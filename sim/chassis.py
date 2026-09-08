"""
Skid-steer chassis dynamics.

A skid-steer robot cannot turn without dragging its wheels sideways. That
scrub is not a detail to paper over -- it is the defining characteristic of
the drivetrain, and it produces, for free, every behaviour that makes these
robots annoying to control:

  * the robot turns more slowly than the wheel-speed difference implies,
    so its effective track is wider than its geometric track
  * the encoders over-report motion during turns, because the wheels are
    spinning against a surface they are also sliding across
  * hard turns eat traction that is then unavailable for acceleration
  * the turn rate depends on the surface, the load, and the speed

Rather than fudge these with a coefficient, the model computes a friction
force at each of the four contact patches from the local slip velocity, caps
each by a friction ellipse, and sums forces and moments. Everything above
then emerges rather than being imposed.

Coordinate conventions:
    world frame   x along the pitch length, y across it
    body frame    +x forward (the direction the horns point), +y left
"""

from __future__ import annotations

import math

import config
import motors
from sim import drivetrain
from sim.geometry import (
    Vec, rotate, to_world, clamp,
)

GRAVITY = 9.81


class Chassis:
    """Rigid body plus four driven wheels."""

    def __init__(self, motor: motors.Motor, x: float, y: float,
                 theta: float) -> None:
        self.pos: Vec = (x, y)
        self.theta = theta
        self.vel: Vec = (0.0, 0.0)      # world frame
        self.omega = 0.0                # yaw rate, rad/s

        self.mass = config.ROBOT_MASS_KG
        self.inertia = config.robot_inertia()

        self.wheels = drivetrain.make_wheels(motor)
        self.motor = motor

        # Previous body-frame acceleration, used for weight transfer.
        self._accel_body: Vec = (0.0, 0.0)

        # Accumulated external impulses (collisions), applied next step.
        self._impulse: Vec = (0.0, 0.0)
        self._angular_impulse = 0.0

        self.battery_v = config.BATTERY_NOMINAL_V
        self._pid_accum = 0.0
        self._enc_accum = 0.0

    # -- command interface -------------------------------------------------

    def set_wheel_targets(self, omega_left: float, omega_right: float) -> None:
        """Targets in rad/s at the wheel. Left pair and right pair.

        This is what the radio actually carries: four motors, two independent
        sides. Converting a desired (v, omega) into these is the caller's job,
        and the track width they assume is a choice they can get wrong.
        """
        self.wheels[0].target_omega = omega_left    # FL
        self.wheels[2].target_omega = omega_left    # RL
        self.wheels[1].target_omega = omega_right   # FR
        self.wheels[3].target_omega = omega_right   # RR

    def stop(self) -> None:
        self.set_wheel_targets(0.0, 0.0)

    # -- forces ------------------------------------------------------------

    def _normal_loads(self) -> list[float]:
        """Per-wheel vertical load, including weight transfer.

        Static load is mg/4. Accelerating forward shifts load to the rear
        wheels; turning shifts it to the outside. On a robot this short
        (120 mm wheelbase, 50 mm CoG) the transfer at full acceleration is
        comparable to the static load itself, so the front wheels genuinely
        run out of grip first.

        Uses the previous step's acceleration, which at 1 kHz is stable and
        avoids an implicit solve.
        """
        static = self.mass * GRAVITY / 4.0
        ax, ay = self._accel_body
        h = config.COG_HEIGHT_M

        long_shift = self.mass * ax * h / (2.0 * config.WHEELBASE_M)
        lat_shift = self.mass * ay * h / (2.0 * config.TRACK_WIDTH_M)

        loads = []
        for w in self.wheels:
            lx, ly = w.position_local
            n = static
            n -= long_shift if lx > 0 else -long_shift   # front loses, rear gains
            n -= lat_shift if ly > 0 else -lat_shift     # left loses on left turn
            loads.append(max(n, 0.0))                    # a lifted wheel makes no force
        return loads

    def _tyre_forces(self) -> tuple[Vec, float, list[float]]:
        """Compute per-wheel ground forces.

        Returns (total force in BODY frame, total yaw moment, per-wheel
        longitudinal force) -- the last is fed back into each wheel's
        rotational equation of motion.
        """
        loads = self._normal_loads()

        # Body-frame velocity of the chassis centre.
        v_body = rotate(self.vel, -self.theta)

        fx_total = 0.0
        fy_total = 0.0
        moment = 0.0
        long_forces = []

        for w, load in zip(self.wheels, loads):
            lx, ly = w.position_local

            # Velocity of this contact patch, in the body frame.
            vx = v_body[0] - self.omega * ly
            vy = v_body[1] + self.omega * lx

            # Longitudinal slip: how much faster the tyre surface is moving
            # than the ground beneath it. Positive means the wheel is driving.
            slip_long = w.omega * config.WHEEL_RADIUS_M - vx
            slip_lat = vy

            w.slip_long = slip_long
            w.slip_lat = slip_lat
            w.normal_load = load

            ref = config.SLIP_REFERENCE_MPS
            fx = config.MU_LONGITUDINAL * load * math.tanh(slip_long / ref)
            fy = -config.MU_LATERAL * load * math.tanh(slip_lat / ref)

            # Friction ellipse: the two directions share one contact patch,
            # so hard cornering really does steal acceleration.
            cap_x = config.MU_LONGITUDINAL * load
            cap_y = config.MU_LATERAL * load
            if cap_x > 1e-9 and cap_y > 1e-9:
                excess = math.hypot(fx / cap_x, fy / cap_y)
                if excess > 1.0:
                    fx /= excess
                    fy /= excess

            w.ground_force = fx
            long_forces.append(fx)

            fx_total += fx
            fy_total += fy
            moment += lx * fy - ly * fx

        return (fx_total, fy_total), moment, long_forces

    # -- integration -------------------------------------------------------

    def apply_impulse(self, impulse: Vec, point: Vec) -> None:
        """Accumulate a collision impulse applied at a world-space point."""
        self._impulse = (self._impulse[0] + impulse[0],
                         self._impulse[1] + impulse[1])
        rx = point[0] - self.pos[0]
        ry = point[1] - self.pos[1]
        self._angular_impulse += rx * impulse[1] - ry * impulse[0]

    def step(self, dt: float, t: float) -> None:
        # --- on-robot loops, at their own rates ---------------------------
        self._enc_accum += dt
        enc_dt = 1.0 / config.ENCODER_SAMPLE_HZ
        while self._enc_accum >= enc_dt:
            self._enc_accum -= enc_dt
            for w in self.wheels:
                w.sample_encoder(t)

        self._pid_accum += dt
        pid_dt = 1.0 / config.FIRMWARE_PID_HZ
        while self._pid_accum >= pid_dt:
            self._pid_accum -= pid_dt
            for w in self.wheels:
                w.run_pid(pid_dt)

        # --- electrics ----------------------------------------------------
        self.battery_v = drivetrain.battery_voltage(self.wheels)
        motor_torques = []
        for w in self.wheels:
            tau = w.electrical_torque(self.battery_v)
            tau = w.apply_backlash(tau, dt)
            tau += w.friction_torque()
            w.motor_torque = tau
            motor_torques.append(tau)

        # --- tyres --------------------------------------------------------
        (fx_body, fy_body), moment, long_forces = self._tyre_forces()

        # --- wheel rotational dynamics ------------------------------------
        # The ground pushes back on the wheel it is driving. A wheel whose
        # motor makes more torque than the tyre can transmit accelerates and
        # spins -- no special case needed, it falls out of this equation.
        for w, tau, fx in zip(self.wheels, motor_torques, long_forces):
            reaction = fx * config.WHEEL_RADIUS_M
            w.integrate(tau - reaction, dt)

        # --- body dynamics ------------------------------------------------
        force_world = rotate((fx_body, fy_body), self.theta)

        ax = force_world[0] / self.mass + self._impulse[0] / (self.mass * dt)
        ay = force_world[1] / self.mass + self._impulse[1] / (self.mass * dt)
        alpha = moment / self.inertia + self._angular_impulse / (self.inertia * dt)

        self._impulse = (0.0, 0.0)
        self._angular_impulse = 0.0

        self.vel = (self.vel[0] + ax * dt, self.vel[1] + ay * dt)
        self.omega += alpha * dt

        self.pos = (self.pos[0] + self.vel[0] * dt,
                    self.pos[1] + self.vel[1] * dt)
        self.theta += self.omega * dt

        # Cache body-frame acceleration for the next step's weight transfer.
        self._accel_body = rotate((ax, ay), -self.theta)

    # -- queries -----------------------------------------------------------

    def body_velocity(self) -> Vec:
        return rotate(self.vel, -self.theta)

    def forward_speed(self) -> float:
        return self.body_velocity()[0]

    def wheel_world_positions(self) -> list[Vec]:
        return [to_world(w.position_local, self.pos, self.theta)
                for w in self.wheels]

    def encoder_counts(self) -> list[int]:
        return [w.encoder.count for w in self.wheels]

    def encoder_velocities(self) -> list[float]:
        return [w.encoder.velocity_rads for w in self.wheels]

    def total_slip(self) -> float:
        """Aggregate scrub, for the diagnostics overlay."""
        return sum(abs(w.slip_lat) for w in self.wheels) / 4.0

    def reset(self, x: float, y: float, theta: float) -> None:
        self.pos = (x, y)
        self.theta = theta
        self.vel = (0.0, 0.0)
        self.omega = 0.0
        self._accel_body = (0.0, 0.0)
        self._impulse = (0.0, 0.0)
        self._angular_impulse = 0.0
        for w in self.wheels:
            w.reset()


# ---------------------------------------------------------------------------
# Kinematics helpers
#
# Converting a desired (forward speed, yaw rate) into wheel speeds requires
# assuming a track width. The GEOMETRIC track is what the robot physically
# has; the EFFECTIVE track is what it behaves as though it has, because of
# scrub. Anyone using the geometric value will under-turn.
# ---------------------------------------------------------------------------

def twist_to_wheels(v: float, omega: float,
                    track_m: float | None = None) -> tuple[float, float]:
    """(forward m/s, yaw rad/s) -> (left, right) wheel speeds in rad/s."""
    if track_m is None:
        track_m = config.effective_track_m()
    r = config.WHEEL_RADIUS_M
    v_left = (v - omega * track_m / 2.0) / r
    v_right = (v + omega * track_m / 2.0) / r
    return v_left, v_right


def wheels_to_twist(omega_left: float, omega_right: float,
                    track_m: float | None = None) -> tuple[float, float]:
    """Inverse of twist_to_wheels."""
    if track_m is None:
        track_m = config.effective_track_m()
    r = config.WHEEL_RADIUS_M
    v = (omega_left + omega_right) * r / 2.0
    omega = (omega_right - omega_left) * r / track_m
    return v, omega


def max_wheel_speed() -> float:
    """No-load wheel speed at nominal voltage."""
    return motors.get(config.MOTOR).noload_rads


def max_forward_speed() -> float:
    return max_wheel_speed() * config.WHEEL_RADIUS_M
