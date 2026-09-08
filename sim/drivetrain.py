"""
Per-wheel drivetrain: encoder, firmware PID, motor electrics, backlash.

One `Wheel` per motor -- four per robot. Each wheel owns its own encoder and
its own velocity loop, exactly as it would on the real robot, where the
microcontroller closes a fast loop per motor and the PC only sends targets.

The chain per timestep, in order:

    true wheel angle
      -> encoder quantisation           (counts are integers)
      -> finite-difference velocity     (coarse at low speed)
      -> firmware PID                   (sees the QUANTISED velocity, as it
                                         would in reality -- not the truth)
      -> duty cycle, deadband, PWM quantisation
      -> battery voltage (sagging under total current draw)
      -> motor torque  Kt * (V - Ke*w) / R
      -> backlash gate                  (no torque while crossing the slop)
      -> friction (Coulomb + viscous)
      -> net torque handed to the chassis

The chassis then computes the ground reaction force and returns it, and the
wheel integrates its own angular acceleration from the difference. Motor and
chassis are genuinely coupled -- a wheel with more torque than the tyre can
transmit spins up and loses grip, which is what should happen.

A note on what is NOT modelled: motor electrical inductance (L/R is ~1 ms
here, an order of magnitude below the mechanical time constant, so it would
be invisible) and rotor temperature derating.
"""

from __future__ import annotations

import math
from collections import deque

import config
import motors
from sim.geometry import clamp, sign


class Encoder:
    """Quadrature encoder on the motor shaft.

    Counts integer ticks, so velocity from finite differencing is badly
    quantised at low speed -- a real and frequently underestimated problem.
    With 1216 counts/output-rev sampled at 1 kHz, one tick per sample already
    corresponds to about 0.2 rad/s at the output shaft, so anything slower
    than that reads as a stream of zeros and ones rather than a smooth value.
    """

    def __init__(self, counts_per_output_rev: float, sample_hz: float,
                 window: int) -> None:
        self.counts_per_rad = counts_per_output_rev / (2.0 * math.pi)
        self.sample_dt = 1.0 / sample_hz
        self.window = max(2, int(window))
        self._history: deque[tuple[float, int]] = deque(maxlen=self.window)
        self.count = 0
        self.velocity_rads = 0.0
        self._accum = 0.0

    def reset(self) -> None:
        self._history.clear()
        self.count = 0
        self.velocity_rads = 0.0
        self._accum = 0.0

    def sample(self, true_angle_rad: float, t: float) -> None:
        """Latch a new integer count and recompute the velocity estimate."""
        self.count = int(true_angle_rad * self.counts_per_rad)
        self._history.append((t, self.count))
        if len(self._history) >= 2:
            t0, c0 = self._history[0]
            t1, c1 = self._history[-1]
            dt = t1 - t0
            if dt > 1e-9:
                self.velocity_rads = (c1 - c0) / self.counts_per_rad / dt


class Wheel:
    """One driven wheel: encoder, velocity loop, motor, gearbox slop."""

    def __init__(self, motor: motors.Motor, position_local: tuple[float, float],
                 side: int) -> None:
        self.motor = motor
        self.position_local = position_local   # (x, y) in the robot frame
        self.side = side                       # +1 left, -1 right

        self.omega = 0.0          # true wheel angular velocity, rad/s
        self.angle = 0.0          # true accumulated wheel angle, rad
        self.target_omega = 0.0   # from the firmware, rad/s

        self.encoder = Encoder(
            motor.encoder_cpr_output,
            config.ENCODER_SAMPLE_HZ,
            config.ENCODER_VELOCITY_WINDOW,
        )

        # Firmware PID state
        self._integral = 0.0
        self._prev_error = 0.0
        self._pid_accum = 0.0
        self.duty = 0.0

        # Gearbox backlash: which direction the teeth are currently loaded in,
        # and how much slop remains to traverse before they load the other way.
        self._engaged_sign = 0.0
        self._lash_remaining = 0.0

        # Diagnostics (read by the renderer and the slip HUD)
        self.current_a = 0.0
        self.motor_torque = 0.0
        self.ground_force = 0.0
        self.slip_long = 0.0
        self.slip_lat = 0.0
        self.normal_load = 0.0

        self.inertia = motor.inertia_output_kgm2 + config.WHEEL_INERTIA_KGM2

    # -- encoder / firmware ------------------------------------------------

    def sample_encoder(self, t: float) -> None:
        self.encoder.sample(self.angle, t)

    def run_pid(self, dt: float) -> None:
        """On-robot velocity loop. Deliberately fed the quantised encoder
        velocity rather than the true wheel speed."""
        measured = self.encoder.velocity_rads
        error = self.target_omega - measured

        self._integral += error * dt
        self._integral = clamp(self._integral,
                               -config.FIRMWARE_I_LIMIT / max(config.FIRMWARE_KI, 1e-9),
                               config.FIRMWARE_I_LIMIT / max(config.FIRMWARE_KI, 1e-9))
        derivative = (error - self._prev_error) / dt if dt > 1e-9 else 0.0
        self._prev_error = error

        duty = (config.FIRMWARE_KP * error
                + config.FIRMWARE_KI * self._integral
                + config.FIRMWARE_KD * derivative)

        duty = clamp(duty, -1.0, 1.0)

        # Anti-windup: stop integrating once saturated.
        if abs(duty) >= 1.0 and sign(error) == sign(self._integral):
            self._integral -= error * dt

        # PWM quantisation
        levels = (1 << config.PWM_BITS) - 1
        duty = round(duty * levels) / levels

        # Static friction deadband: below this the motor simply does not turn.
        if abs(duty) < config.MOTOR_DEADBAND_DUTY:
            duty = 0.0

        self.duty = duty

    # -- physics -----------------------------------------------------------

    def electrical_torque(self, battery_v: float) -> float:
        """Motor torque before friction and before the backlash gate."""
        voltage = self.duty * battery_v
        self.current_a = self.motor.current_at(self.omega, voltage)
        return self.motor.kt * self.current_a

    def apply_backlash(self, torque: float, dt: float) -> float:
        """Gate torque while the gear teeth cross the slop.

        When the driving torque reverses, the teeth must travel the full
        backlash before they contact on the other flank. During that
        traversal no torque reaches the wheel. This is what makes a
        high-ratio gearbox feel vague when reversing direction, and it is
        the part of backlash the controller cannot compensate away.
        """
        s = sign(torque)
        if s == 0.0:
            return 0.0

        if self._engaged_sign == 0.0:
            self._engaged_sign = s
            return torque

        if s == self._engaged_sign:
            self._lash_remaining = 0.0
            return torque

        # Direction reversed: open the slop and traverse it.
        if self._lash_remaining <= 0.0:
            self._lash_remaining = config.GEARBOX_BACKLASH_RAD

        self._lash_remaining -= abs(self.omega) * dt
        if self._lash_remaining <= 0.0:
            self._engaged_sign = s
            self._lash_remaining = 0.0
            return torque
        return 0.0

    def friction_torque(self) -> float:
        """Coulomb plus viscous drag, always opposing motion."""
        visc = -config.MOTOR_VISCOUS_DAMPING * self.omega
        if abs(self.omega) < 1e-6:
            coul = 0.0
        else:
            coul = -config.MOTOR_COULOMB_FRICTION * sign(self.omega)
        return visc + coul

    def integrate(self, net_torque: float, dt: float) -> None:
        alpha = net_torque / self.inertia
        self.omega += alpha * dt
        self.angle += self.omega * dt

    def reset(self) -> None:
        self.omega = 0.0
        self.angle = 0.0
        self.target_omega = 0.0
        self._integral = 0.0
        self._prev_error = 0.0
        self.duty = 0.0
        self._engaged_sign = 0.0
        self._lash_remaining = 0.0
        self.current_a = 0.0
        self.motor_torque = 0.0
        self.ground_force = 0.0
        self.encoder.reset()


def make_wheels(motor: motors.Motor) -> list[Wheel]:
    """Four wheels in the robot frame.

    Order is fixed and relied upon elsewhere:
        0 = front-left, 1 = front-right, 2 = rear-left, 3 = rear-right
    """
    hx = config.WHEELBASE_M / 2.0
    hy = config.TRACK_WIDTH_M / 2.0
    return [
        Wheel(motor, (+hx, +hy), side=+1),   # FL
        Wheel(motor, (+hx, -hy), side=-1),   # FR
        Wheel(motor, (-hx, +hy), side=+1),   # RL
        Wheel(motor, (-hx, -hy), side=-1),   # RR
    ]


WHEEL_NAMES = ("FL", "FR", "RL", "RR")


def battery_voltage(wheels: list[Wheel]) -> float:
    """Terminal voltage after internal-resistance sag.

    All four motors share one pack, so a hard launch on all wheels pulls the
    rail down and every motor gets weaker at once. This is why a robot that
    accelerates fine on blocks bogs down on the pitch.
    """
    total_current = sum(abs(w.current_a) for w in wheels)
    v = config.BATTERY_NOMINAL_V - total_current * config.BATTERY_INTERNAL_R
    return max(v, 0.0)
