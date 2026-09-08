"""
State estimation: delayed, noisy, incomplete observations -> a usable belief.

THE FIXED-LAG ARCHITECTURE
--------------------------
Measurements do not arrive in the order they were taken. Telemetry has ~4 ms
of latency; vision has ~16 ms. So an encoder reading sampled AFTER a camera
frame routinely arrives BEFORE it. A filter that simply absorbs whatever
turns up will apply measurements out of order and quietly corrupt itself.

The fix used here is the one real systems use: run the filter deliberately
behind the present, at `now - FILTER_LAG_S`, and buffer every measurement
until the filter time catches up to its timestamp. Measurements are then
always applied in true chronological order. The lag costs nothing, because
the belief is forward-predicted for decisions anyway -- and it has to be,
since acting on the present is already too late.

    truth ────────────────────────────────────●  now
    filter ─────────────────●  now - 30ms      ╎
                            └──── predict ─────┴──▶ now + latency
                                                       (where we aim)

WHAT EACH SENSOR IS GOOD FOR
----------------------------
    vision      absolute position. Slow, noisy, drops out, but never drifts.
    encoders    velocity. Fast and smooth, but measures WHEELS, not the
                robot -- and on a skid-steer the two diverge badly in turns.

So vision anchors position and encoders sharpen velocity, and the encoder
contribution is gated on how hard the robot is turning. Integrating encoders
for position would be the classic mistake; this module never does it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

import config
from ai.calibration import Calibration, LatencyIdentifier
from sim import chassis
from sim.geometry import wrap_angle, clamp


# A detection further than this from the prediction is treated as a possible
# false positive. Sized well above any plausible one-frame ball movement
# (2 m/s over an 11 ms frame is 22 mm) and well below the pitch, so real
# detections always pass and spurious blobs elsewhere do not.
BALL_GATE_FLOOR_M = 0.25


# ---------------------------------------------------------------------------
# Belief output
# ---------------------------------------------------------------------------

@dataclass
class RobotBelief:
    pos: tuple[float, float] = (0.0, 0.0)
    theta: float = 0.0
    v: float = 0.0
    omega: float = 0.0
    valid: bool = False

    def vel_vector(self) -> tuple[float, float]:
        return (self.v * math.cos(self.theta), self.v * math.sin(self.theta))


@dataclass
class Belief:
    t: float = 0.0
    ball_pos: tuple[float, float] = (0.0, 0.0)
    ball_vel: tuple[float, float] = (0.0, 0.0)
    ball_valid: bool = False
    robots: list[RobotBelief] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Ball filter -- linear KF with rolling drag
# ---------------------------------------------------------------------------

class BallFilter:
    def __init__(self, cal: Calibration) -> None:
        self.x = np.zeros(4)
        self.P = np.eye(4) * 1.0
        sig = cal.position_sigma()
        self.R = np.eye(2) * (sig ** 2)
        self.initialised = False
        self._reject_streak = 0
        self._contested = False

    def set_contested(self, contested: bool) -> None:
        """Tell the filter a robot is close enough to be pushing the ball.

        The process model here is constant velocity plus rolling drag, which
        is exact for a free ball and simply wrong for one being shoved by a
        5 kg robot. Measured with perfect sensors: a free ball tracks to
        0.000 mm, a ball in contact to ~4 mm with spikes past 150 mm.

        Rather than model the collision -- which needs the robot geometry and
        would be a second physics engine inside the estimator -- inflate the
        process noise while contact is plausible. The filter then leans on
        the camera instead of on a model it knows does not apply. This is
        adaptive Q, and it needs no ground truth: robot positions come from
        the filter's own belief.
        """
        self._contested = contested

    def predict(self, dt: float) -> None:
        if dt <= 0:
            return
        F = np.eye(4)
        F[0, 2] = dt
        F[1, 3] = dt
        self.x = F @ self.x

        # Rolling resistance is a constant deceleration opposing motion, not
        # a linear drag. Applied outside F because it is nonlinear; the error
        # this introduces in P is negligible at these timesteps.
        sp = math.hypot(self.x[2], self.x[3])
        if sp > 1e-6:
            new_sp = max(0.0, sp - config.BALL_ROLL_DECEL * dt)
            k = new_sp / sp
            self.x[2] *= k
            self.x[3] *= k

        # Process noise. Velocity noise is large because collisions are
        # entirely unmodelled here -- the ball can reverse in one timestep and
        # the filter must be willing to believe it.
        q_pos = (0.004 ** 2) * dt
        q_vel = (2.5 ** 2) * dt
        if self._contested:
            # A robot is close enough to be accelerating the ball, so the
            # constant-velocity model does not hold. Widen the covariance and
            # let the camera lead.
            q_vel *= config.BALL_CONTESTED_Q_GAIN
            q_pos *= config.BALL_CONTESTED_Q_GAIN
        Q = np.diag([q_pos, q_pos, q_vel, q_vel])
        self.P = F @ self.P @ F.T + Q

    def update(self, z: tuple[float, float]) -> bool:
        zz = np.array(z)
        if not self.initialised:
            self.x[:2] = zz
            self.x[2:] = 0.0
            self.P = np.eye(4) * 0.5
            self.initialised = True
            return True

        H = np.zeros((2, 4))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        y = zz - H @ self.x
        S = H @ self.P @ H.T + self.R

        try:
            d2 = float(y.T @ np.linalg.inv(S) @ y)
        except np.linalg.LinAlgError:
            d2 = 0.0
        innov_m = float(math.hypot(y[0], y[1]))

        # Outlier gate, on TWO conditions that must BOTH hold.
        #
        # A Mahalanobis test alone is wrong here. When the filter is
        # confident (small P) and the sensor is good (small R), S collapses
        # and d2 explodes for a millimetre of innovation -- so the gate
        # starts rejecting perfectly good detections precisely when the ball
        # is struck and the constant-velocity model briefly stops applying.
        # That is the worst possible moment to stop believing the camera.
        #
        # What actually distinguishes a false-positive blob is that it is
        # somewhere else entirely -- metres away, not millimetres. So the
        # absolute floor is the real discriminator, and the statistical test
        # only refines it.
        if d2 > 60.0 and innov_m > BALL_GATE_FLOOR_M:
            self._reject_streak += 1
            if self._reject_streak > 5:
                # We have rejected too long: the filter, not the camera, is
                # the one that is wrong. Re-acquire on the measurement, but
                # KEEP the velocity estimate -- zeroing it throws away the
                # one thing that survives an occlusion intact and makes the
                # ball appear to stop dead every time it is re-found.
                self.x[:2] = zz
                self.P = np.eye(4) * 0.5
                self._reject_streak = 0
                return True
            return False
        self._reject_streak = 0

        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        return True

    @property
    def pos(self):
        return (float(self.x[0]), float(self.x[1]))

    @property
    def vel(self):
        return (float(self.x[2]), float(self.x[3]))


# ---------------------------------------------------------------------------
# Robot filter -- EKF on the unicycle model
# ---------------------------------------------------------------------------

class RobotFilter:
    """State [x, y, theta, v, omega].

    The unicycle constraint is worth having: a skid-steer robot's velocity is
    along its heading (give or take scrub), so knowing heading well makes the
    position prediction much better than a free 2D velocity model would.
    """

    def __init__(self, cal: Calibration) -> None:
        self.x = np.zeros(5)
        self.P = np.eye(5)
        self.cal = cal
        pos_sig = cal.position_sigma()
        self.R_pos = np.eye(2) * (pos_sig ** 2)
        self.R_theta = np.array([[cal.heading_sigma() ** 2]])
        self.initialised = False
        self.cmd = (0.0, 0.0)          # last known commanded (v, omega)
        self.last_slip_scale = 1.0

    def set_command(self, v: float, omega: float) -> None:
        self.cmd = (v, omega)

    def predict(self, dt: float) -> None:
        if dt <= 0:
            return
        x, y, th, v, w = self.x
        tau = max(config.DRIVETRAIN_TAU_S, 1e-3)
        a = 1.0 - math.exp(-dt / tau)
        vc, wc = self.cmd

        nx = x + v * math.cos(th) * dt
        ny = y + v * math.sin(th) * dt
        nth = wrap_angle(th + w * dt)
        nv = v + (vc - v) * a
        nw = w + (wc - w) * a

        F = np.eye(5)
        F[0, 2] = -v * math.sin(th) * dt
        F[0, 3] = math.cos(th) * dt
        F[1, 2] = v * math.cos(th) * dt
        F[1, 3] = math.sin(th) * dt
        F[2, 4] = dt
        F[3, 3] = 1.0 - a
        F[4, 4] = 1.0 - a

        self.x = np.array([nx, ny, nth, nv, nw])

        q_pos = (0.003 ** 2) * dt
        q_th = (0.02 ** 2) * dt
        q_v = (1.2 ** 2) * dt
        q_w = (6.0 ** 2) * dt
        Q = np.diag([q_pos, q_pos, q_th, q_v, q_w])
        self.P = F @ self.P @ F.T + Q

    def update_position(self, z) -> None:
        zz = np.array(z)
        if not self.initialised:
            self.x[0:2] = zz
            self.P = np.eye(5) * 0.5
            self.initialised = True
            return
        H = np.zeros((2, 5))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        y = zz - H @ self.x
        S = H @ self.P @ H.T + self.R_pos
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P
        self.x[2] = wrap_angle(self.x[2])

    def update_heading(self, theta_meas: float) -> None:
        H = np.zeros((1, 5))
        H[0, 2] = 1.0
        # Innovation MUST be wrapped. Without this, a robot sitting at +pi
        # jittering to -pi produces a 2*pi innovation and the filter explodes.
        y = np.array([wrap_angle(theta_meas - self.x[2])])
        S = H @ self.P @ H.T + self.R_theta
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y).ravel()
        self.P = (np.eye(5) - K @ H) @ self.P
        self.x[2] = wrap_angle(self.x[2])

    def update_encoder_twist(self, v_meas: float, w_meas: float) -> None:
        """Fuse encoder-derived body twist, discounted by expected scrub.

        The measurement is computed from wheel speeds through a fixed
        kinematic model. That model is wrong in a way that grows with turn
        rate, so rather than trust it uniformly we inflate its covariance
        with |omega|. At a standstill the encoders are excellent; mid-spin
        they are close to useless, and the filter should know the difference.
        """
        scale = 1.0 + config.SLIP_GATE_GAIN * abs(w_meas)
        if not config.USE_SLIP_GATING:
            scale = 1.0
        self.last_slip_scale = scale

        R = np.diag([(0.05 * scale) ** 2, (0.30 * scale) ** 2])
        H = np.zeros((2, 5))
        H[0, 3] = 1.0
        H[1, 4] = 1.0
        y = np.array([v_meas, w_meas]) - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P
        self.x[2] = wrap_angle(self.x[2])

    def belief(self) -> RobotBelief:
        return RobotBelief(
            pos=(float(self.x[0]), float(self.x[1])),
            theta=float(self.x[2]),
            v=float(self.x[3]),
            omega=float(self.x[4]),
            valid=self.initialised,
        )


# ---------------------------------------------------------------------------
# Estimator
# ---------------------------------------------------------------------------

class Estimator:
    def __init__(self, self_index: int) -> None:
        self.cal = Calibration()
        self.self_index = self_index
        self.opp_index = 1 - self_index

        self.ball = BallFilter(self.cal)
        self.robots = [RobotFilter(self.cal), RobotFilter(self.cal)]

        self.latency_id = LatencyIdentifier()
        self.filter_t = 0.0
        self._started = False

        self._pending: list[tuple[float, str, object]] = []
        self._last_vision_t = -1.0
        self._last_telem_t = -1.0

        self.vision_gap = 0.0        # seconds since the last usable ball fix
        # total_latency: command -> visible in belief (what the identifier
        # measures). actuation_delay: command -> robot responds, which is the
        # only part the forward predictor should add on top of the filter lag.
        self.total_latency = config.FIXED_LATENCY_S
        self.actuation_delay = config.FIXED_LATENCY_S
        # The opponent's live command, when the intent channel is available.
        # Read by the planner; never fitted, never stored across matches.
        self._last_opp_intent: tuple[float, float] | None = None

    # -- measurement intake ------------------------------------------------

    def _ingest_vision(self, frame) -> None:
        if frame is None or frame.t_exposure <= self._last_vision_t:
            return
        self._last_vision_t = frame.t_exposure
        self._pending.append((frame.t_exposure, "vision", frame))

    def _ingest_telemetry(self, packet) -> None:
        if packet is None or packet.t_sample <= self._last_telem_t:
            return
        self._last_telem_t = packet.t_sample
        self._pending.append((packet.t_sample, "telem", packet))

    # -- measurement application -------------------------------------------

    def _apply_vision(self, frame) -> None:
        # Ball
        if frame.ball_px is not None:
            z = self.cal.ball_to_world(frame.ball_px)
            if self.ball.update(z):
                self.vision_gap = 0.0
        # A spurious blob is offered to the same gate the real one passes
        # through -- which is the honest test of whether the gate works.
        elif frame.false_ball_px is not None:
            z = self.cal.ball_to_world(frame.false_ball_px)
            self.ball.update(z)

        # Robots
        for i, mk in enumerate(frame.markers):
            if i >= len(self.robots):
                break
            pos, theta = self.cal.pose_from_markers(mk.centre_px, mk.dot_px)
            if pos is not None:
                self.robots[i].update_position(pos)
            if theta is not None:
                self.robots[i].update_heading(theta)

    def _apply_telemetry(self, packet) -> None:
        for i, rt in enumerate(packet.robots):
            if i >= len(self.robots) or not rt.wheel_rads:
                continue
            if i == self.opp_index and not config.USE_OPPONENT_ENCODERS:
                continue
            if i == self.self_index and not config.USE_ENCODER_FUSION:
                continue
            wl = (rt.wheel_rads[0] + rt.wheel_rads[2]) / 2.0   # FL, RL
            wr = (rt.wheel_rads[1] + rt.wheel_rads[3]) / 2.0   # FR, RR
            v, w = chassis.wheels_to_twist(wl, wr)
            self.robots[i].update_encoder_twist(v, w)

    # -- main --------------------------------------------------------------

    def step(self, sensors, own_command: tuple[float, float]) -> None:
        now = sensors.now()
        if not self._started:
            self.filter_t = now - config.FILTER_LAG_S
            self._started = True

        self._ingest_vision(sensors.vision())
        self._ingest_telemetry(sensors.telemetry())

        # Our own commanded twist drives the prediction for our own filter.
        self.robots[self.self_index].set_command(*own_command)

        # The opponent's command, if the intent channel is available. This is
        # the only place it enters the estimator, and it is used exactly as
        # our own command is: as a control input to a known dynamics model.
        opp_cmd = sensors.opponent_intent()
        self._last_opp_intent = opp_cmd
        if opp_cmd is not None:
            self.robots[self.opp_index].set_command(*opp_cmd)

        target_t = now - config.FILTER_LAG_S
        if target_t <= self.filter_t:
            return

        # Apply everything whose timestamp falls in the interval we are about
        # to cross, in strict chronological order.
        due = [m for m in self._pending if m[0] <= target_t]
        due.sort(key=lambda m: m[0])
        self._pending = [m for m in self._pending if m[0] > target_t]

        for t_meas, kind, data in due:
            dt = t_meas - self.filter_t
            if dt > 0:
                self._predict_all(dt)
                self.filter_t = t_meas
            if kind == "vision":
                self._apply_vision(data)
            else:
                self._apply_telemetry(data)

        remaining = target_t - self.filter_t
        if remaining > 0:
            self._predict_all(remaining)
            self.filter_t = target_t

        self.vision_gap = now - max(self._last_vision_t, 0.0)

        # Latency identification, from our own command against our own
        # observed motion. Never involves the opponent.
        me = self.robots[self.self_index]
        self.latency_id.push_command(now, own_command[0])
        if me.initialised:
            self.latency_id.push_observation(self.filter_t, float(me.x[3]))
        self.total_latency = self.latency_id.update(now)

        # The identifier compares commands stamped at `now` against beliefs
        # stamped at `filter_t`, so the lag it recovers ALREADY contains the
        # filter lag. What the predictor actually needs is only the part
        # after the belief: the actuation delay (radio + motor response).
        #
        # Adding the identified figure on top of (now - filter_t) double
        # counts the filter lag and makes the robot aim roughly 30 ms too far
        # ahead -- about 60 mm of lead error at full speed, which reads as
        # the robot consistently overshooting the ball.
        if config.AUTO_LATENCY_ID:
            self.actuation_delay = max(
                0.0, self.total_latency - config.FILTER_LAG_S)
        else:
            self.actuation_delay = config.FIXED_LATENCY_S

    def _mark_contested(self) -> None:
        """Flag the ball as contested using ONLY the filter's own belief."""
        bp = self.ball.pos
        reach = (config.ROBOT_LENGTH_M / 2.0 + config.HORN_LENGTH_M
                 + config.BALL_RADIUS_M + 0.05)
        near = False
        for r in self.robots:
            if not r.initialised:
                continue
            dx = float(r.x[0]) - bp[0]
            dy = float(r.x[1]) - bp[1]
            if dx * dx + dy * dy < reach * reach:
                near = True
                break
        self.ball.set_contested(near)

    def _predict_all(self, dt: float) -> None:
        self._mark_contested()
        # Cap the step so a long stall cannot integrate one huge jump.
        while dt > 0:
            step = min(dt, 0.02)
            self.ball.predict(step)
            for r in self.robots:
                r.predict(step)
            dt -= step

    # -- output ------------------------------------------------------------

    def belief(self) -> Belief:
        return Belief(
            t=self.filter_t,
            ball_pos=self.ball.pos,
            ball_vel=self.ball.vel,
            ball_valid=self.ball.initialised,
            robots=[r.belief() for r in self.robots],
        )

    def compensated_belief(self, now: float) -> Belief:
        """Belief forward-predicted to where the world will be when our
        command actually lands.

        This is the single highest-value correction in the project. Without
        it every decision is made against a world that is 30-50 ms stale, the
        controller has to be detuned to stay stable, and the robot feels
        sluggish and always slightly behind the ball.
        """
        b = self.belief()
        if not config.USE_LATENCY_COMP:
            return b

        horizon = (now - self.filter_t) + self.actuation_delay
        horizon = clamp(horizon, 0.0, 0.35)

        bp, bv = predict_ball(b.ball_pos, b.ball_vel, horizon)
        out = Belief(t=now + self.total_latency, ball_pos=bp, ball_vel=bv,
                     ball_valid=b.ball_valid, robots=[])
        for rb in b.robots:
            out.robots.append(predict_robot(rb, horizon))
        return out


# ---------------------------------------------------------------------------
# Forward models -- shared by the estimator, the planner and reachability
# ---------------------------------------------------------------------------

def predict_ball(pos, vel, horizon: float, bounce: bool = True):
    """Roll the ball forward, including wall bounces.

    Bounces matter: a prediction that ignores them sends the robot chasing a
    point beyond the wall. The goal mouths are treated as openings, so a ball
    heading in does not bounce off a wall that is not there.
    """
    x, y = pos
    vx, vy = vel
    dt = 0.01
    t = 0.0
    hx, hy = config.HALF_LENGTH_M, config.HALF_WIDTH_M
    r = config.BALL_RADIUS_M
    e = config.WALL_RESTITUTION

    while t < horizon:
        step = min(dt, horizon - t)
        sp = math.hypot(vx, vy)
        if sp > 1e-6:
            ns = max(0.0, sp - config.BALL_ROLL_DECEL * step)
            k = ns / sp
            vx *= k
            vy *= k
        x += vx * step
        y += vy * step

        if bounce:
            if y + r > hy and vy > 0:
                y = hy - r
                vy = -vy * e
            elif y - r < -hy and vy < 0:
                y = -hy + r
                vy = -vy * e
            in_mouth = abs(y) <= config.HALF_GOAL_M
            if not in_mouth:
                if x + r > hx and vx > 0:
                    x = hx - r
                    vx = -vx * e
                elif x - r < -hx and vx < 0:
                    x = -hx + r
                    vx = -vx * e
        t += step
    return (x, y), (vx, vy)


def predict_robot(rb: RobotBelief, horizon: float,
                  cmd: tuple[float, float] | None = None) -> RobotBelief:
    """Roll a robot forward under the unicycle model.

    With `cmd`, the robot is assumed to be driving toward that command
    through the known first-order lag. Without it, current velocities simply
    persist and decay.
    """
    x, y = rb.pos
    th, v, w = rb.theta, rb.v, rb.omega
    tau = max(config.DRIVETRAIN_TAU_S, 1e-3)
    dt = 0.01
    t = 0.0
    while t < horizon:
        step = min(dt, horizon - t)
        if cmd is not None:
            a = 1.0 - math.exp(-step / tau)
            v += (cmd[0] - v) * a
            w += (cmd[1] - w) * a
        x += v * math.cos(th) * step
        y += v * math.sin(th) * step
        th = wrap_angle(th + w * step)
        t += step
    return RobotBelief(pos=(x, y), theta=th, v=v, omega=w, valid=rb.valid)
