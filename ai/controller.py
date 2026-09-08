"""
Low-level motion control: a target pose in, wheel speeds out.

The robot is non-holonomic, which shapes everything here. It cannot strafe.
To reach a point beside it, it must turn first, and to reach a point behind
it, the fastest route is very often to REVERSE rather than execute a
three-point turn. Any controller that refuses to drive backwards throws away
roughly half the robot's agility, so this one picks whichever direction
gives the smaller heading error and commits.

Speed is gated on heading error (via cos), so the robot slows while badly
aligned and accelerates as it comes onto line. Without that gate it drives
fast along a curve toward where the target used to be, which on a skid-steer
means scrubbing away most of its traction for nothing.
"""

from __future__ import annotations

import math

import config
from game.input import max_forward_speed, max_yaw_rate, twist_to_wheels_clamped
from sim.geometry import wrap_angle, clamp


# Gains. Tuned against the latency-compensated belief -- if latency
# compensation is disabled these are too hot and the robot will oscillate,
# which is exactly the lesson the flag exists to teach.
K_HEADING = 3.2
K_RANGE = 3.4
ARRIVE_RADIUS = 0.05
SPIN_ERROR = math.radians(70.0)     # beyond this, turn in place first


def limits() -> tuple[float, float]:
    return (max_forward_speed() * config.SPEED_CAP_FRAC,
            max_yaw_rate() * config.SPEED_CAP_FRAC)


def drive_to(pos, theta, target, allow_reverse: bool = True,
             final_heading: float | None = None,
             slow_radius: float = 0.35,
             through: bool = False) -> tuple[float, float]:
    """Return the (v, omega) that drives from the current pose to `target`.

    `through` means the target is a VIA-POINT, not a destination: drive at it
    without decelerating on approach and without stopping on arrival.

    This distinction is why a player who simply drives forward used to win
    every time. Defensive and line-up targets sit close to the robot, so the
    arrival slowdown applied almost permanently: the AI commanded over 1 m/s
    only 26% of the time and spent 57% of the match below 0.15 m/s, against a
    2.6 m/s top speed and with nothing blocking it. It was not out-thought,
    it was out-run -- it shuffled between waypoints while the opponent
    sprinted.

    Somewhere to be on the way is not somewhere to stop.
    """
    v_max, w_max = limits()
    if through:
        slow_radius = 0.0

    dx = target[0] - pos[0]
    dy = target[1] - pos[1]
    rng = math.hypot(dx, dy)

    if rng < ARRIVE_RADIUS and not through:
        if final_heading is None:
            return 0.0, 0.0
        err = wrap_angle(final_heading - theta)
        return 0.0, clamp(K_HEADING * err, -w_max, w_max)

    bearing = math.atan2(dy, dx)
    err = wrap_angle(bearing - theta)

    # A TARGET UNDERFOOT HAS NO MEANINGFUL BEARING.
    #
    # atan2 of a vector a few millimetres long swings through 180 degrees as
    # the robot creeps, so steering by it makes the robot pirouette on the
    # spot instead of travelling anywhere. The arrival branch above would have
    # caught that -- except every defensive and repositioning candidate is
    # `through`, which skips it, so those laws chased a point they were
    # already standing on.
    #
    # Measured: the fraction of one-second windows with near-zero NET
    # displacement despite non-zero distance travelled was 37.9% in
    # defend:cover0.3, 28.0% in defend:cutoff_deep and 21.9% in
    # reposition:peelL -- against 10.3% for the striker's wall sweep, which
    # already had this guard. That is the vibration, quantified.
    #
    # Inside the floor, fade the steering out rather than cutting it, so the
    # robot glides through the point instead of stopping dead on it.
    if rng < config.TARGET_BEARING_FLOOR_M:
        fade = (rng / config.TARGET_BEARING_FLOOR_M) ** 2
        err *= fade

    reverse = False
    if allow_reverse and abs(err) > math.pi / 2:
        reverse = True
        err = wrap_angle(bearing + math.pi - theta)

    # THE BEARING TO A NEARBY TARGET IS NOISE.
    #
    # A defensive cover point tracks the moving ball and therefore sits a few
    # centimetres from the robot most of the time. At that range the BEARING to
    # it swings through tens of degrees per tick for millimetres of motion, so
    # steering hard by it makes the robot turn, overshoot, turn back, and
    # vibrate on the spot without travelling anywhere.
    #
    # Measured, fraction of one-second windows with near-zero net displacement
    # despite non-zero distance travelled:
    #     defend:cover0.3     37.9%
    #     defend:cutoff_deep  28.0%
    #     reposition:peelL    21.9%
    # against 7-10% for the attacking behaviours, which had this same fault
    # fixed separately. Steering authority is therefore faded out as the target
    # closes, and the spin-on-the-spot branch is disabled entirely -- spinning
    # to face something you are already standing on achieves nothing.
    close = rng < config.DRIVE_MIN_BEARING_M
    gain = K_HEADING
    if close:
        gain *= max(rng / config.DRIVE_MIN_BEARING_M, 0.15)

    omega = clamp(gain * err, -w_max, w_max)

    # Slow into the target, and slow while poorly aligned.
    speed = v_max if through else min(v_max, K_RANGE * rng)
    if slow_radius > 0.0 and rng < slow_radius:
        speed *= max(0.25, rng / slow_radius)

    if abs(err) > SPIN_ERROR and not close:
        # Too far off line to make useful progress -- rotate on the spot
        # rather than carve a wide arc into a wall.
        v = 0.0
    else:
        v = speed * math.cos(err)
        if reverse:
            v = -v

    return v, omega


def face(theta: float, target_heading: float) -> tuple[float, float]:
    _, w_max = limits()
    return 0.0, clamp(K_HEADING * wrap_angle(target_heading - theta),
                      -w_max, w_max)


def quantize_like_keyboard(v: float, omega: float) -> tuple[float, float]:
    """Force the AI onto the same discrete command set a keyboard produces.

    Off by default. Available for anyone who feels that continuous control
    against a keyboard player is the unfair part -- it is not where the AI's
    real advantage comes from, but the knob costs nothing.
    """
    v_max, w_max = limits()
    vq = round(v / v_max) * v_max if v_max > 0 else 0.0
    wq = round(omega / w_max) * w_max if w_max > 0 else 0.0
    return clamp(vq, -v_max, v_max), clamp(wq, -w_max, w_max)


def to_wheels(v: float, omega: float) -> tuple[float, float]:
    """Final conversion, including the shared saturation policy.

    Uses config.effective_track_m(), the AI's BELIEF about its own scrub. That
    belief is deliberately imperfect -- the real effective track varies with
    turn rate -- so the robot under- and over-turns slightly depending on the
    manoeuvre, and the estimator has to notice.
    """
    if config.AI_COMMAND_QUANTIZATION:
        v, omega = quantize_like_keyboard(v, omega)
    return twist_to_wheels_clamped(v, omega)
