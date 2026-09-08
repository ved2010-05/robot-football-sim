"""
Human input: keyboard to virtual analog stick.

A raw keyboard is on/off. An analog stick is not. Since the real robot will
be driven with a stick, giving the player a binary input here would handicap
them against an AI issuing continuous commands -- and the whole premise is
that the two robots are identical, so the only asymmetry should be in the
decisions, not the actuation.

Held keys therefore ramp a virtual stick toward full deflection, and release
ramps it back to centre, with separate rates for the two directions (real
players release faster than they push).

THE INTENT CHANNEL
------------------
Two things read this controller, at two different times:

    the human's ROBOT   receives the command late, through a DelayLine
                        carrying input latency plus radio latency

    the AI              may read `intent()` immediately, with no delay

That gap is the whole point. The player presses a key at t=0; their robot
starts responding at t~20 ms; the AI knew at t=0. It is reading the command,
not the robot -- which is precisely what the real system will be able to do,
since the gamepad is wired into the same PC that runs the AI.

`intent()` is gated by config.USE_INTENT_CHANNEL. With it off, the AI must
infer the opponent's motion from vision and encoders like anyone else.
"""

from __future__ import annotations

import math

import config
from sim import chassis
from sim.geometry import clamp


def max_forward_speed() -> float:
    return chassis.max_forward_speed() * config.COMMAND_MAX_SPEED_FRAC


def max_yaw_rate() -> float:
    """Physical yaw ceiling, scaled by the shared command limit."""
    physical = (2.0 * chassis.max_wheel_speed() * config.WHEEL_RADIUS_M
                / config.effective_track_m())
    return physical * config.COMMAND_MAX_YAW_FRAC


def twist_to_wheels_clamped(v: float, omega: float) -> tuple[float, float]:
    """(v, omega) -> wheel speeds, scaled down together if either saturates.

    Scaling BOTH preserves the ratio, so a robot asked for more than it has
    drives a wider arc rather than losing its turn entirely. Clipping them
    independently would change the commanded path shape, which is worse: the
    robot would silently stop turning at high speed.
    """
    wl, wr = chassis.twist_to_wheels(v, omega)
    wmax = chassis.max_wheel_speed()
    peak = max(abs(wl), abs(wr))
    if peak > wmax and peak > 1e-9:
        s = wmax / peak
        wl *= s
        wr *= s
    return wl, wr


class VirtualStick:
    """Two ramped axes in [-1, 1]."""

    def __init__(self) -> None:
        self.drive = 0.0   # +1 forward
        self.turn = 0.0    # +1 left (counter-clockwise, matching +omega)

    @staticmethod
    def _ramp(current: float, target: float, dt: float) -> float:
        if target == 0.0:
            rate = config.STICK_RAMP_DOWN
        elif current * target < 0.0:
            # Reversing: use the faster rate, since letting go and pushing
            # the other way is one motion to the player.
            rate = config.STICK_RAMP_DOWN
        else:
            rate = config.STICK_RAMP_UP
        step = rate * dt
        if current < target:
            return min(current + step, target)
        if current > target:
            return max(current - step, target)
        return current

    def update(self, keys: set[str], dt: float) -> None:
        fwd = config.KEY_FORWARD in keys
        back = config.KEY_BACK in keys
        left = config.KEY_LEFT in keys
        right = config.KEY_RIGHT in keys

        drive_target = (1.0 if fwd else 0.0) - (1.0 if back else 0.0)
        turn_target = (1.0 if left else 0.0) - (1.0 if right else 0.0)

        self.drive = self._ramp(self.drive, drive_target, dt)
        self.turn = self._ramp(self.turn, turn_target, dt)

    def reset(self) -> None:
        self.drive = 0.0
        self.turn = 0.0


class HumanController:
    """Turns key state into a wheel command, and exposes intent."""

    def __init__(self) -> None:
        self.stick = VirtualStick()
        self.keys: set[str] = set()
        self._last_twist = (0.0, 0.0)

    # -- key events --------------------------------------------------------

    def key_down(self, key: str) -> None:
        self.keys.add(key.lower())

    def key_up(self, key: str) -> None:
        self.keys.discard(key.lower())

    def clear(self) -> None:
        self.keys.clear()
        self.stick.reset()
        self._last_twist = (0.0, 0.0)

    # -- output ------------------------------------------------------------

    def update(self, dt: float) -> tuple[float, float]:
        """Advance the stick and return the commanded (v, omega)."""
        self.stick.update(self.keys, dt)
        v = self.stick.drive * max_forward_speed()
        omega = self.stick.turn * max_yaw_rate()
        self._last_twist = (v, omega)
        return v, omega

    def wheel_command(self) -> tuple[float, float]:
        v, omega = self._last_twist
        return twist_to_wheels_clamped(v, omega)

    def intent(self) -> tuple[float, float] | None:
        """The command as issued, with no transport delay.

        This is what makes the AI able to react before the opponent's robot
        visibly moves. Returns None when the channel is disabled, and the AI
        must then fall back to vision and encoders.
        """
        if not config.USE_INTENT_CHANNEL:
            return None
        return self._last_twist


class ScriptedController(HumanController):
    """A stand-in opponent for headless runs and regression tests.

    Not a model of a human and not learned from one -- just a fixed policy so
    matches can be run without a person at the keyboard. Chases the ball and
    turns toward it.
    """

    def __init__(self, aggression: float = 1.0) -> None:
        super().__init__()
        self.aggression = aggression
        self._twist = (0.0, 0.0)

    def drive_toward(self, robot_pos, robot_theta, target) -> None:
        from sim.geometry import angle_diff
        dx = target[0] - robot_pos[0]
        dy = target[1] - robot_pos[1]
        rng = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        err = angle_diff(bearing, robot_theta)

        # Back up rather than pirouette when the target is behind.
        reverse = abs(err) > math.pi / 2
        if reverse:
            err = angle_diff(bearing + math.pi, robot_theta)

        turn = clamp(err * 2.5, -1.0, 1.0)
        drive = clamp(rng * 2.0, 0.0, 1.0) * (1.0 - 0.6 * abs(turn))
        if reverse:
            drive = -drive

        v = drive * max_forward_speed() * self.aggression
        omega = turn * max_yaw_rate()
        self._twist = (v, omega)
        self._last_twist = (v, omega)

    def update(self, dt: float) -> tuple[float, float]:
        return self._twist
