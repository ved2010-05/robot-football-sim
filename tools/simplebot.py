"""
SimpleBot: the one-trick benchmark opponent.

It does exactly one thing, and only ever one of two things at a time:

    ANGLE     turn on the spot until it is pointing where it wants to go
    STRAIGHT  drive forward in a straight line

No curves, no arcs, no replanning mid-move, no defending, no interception,
no model of anything. It picks the point behind the ball on the ball-to-goal
line, turns until its horns point at it, drives there, turns until its horns
point at the goal, and then drives straight through the ball into the net.

WHY THIS IS THE RIGHT YARDSTICK
-------------------------------
It is the simplest policy that can actually score, and it is the machine
version of "just go forward with small turns" -- the thing that was beating
the clever planner every time. A rollout planner with state estimation,
latency compensation, reachability analysis and an opponent model ought to
beat this comfortably. If it does not, the sophistication is decorative.

It is also completely deterministic, so it makes a stable baseline: any
change in match results is a change in the AI, not in the opponent.

Two properties make it strong despite being trivial, and both are things the
clever planner was failing at:

  * it COMMITS. Once it starts a straight run it keeps going, so it actually
    reaches speed. The drivetrain needs ~0.5 s of steady command to get
    moving, and this never interrupts itself.
  * it never decelerates on approach, because it has no concept of arriving
    anywhere except the goal.
"""

from __future__ import annotations

import math

import config
from sim.geometry import wrap_angle, clamp
from sim.kinematics import (max_forward_speed, max_yaw_rate,
                            twist_to_wheels_clamped)


# How closely it must be pointing at its target before it will drive.
ANGLE_TOL = math.radians(7.0)
# Hysteresis so it does not flicker between turning and driving on the
# boundary -- once driving, it tolerates more error before turning again.
ANGLE_RESUME = math.radians(22.0)


class SimpleBot:
    def __init__(self, world, index: int, speed_frac: float = 1.0,
                 turn_frac: float = 1.0) -> None:
        self.world = world
        self.index = index
        self.speed_frac = speed_frac
        self.turn_frac = turn_frac
        self._twist = (0.0, 0.0)
        self._driving = False
        self.mode = "angle"

    # -- the entire strategy ----------------------------------------------

    def _target(self):
        """Where to point: the strike point, or the goal once lined up."""
        me = self.world.robots[self.index]
        ball = self.world.ball
        goal = self.world.goal_centre(self.index)

        bx, by = ball.pos
        dx, dy = goal[0] - bx, goal[1] - by
        n = max(math.hypot(dx, dy), 1e-6)
        ux, uy = dx / n, dy / n

        # Stand-off so the ball ends up in the pocket, not under the nose.
        stand = (config.ROBOT_LENGTH_M / 2.0
                 + config.HORN_LENGTH_M * 0.5
                 + ball.radius)
        strike = (bx - ux * stand, by - uy * stand)

        # Are we already behind the ball, lined up with the goal?
        rx, ry = me.pos[0] - bx, me.pos[1] - by
        rn = max(math.hypot(rx, ry), 1e-6)
        behindness = (rx * ux + ry * uy) / rn

        if behindness < -0.85 and rn < stand * 2.6:
            return goal, True          # drive through the ball at the goal
        return strike, False           # go and get behind it first

    def update(self, dt: float):
        me = self.world.robots[self.index]
        target, _shooting = self._target()

        dx = target[0] - me.pos[0]
        dy = target[1] - me.pos[1]
        err = wrap_angle(math.atan2(dy, dx) - me.theta)

        tol = ANGLE_RESUME if self._driving else ANGLE_TOL
        if abs(err) > tol:
            # ANGLE: turn on the spot. No forward component at all.
            self._driving = False
            self.mode = "angle"
            w = math.copysign(max_yaw_rate() * self.turn_frac, err)
            self._twist = (0.0, w)
        else:
            # STRAIGHT: full speed ahead, no steering.
            self._driving = True
            self.mode = "straight"
            self._twist = (max_forward_speed() * self.speed_frac, 0.0)

        return self._twist

    # -- controller interface ---------------------------------------------

    def wheel_command(self):
        return twist_to_wheels_clamped(*self._twist)

    def intent(self):
        if not config.USE_INTENT_CHANNEL:
            return None
        return self._twist

    def clear(self):
        self._twist = (0.0, 0.0)
        self._driving = False

    def key_down(self, k):
        pass

    def key_up(self, k):
        pass
