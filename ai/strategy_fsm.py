"""
Baseline state machine.

Kept as the fallback when tracking degrades, and as the benchmark the rollout
planner has to beat. It is not a strawman -- with good state estimation and
latency compensation behind it, this is already a difficult opponent.

Its one structural weakness is the one that motivates the MPC: it is greedy.
It will never choose to back away from the ball in order to be better placed
two seconds later, because reversing always scores worse right now. On a
non-holonomic robot that costs a lot, and it is exactly what the rollout
planner discovers on its own.
"""

from __future__ import annotations

import math

import config
from ai.estimator import Belief, RobotBelief, predict_ball
from ai.controller import limits
from ai.tactics import effective_aim, clamp_target
from sim.geometry import wrap_angle, clamp, dist


def own_goal(attack_dir: float):
    return (-attack_dir * config.HALF_LENGTH_M, 0.0)


def their_goal(attack_dir: float):
    return (attack_dir * config.HALF_LENGTH_M, 0.0)


def travel_time(me: RobotBelief, target) -> float:
    """Rough time for a non-holonomic robot to reach a point.

    Turn first, then drive. Crude, but it captures the thing a holonomic
    estimate would miss: a point 30 cm directly to the side is far more
    expensive than a point 30 cm straight ahead.
    """
    v_max, w_max = limits()
    dx = target[0] - me.pos[0]
    dy = target[1] - me.pos[1]
    rng = math.hypot(dx, dy)
    if rng < 1e-6:
        return 0.0
    bearing = math.atan2(dy, dx)
    err = abs(wrap_angle(bearing - me.theta))
    err = min(err, math.pi - err)          # reversing is allowed
    turn = err / max(w_max, 1e-6)
    drive = rng / max(v_max, 1e-6)
    return turn * 0.7 + drive


def earliest_intercept(me: RobotBelief, ball_pos, ball_vel,
                       horizon: float = 1.6):
    """First point on the ball's predicted path we can actually get to.

    Chasing the ball's CURRENT position guarantees arriving where it no
    longer is. Solving for the first reachable point on its future path is
    the single biggest behavioural difference between a robot that looks
    like it is following the ball and one that looks like it is playing.
    """
    t = 0.0
    step = 0.05
    best = ball_pos
    while t < horizon:
        p, _ = predict_ball(ball_pos, ball_vel, t)
        if travel_time(me, p) <= t:
            return p, t
        best = p
        t += step
    return best, horizon


def aim_point(ball_pos, opp: RobotBelief, attack_dir: float):
    """Pick a spot in the goal mouth away from the opponent.

    A simple version of the reachability-based shot selection: split the
    mouth into candidate points and take the one the opponent is furthest
    from being able to cover, weighted slightly toward the centre so the AI
    does not always aim at an impossible angle into a post.
    """
    gx = attack_dir * config.HALF_LENGTH_M
    hg = config.HALF_GOAL_M * 0.78          # stay off the posts
    best, best_score = (gx, 0.0), -1e18
    for i in range(7):
        y = -hg + (2 * hg) * i / 6.0
        cand = (gx, y)
        # Distance from the opponent to the shot line, roughly.
        d = _point_line_distance(opp.pos, ball_pos, cand)
        centre_bias = 1.0 - 0.45 * abs(y) / max(hg, 1e-6)
        score = d * centre_bias
        if score > best_score:
            best_score = score
            best = cand
    return best


def _point_line_distance(p, a, b) -> float:
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return dist(p, a)
    t = clamp(((px - ax) * dx + (py - ay) * dy) / L2, 0.0, 1.0)
    cx, cy = ax + dx * t, ay + dy * t
    return math.hypot(px - cx, py - cy)


class FSM:
    def __init__(self, self_index: int) -> None:
        self.self_index = self_index
        self.opp_index = 1 - self_index
        self.attack_dir = 1.0 if self_index == 0 else -1.0
        self.state = "attack"

    def decide(self, b: Belief):
        """Return (target_point, allow_reverse, final_heading, aim)."""
        me = b.robots[self.self_index]
        opp = b.robots[self.opp_index]
        ball = b.ball_pos
        bvel = b.ball_vel

        goal = their_goal(self.attack_dir)
        mine = own_goal(self.attack_dir)
        aim = aim_point(ball, opp, self.attack_dir)
        # A ball on a wall or in a corner has to be walked out before
        # any shot exists; aiming at the goal from there wedges it in.
        aim, diverted = effective_aim(ball, aim, self.attack_dir)
        self.diverted = diverted

        stand = (config.ROBOT_LENGTH_M / 2.0
                 + config.HORN_LENGTH_M * 0.5
                 + config.BALL_RADIUS_M)

        # --- is the ball a threat to our goal? ----------------------------
        toward_us = (bvel[0] * -self.attack_dir) > 0.25
        ball_our_half = (ball[0] * self.attack_dir) < -0.15
        opp_closer = (dist(opp.pos, ball) + 0.10) < dist(me.pos, ball)

        if toward_us and ball_our_half:
            self.state = "defend"
            # Sit on the line between the ball and our goal, near enough to
            # our own end to actually block rather than merely follow.
            dx = mine[0] - ball[0]
            dy = mine[1] - ball[1]
            n = max(math.hypot(dx, dy), 1e-6)
            back = min(0.55, n * 0.55)
            target = (ball[0] + dx / n * back, ball[1] + dy / n * back)
            return clamp_target(target), True, None, aim

        if opp_closer and ball_our_half:
            self.state = "cover"
            dx = mine[0] - ball[0]
            dy = mine[1] - ball[1]
            n = max(math.hypot(dx, dy), 1e-6)
            target = (ball[0] + dx / n * 0.40, ball[1] + dy / n * 0.40)
            return clamp_target(target), True, None, aim

        # --- attack -------------------------------------------------------
        speed = math.hypot(*bvel)
        if speed > 0.45:
            self.state = "intercept"
            p, _ = earliest_intercept(me, ball, bvel)
            return clamp_target(p), True, None, aim

        # Where we must stand to push the ball at the aim point.
        dx = aim[0] - ball[0]
        dy = aim[1] - ball[1]
        n = max(math.hypot(dx, dy), 1e-6)
        ux, uy = dx / n, dy / n
        strike = (ball[0] - ux * stand, ball[1] - uy * stand)

        rx = me.pos[0] - ball[0]
        ry = me.pos[1] - ball[1]
        rn = max(math.hypot(rx, ry), 1e-6)
        behindness = (rx * ux + ry * uy) / rn

        if behindness < -0.82 and rn < stand * 2.4:
            self.state = "strike"
            return clamp_target(aim), False, None, aim

        if behindness < -0.25:
            self.state = "line-up"
            return clamp_target(strike), True, math.atan2(uy, ux), aim

        # Wrong side of the ball. Arc around it rather than shoving it the
        # wrong way in passing.
        self.state = "wrap"
        side = 1.0 if (ux * ry - uy * rx) > 0 else -1.0
        px, py = -uy * side, ux * side
        target = (ball[0] + px * stand * 1.85 - ux * stand * 0.7,
                  ball[1] + py * stand * 1.85 - uy * stand * 0.7)
        return target, True, None, aim
