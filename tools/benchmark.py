"""
Honest benchmark: does the AI actually exploit the advantages it was given?

WHY THE CHASE BASELINE IS THE WRONG YARDSTICK
---------------------------------------------
`TruthChaser` reads ground truth and re-decides every single tick with zero
reaction delay. Several of the AI's headline advantages are, by construction,
advantages *over a human*:

    the intent channel        worth ~20 ms of lookahead into what the player
                              is about to do -- but TruthChaser's "intent" is
                              recomputed instantly every tick, so knowing it
                              early buys nothing

    the commitment window     the planner assumes the opponent cannot change
                              command for OPPONENT_ASSUMED_REACTION_S. That
                              assumption is simply FALSE against TruthChaser,
                              so the planner is reasoning from a wrong model
                              and should be expected to do worse

Measuring those against a zero-latency bot therefore understates them and, in
the second case, actively penalises them. `HumanLike` below is a test fixture
with a human-shaped response: a reaction delay before it notices anything
changed, and a tendency to hold a command once committed.

It is NOT a model of any particular player, nothing here is fitted or
learned, and the AI never sees it. It exists only so the benchmark measures
the thing the AI was actually designed for.

    python -m tools.benchmark --matches 8
"""

from __future__ import annotations

import argparse
import collections
import math
import statistics

import config
from sim.geometry import wrap_angle, clamp


class HumanLike:
    """A test opponent with human-shaped reaction and commitment.

    Deliberately simple. Three properties, all fixed constants:
      * reacts only every `reaction` seconds, not continuously
      * holds whatever it last decided in between
      * adds a little sloppiness so it is not a metronome
    """

    def __init__(self, world, index: int, reaction: float = 0.25,
                 sloppiness: float = 0.12, seed: int = 0) -> None:
        import random
        self.world = world
        self.index = index
        self.reaction = reaction
        self.sloppiness = sloppiness
        self.rng = random.Random(seed)
        self._twist = (0.0, 0.0)
        self._since = 0.0

    def _decide(self):
        from game.input import max_forward_speed, max_yaw_rate
        me = self.world.robots[self.index]
        ball = self.world.ball
        goal = self.world.goal_centre(self.index)

        bx, by = ball.pos
        dx, dy = goal[0] - bx, goal[1] - by
        n = max(math.hypot(dx, dy), 1e-6)
        ux, uy = dx / n, dy / n
        stand = (config.ROBOT_LENGTH_M / 2.0
                 + config.HORN_LENGTH_M * 0.5 + ball.radius)

        rx, ry = me.pos[0] - bx, me.pos[1] - by
        rn = max(math.hypot(rx, ry), 1e-6)
        behind = (rx * ux + ry * uy) / rn

        if behind < -0.8 and rn < stand * 2.2:
            target = goal
        elif behind < -0.2:
            target = (bx - ux * stand, by - uy * stand)
        else:
            side = 1.0 if (ux * ry - uy * rx) > 0 else -1.0
            px, py = -uy * side, ux * side
            target = (bx + px * stand * 1.9 - ux * stand * 0.6,
                      by + py * stand * 1.9 - uy * stand * 0.6)

        tdx, tdy = target[0] - me.pos[0], target[1] - me.pos[1]
        rng = math.hypot(tdx, tdy)
        bearing = math.atan2(tdy, tdx)
        err = wrap_angle(bearing - me.theta)
        rev = abs(err) > math.pi / 2
        if rev:
            err = wrap_angle(bearing + math.pi - me.theta)

        turn = clamp(err * 2.5, -1.0, 1.0)
        drive = clamp(rng * 2.0, 0.0, 1.0) * (1.0 - 0.6 * abs(turn))
        if rev:
            drive = -drive

        s = self.sloppiness
        turn += self.rng.uniform(-s, s)
        drive += self.rng.uniform(-s, s)
        return (clamp(drive, -1, 1) * max_forward_speed(),
                clamp(turn, -1, 1) * max_yaw_rate())

    def update(self, dt: float):
        self._since += dt
        if self._since >= self.reaction:
            self._since = 0.0
            self._twist = self._decide()
        return self._twist

    def wheel_command(self):
        from game.input import twist_to_wheels_clamped
        return twist_to_wheels_clamped(*self._twist)

    def intent(self):
        if not config.USE_INTENT_CHANNEL:
            return None
        return self._twist

    def clear(self):
        self._twist = (0.0, 0.0)
        self._since = 0.0

    def key_down(self, k):
        pass

    def key_up(self, k):
        pass


def play(seed: int, seconds: float, reaction: float):
    from game.match import Match
    from ai.agent import Agent

    m = Match(None, None, seed=seed)
    bot = HumanLike(m.world, 1, reaction=reaction, seed=seed)
    agent = Agent(m, robot_index=0, opponent_controller=bot)
    m.controllers[0] = agent
    m.controllers[1] = bot

    config.MATCH_DURATION_S = seconds
    m.clock = seconds
    dt = config.PHYSICS_DT
    while m.phase.value != "finished":
        m.step(dt)
    return m.world.score[0] - m.world.score[1], m.world.score


def series(matches: int, seconds: float, reaction: float):
    diffs, scores = [], []
    for i in range(matches):
        d, sc = play(2000 + i, seconds, reaction)
        diffs.append(d)
        scores.append(f"{sc[0]}-{sc[1]}")
    return diffs, scores


def report(label, diffs, scores):
    w = sum(1 for d in diffs if d > 0)
    dr = sum(1 for d in diffs if d == 0)
    l = sum(1 for d in diffs if d < 0)
    print(f"{label:<34}{statistics.mean(diffs):>+8.2f}   "
          f"{w}W {dr}D {l}L   {' '.join(scores)}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--matches", type=int, default=6)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--reaction", type=float, default=0.25)
    args = ap.parse_args()

    print(f"AI vs a human-shaped opponent "
          f"({args.reaction*1000:.0f} ms reaction, holds between decisions)")
    print(f"{args.matches} matches x {args.seconds:.0f}s\n")
    print(f"{'configuration':<34}{'goal diff':>8}   record   scores")
    print("-" * 78)

    trials = [
        ("everything on", {}),
        ("no intent channel", {"USE_INTENT_CHANNEL": False}),
        ("no latency compensation", {"USE_LATENCY_COMP": False}),
        ("no reachability", {"USE_REACHABILITY": False}),
        ("no opponent encoders", {"USE_OPPONENT_ENCODERS": False}),
        ("no slip gating", {"USE_SLIP_GATING": False}),
        ("FSM instead of MPC", {"PLANNER": "fsm"}),
    ]
    base = None
    for label, over in trials:
        saved = {k: getattr(config, k) for k in over}
        for k, v in over.items():
            setattr(config, k, v)
        try:
            diffs, scores = series(args.matches, args.seconds, args.reaction)
            report(label, diffs, scores)
            if base is None:
                base = statistics.mean(diffs)
        finally:
            for k, v in saved.items():
                setattr(config, k, v)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
