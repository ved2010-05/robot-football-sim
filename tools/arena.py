"""
One process, one config point, N matches, one JSON line out.

WHY THIS EXISTS
---------------
Sweeping a parameter by reloading modules inside a single process has bitten
this project twice: `game.match` holds a binding to the OLD `sim.world`, so
half the stack runs on the previous value and the sweep reports a sharp,
confident, completely fake optimum. (It did: a 75 mm horn "optimum" that
INVERTED when re-run properly.)

So: one fresh interpreter per parameter value. This module is the child. It
applies its overrides to `config` BEFORE importing anything that reads them,
re-derives the dependent constants, plays the matches, and prints JSON.

    python -m tools.arena --opponent simple --matches 8 --set STRIKE_AIM_DEG=18
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys


def _apply(pairs: list[str]) -> dict:
    """Set config values, then re-derive anything computed from them."""
    import config
    applied = {}
    for p in pairs:
        key, _, raw = p.partition("=")
        key = key.strip()
        if not hasattr(config, key):
            raise SystemExit(f"config has no attribute {key!r}")
        cur = getattr(config, key)
        if isinstance(cur, bool):
            val = raw.strip().lower() in ("1", "true", "yes", "on")
        elif isinstance(cur, int) and not isinstance(cur, bool):
            val = int(float(raw))
        elif isinstance(cur, float):
            val = float(raw)
        else:
            val = raw
        setattr(config, key, val)
        applied[key] = val

    # Derived constants must move with their parents, or the sweep measures
    # an inconsistent world (a longer arena with the old half-length).
    config.HALF_LENGTH_M = config.ARENA_LENGTH_M / 2.0
    config.HALF_WIDTH_M = config.ARENA_WIDTH_M / 2.0
    config.HALF_GOAL_M = config.GOAL_WIDTH_M / 2.0
    return applied


def _make_opponent(kind: str, world, index: int, seed: int):
    if kind == "simple":
        from tools.simplebot import SimpleBot
        return SimpleBot(world, index)
    if kind == "runner":
        from tools.runner import StraightRunner
        return StraightRunner(world, index)
    if kind == "human":
        from tools.benchmark import HumanLike
        return HumanLike(world, index, reaction=0.25, seed=seed)
    if kind == "chase":
        from game.main import TruthChaser
        return TruthChaser(world, index)
    raise SystemExit(f"unknown opponent {kind!r}")


class Probe:
    """Low-variance proxies for "is this AI actually attacking?".

    Goals are the thing that matters and a hopeless thing to tune on: measured
    over 12 matches the AI scored 6 goals, ALL of them inside 2 matches, giving
    a per-match standard deviation of 1.61 against a mean of -0.08. Resolving a
    half-goal change through that needs roughly 40 matches per config point.

    Shots and territory are the same behaviour sampled far more often, so they
    move out of the noise in a tenth of the matches. A change that raises shots
    and does not raise goals is not automatically good -- but a change that
    raises neither is dead, and that can now be established cheaply.
    """

    def __init__(self, world, index: int) -> None:
        import config
        self.world = world
        self.index = index
        self.adir = 1.0 if world.goal_centre(index)[0] > 0 else -1.0
        self.shots = 0
        self.on_target = 0
        self.att_third = 0
        self.own_third = 0
        self.ticks = 0
        self._was_shot = False

    def _on_target(self, ball) -> bool:
        """Is the ball rolling at the goal mouth fast enough to reach it?"""
        import config
        vx, vy = ball.vel
        speed = math.hypot(vx, vy)
        if speed < 0.60:
            return False
        if vx * self.adir <= 0.0:
            return False
        line = self.adir * config.HALF_LENGTH_M
        dx = line - ball.pos[0]
        t = dx / vx
        if t <= 0.0:
            return False
        # Rolling friction: will it still be moving when it gets there?
        if speed - config.BALL_ROLL_DECEL * t <= 0.0:
            return False
        y = ball.pos[1] + vy * t
        return abs(y) < config.HALF_GOAL_M

    def sample(self) -> None:
        import config
        self.ticks += 1
        ball = self.world.ball
        x = ball.pos[0] * self.adir
        if x > config.HALF_LENGTH_M * 0.33:
            self.att_third += 1
        elif x < -config.HALF_LENGTH_M * 0.33:
            self.own_third += 1

        # A "shot" is the RISING EDGE of the ball being struck goalward, so one
        # strike counts once however long the ball rolls.
        shot = math.hypot(*ball.vel) > 0.60 and ball.vel[0] * self.adir > 0.0
        if shot and not self._was_shot:
            self.shots += 1
            if self._on_target(ball):
                self.on_target += 1
        self._was_shot = shot

    def summary(self) -> dict:
        n = max(self.ticks, 1)
        return {
            "shots": self.shots,
            "on_target": self.on_target,
            "att_third_pct": 100.0 * self.att_third / n,
            "own_third_pct": 100.0 * self.own_third / n,
        }


def play(seed: int, seconds: float, opponent: str, ai: str = "agent") -> dict:
    import config
    from game.match import Match
    from ai.agent import Agent

    config.MATCH_DURATION_S = seconds
    m = Match(None, None, seed=seed)
    bot = _make_opponent(opponent, m.world, 1, seed)
    # `--ai` swaps OUR side for one of the benchmark controllers. That gives a
    # control row measured through exactly the same probe, which is the only
    # way to say whether a number like "14% of the match in the attacking
    # third" is good or terrible.
    if ai == "agent":
        me = Agent(*m.hal(0, bot), robot_index=0, truth=m.world)
    else:
        me = _make_opponent(ai, m.world, 0, seed)
    m.controllers[0] = me
    m.controllers[1] = bot
    m.clock = seconds

    probe = Probe(m.world, 0)
    dt = config.PHYSICS_DT
    tick = 0
    while m.phase.value != "finished":
        m.step(dt)
        tick += 1
        if tick % 10 == 0:          # 100 Hz, matching the control rate
            probe.sample()

    out = {
        "for": m.world.score[0],
        "against": m.world.score[1],
        "diff": m.world.score[0] - m.world.score[1],
    }
    out.update(probe.summary())
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--opponent", default="simple",
                    choices=("simple", "runner", "human", "chase"))
    ap.add_argument("--matches", type=int, default=8)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--seed0", type=int, default=3000)
    ap.add_argument("--ai", default="agent",
                    choices=("agent", "simple", "runner", "human", "chase"),
                    help="what plays on OUR side; non-agent values are controls")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    ap.add_argument("--label", default="")
    a = ap.parse_args()

    applied = _apply(a.set)

    rows = [play(a.seed0 + i, a.seconds, a.opponent, a.ai)
            for i in range(a.matches)]
    diffs = [r["diff"] for r in rows]
    out = {
        "label": a.label,
        "ai": a.ai,
        "opponent": a.opponent,
        "set": applied,
        "matches": a.matches,
        "goals_for": sum(r["for"] for r in rows),
        "goals_against": sum(r["against"] for r in rows),
        "mean_diff": statistics.mean(diffs),
        "stdev_diff": statistics.pstdev(diffs) if len(diffs) > 1 else 0.0,
        "wins": sum(1 for d in diffs if d > 0),
        "draws": sum(1 for d in diffs if d == 0),
        "losses": sum(1 for d in diffs if d < 0),
        "scores": [f"{r['for']}-{r['against']}" for r in rows],
        "shots": sum(r["shots"] for r in rows) / a.matches,
        "on_target": sum(r["on_target"] for r in rows) / a.matches,
        "att_third_pct": statistics.mean(r["att_third_pct"] for r in rows),
        "own_third_pct": statistics.mean(r["own_third_pct"] for r in rows),
        "shots_sd": (statistics.pstdev([r["shots"] for r in rows])
                     if a.matches > 1 else 0.0),
    }
    print("JSON " + json.dumps(out), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
