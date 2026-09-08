"""
Mode census: where does the match actually GO?

Aggregate goal difference tells you whether a change helped. It does not tell
you why, and it hides the thing that matters most here -- that the AI can
spend a third of every match in a reflex and 3% of it actually attacking.

Prints the fraction of control ticks spent in each mode, plus the facts the
attack gate turns on, so a blocked tactic is visible as a number instead of
having to be inferred from the score.

    python -m tools.census --opponent simple --matches 6
"""

from __future__ import annotations

import argparse
import collections
import json
import sys

from tools.arena import _apply, _make_opponent


def census(seed: int, seconds: float, opponent: str) -> collections.Counter:
    import config
    from game.match import Match
    from ai.agent import Agent

    config.MATCH_DURATION_S = seconds
    m = Match(None, None, seed=seed)
    bot = _make_opponent(opponent, m.world, 1, seed)
    agent = Agent(m, robot_index=0, opponent_controller=bot)
    m.controllers[0] = agent
    m.controllers[1] = bot
    m.clock = seconds

    c = collections.Counter()
    dt = config.PHYSICS_DT
    while m.phase.value != "finished":
        m.step(dt)
        st = agent.debug.get("state")
        if st:
            c[st] += 1
            c["_ticks"] += 1
    c["_for"] = m.world.score[0]
    c["_against"] = m.world.score[1]
    return c


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--opponent", default="simple")
    ap.add_argument("--matches", type=int, default=6)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--seed0", type=int, default=3000)
    ap.add_argument("--set", action="append", default=[])
    a = ap.parse_args()
    _apply(a.set)

    total = collections.Counter()
    for i in range(a.matches):
        total.update(census(a.seed0 + i, a.seconds, a.opponent))

    ticks = total.pop("_ticks", 1)
    gf, ga = total.pop("_for", 0), total.pop("_against", 0)
    print(f"\n  {a.matches} matches vs {a.opponent}: {gf} for, {ga} against"
          f"   ({ticks} control ticks)")
    for name, n in total.most_common():
        print(f"    {name:<24} {100.0 * n / ticks:5.2f}%")
    return 0


if __name__ == "__main__":
    sys.exit(main())
