"""
Batch match runner: no window, no keyboard, many matches, one number at the end.

This is the tool that makes tuning honest. Changing a scoring weight and then
playing three games against it tells you almost nothing -- the variance
between matches is larger than most changes you will make. Running fifty
matches and comparing goal difference tells you something.

    python -m tools.headless --matches 20
    python -m tools.headless --matches 20 --planner fsm
    python -m tools.headless --compare              mpc vs fsm, same seeds
    python -m tools.headless --ablate               what each capability is worth

The `--ablate` mode is the interesting one: it disables each correction and
capability in turn and reports the cost in goal difference and belief error.
That is how you find out whether something you built is actually earning its
place, rather than assuming it is.
"""

from __future__ import annotations

import argparse
import importlib
import statistics
import sys
import time

import config


def _fresh_modules():
    """Reload the AI stack so config changes take effect."""
    import ai.calibration, ai.estimator, ai.controller
    import ai.strategy_fsm, ai.reachability, ai.opponent
    import ai.strategy_mpc, ai.agent
    import game.input, game.match, game.main
    for mod in (game.input, ai.calibration, ai.estimator, ai.controller,
                ai.strategy_fsm, ai.reachability, ai.opponent,
                ai.strategy_mpc, ai.agent, game.match, game.main):
        importlib.reload(mod)
    return game.match, game.main, ai.agent, game.input


def play(seed: int, seconds: float, quiet: bool = True):
    """One match: the agent against the chase baseline."""
    gm, gmain, aagent, ginput = _fresh_modules()

    config.MATCH_DURATION_S = seconds
    human = ginput.HumanController()
    m = gm.Match(None, human, seed=seed)
    agent = aagent.Agent(*m.hal(0, human), robot_index=0, truth=m.world)
    m.controllers[0] = agent
    m.controllers[1] = gmain.TruthChaser(m.world, 1)

    dt = config.PHYSICS_DT
    while m.phase.value != "finished":
        m.step(dt)

    return {
        "for": m.world.score[0],
        "against": m.world.score[1],
        "diff": m.world.score[0] - m.world.score[1],
        "resets": m.stuck_resets,
        "err": agent.error_summary(),
        "latency": agent.estimator.total_latency,
    }


def series(matches: int, seconds: float, label: str = ""):
    results = []
    t0 = time.perf_counter()
    for i in range(matches):
        r = play(seed=1000 + i, seconds=seconds)
        results.append(r)
        print(f"  [{i+1:>3}/{matches}] {label:<10} "
              f"{r['for']:>3} - {r['against']:<3} "
              f"(diff {r['diff']:+3})  "
              f"ball err {r['err']['ball']*1000:5.1f} mm",
              flush=True)
    wall = time.perf_counter() - t0
    diffs = [r["diff"] for r in results]
    summary = {
        "label": label,
        "matches": matches,
        "goals_for": sum(r["for"] for r in results),
        "goals_against": sum(r["against"] for r in results),
        "mean_diff": statistics.mean(diffs),
        "stdev_diff": statistics.pstdev(diffs) if len(diffs) > 1 else 0.0,
        "wins": sum(1 for d in diffs if d > 0),
        "draws": sum(1 for d in diffs if d == 0),
        "losses": sum(1 for d in diffs if d < 0),
        "ball_err": statistics.mean(r["err"]["ball"] for r in results),
        "resets": sum(r["resets"] for r in results),
        "wall": wall,
    }
    return summary


def show(s: dict) -> None:
    print(f"\n  {s['label'] or 'result'}")
    print(f"    record          {s['wins']}W {s['draws']}D {s['losses']}L "
          f"over {s['matches']} matches")
    print(f"    goals           {s['goals_for']} for, "
          f"{s['goals_against']} against")
    print(f"    goal diff       {s['mean_diff']:+.2f} per match "
          f"(sd {s['stdev_diff']:.2f})")
    print(f"    ball belief err {s['ball_err']*1000:.1f} mm")
    print(f"    ball resets     {s['resets']}")
    print(f"    wall time       {s['wall']:.1f} s")


# ---------------------------------------------------------------------------

ABLATIONS = [
    ("baseline", {}),
    ("no latency compensation", {"USE_LATENCY_COMP": False}),
    ("no two-plane homography", {"USE_TWO_PLANE_HOMOGRAPHY": False}),
    ("no undistortion", {"USE_UNDISTORT": False}),
    ("no encoder fusion", {"USE_ENCODER_FUSION": False,
                           "USE_OPPONENT_ENCODERS": False}),
    ("no slip gating", {"USE_SLIP_GATING": False}),
    ("no intent channel", {"USE_INTENT_CHANNEL": False}),
    ("no reachability", {"USE_REACHABILITY": False}),
    ("FSM instead of MPC", {"PLANNER": "fsm"}),
    ("speed capped to 60%", {"SPEED_CAP_FRAC": 0.6}),
    ("250 ms reaction handicap", {"AI_REACTION_DELAY_S": 0.25}),
]


def ablate(matches: int, seconds: float) -> None:
    print("Ablation study: what is each capability actually worth?\n")
    rows = []
    for label, overrides in ABLATIONS:
        saved = {k: getattr(config, k) for k in overrides}
        for k, v in overrides.items():
            setattr(config, k, v)
        try:
            s = series(matches, seconds, label=label)
            rows.append(s)
        finally:
            for k, v in saved.items():
                setattr(config, k, v)

    base = rows[0]["mean_diff"] if rows else 0.0
    print("\n" + "=" * 74)
    print(f"{'configuration':<28}{'goal diff':>11}{'vs base':>10}"
          f"{'ball err':>11}{'W-D-L':>12}")
    print("-" * 74)
    for s in rows:
        delta = s["mean_diff"] - base
        print(f"{s['label']:<28}{s['mean_diff']:>+11.2f}{delta:>+10.2f}"
              f"{s['ball_err']*1000:>10.1f}mm"
              f"{s['wins']:>6}-{s['draws']}-{s['losses']}")
    print("=" * 74)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--matches", type=int, default=10)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--planner", choices=("mpc", "fsm"), default=None)
    ap.add_argument("--motor", default=None)
    ap.add_argument("--compare", action="store_true",
                    help="run MPC and FSM over the same seeds")
    ap.add_argument("--ablate", action="store_true",
                    help="measure what each capability contributes")
    args = ap.parse_args()

    if args.motor:
        config.MOTOR = args.motor
    if args.planner:
        config.PLANNER = args.planner

    if args.ablate:
        ablate(args.matches, args.seconds)
        return 0

    if args.compare:
        out = []
        for planner in ("mpc", "fsm"):
            config.PLANNER = planner
            out.append(series(args.matches, args.seconds, label=planner))
        for s in out:
            show(s)
        gap = out[0]["mean_diff"] - out[1]["mean_diff"]
        print(f"\n  MPC minus FSM: {gap:+.2f} goals per match")
        return 0

    show(series(args.matches, args.seconds, label=config.PLANNER))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
