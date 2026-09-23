"""
The honesty check: our side against every opponent, next to the control.

A change measured only against SimpleBot is a change measured against one
opponent's habits. This runs the agent AND the benchmark control across every
opponent, from identical seeds, through the identical probe, so the comparison
that matters -- "is the AI better than the trivial policy, against everyone?"
-- is one table rather than six remembered numbers.

    python -m tools.verify --matches 16 --seconds 120
    python -m tools.verify --matches 16 --set ATTACK_MODE=sequence
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

OPPONENTS = ("simple", "runner", "human")
SIDES = ("agent", "simple")


def one(ai, opp, matches, seconds, sets, seed0=3000):
    cmd = [sys.executable, "-m", "tools.arena", "--ai", ai,
           "--opponent", opp, "--matches", str(matches),
           "--seconds", str(seconds), "--seed0", str(seed0)]
    for kv in sets:
        cmd += ["--set", kv]
    env = dict(os.environ)
    env.update(OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1",
               MKL_NUM_THREADS="1", NUMEXPR_NUM_THREADS="1")
    p = subprocess.run(cmd, capture_output=True, text=True, env=env)
    for line in p.stdout.splitlines():
        if line.startswith("JSON "):
            return json.loads(line[5:])
    sys.stderr.write(f"FAILED {ai} vs {opp}\n{p.stdout[-1500:]}{p.stderr[-1500:]}\n")
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--matches", type=int, default=16)
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--jobs", type=int, default=3)
    ap.add_argument("--seed0", type=int, default=3000)
    ap.add_argument("--sides", default=",".join(SIDES),
                    help="comma list: agent,simple. 'agent' alone skips the "
                         "control rows, which do not change when the AI does")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    a = ap.parse_args()
    sides = tuple(s for s in a.sides.split(",") if s)

    jobs = [(ai, opp) for ai in sides for opp in OPPONENTS]
    print(f"{len(jobs)} runs x {a.matches} matches x {a.seconds:.0f}s", flush=True)
    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        futs = {j: ex.submit(one, j[0], j[1], a.matches, a.seconds, a.set,
                             a.seed0)
                for j in jobs}
        out = {j: f.result() for j, f in futs.items()}

    print(f"\n  {'our side':<10}{'opponent':<10}{'goals':>10}"
          f"{'diff':>9}{'+-':>6}{'W-D-L':>10}{'att3rd':>9}")
    for ai in sides:
        for opp in OPPONENTS:
            r = out.get((ai, opp))
            if r is None:
                print(f"  {ai:<10}{opp:<10}{'FAILED':>10}")
                continue
            se = r["stdev_diff"] / max(a.matches ** 0.5, 1e-9)
            label = "AI" if ai == "agent" else "control"
            print(f"  {label:<10}{opp:<10}"
                  f"{r['goals_for']:>5}-{r['goals_against']:<4}"
                  f"{r['mean_diff']:>+9.2f}{se:>6.2f}"
                  f"{r['wins']:>4}-{r['draws']}-{r['losses']:<4}"
                  f"{r['att_third_pct']:>8.1f}%"
                  f"  shots {r['shots']:.1f} on tgt {r['on_target']:.2f}")
    print("\n  'control' is the 40-line SimpleBot playing our side, same seeds.")
    print("  The AI has to beat that row, not just beat the opponent.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
