"""
Parameter sweep: one FRESH PROCESS per value, run in parallel.

    python -m tools.sweep --param STRIKE_AIM_DEG --values 8,12,18,25 \
        --opponent simple --matches 10

    # two parameters at once, cartesian:
    python -m tools.sweep --param A --values 1,2 --param B --values 3,4

Reads the noise floor before it reads the ranking: with N matches the
standard error on mean goal difference is sd/sqrt(N), and anything inside
about 0.5 goals of the best is reported as a tie, not a winner.
"""

from __future__ import annotations

import argparse
import itertools
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor


def run_point(assignments, opponent, matches, seconds, seed0):
    cmd = [sys.executable, "-m", "tools.arena",
           "--opponent", opponent,
           "--matches", str(matches),
           "--seconds", str(seconds),
           "--seed0", str(seed0)]
    for k, v in assignments:
        cmd += ["--set", f"{k}={v}"]
    # Pin every child to ONE compute thread.
    #
    # numpy's backend opens a thread pool per process, sized to the machine.
    # Six sweep points therefore put ~72 threads on 12 cores and spend their
    # time context-switching: measured, a 120 s match cost 25 s of wall clock
    # run alone and 113 s with four points in flight -- parallelism was buying
    # almost nothing. The simulation is single-threaded work; the pools are
    # pure overhead here.
    env = dict(os.environ)
    env.update(OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1",
               MKL_NUM_THREADS="1", NUMEXPR_NUM_THREADS="1",
               VECLIB_MAXIMUM_THREADS="1")
    p = subprocess.run(cmd, capture_output=True, text=True, env=env)
    for line in p.stdout.splitlines():
        if line.startswith("JSON "):
            return json.loads(line[5:])
    sys.stderr.write(f"FAILED {assignments}\n{p.stdout[-2000:]}\n{p.stderr[-2000:]}\n")
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--param", action="append", required=True)
    ap.add_argument("--values", action="append", required=True)
    ap.add_argument("--opponent", default="simple")
    ap.add_argument("--matches", type=int, default=10)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--seed0", type=int, default=3000)
    # THREE, NOT SIX.
    #
    # The 12 logical cores are 6 physical ones, and this simulation is
    # memory-bandwidth bound, so concurrency past about three jobs does not
    # add throughput -- it destroys it. Measured on an otherwise idle machine:
    # a 30 s match costs 4.4 s of wall clock alone and 23 s with six other
    # sims running, a 5.3x collapse. Whole afternoons of sweeps were lost to
    # this: at --jobs 6 the workers sat at 9% CPU and a 4-point sweep had not
    # produced a single result after 34 minutes.
    ap.add_argument("--jobs", type=int, default=3)
    a = ap.parse_args()

    if len(a.param) != len(a.values):
        raise SystemExit("--param and --values must be given in pairs")

    grids = [[(p, v) for v in vals.split(",")]
             for p, vals in zip(a.param, a.values)]
    points = list(itertools.product(*grids))

    print(f"{len(points)} points x {a.matches} matches vs {a.opponent}"
          f"  ({a.jobs} at a time)", flush=True)

    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        futs = [ex.submit(run_point, pt, a.opponent, a.matches,
                          a.seconds, a.seed0) for pt in points]
        results = []
        for pt, f in zip(points, futs):
            r = f.result()
            if r is None:
                continue
            r["_point"] = pt
            results.append(r)
            desc = " ".join(f"{k}={v}" for k, v in pt)
            se = r["stdev_diff"] / max(a.matches ** 0.5, 1e-9)
            print(f"  {desc:<40} {r['mean_diff']:+6.2f} +-{se:4.2f}  "
                  f"{r['goals_for']:>3}F{r['goals_against']:>3}A  "
                  f"shots {r.get('shots', 0):5.1f}  "
                  f"on-tgt {r.get('on_target', 0):4.1f}  "
                  f"att3rd {r.get('att_third_pct', 0):4.1f}%", flush=True)

    if not results:
        return 1
    results.sort(key=lambda r: -r["mean_diff"])
    best = results[0]
    print("\n  ranked by goal difference")
    for r in results:
        desc = " ".join(f"{k}={v}" for k, v in r["_point"])
        gap = best["mean_diff"] - r["mean_diff"]
        tag = "  <- best" if r is best else ("  (tie)" if gap < 0.5 else "")
        print(f"    {desc:<40} {r['mean_diff']:+6.2f}"
              f"  {r['goals_for']:>3}F{r['goals_against']:>3}A{tag}")

    # Goals are the objective; shots are the SENSITIVE reading of the same
    # behaviour. Measured over 12 matches the AI scored 6 goals, all of them
    # inside 2 matches -- per-match sd 1.61 against a mean of -0.08. Ranking
    # config points on that column is ranking variance. Shots and territory
    # are the same attacking behaviour sampled a hundred times more often, so
    # steer on them and confirm the winner on goals with more matches.
    by_shots = sorted(results, key=lambda r: -r.get("on_target", 0))
    print("\n  ranked by shots on target (lower variance -- steer on this)")
    for r in by_shots:
        desc = " ".join(f"{k}={v}" for k, v in r["_point"])
        print(f"    {desc:<40} on-tgt {r.get('on_target', 0):5.2f}"
              f"  shots {r.get('shots', 0):5.1f}"
              f"  att3rd {r.get('att_third_pct', 0):4.1f}%"
              f"  own3rd {r.get('own_third_pct', 0):4.1f}%")
    print("\n  a goal-difference gap under 0.5 is noise.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
