"""
Evaluate one configuration of the AI, and compare it with another, honestly.

    python -m tools.evaluate --label base --matches 24
    python -m tools.evaluate --label wide --matches 24 --set ROUTE_CLEARANCE_M=0.30
    python -m tools.evaluate --report base,wide

WHAT THIS FIXES ABOUT THE OLDER TOOLS
-------------------------------------
* Every match is saved as a row in scratch/evals/<label>.json. Running the
  same label again with a new --seed0 ADDS matches instead of replacing them,
  so a result that is still inside its error bar can be firmed up rather than
  re-run from nothing.
* Work is split by (opponent, seed chunk), not by opponent, so all the
  workers stay busy until the end instead of one slow opponent holding the
  run open.
* Every number is printed with its standard error, and a comparison says in
  words whether the difference is outside the noise. A change inside the noise
  is reported as "no detectable effect", never as a gain.
* Two human-shaped fixtures, not one, so a change that only exploits the
  habits of one fixture shows up as helping one row and not the other.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

OPPONENTS = ("simple", "runner", "human", "human_fast")
OUT_DIR = os.path.join("scratch", "evals")


def _snapshot(label):
    """Copy the code as it is NOW, and run every match from that copy.

    A run takes tens of minutes, and each chunk is a fresh interpreter that
    imports whatever is on disk when it starts. Edit a file mid-run and the
    later chunks measure different code from the earlier ones, under one
    label, silently. Freezing a copy at launch makes that impossible.
    """
    import shutil
    dst = os.path.join(OUT_DIR, "trees", label)
    if os.path.exists(dst):
        shutil.rmtree(dst)
    ign = shutil.ignore_patterns("__pycache__", "*.pyc")
    for d in ("ai", "sim", "game", "tools"):
        shutil.copytree(d, os.path.join(dst, d), ignore=ign)
    for f in ("config.py", "motors.py"):
        shutil.copy2(f, dst)
    return dst


def _run_chunk(opp, seed0, n, seconds, sets, cwd=None):
    cmd = [sys.executable, "-m", "tools.arena", "--ai", "agent",
           "--opponent", opp, "--matches", str(n), "--seconds", str(seconds),
           "--seed0", str(seed0)]
    for kv in sets:
        cmd += ["--set", kv]
    env = dict(os.environ)
    env.update(OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1",
               MKL_NUM_THREADS="1", NUMEXPR_NUM_THREADS="1")
    p = subprocess.run(cmd, capture_output=True, text=True, env=env, cwd=cwd)
    for line in p.stdout.splitlines():
        if line.startswith("JSON "):
            return opp, json.loads(line[5:])["rows"]
    sys.stderr.write(f"FAILED {opp} {seed0}\n{p.stderr[-2000:]}\n")
    return opp, []


def _load(label):
    path = os.path.join(OUT_DIR, f"{label}.json")
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    return {"sets": None, "seconds": None, "rows": {}}


def _save(label, data):
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, f"{label}.json"), "w",
              encoding="utf-8") as f:
        json.dump(data, f)


def _stats(rows, key):
    xs = [r[key] for r in rows]
    if not xs:
        return 0.0, 0.0
    m = statistics.mean(xs)
    se = statistics.stdev(xs) / math.sqrt(len(xs)) if len(xs) > 1 else 0.0
    return m, se


def report(labels):
    datas = {lb: _load(lb) for lb in labels}
    for lb, d in datas.items():
        print(f"\n== {lb}   set={d.get('sets')}")
        print(f"  {'opponent':<11}{'n':>4}{'GF':>7}{'GA':>7}{'diff':>8}{'+-':>6}"
              f"{'on tgt':>8}{'dead%':>7}{'dngOwn%':>9}{'dngTheir%':>10}")
        tot, tot_var = 0.0, 0.0
        for opp in OPPONENTS:
            rows = list(d["rows"].get(opp, {}).values())
            if not rows:
                continue
            gf, _ = _stats(rows, "for")
            ga, _ = _stats(rows, "against")
            df, se = _stats(rows, "diff")
            tot += df
            tot_var += se * se
            print(f"  {opp:<11}{len(rows):>4}{gf:>7.2f}{ga:>7.2f}{df:>+8.2f}"
                  f"{se:>6.2f}{_stats(rows, 'on_target')[0]:>8.2f}"
                  f"{_stats(rows, 'dead_pct')[0]:>7.1f}"
                  f"{_stats(rows, 'danger_own_pct')[0]:>9.1f}"
                  f"{_stats(rows, 'danger_their_pct')[0]:>10.1f}")
        print(f"  {'SUM':<11}{'':>4}{'':>7}{'':>7}{tot:>+8.2f}"
              f"{math.sqrt(tot_var):>6.2f}")

    if len(labels) < 2:
        return
    base, rest = labels[0], labels[1:]
    for lb in rest:
        print(f"\n== {lb}  minus  {base}   (goal diff per match; paired on "
              f"shared seeds where possible)")
        tot, tot_var = 0.0, 0.0
        for opp in OPPONENTS:
            a = datas[base]["rows"].get(opp, {})
            b = datas[lb]["rows"].get(opp, {})
            if not a or not b:
                continue
            shared = sorted(set(a) & set(b))
            if len(shared) >= 4:
                ds = [b[s]["diff"] - a[s]["diff"] for s in shared]
                m = statistics.mean(ds)
                se = statistics.stdev(ds) / math.sqrt(len(ds))
                n = len(ds)
            else:
                ma, sa = _stats(list(a.values()), "diff")
                mb, sb = _stats(list(b.values()), "diff")
                m, se, n = mb - ma, math.sqrt(sa * sa + sb * sb), len(b)
            tot += m
            tot_var += se * se
            verdict = ("better" if m > 2 * se else "WORSE" if m < -2 * se
                       else "no detectable effect")
            print(f"  {opp:<11} n={n:<4}{m:>+7.2f} +- {se:.2f}   {verdict}")
        tse = math.sqrt(tot_var)
        verdict = ("better" if tot > 2 * tse else "WORSE" if tot < -2 * tse
                   else "no detectable effect")
        print(f"  {'SUM':<11}       {tot:>+7.2f} +- {tse:.2f}   {verdict}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--label")
    ap.add_argument("--matches", type=int, default=24,
                    help="per opponent")
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--seed0", type=int, default=5000)
    ap.add_argument("--chunk", type=int, default=4)
    ap.add_argument("--jobs", type=int, default=3)
    ap.add_argument("--opponents", default=",".join(OPPONENTS))
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    ap.add_argument("--report", help="comma list of labels; first is the base")
    a = ap.parse_args()

    if a.report:
        report(a.report.split(","))
        return 0
    if not a.label:
        raise SystemExit("--label is required")

    data = _load(a.label)
    if data["sets"] is not None and sorted(data["sets"]) != sorted(a.set):
        raise SystemExit(f"label {a.label!r} was run with {data['sets']}; "
                         f"refusing to mix in {a.set}")
    data["sets"], data["seconds"] = a.set, a.seconds

    opps = [o for o in a.opponents.split(",") if o]
    jobs = []
    for opp in opps:
        for s in range(a.seed0, a.seed0 + a.matches, a.chunk):
            jobs.append((opp, s, min(a.chunk, a.seed0 + a.matches - s)))
    print(f"{a.label}: {len(opps)} opponents x {a.matches} matches, "
          f"{len(jobs)} chunks on {a.jobs} workers", flush=True)
    tree = _snapshot(a.label)
    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        futs = [ex.submit(_run_chunk, o, s, n, a.seconds, a.set, tree)
                for o, s, n in jobs]
        for f in futs:
            opp, rows = f.result()
            bucket = data["rows"].setdefault(opp, {})
            for r in rows:
                bucket[str(r["seed"])] = r
            _save(a.label, data)
    report([a.label])
    return 0


if __name__ == "__main__":
    sys.exit(main())
