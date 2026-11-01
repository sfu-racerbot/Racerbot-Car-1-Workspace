#!/usr/bin/env python3
"""Compare gap_follow settings across tracks, seeds, directions and floor frictions.

    python3 tools/f1tenth_sim/sweep2.py --out results.jsonl \\
        --track course_2026_10_30,levine_blocked --seeds 12 --friction 0.55,0.42 \\
        base: lat4:max_lateral_accel=4.0 lat4s:max_lateral_accel=4.0,max_steering_angle=0.436

Each candidate is name:param=value,param=value (or name: for the shipped YAML). One
row per candidate, friction and track is appended to --out as each finishes, so a
crash loses little. summ2.py prints them. Three seeds misled a tuning decision on
2026-10-30, so use at least six before believing a difference.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eval_gap  # noqa: E402


def parse_candidate(spec: str) -> tuple:
    """(name, [name=value, ...]) from ``name:param=value,param=value``."""
    name, sep, values = spec.partition(":")
    if not sep or not name:
        raise SystemExit(f"a candidate is name:param=value,... got {spec!r}")
    return name, [item for item in values.split(",") if item]


def summarize(candidate: str, friction, track: str, runs: list) -> dict:
    """One row from the eval_gap results of ``runs`` on one track."""
    passed = [r for r in runs if r["passed"]]
    clearances = [r["min_body_clearance_m"] for r in runs if r["min_body_clearance_m"] is not None]
    return dict(
        cand=candidate, mu=friction, track=track, passed=len(passed), total=len(runs),
        lap=round(sum(r["sim_time_s"] / max(r["laps"], 1e-6) for r in passed) / len(passed), 2) if passed else None,
        contact=sum(1 for r in runs if r.get("contact_steps", 0) > 0),
        minclear=min(clearances, default=None),
        stops=sum(r["stop_steps"] for r in runs))


def _one(job):
    track, seed, params, timeout, reverse, calibration = job
    return eval_gap.run_track(track, params, seed, timeout, reverse, calibration)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("candidates", nargs="+", help="name:param=value,param=value")
    ap.add_argument("--out", required=True, help="JSON lines file to append to")
    ap.add_argument("--track", required=True, help="as eval_gap: a name, group, or comma list")
    ap.add_argument("--seeds", type=int, default=6)
    ap.add_argument("--first-seed", type=int, default=12345)
    ap.add_argument("--timeout", type=float, default=150.0, help="simulated seconds per lap")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--friction", default="", help="comma list (default: the calibration's)")
    ap.add_argument("--floor", choices=sorted(eval_gap.FLOORS),
                    help="a measured floor profile (default: the plain calibration)")
    ap.add_argument("--forward-only", action="store_true", help="skip the reverse direction")
    ap.add_argument("--set", action="append", default=[], help="a parameter for every candidate")
    args = ap.parse_args(argv)
    candidates = [parse_candidate(spec) for spec in args.candidates]
    tracks = eval_gap.select_tracks(args.track)
    frictions = [float(x) for x in args.friction.split(",") if x] or [None]
    directions = (False,) if args.forward_only else (False, True)
    for track in tracks:
        eval_gap.track_path(track)     # build layouts once, before forking
    for name, values in candidates:
        params = eval_gap.parse_set(args.set + values)
        for friction in frictions:
            calibration = eval_gap.with_friction(friction, eval_gap.with_floor(args.floor))
            jobs = [(t, args.first_seed + k, params, args.timeout, rev, calibration)
                    for t in tracks for k in range(args.seeds) for rev in directions]
            started = time.time()
            with ProcessPoolExecutor(max_workers=args.workers) as pool:
                results = list(pool.map(_one, jobs))
            with open(args.out, "a") as out:
                for track in tracks:
                    row = summarize(name, friction, track, [r for r in results if r["track"] == track])
                    out.write(json.dumps(row) + "\n")
            print(f"{name} friction {friction}: {time.time() - started:.0f} s", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
