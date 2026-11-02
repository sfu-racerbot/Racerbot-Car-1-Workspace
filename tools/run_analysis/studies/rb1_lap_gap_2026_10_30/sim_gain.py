"""Lap time of car 1's gap_follow on the 2026-10-30 course against how much of each speed command
the car really delivers (speed_command_scale), everything else equal.

    rb python3 sim_gain.py <name>=<command scale>[,<odom scale>] ... [--seeds 6] [--workers 2]

The command scale is true speed per unit of commanded speed: 0.79 is car 1 as raced on 2026-10-30
(speed_gain.py, pooled over its solo race runs), 1.0 is car 1 at its 4600 command gain.
"""
import argparse
import dataclasses
import statistics
import sys
from concurrent.futures import ProcessPoolExecutor

from _paths import LINE, WS  # noqa: E402
import eval_gap  # noqa: E402
from sim_fidelity import calibration_for_car, with_floor  # noqa: E402

TRACK = "course_2026_10_30_scans"
FLOOR = "sfu_course_2026_10_30"


def job(args):
    name, cal, seed, params = args
    r = eval_gap.run_track(TRACK, params, seed, 90.0, calibration=cal, car_id="rb1")
    return name, seed, r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("arms", nargs="+")
    ap.add_argument("--seeds", type=int, default=6)
    ap.add_argument("--seed", type=int, default=12345)
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--set", action="append", default=[])
    a = ap.parse_args()
    params = eval_gap.parse_set(a.set, "rb1")
    eval_gap.track_path(TRACK)
    base = with_floor(FLOOR, calibration_for_car("rb1"))
    jobs = []
    for arm in a.arms:
        name, _, vals = arm.partition("=")
        parts = [float(v) for v in vals.split(",")]
        kw = {"speed_command_scale": parts[0]}
        if len(parts) > 1:
            kw["odom_speed_scale"] = parts[1]
        cal = dataclasses.replace(base, **kw)
        jobs += [(name, cal, a.seed + k, params) for k in range(a.seeds)]
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        out = list(pool.map(job, jobs))
    by = {}
    for name, seed, r in out:
        by.setdefault(name, []).append(r)
        print(f"{name:10s} seed={seed} passed={r['passed']} laps={r['laps']:.2f} time={r['sim_time_s']:6.2f}s "
              f"vmean={r['mean_speed']:.2f} contact={r.get('contact_steps', 0)} stops={r['stop_steps']}", flush=True)
    for name, rs in by.items():
        ok = [r['sim_time_s'] for r in rs if r['passed']]
        print(f"== {name}: {len(ok)}/{len(rs)} passed, lap median {statistics.median(ok) if ok else float('nan'):.2f} s "
              f"(min {min(ok) if ok else float('nan'):.2f}, max {max(ok) if ok else float('nan'):.2f}), "
              f"vmean {statistics.mean(r['mean_speed'] for r in rs):.2f}, "
              f"contact ticks {sum(r.get('contact_steps', 0) for r in rs)}")


if __name__ == "__main__":
    main()
