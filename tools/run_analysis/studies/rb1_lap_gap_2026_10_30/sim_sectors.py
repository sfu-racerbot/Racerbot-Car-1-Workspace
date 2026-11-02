"""Per-sector simulated time, car 1 as raced against car 1 at the corrected command gain.

Traces car 1's gap_follow on course_2026_10_30_scans with the measured floor, per arm and seed,
projects the true path onto the real course line (same 10 sectors as sectime.py) and prints
the median time per sector and the binding cap there.
"""
import collections
import dataclasses
import statistics
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from _paths import LINE, WS  # noqa: E402
import eval_gap  # noqa: E402
import trace_gap  # noqa: E402
from sim_fidelity import calibration_for_car, with_floor  # noqa: E402
from run_analysis.laps import Course  # noqa: E402
from run_analysis.loc import load_line  # noqa: E402

TRACK, FLOOR = "course_2026_10_30_scans", "sfu_course_2026_10_30"
ARMS = {"asraced": 0.79, "fixed": 1.0}
SEEDS = range(12345, 12351)
NSEC = 10


def job(a):
    name, scale, seed = a
    params = eval_gap.parse_set([], "rb1")
    cal = dataclasses.replace(with_floor(FLOOR, calibration_for_car("rb1")),
                              speed_command_scale=scale, odom_speed_scale=1.16)
    rows, laps = trace_gap.trace(TRACK, params, seed, 90.0, calibration=cal, car_id="rb1")
    return name, seed, [(float(r["t"]), float(r["x"]), float(r["y"]), float(r["speed_true"]),
                         float(r["cmd_speed"]), r["binding"]) for r in rows], laps


def main():
    eval_gap.track_path(TRACK)
    course = Course(load_line(LINE))
    L = course.length
    jobs = [(n, s, seed) for n, s in ARMS.items() for seed in SEEDS]
    with ProcessPoolExecutor(max_workers=2) as pool:
        out = list(pool.map(job, jobs))
    sect = {n: [[] for _ in range(NSEC)] for n in ARMS}
    binds = {n: [collections.Counter() for _ in range(NSEC)] for n in ARMS}
    for name, seed, rows, laps in out:
        a = np.array([r[:5] for r in rows])
        t, x, y = a[:, 0], a[:, 1], a[:, 2]
        s, lat = course.project(x, y)
        if seed == 12345:
            print(f"{name}: lap flag {laps}, |lateral offset| median {np.median(np.abs(lat)):.2f} m (frame check)")
        p = course.unwrap(s) + s[0]
        mono = np.maximum.accumulate(p)
        for k in range(NSEC):
            for lap in range(2):
                a0 = k * L / NSEC + lap * L
                a1 = a0 + L / NSEC
                if a0 < p[0] or mono[-1] < a1:
                    continue
                t0, t1 = np.interp(a0, mono, t), np.interp(a1, mono, t)
                sect[name][k].append(t1 - t0)
                for i in np.flatnonzero((t >= t0) & (t <= t1)):
                    binds[name][k][rows[i][5] or "-"] += 1
                break
    tot = {}
    print(f"{'sector':>6} {'from':>5} | " + " | ".join(f"{n:>8} t  binding" for n in ARMS) + " | gain")
    for k in range(NSEC):
        cells, meds = [], []
        for n in ARMS:
            m = statistics.median(sect[n][k]) if sect[n][k] else float("nan")
            meds.append(m)
            tot[n] = tot.get(n, 0) + (m if m == m else 0)
            top = binds[n][k].most_common(1)
            cells.append(f"{m:6.2f} {top[0][0] if top else '-':>14s}")
        print(f"{k:6d} {k * L / NSEC:5.1f} | " + " | ".join(cells) + f" | {meds[0] - meds[1]:+.2f} s")
    print("sum of sector medians: " + ", ".join(f"{n} {v:.2f} s" for n, v in tot.items()))


if __name__ == "__main__":
    main()
