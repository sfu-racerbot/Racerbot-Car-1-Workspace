#!/usr/bin/env python3
"""The lap a perfect speed profile would run, on a course line or on a path really driven.

    python3 tools/run_analysis/lap_theory.py --map data/racerbot_auto/20261030-185036-fixed
    python3 tools/run_analysis/lap_theory.py --map data/racerbot_auto/20261030-185036-fixed \\
        --driven data/racerbot_runs/20261030-185036 --from 50 --to 92

Answers "is the time lost in the line or in the speed". The profile runs at the
lateral limit in corners and at the acceleration and braking limits between them.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

SMOOTH_M = 0.4     # m of smoothing before curvature, or scan jitter reads as corners


def curvature(xy, spacing: float, smooth_m: float = SMOOTH_M, closed: bool = True) -> np.ndarray:
    """|curvature| of evenly spaced points."""
    from scipy.ndimage import gaussian_filter1d
    mode = "wrap" if closed else "nearest"
    sigma = smooth_m / spacing
    x, y = gaussian_filter1d(xy[:, 0], sigma, mode=mode), gaussian_filter1d(xy[:, 1], sigma, mode=mode)
    dx, dy = np.gradient(x), np.gradient(y)
    ddx, ddy = np.gradient(dx), np.gradient(dy)
    return np.abs(dx * ddy - dy * ddx) / np.maximum((dx ** 2 + dy ** 2) ** 1.5, 1e-9)


def speed_profile(kappa, spacing: float, lateral: float, braking: float, accel: float,
                  top_speed: float, closed: bool = True) -> np.ndarray:
    """The fastest speed at each point within the limits."""
    v = np.minimum(top_speed, np.sqrt(lateral / np.maximum(kappa, 1e-4)))
    n = len(v)
    for _ in range(3 if closed else 1):
        for i in range(n if closed else n - 1):
            j = (i + 1) % n
            v[j] = min(v[j], math.sqrt(v[i] ** 2 + 2.0 * accel * spacing))
        for i in reversed(range(n if closed else n - 1)):
            j = (i + 1) % n
            v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2.0 * braking * spacing))
    return v


def resample(xy, spacing: float, closed: bool = True) -> np.ndarray:
    points = np.vstack([xy, xy[:1]]) if closed else np.asarray(xy)
    along = np.r_[0.0, np.cumsum(np.hypot(*np.diff(points, axis=0).T))]
    q = np.arange(0.0, along[-1], spacing)
    return np.c_[np.interp(q, along, points[:, 0]), np.interp(q, along, points[:, 1])]


def lap_time(xy, lateral: float, braking: float, accel: float, top_speed: float,
             spacing: float = 0.1, closed: bool = True) -> tuple:
    """(seconds, metres) for one pass of ``xy`` at the perfect profile."""
    path = resample(xy, spacing, closed)
    v = speed_profile(curvature(path, spacing, closed=closed), spacing, lateral, braking, accel,
                      top_speed, closed)
    return float(np.sum(spacing / np.maximum(v, 0.2))), float(len(path) * spacing)


def relax_line(xy, distance_at, margin: float, iterations: int = 800) -> np.ndarray:
    """Shorten and straighten a closed line, keeping every point ``margin`` from walls.
    ``distance_at(x, y)`` gives metres to the nearest wall."""
    q = np.array(xy, dtype=float)
    for _ in range(iterations):
        moved = q + 0.25 * (np.roll(q, 1, 0) + np.roll(q, -1, 0) - 2.0 * q)
        allowed = distance_at(moved[:, 0], moved[:, 1]) >= margin
        q[allowed] = moved[allowed]
    return q


def main(argv=None) -> int:
    from run_analysis.laps import Course
    from run_analysis.loc import COL, DEFAULT_OUT, Grid, load_line, load_track
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", required=True, help="directory holding map.yaml and raceline_raw.csv")
    ap.add_argument("--line", help="course line CSV (default: <map>/raceline_raw.csv)")
    ap.add_argument("--margin", type=float, default=0.32, help="wall margin for the cut line, m")
    ap.add_argument("--driven", help="a run located with loc.py: judge the path it drove")
    ap.add_argument("--from", dest="t0", type=float, default=0.0, help="seconds into that run")
    ap.add_argument("--to", dest="t1", type=float, default=math.inf, help="seconds into that run")
    ap.add_argument("--loc", default=str(DEFAULT_OUT))
    ap.add_argument("--lateral", nargs="*", type=float, default=[2.5, 3.0, 3.5, 4.0, 5.0])
    ap.add_argument("--braking", type=float, default=3.0)
    ap.add_argument("--accel", type=float, default=3.5)
    ap.add_argument("--top-speed", type=float, default=5.0)
    args = ap.parse_args(argv)
    grid = Grid.from_map(args.map)
    line = load_line(args.line or Path(args.map).expanduser() / "raceline_raw.csv")
    course = Course(line)
    if args.driven:
        a = load_track(args.driven, args.loc)
        t = a[:, COL["t"]] - a[0, COL["t"]]
        a = a[(t >= args.t0) & (t <= args.t1)]
        s, _ = course.project(a[:, COL["x"]], a[:, COL["y"]])
        fraction = abs(course.unwrap(s)[-1]) / course.length
        path = a[:, [COL["x"], COL["y"]]]
        actual = (a[-1, COL["t"]] - a[0, COL["t"]]) / fraction
        print(f"{Path(args.driven).name}: {fraction * 100:.0f}% of a lap, actual {actual:.2f} s/lap")
        for lateral in args.lateral:
            seconds, _m = lap_time(path, lateral, args.braking, args.accel, args.top_speed, closed=False)
            print(f"   perfect speed on this path at {lateral} m/s^2: {seconds / fraction:.2f} s/lap")
        return 0
    cut = relax_line(course.xy[::2], lambda x, y: grid.sample(grid.distance, x, y), args.margin)
    for name, path in (("course line", course.xy[::2]), (f"cut line, {args.margin} m margin", cut)):
        length = lap_time(path, 3.0, args.braking, args.accel, args.top_speed)[1]
        print(f"{name}: {length:.1f} m")
        for lateral in args.lateral:
            print(f"   {lateral} m/s^2: {lap_time(path, lateral, args.braking, args.accel, args.top_speed)[0]:.2f} s")
    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    raise SystemExit(main())
