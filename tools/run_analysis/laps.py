#!/usr/bin/env python3
"""True lap times and pace from runs located with loc.py, measured along a course line.

    python3 tools/run_analysis/laps.py data/racerbot_runs/20261030-185036 \\
        --line data/racerbot_auto/20261030-185036-fixed/raceline_raw.csv

A lap is a full course length of progress within one LB hold. A hold too short to
close a lap still gets a pace: the fastest --pace-window metres scaled up to a lap.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

SAMPLE = 0.05      # m between centre line points
JUMP = 1.5         # m of progress in one sample that is a re-localisation, not driving


class Course:
    """A closed line, resampled evenly, measuring progress and lateral offset."""

    def __init__(self, line, spacing: float = SAMPLE):
        closed = np.vstack([line, line[:1]])
        along = np.r_[0.0, np.cumsum(np.hypot(*np.diff(closed, axis=0).T))]
        self.length = float(along[-1])
        self.spacing = spacing
        q = np.arange(0.0, self.length, spacing)
        self.xy = np.c_[np.interp(q, along, closed[:, 0]), np.interp(q, along, closed[:, 1])]
        self.s = q
        self._tx, self._ty = np.gradient(self.xy[:, 0]), np.gradient(self.xy[:, 1])

    def project(self, x, y) -> tuple:
        """(arc length of the nearest line point, signed offset, left positive)."""
        x, y = np.atleast_1d(x), np.atleast_1d(y)
        s, lateral = np.zeros(len(x)), np.zeros(len(x))
        for i in range(0, len(x), 2000):
            px, py = x[i:i + 2000][:, None], y[i:i + 2000][:, None]
            j = ((px - self.xy[:, 0]) ** 2 + (py - self.xy[:, 1]) ** 2).argmin(axis=1)
            s[i:i + 2000] = self.s[j]
            norm = np.hypot(self._tx[j], self._ty[j])
            lateral[i:i + 2000] = ((px[:, 0] - self.xy[j, 0]) * -self._ty[j]
                                   + (py[:, 0] - self.xy[j, 1]) * self._tx[j]) / norm
        return s, lateral

    def unwrap(self, s) -> np.ndarray:
        """Cumulative progress from wrapped arc length; jumps count as nothing."""
        ds = np.diff(s)
        half = self.length / 2.0
        ds = np.where(ds > half, ds - self.length, np.where(ds < -half, ds + self.length, ds))
        ds = np.where(np.abs(ds) > JUMP, 0.0, ds)
        return np.r_[0.0, np.cumsum(ds)]


def holds(lb) -> list:
    """(first, last + 1) index pairs of each run of true values in ``lb``."""
    edges = np.flatnonzero(np.diff(np.r_[0, np.asarray(lb).astype(int), 0]))
    return list(zip(edges[::2], edges[1::2]))


def lap_times(t, progress, length: float) -> list:
    """Seconds per whole lap of ``progress`` (either direction), from its start."""
    t, p = np.asarray(t), np.asarray(progress, dtype=float)
    if len(p) < 2:
        return []
    p = (p - p[0]) * (np.sign(p[-1] - p[0]) or 1.0)
    mono = np.maximum.accumulate(p)
    laps = int(mono[-1] // length)
    marks = [float(np.interp(k * length, mono, t)) for k in range(laps + 1)]
    return [marks[i + 1] - marks[i] for i in range(laps)]


def best_pace(t, progress, length: float, window: float = 25.0) -> float:
    """Seconds per lap at the fastest ``window`` metres, or nan if it never got that far.
    A fast stretch flatters this, so it is an upper bound on pace and not a lap time."""
    t, p = np.asarray(t), np.asarray(progress, dtype=float)
    if len(p) < 2:
        return math.nan
    p = (p - p[0]) * (np.sign(p[-1] - p[0]) or 1.0)
    mono = np.maximum.accumulate(p)
    ends = np.searchsorted(mono, mono + window)
    valid = ends < len(t)
    if not valid.any():
        return math.nan
    return float(np.min(t[ends[valid]] - t[valid]) * length / window)


def main(argv=None) -> int:
    from run_analysis.loc import COL, DEFAULT_OUT, load_line, load_track
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="run directories")
    ap.add_argument("--line", required=True, help="course line CSV")
    ap.add_argument("--loc", default=str(DEFAULT_OUT), help="where loc.py wrote (default %(default)s)")
    ap.add_argument("--pace-window", type=float, default=25.0, help="metres for the pace figure")
    args = ap.parse_args(argv)
    course = Course(load_line(args.line))
    print(f"course line {course.length:.2f} m")
    for run in args.runs:
        a = load_track(run, args.loc)
        print(f"{Path(run).name}: {a[:, COL['h10']].mean():.2f} of beams within 10 cm")
        s, _ = course.project(a[:, COL["x"]], a[:, COL["y"]])
        progress = course.unwrap(s)
        for i, j in holds(a[:, COL["lb"]] > 0):
            if j - i < 20:
                continue
            t = a[i:j, COL["t"]]
            laps = lap_times(t, progress[i:j], course.length)
            pace = best_pace(t, progress[i:j], course.length, args.pace_window)
            print(f"   hold {t[-1] - t[0]:6.1f} s, {abs(progress[j - 1] - progress[i]):6.1f} m, "
                  f"laps {[round(x, 1) for x in laps]}, best pace {pace:.1f} s/lap")
    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    raise SystemExit(main())
