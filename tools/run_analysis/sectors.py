#!/usr/bin/env python3
"""True speed by position on the course, one column per run: where each run loses time.

    python3 tools/run_analysis/sectors.py data/rb2/racerbot_runs/20261030-185036 \\
        data/rb2/racerbot_runs/20261030-181819 --skip 30 2.5 \\
        --line data/rb2/racerbot_auto/20261030-185036-fixed/raceline_raw.csv

Speed comes from the located pose, not the wheels, using each run's longest LB hold.
--skip drops the first seconds of that hold in each run (a standing start is not pace).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

MAX_TRUE_SPEED = 8.0   # m/s, faster than this between two poses is a re-localisation


def true_speed(t, x, y, smooth: int = 5) -> np.ndarray:
    """Speed from pose differences, poses smoothed over ``smooth`` samples."""
    kernel = np.ones(smooth) / smooth
    sx, sy = np.convolve(x, kernel, "same"), np.convolve(y, kernel, "same")
    return np.hypot(np.gradient(sx), np.gradient(sy)) / np.maximum(np.gradient(t), 1e-3)


def sector_medians(s, value, length: float, sector: float, mask=None, min_samples: int = 2) -> np.ndarray:
    """Median of ``value`` per ``sector`` metres of arc length ``s``; nan if too few samples."""
    s, value = np.asarray(s), np.asarray(value, dtype=float)
    mask = np.ones(len(s), bool) if mask is None else np.asarray(mask, bool)
    bins = (s // sector).astype(int)
    out = np.full(int(length // sector) + 1, np.nan)
    for q in range(len(out)):
        chosen = mask & (bins == q)
        if chosen.sum() >= min_samples:
            out[q] = np.median(value[chosen])
    return out


def time_at_speeds(speeds, sector: float) -> float:
    """Seconds to cover the sectors that have a speed, each at that speed."""
    speeds = np.asarray(speeds, dtype=float)
    return float(np.nansum(sector / speeds))


def longest_hold(a, lb_col: int) -> np.ndarray:
    from run_analysis.laps import holds
    i, j = max(holds(a[:, lb_col] > 0), key=lambda h: h[1] - h[0])
    return a[i:j]


def main(argv=None) -> int:
    from run_analysis.laps import Course
    from run_analysis.loc import COL, DEFAULT_OUT, load_line, load_track
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="run directories")
    ap.add_argument("--line", required=True, help="course line CSV")
    ap.add_argument("--loc", default=str(DEFAULT_OUT), help="where loc.py wrote (default %(default)s)")
    ap.add_argument("--skip", nargs="*", type=float, default=[], help="seconds to drop per run (default 2.5)")
    ap.add_argument("--sector", type=float, default=2.5, help="metres per sector")
    ap.add_argument("--plot", help="also write a PNG of speed against position here")
    args = ap.parse_args(argv)
    course = Course(load_line(args.line))
    table = {}
    for k, run in enumerate(args.runs):
        a = longest_hold(load_track(run, args.loc), COL["lb"])
        t = a[:, COL["t"]]
        keep = t - t[0] > (args.skip[k] if k < len(args.skip) else 2.5)
        a, t = a[keep], t[keep]
        v = true_speed(t, a[:, COL["x"]], a[:, COL["y"]])
        s, _ = course.project(a[:, COL["x"]], a[:, COL["y"]])
        good = (a[:, COL["h20"]] > 0.6) & (v < MAX_TRUE_SPEED)
        table[Path(run).name[-6:]] = sector_medians(s, v, course.length, args.sector, good)
        print(f"{Path(run).name}: true {v[good].mean():.2f} m/s, wheels {a[good, COL['wheel_v']].mean():.2f} m/s")
    names = list(table)
    print("sector m  " + " ".join(f"{n:>7}" for n in names))
    for q in range(len(next(iter(table.values())))):
        print(f"{q * args.sector:8.1f}  " + " ".join(f"{table[n][q]:7.2f}" for n in names))
    print("lap, s    " + " ".join(f"{time_at_speeds(table[n], args.sector):7.2f}" for n in names))
    print("sectors   " + " ".join(f"{int(np.isfinite(table[n]).sum()):7d}" for n in names))
    best = np.nanmax(np.array([table[n] for n in names]), axis=0)
    print(f"best speed anyone showed in each sector adds up to {time_at_speeds(best, args.sector):.2f} s")
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.figure(figsize=(12, 5))
        for n in names:
            plt.plot((np.arange(len(table[n])) + 0.5) * args.sector, table[n], label=n)
        plt.xlabel("position along course line (m)")
        plt.ylabel("true speed (m/s)")
        plt.legend()
        plt.grid(alpha=0.3)
        plt.tight_layout()
        plt.savefig(args.plot, dpi=80)
    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    raise SystemExit(main())
