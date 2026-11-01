#!/usr/bin/env python3
"""The real car's grip curve: lateral acceleration asked for against what it achieved.

    python3 tools/run_analysis/gripcurve.py data/racerbot_runs/20261030-18*

Asked is true speed squared times the commanded curvature, with the command delayed
by the rack's lag. Achieved is true speed times the gyro. True speed comes from loc.py,
because the wheels over-read. The car has no sharp limit: it follows the request, then
rolls off progressively, which one friction number cannot describe.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

WHEELBASE = 0.36        # m, docs/hardware-reference.md
STEER_LAG = 0.15        # s, rack lag measured on run 20261019-200703
FULL_LOCK = 0.46        # rad, what the rack reaches; Team A commands past it
BINS = ((0, .5), (.5, 1), (1, 1.5), (1.5, 2), (2, 2.5), (2.5, 3), (3, 3.5), (3.5, 4),
        (4, 5), (5, 6), (6, 8), (8, 12), (12, 30))


def asked_achieved(t, speed, gyro_z, steer_cmd, wheelbase: float = WHEELBASE,
                   lag: float = STEER_LAG, full_lock: float = FULL_LOCK) -> tuple:
    """(asked, achieved) lateral acceleration, m/s^2, per sample."""
    t, speed = np.asarray(t), np.asarray(speed)
    steer = np.interp(t - lag, t, np.clip(steer_cmd, -full_lock, full_lock))
    return speed ** 2 * np.tan(np.abs(steer)) / wheelbase, np.abs(speed * np.asarray(gyro_z))


def curve(asked, achieved, bins=BINS, min_samples: int = 10) -> list:
    """Rows of (bin low, bin high, samples, median achieved, p90 achieved)."""
    asked, achieved = np.asarray(asked), np.asarray(achieved)
    rows = []
    for low, high in bins:
        chosen = (asked >= low) & (asked < high)
        if chosen.sum() >= min_samples:
            rows.append((low, high, int(chosen.sum()), float(np.median(achieved[chosen])),
                         float(np.percentile(achieved[chosen], 90))))
    return rows


def main(argv=None) -> int:
    from run_analysis.loc import COL, DEFAULT_OUT, load_track
    from run_analysis.sectors import MAX_TRUE_SPEED, true_speed
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="run directories already located with loc.py")
    ap.add_argument("--loc", default=str(DEFAULT_OUT), help="where loc.py wrote (default %(default)s)")
    ap.add_argument("--min-speed", type=float, default=1.0, help="ignore samples below this, m/s")
    args = ap.parse_args(argv)
    asked, achieved = [], []
    for run in args.runs:
        try:
            a = load_track(run, args.loc)
        except FileNotFoundError as error:
            print(error, file=sys.stderr)
            continue
        if len(a) < 100:
            continue
        v = true_speed(a[:, COL["t"]], a[:, COL["x"]], a[:, COL["y"]])
        ask, got = asked_achieved(a[:, COL["t"]], v, a[:, COL["gyro_z"]], a[:, COL["steer_cmd"]])
        keep = (a[:, COL["lb"]] > 0) & (a[:, COL["h20"]] > 0.8) & (v > args.min_speed) & (v < MAX_TRUE_SPEED)
        asked.append(ask[keep])
        achieved.append(got[keep])
    if not asked:
        print("no located runs", file=sys.stderr)
        return 1
    asked, achieved = np.concatenate(asked), np.concatenate(achieved)
    print(f"{len(asked)} samples")
    print(" asked m/s^2     n   achieved median   p90   ratio")
    for low, high, n, median, p90 in curve(asked, achieved):
        print(f"  {low:4.1f}-{high:4.1f} {n:6d}   {median:13.2f} {p90:5.2f} {median / ((low + high) / 2.0):7.2f}")
    print(f"achieved p95/p99/p99.9: {np.percentile(achieved, [95, 99, 99.9]).round(2)}")
    return 0 if math.isfinite(float(achieved.max())) else 1


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    raise SystemExit(main())
