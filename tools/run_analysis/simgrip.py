#!/usr/bin/env python3
"""The simulator's grip curve, to set beside the real car's from gripcurve.py.

    python3 tools/run_analysis/simgrip.py --track course_2026_10_30 --friction 0.55,0.42

Drives the track in the simulator, at --lateral values high enough to reach the top
of the curve, and reports lateral acceleration asked for against what the simulated
car achieved, computed the same way gripcurve.py computes it on the floor.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "tools"), str(ROOT / "tools" / "f1tenth_sim")]


def yaw_rate(t, yaw_deg, smooth: int = 5) -> np.ndarray:
    """rad/s from a yaw trace in degrees that may wrap at +/-180."""
    yaw = np.unwrap(np.radians(yaw_deg))
    return np.gradient(np.convolve(yaw, np.ones(smooth) / smooth, "same"), t)


def main(argv=None) -> int:
    import eval_gap
    import trace_gap
    from run_analysis.gripcurve import asked_achieved, curve
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--track", default="course_2026_10_30")
    ap.add_argument("--friction", default="", help="comma list (default: the calibration's)")
    ap.add_argument("--lateral", default="3.0,5.0", help="max_lateral_accel values to drive at")
    ap.add_argument("--seed", type=int, default=12345)
    ap.add_argument("--timeout", type=float, default=45.0, help="simulated seconds per drive")
    ap.add_argument("--set", action="append", default=[], help="a parameter, name=value")
    args = ap.parse_args(argv)
    for friction in [float(x) for x in args.friction.split(",") if x] or [None]:
        asked, achieved = [], []
        for lateral in (float(x) for x in args.lateral.split(",")):
            params = eval_gap.parse_set(args.set + [f"max_lateral_accel={lateral}"])
            rows, _laps = trace_gap.trace(args.track, params, args.seed, args.timeout,
                                          eval_gap.with_friction(friction))
            t = np.array([r["t"] for r in rows], dtype=float)
            v = np.array([r["speed_true"] for r in rows], dtype=float)
            ask, got = asked_achieved(t, v, [r["yaw_rate_true"] for r in rows],
                                      [r["steering"] for r in rows])
            asked.append(ask[v > 1.0])
            achieved.append(got[v > 1.0])
        asked, achieved = np.concatenate(asked), np.concatenate(achieved)
        print(f"friction {friction}: {len(asked)} samples")
        for low, high, n, median, _p90 in curve(asked, achieved, min_samples=4):
            print(f"   asked {low:4.1f}-{high:4.1f}  n {n:4d}  achieved {median:5.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
