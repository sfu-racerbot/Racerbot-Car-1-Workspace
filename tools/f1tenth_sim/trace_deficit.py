#!/usr/bin/env python3
"""Where a simulated lap loses time against a perfect speed profile on its own path, and why.

    python3 tools/f1tenth_sim/trace_gap.py --track course_2026_10_30_scans --floor sfu_course_2026_10_30 \\
        --timeout 60 --out /tmp/trace.csv
    python3 tools/f1tenth_sim/trace_deficit.py /tmp/trace.csv --lateral 3.0

Takes trace_gap.py's per-tick CSV. Along the path the car actually drove, it computes the
fastest speed profile that path allows (lap_theory.speed_profile, at the --lateral grip,
--braking and --accel limits and the VESC top speed), then per 2.5 m of path prints the
time lost against it, the simulated and ideal speeds, and which of gap_follow's speed caps
was binding there. The total says how much a speed-planning change could buy without
touching the line; the sections say where.
"""

from __future__ import annotations

import argparse
import collections
import csv
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SPACING = 0.1           # m between resampled path points
TOP_SPEED = 5.04        # m/s, vesc.yaml speed_max / speed_to_erpm_gain


def read_trace(path) -> dict:
    """trace_gap.py's CSV as float arrays, plus the binding cap names; blanks become nan."""
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"{path} has no rows")

    def column(name):
        return np.array([float(r[name]) if r.get(name) not in ("", None, "None") else np.nan for r in rows])
    speed = "speed_true" if "speed_true" in rows[0] else "speed_meas"
    return dict(t=column("t"), x=column("x"), y=column("y"), v=column(speed), steering=column("steering"),
                binding=[r.get("binding", "") for r in rows])


def deficit(trace: dict, lateral: float, braking: float, accel: float, top_speed: float = TOP_SPEED,
            section: float = 2.5) -> dict:
    """Ideal and actual time along the driven path, total and per ``section`` metres."""
    from run_analysis.lap_theory import curvature, speed_profile
    x, y, t = trace["x"], trace["y"], trace["t"]
    along = np.r_[0.0, np.cumsum(np.hypot(np.diff(x), np.diff(y)))]
    q = np.arange(0.0, along[-1], SPACING)
    path = np.column_stack([np.interp(q, along, x), np.interp(q, along, y)])
    ideal_v = speed_profile(curvature(path, SPACING, closed=False), SPACING, lateral, braking, accel,
                            top_speed, closed=False)
    ideal_t = np.r_[0.0, np.cumsum(SPACING / np.maximum(ideal_v[:-1], 0.2))]
    actual_t = np.interp(q, along, t) - t[0]
    rows = []
    for start in np.arange(0.0, q[-1] - section, section):
        i, j = int(round(start / SPACING)), int(round((start + section) / SPACING))
        here = (along >= start) & (along < start + section)
        counts = collections.Counter(b for b, h in zip(trace["binding"], here) if h and b)
        rows.append(dict(at=float(start), lost=float((actual_t[j] - actual_t[i]) - (ideal_t[j] - ideal_t[i])),
                         speed=float(np.nanmedian(trace["v"][here])) if here.any() else np.nan,
                         ideal=float(np.median(ideal_v[i:j])),
                         steering=float(np.nanmedian(np.abs(trace["steering"][here]))) if here.any() else np.nan,
                         binding=dict(counts.most_common(3))))
    return dict(length=float(q[-1]), ideal=float(ideal_t[-1]), actual=float(actual_t[-1]), sections=rows)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("trace", help="trace_gap.py --out CSV")
    ap.add_argument("--lateral", type=float, default=3.0, help="m/s^2 of cornering grip (default %(default)s)")
    ap.add_argument("--braking", type=float, default=3.0, help="m/s^2 (default %(default)s)")
    ap.add_argument("--accel", type=float, default=3.5, help="m/s^2 (default %(default)s)")
    ap.add_argument("--top-speed", type=float, default=TOP_SPEED)
    args = ap.parse_args(argv)
    d = deficit(read_trace(args.trace), args.lateral, args.braking, args.accel, args.top_speed)
    print(f"{d['length']:.1f} m driven: {d['actual']:.2f} s, a perfect profile on that path {d['ideal']:.2f} s "
          f"({d['actual'] - d['ideal']:+.2f} s)")
    print("    at   lost  speed  ideal  |steer|  binding")
    for r in d["sections"]:
        total = sum(r["binding"].values()) or 1
        names = " ".join(f"{k}:{100 * n // total}%" for k, n in r["binding"].items())
        print(f"{r['at']:6.1f} {r['lost']:+6.2f} {r['speed']:6.2f} {r['ideal']:6.2f} {r['steering']:8.3f}  {names}")
    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(ROOT / "tools"))
    raise SystemExit(main())
