"""Scan and drive timing per run, LB-held stretches only.

scan rate        : /scan messages per second (receive time)
beams            : ranges per scan, and finite fraction
scan age         : receive time - header stamp (sensor driver + transport; needs both clocks on the Jetson)
scan->drive      : /drive receive time - receive time of the latest /scan before it
drive rate       : /drive per second
"""
import sys
from pathlib import Path

import numpy as np

from _paths import CACHE, LINE, LOC  # noqa: E402,F401
for run in sys.argv[1:]:
    b = np.load(CACHE / "bag" / f"{run}.npz")
    sc, dr, j = b["scan"], b["drive"], b["joy"]
    lbt = j[j[:, 1] > 0, 0]

    def held(t):
        if len(lbt) == 0:
            return np.zeros(len(t), bool)
        k = np.searchsorted(lbt, t)
        near = np.minimum(np.abs(t - lbt[np.clip(k - 1, 0, len(lbt) - 1)]), np.abs(t - lbt[np.clip(k, 0, len(lbt) - 1)]))
        return near < 0.1
    hs, hd = held(sc[:, 0]), held(dr[:, 0])
    srate = 1 / np.median(np.diff(sc[:, 0]))
    sdt = np.diff(sc[:, 0])
    age = (sc[:, 0] - sc[:, 1])[hs]
    k = np.searchsorted(sc[:, 0], dr[:, 0]) - 1
    ok = (k >= 0) & hd
    lat = dr[ok, 0] - sc[k[ok], 0]
    ddt = np.diff(dr[hd, 0])
    print(f"{run}: scan {srate:4.1f} Hz (p99 gap {np.percentile(sdt, 99) * 1e3:5.1f} ms, gaps>50ms {np.sum(sdt > 0.05)}), "
          f"beams {int(np.median(sc[:, 2]))} ({np.median(sc[:, 7] / sc[:, 2]):.2f} finite), "
          f"scan_time {np.median(sc[:, 3]) * 1e3:.1f} ms, angle {np.degrees(sc[0, 5]):.0f}..{np.degrees(sc[0, 6]):.0f} deg | "
          f"scan age med {np.median(age) * 1e3:6.1f} ms p95 {np.percentile(age, 95) * 1e3:6.1f} | "
          f"scan->drive med {np.median(lat) * 1e3:5.1f} ms p95 {np.percentile(lat, 95) * 1e3:5.1f} p99 {np.percentile(lat, 99) * 1e3:5.1f} | "
          f"drive {1 / np.median(ddt):4.1f} Hz, gaps>50ms {np.sum(ddt > 0.05)}/{len(ddt)}")
