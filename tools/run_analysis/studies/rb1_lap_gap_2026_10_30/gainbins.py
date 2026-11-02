"""True gain (wheel eRPM / map speed) binned by true speed and by acceleration, per car.

Straight samples only (|yaw rate| < 0.35 rad/s), LB held, map fit >= 0.85 (from speedgain.py).
Also: distance by wheel eRPM against map distance over each whole LB hold.
"""
import sys
from pathlib import Path

import numpy as np

from _paths import CACHE, LINE, LOC  # noqa: E402,F401
cars = {"car1": sys.argv[1].split(","), "car2": sys.argv[2].split(",")}
bins = [0.5, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0]
for car, runs in cars.items():
    V, G, A, W = [], [], [], []
    for r in runs:
        d = np.load(CACHE / f"steady_{r}.npz")
        t, v, w, erpm, mv = d["t"], d["v"], d["w"], d["erpm"], d["mv"]
        acc = np.gradient(np.nan_to_num(v), t)
        m = mv & np.isfinite(v) & np.isfinite(erpm) & (np.abs(w) < 0.35) & (v > 0.5)
        V.append(v[m]); G.append(erpm[m] / v[m]); A.append(acc[m]); W.append(w[m])
    V, G, A = map(np.concatenate, (V, G, A))
    print(f"== {car}: {len(V)} straight samples; true gain median by true speed (n), and split by accel")
    for lo, hi in zip(bins[:-1], bins[1:]):
        m = (V >= lo) & (V < hi)
        if m.sum() < 5:
            continue
        acc_m = m & (A > 0.5)
        coast = m & (np.abs(A) <= 0.5)
        dec = m & (A < -0.5)
        f = lambda mm: f"{np.median(G[mm]):5.0f} ({mm.sum():3d})" if mm.sum() >= 3 else "   -       "
        print(f"   {lo:.1f}-{hi:.1f} m/s: all {f(m)}  accel>0.5 {f(acc_m)}  |a|<=0.5 {f(coast)}  decel {f(dec)}")
