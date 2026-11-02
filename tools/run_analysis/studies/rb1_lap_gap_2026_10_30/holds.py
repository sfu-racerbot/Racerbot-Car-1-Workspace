"""Absolute LB-hold windows (local wall clock) for every run, to see which overlap across cars."""
import datetime as dt
import sys
from pathlib import Path

import numpy as np

from _paths import CACHE, LINE, LOC  # noqa: E402,F401
tz = dt.timezone(dt.timedelta(hours=-7))
holds = []
for arg in sys.argv[1:]:
    car, run = arg.split(":")
    j = np.load(CACHE / "bag" / f"{run}.npz")["joy"]
    lb = j[:, 1] > 0
    edges = np.flatnonzero(np.diff(np.concatenate([[0], lb.astype(int), [0]])))
    for s, e in zip(edges[::2], edges[1::2]):
        t0, t1 = j[s, 0], j[e - 1, 0]
        if t1 - t0 > 3:
            holds.append((t0, t1, car, run))
holds.sort()
for t0, t1, car, run in holds:
    other = [f"{c}:{r}" for a, b, c, r in holds if c != car and a < t1 and b > t0]
    f = lambda t: dt.datetime.fromtimestamp(t, tz).strftime("%H:%M:%S")
    print(f"{car} {run} {f(t0)}-{f(t1)} ({t1 - t0:5.1f}s)  overlaps: {', '.join(other) or 'none'}")
