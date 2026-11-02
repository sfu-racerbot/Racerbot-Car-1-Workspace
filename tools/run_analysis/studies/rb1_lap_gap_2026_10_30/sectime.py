"""Composite lap from per-sector traversal times, per group of runs, on the true (map) track.

Each traversal of a sector (entry to exit, both inside one LB hold, map fit >= 0.85 throughout,
entered at least SKIP seconds after the hold began) gives a time; the group's median per sector
summed is its composite lap. Also: median true speed, commanded /drive speed and odom speed
in each sector, and the deficit of group A against group B.

    python3 sectime.py A:run,run B:run,run [--sectors N] [--skip S]
"""
import argparse
import sys
from pathlib import Path

import numpy as np

from _paths import CACHE, LINE, LOC  # noqa: E402,F401  (also puts tools/ on sys.path)
from run_analysis.laps import Course, holds  # noqa: E402
from run_analysis.loc import load_line  # noqa: E402


ap = argparse.ArgumentParser()
ap.add_argument("groups", nargs="+")
ap.add_argument("--sectors", type=int, default=10)
ap.add_argument("--skip", type=float, default=4.0)
args = ap.parse_args()
course = Course(load_line(LINE))
L = course.length
edges = np.linspace(0, L, args.sectors + 1)

result = {}
for g in args.groups:
    name, runs = g.split(":")
    times = [[] for _ in range(args.sectors)]
    vt, vc, vo = ([[] for _ in range(args.sectors)] for _ in range(3))
    for run in runs.split(","):
        a = np.load(LOC / f"{run}.npz")["a"]
        st = np.load(CACHE / f"steady_{run}.npz")
        t, x, y, h10, lb = a[:, 0], a[:, 1], a[:, 2], a[:, 4], a[:, 10] > 0
        # merge LB releases shorter than 0.5 s: the car keeps rolling through them
        idx = np.flatnonzero(lb)
        for u, w in zip(idx[:-1], idx[1:]):
            if w - u > 1 and t[w] - t[u] < 0.5:
                lb[u:w] = True
        s, _ = course.project(x, y)
        prog = course.unwrap(s)
        for i, j in holds(lb):
            if j - i < 20:
                continue
            tt, pp, hh = t[i:j], prog[i:j], h10[i:j]
            sign = np.sign(pp[-1] - pp[0]) or 1.0
            p = s[i] + (pp - pp[0]) * sign          # absolute arc position, increasing
            if sign < 0:
                raise SystemExit(f"{run}: course driven backwards, flip the line")
            k0, k1 = int(np.floor(p[0] / L * args.sectors)), int(np.floor(p[-1] / L * args.sectors))
            for k in range(k0 + 1, k1):             # whole sectors only
                a0, a1 = k * L / args.sectors, (k + 1) * L / args.sectors
                if not (p[0] <= a0 and p[-1] >= a1):
                    continue
                t0, t1 = np.interp(a0, np.maximum.accumulate(p), tt), np.interp(a1, np.maximum.accumulate(p), tt)
                if t0 - tt[0] < args.skip:
                    continue
                m = (tt >= t0) & (tt <= t1)
                if m.sum() < 3 or hh[m].min() < 0.6 or np.median(hh[m]) < 0.8:
                    continue
                xi, yi = x[i:j][m], y[i:j][m]
                if np.max(np.hypot(np.diff(xi), np.diff(yi))) > 0.6:
                    continue
                sec = k % args.sectors
                times[sec].append(t1 - t0)
                mm = (st["t"] >= t0) & (st["t"] <= t1)
                vt[sec].append(np.nanmedian(st["v"][mm]))
                vc[sec].append(np.nanmedian(st["drv"][mm]))
                vo[sec].append(np.nanmedian(st["odom"][mm]))
    med = np.array([np.median(x) if x else np.nan for x in times])
    n = np.array([len(x) for x in times])
    result[name] = (med, n, [np.nanmedian(x) if x else np.nan for x in vt],
                    [np.nanmedian(x) if x else np.nan for x in vc], [np.nanmedian(x) if x else np.nan for x in vo])

names = list(result)
print(f"course {L:.2f} m, {args.sectors} sectors of {L / args.sectors:.2f} m; flying passes only (skip {args.skip} s)")
hdr = "sector  from-m " + "".join(f"| {nm:>8s} t(s) n  true cmd odom " for nm in names)
print(hdr)
for k in range(args.sectors):
    row = f"  {k:2d}   {edges[k]:5.1f} "
    for nm in names:
        med, n, vt, vc, vo = result[nm]
        row += f"| {med[k]:13.2f} {n[k]:2d}  {vt[k]:4.2f} {vc[k]:4.2f} {vo[k]:4.2f} "
    if len(names) >= 2:
        row += f"| d {result[names[0]][0][k] - result[names[1]][0][k]:+.2f} s"
    print(row)
for nm in names:
    print(f"composite lap {nm}: {np.nansum(result[nm][0]):.2f} s")
if len(names) >= 2:
    print(f"deficit {names[0]} - {names[1]}: {np.nansum(result[names[0]][0]) - np.nansum(result[names[1]][0]):+.2f} s")
