"""True speed against wheel eRPM, the motor command and /odom, per run.

True speed comes from loc.py's map track (rear axle), independent of the wheels.
Steady window: LB held, map fit h10 >= 0.85, |yaw rate| < 0.35 rad/s,
commanded eRPM within +-3% over the preceding 0.6 s, true speed > 1 m/s.
"""
import sys
from pathlib import Path

import numpy as np

from _paths import CACHE, LINE, LOC  # noqa: E402,F401
HALF = 0.25   # s, half-window for the path-length speed


def true_speed(a):
    t, x, y, yaw, h10 = a[:, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 4]
    seg = np.hypot(np.diff(x), np.diff(y))
    s = np.concatenate([[0], np.cumsum(seg)])
    v = np.full(len(t), np.nan)
    w = np.full(len(t), np.nan)
    uy = np.unwrap(yaw)
    for i in range(len(t)):
        j0 = np.searchsorted(t, t[i] - HALF)
        j1 = np.searchsorted(t, t[i] + HALF, side="right") - 1
        if j1 - j0 < 6 or t[j1] - t[j0] < 1.6 * HALF:
            continue
        if np.min(h10[j0:j1 + 1]) < 0.85:
            continue
        dt = t[j1] - t[j0]
        if np.max(np.diff(t[j0:j1 + 1])) > 0.12:
            continue
        v[i] = (s[j1] - s[j0]) / dt
        w[i] = (uy[j1] - uy[j0]) / dt
    return v, w


def at(src_t, src_v, t):
    return np.interp(t, src_t, src_v, left=np.nan, right=np.nan)


def steady_mask(b, t, lb):
    m = b["mcmd"]
    ok = np.zeros(len(t), bool)
    for i, ti in enumerate(t):
        j0, j1 = np.searchsorted(m[:, 0], ti - 0.6), np.searchsorted(m[:, 0], ti)
        if j1 - j0 < 15:
            continue
        seg = m[j0:j1, 1]
        mu = seg.mean()
        ok[i] = mu > 0 and (seg.max() - seg.min()) < 0.06 * mu
    return ok & (lb > 0)


def main():
    rows = []
    for run in sys.argv[1:]:
        a = np.load(LOC / f"{run}.npz")["a"]
        b = np.load(CACHE / "bag" / f"{run}.npz")
        t = a[:, 0]
        v, w = true_speed(a)
        core = b["core"]
        erpm = at(core[:, 0], core[:, 2], t)
        volt = at(core[:, 0], core[:, 3], t)
        cmd = at(b["mcmd"][:, 0], b["mcmd"][:, 1], t)
        odom = at(b["odom"][:, 0], b["odom"][:, 2], t)
        drv = at(b["drive"][:, 0], b["drive"][:, 2], t)
        lb = a[:, 10]
        st = steady_mask(b, t, lb) & (np.abs(w) < 0.35) & (v > 1.0)
        mv = (lb > 0) & (v > 0.5) & np.isfinite(v)

        def q(x, m):
            x = x[m & np.isfinite(x)]
            return (np.median(x), np.percentile(x, 25), np.percentile(x, 75), len(x)) if len(x) else (np.nan,) * 3 + (0,)
        g = q(erpm / v, st)
        tr = q(erpm / cmd, st)
        od = q(odom / v, st)
        vd = q(v / drv, st)
        vdm = q(v / drv, mv)
        vs = q(v, st)
        vl = q(volt, mv)
        print(f"{run}: n={g[3]:4d} steady  v~{vs[0]:.2f}  true_gain {g[0]:6.0f} [{g[1]:.0f},{g[2]:.0f}]"
              f"  erpm/cmd {tr[0]:.3f} [{tr[1]:.3f},{tr[2]:.3f}]  odom/true {od[0]:.3f}"
              f"  true/drive {vd[0]:.3f} (all moving {vdm[0]:.3f})  V {vl[0]:.2f}")
        rows.append((run, g, tr, od, vd))
        np.savez(CACHE / f"steady_{run}.npz", t=t, v=v, w=w, erpm=erpm, cmd=cmd, odom=odom, drv=drv, st=st, mv=mv, volt=volt)


main()
