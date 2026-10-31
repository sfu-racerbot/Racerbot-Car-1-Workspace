#!/usr/bin/env python3
"""Is the car on this map's start pose? Compares one /scan with map.pgm. Read-only: subscribes
to /scan, publishes nothing, cannot move the car. GO within 0.3 m and 12 deg, else says how far off.

    python3 check_start.py
"""
import math, sys
from pathlib import Path
import numpy as np
import yaml
from scipy.ndimage import distance_transform_edt as edt, gaussian_filter, map_coordinates
HERE = Path(__file__).resolve().parent
LASER_X = 0.26  # base_link -> laser, bringup_launch.py
MAX_OFF_M, MAX_OFF_DEG = 0.3, 12.0
MIN_INLIERS = 0.80

def load():
    y = yaml.safe_load((HERE / 'map.yaml').read_text())
    res = float(y['resolution']); ox, oy = y['origin'][:2]
    data = (HERE / 'map.pgm').read_bytes(); pos = 0; tokens = []
    while len(tokens) < 4:  # P5, width, height, maxval; comments may sit between them
        end = data.index(b'\n', pos); line = data[pos:end]; pos = end + 1
        if not line.startswith(b'#'): tokens += line.split()
    w, h = int(tokens[1]), int(tokens[2])
    img = np.frombuffer(data, np.uint8, w * h, pos).reshape(h, w)[::-1]
    d = edt(img != 0) * res
    s = {k.strip(): float(v) for k, v in (l.split(':') for l in (HERE / 'start_pose.yaml').read_text().splitlines() if ':' in l and not l.startswith('#'))}
    return d, gaussian_filter(np.minimum(d, 0.7), 0.7), res, ox, oy, (s['x'], s['y'], s['yaw'])

def inliers(raw, res, ox, oy, bx, by, p):
    c, s = math.cos(p[2]), math.sin(p[2])
    col = ((p[0] + c * bx - s * by - ox) / res).astype(int); row = ((p[1] + s * bx + c * by - oy) / res).astype(int)
    ok = (col >= 0) & (col < raw.shape[1]) & (row >= 0) & (row < raw.shape[0])
    v = np.full(col.shape, 9.0); v[ok] = raw[row[ok], col[ok]]
    return float((v < 0.10).mean())

def refine(F, res, ox, oy, bx, by, p):
    x, y, th = p; gy, gx = np.gradient(F, res)
    S = lambda A, X, Y: map_coordinates(A, [(Y - oy) / res - .5, (X - ox) / res - .5], order=1, mode='nearest')
    for it in range(20):
        c, s = math.cos(th), math.sin(th); X = x + c * bx - s * by; Y = y + s * bx + c * by
        d, a, b = S(F, X, Y), S(gx, X, Y), S(gy, X, Y); w = (d < max(.4 * .8 ** it, .12)).astype(float)
        J = np.stack([a, b, a * (-s * bx - c * by) + b * (c * bx - s * by)], 1)
        dx = -np.linalg.solve((J * w[:, None]).T @ J + np.diag([1e-3, 1e-3, 1e-2]), (J * w[:, None]).T @ d)
        x += dx[0]; y += dx[1]; th += dx[2]
    return x, y, th

def verdict(ranges, amin, ainc):
    raw, F, res, ox, oy, start = load()
    r = np.asarray(ranges); a = amin + ainc * np.arange(len(r)); ok = np.isfinite(r) & (r > .1) & (r < 12)
    bx = LASER_X + r[ok] * np.cos(a[ok]); by = r[ok] * np.sin(a[ok])
    best = max((refine(F, res, ox, oy, bx, by, (start[0] + dx, start[1] + dy, start[2] + dt))
                for dx in (-.4, 0, .4) for dy in (-.4, 0, .4) for dt in (-.3, 0, .3)),
               key=lambda p: inliers(raw, res, ox, oy, bx, by, p))
    h = inliers(raw, res, ox, oy, bx, by, best)
    off = math.hypot(best[0] - start[0], best[1] - start[1])
    deg = abs(math.degrees((best[2] - start[2] + math.pi) % (2 * math.pi) - math.pi))
    go = h >= MIN_INLIERS and off <= MAX_OFF_M and deg <= MAX_OFF_DEG
    return go, h, off, deg, best, start

def report(res):
    go, h, off, deg, best, start = res
    print(f"{'GO' if go else 'NO-GO'}: scan fits the map {h*100:.0f}% (needs {MIN_INLIERS*100:.0f}%), car is {off:.2f} m and {deg:.0f} deg from the start pose "
          f"(limits {MAX_OFF_M} m, {MAX_OFF_DEG:.0f} deg).")
    if not go and h >= MIN_INLIERS:
        c, s = math.cos(start[2]), math.sin(start[2]); dx, dy = start[0] - best[0], start[1] - best[1]
        print(f"  Move the car about {c*dx+s*dy:+.2f} m forward and {-s*dx+c*dy:+.2f} m to its left (start-pose axes), then turn it to face {math.degrees(start[2]):.0f} deg on the map.")
    return 0 if go else 1

if __name__ == '__main__':
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    rclpy.init(); n = rclpy.create_node('check_start'); got = []
    n.create_subscription(LaserScan, '/scan', got.append, qos_profile_sensor_data)
    for _ in range(100):
        rclpy.spin_once(n, timeout_sec=.1)
        if got: break
    if not got: sys.exit('NO-GO: no /scan in 10 s. Is bringup_launch.py running, and this terminal sourced?')
    sys.exit(report(verdict(got[0].ranges, got[0].angle_min, got[0].angle_increment)))
