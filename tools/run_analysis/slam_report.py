"""Post-run SLAM health metrics; numerical functions require only NumPy."""
from __future__ import annotations

import math

import numpy as np


def heading_drift(a, b, window_s=1.0):
    """Compare unwrapped headings at common times, relative to their first difference."""
    a = np.asarray(a, dtype=float).reshape(-1, 2)
    b = np.asarray(b, dtype=float).reshape(-1, 2)
    missing = dict(final_deg=math.nan, max_abs_deg=math.nan, rms_rate_deg_s=math.nan)
    if len(a) < 2 or len(b) < 2 or window_s <= 0:
        return missing
    a = a[np.argsort(a[:, 0])]
    b = b[np.argsort(b[:, 0])]
    mask = (a[:, 0] >= b[0, 0]) & (a[:, 0] <= b[-1, 0])
    t = a[mask, 0]
    if len(t) < 2:
        return missing
    drift = np.unwrap(a[:, 1])[mask] - np.interp(t, b[:, 0], np.unwrap(b[:, 1]))
    drift -= drift[0]
    count = int((t[-1] - t[0]) / window_s)
    ends = t[0] + np.arange(count + 1) * window_s
    changes = np.diff(np.interp(ends, t, drift)) / window_s
    return dict(final_deg=float(np.degrees(drift[-1])),
                max_abs_deg=float(np.degrees(np.max(np.abs(drift)))),
                rms_rate_deg_s=float(np.degrees(np.sqrt(np.mean(changes ** 2))))
                if len(changes) else math.nan)


def slam_pose(slam):
    """Return map-frame base poses, composing TF when /pose is missing."""
    if not slam:
        return np.empty((0, 4))
    pose = np.asarray(slam.get('pose', []), dtype=float).reshape(-1, 4)
    if len(pose):
        return pose
    mo = np.asarray(slam.get('map_odom', []), dtype=float).reshape(-1, 4)
    base = np.asarray(slam.get('odom_base', []), dtype=float).reshape(-1, 4)
    if not len(mo) or not len(base):
        return np.empty((0, 4))
    mo = mo[np.argsort(mo[:, 0])]
    base = base[(base[:, 0] >= mo[0, 0]) & (base[:, 0] <= mo[-1, 0])]
    t = base[:, 0]
    x = np.interp(t, mo[:, 0], mo[:, 1])
    y = np.interp(t, mo[:, 0], mo[:, 2])
    yaw = np.interp(t, mo[:, 0], np.unwrap(mo[:, 3]))
    c, s = np.cos(yaw), np.sin(yaw)
    return np.column_stack((t, x+c*base[:, 1]-s*base[:, 2],
                            y+s*base[:, 1]+c*base[:, 2], yaw+base[:, 3]))


def slam_yaw(slam):
    return slam_pose(slam)[:, [0, 3]]


def map_consistency(scan_t, ranges, angle_min, angle_increment, range_max,
                    poses, grid_map, laser_x=0.26, cells=1):
    """Fraction of finite LiDAR endpoints close to an occupied map cell."""
    poses = np.asarray(poses, dtype=float).reshape(-1, 4)
    if grid_map is None or len(poses) < 2:
        return math.nan
    resolution, ox, oy, grid = grid_map
    ranges = np.asarray(ranges, dtype=float)
    hits = total = 0
    for t, scan in zip(scan_t, ranges):
        if t < poses[0, 0] or t > poses[-1, 0]:
            continue
        valid = np.isfinite(scan) & (scan < range_max)
        angle = angle_min + np.arange(len(scan))[valid] * angle_increment
        r = scan[valid]
        x = np.interp(t, poses[:, 0], poses[:, 1])
        y = np.interp(t, poses[:, 0], poses[:, 2])
        yaw = np.interp(t, poses[:, 0], np.unwrap(poses[:, 3]))
        ix = np.floor((x + laser_x*np.cos(yaw) + r*np.cos(yaw+angle) - ox) / resolution).astype(int)
        iy = np.floor((y + laser_x*np.sin(yaw) + r*np.sin(yaw+angle) - oy) / resolution).astype(int)
        matched = np.zeros(len(ix), dtype=bool)
        for dy in range(-cells, cells + 1):
            for dx in range(-cells, cells + 1):
                xx, yy = ix + dx, iy + dy
                inside = (xx >= 0) & (xx < grid.shape[1]) & (yy >= 0) & (yy < grid.shape[0])
                matched[inside] |= grid[yy[inside], xx[inside]] > 50
        hits += int(matched.sum())
        total += len(ix)
    return hits / total if total else math.nan


def lap_windows(laps, run_start, run_end):
    start = run_start
    windows = []
    for lap in sorted(laps, key=lambda item: item['t']):
        end = lap['t']
        if start <= end <= run_end:
            windows.append((lap['lap'], start, end))
            start = end
    return windows


def scan_drop_timeline(drop_times, proc_records, node='/slam_toolbox', bin_s=10.0):
    drops = list(drop_times)
    times = drops + [r['t'] for r in proc_records]
    if not times:
        return []
    start = min(times)
    result = []
    for index in range(int((max(times)-start) // bin_s) + 1):
        t0 = start + index*bin_s
        cpu = [r['cpu_pct'] for r in proc_records if r['node'] == node
               and r['cpu_pct'] is not None and t0 <= r['t'] < t0 + bin_s]
        result.append(dict(t0=t0, drops=sum(t0 <= t < t0+bin_s for t in drops),
                           cpu_mean=float(np.mean(cpu)) if cpu else math.nan,
                           cpu_max=max(cpu) if cpu else math.nan))
    return result


def path_length(rows):
    rows = np.asarray(rows, dtype=float)
    return float(np.hypot(np.diff(rows[:, 1]), np.diff(rows[:, 2])).sum()) if len(rows) else 0.0
