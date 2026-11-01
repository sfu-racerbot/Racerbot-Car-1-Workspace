#!/usr/bin/env python3
"""Locate a recorded run on a saved map, scan by scan: true position, independent of the wheels.

    python3 tools/run_analysis/loc.py data/racerbot_runs/20261030-185036 \\
        --map data/racerbot_auto/20261030-185036-fixed

Tracking: wheel speed and the VESC gyro predict the next pose, then a Gauss-Newton
match against the map's distance field corrects it. If the match stays poor, a global
search over the free space near the course line recovers the pose (the car may have
been carried). Moving scans are matched at 20 Hz, parked ones at 2 Hz.

Results go to --out (default ~/.cache/racerbot/loc), never into data/ unless asked.
Each <run>.npz holds ``a``, one row per matched scan, with the columns in COLUMNS.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

LASER_X = 0.26       # m, docs/hardware-reference.md: LiDAR ahead of the rear axle
MAX_RANGE = 12.0     # m, longer returns add little and cost time
FIELD_CAP = 0.5      # m, the distance field is flat past this
GOOD_H10 = 0.75      # fraction of beams within 10 cm below which a match is poor
POOR_SCANS = 6       # poor matches in a row before a global search
SEARCH_EVERY = 3.0   # s, at most one global search this often
DEFAULT_OUT = Path("~/.cache/racerbot/loc").expanduser()
COLUMNS = ("t", "x", "y", "yaw", "h10", "h20", "wheel_v", "gyro_z", "steer_cmd", "speed_cmd", "lb")
COL = {name: i for i, name in enumerate(COLUMNS)}


class Grid:
    """An occupancy grid and its smoothed distance field."""

    def __init__(self, occupied: np.ndarray, free: np.ndarray, resolution: float,
                 origin_x: float, origin_y: float):
        from scipy.ndimage import distance_transform_edt, gaussian_filter
        self.occ, self.free = occupied, free
        self.res, self.ox, self.oy = float(resolution), float(origin_x), float(origin_y)
        self.distance = distance_transform_edt(~occupied) * self.res
        self.d = gaussian_filter(np.minimum(self.distance, FIELD_CAP + 0.2), 0.7)
        self.gy, self.gx = np.gradient(self.d, self.res)

    @classmethod
    def from_map(cls, map_dir) -> "Grid":
        """Load <map_dir>/map.yaml and its image, row 0 at the lowest y."""
        import yaml
        from PIL import Image
        map_dir = Path(map_dir).expanduser()
        meta = yaml.safe_load((map_dir / "map.yaml").read_text())
        image = np.array(Image.open(map_dir / meta["image"]))[::-1]
        return cls(image < 100, image > 250, meta["resolution"], *meta["origin"][:2])

    def sample(self, field: np.ndarray, x, y):
        from scipy.ndimage import map_coordinates
        return map_coordinates(field, [(np.asarray(y) - self.oy) / self.res - 0.5,
                                       (np.asarray(x) - self.ox) / self.res - 0.5],
                               order=1, mode="nearest")


def match(grid: Grid, bx, by, pose, iters: int = 12, trim: float = 0.30):
    """Refine ``pose`` (x, y, yaw) so points (bx, by) in base_link lie on walls.
    Returns (pose, fraction within 10 cm, fraction within 20 cm)."""
    x, y, th = pose
    for it in range(iters):
        c, s = math.cos(th), math.sin(th)
        wx, wy = x + c * bx - s * by, y + s * bx + c * by
        d, gx, gy = grid.sample(grid.d, wx, wy), grid.sample(grid.gx, wx, wy), grid.sample(grid.gy, wx, wy)
        weight = (d < max(trim * 0.7 ** it, 0.12)).astype(float)
        if weight.sum() < 30:
            break
        jac = np.stack([gx, gy, gx * (-s * bx - c * by) + gy * (c * bx - s * by)], 1)
        hess = (jac * weight[:, None]).T @ jac + np.diag([1e-3, 1e-3, 1e-2])
        step = -np.linalg.solve(hess, (jac * weight[:, None]).T @ d)
        x, y, th = x + step[0], y + step[1], th + step[2]
        if math.hypot(step[0], step[1]) < 1e-3 and abs(step[2]) < 3e-4:
            break
    c, s = math.cos(th), math.sin(th)
    d = grid.sample(grid.d, x + c * bx - s * by, y + s * bx + c * by)
    return (x, y, th), float((d < 0.10).mean()), float((d < 0.20).mean())


def search_points(grid: Grid, line, spacing: float = 0.25, reach: float = 1.5) -> np.ndarray:
    """Free cells on a ``spacing`` grid within ``reach`` of the course ``line``."""
    step = max(1, int(round(spacing / grid.res)))
    rows, cols = np.nonzero(grid.free[::step, ::step])
    px = grid.ox + (cols * step + 0.5) * grid.res
    py = grid.oy + (rows * step + 0.5) * grid.res
    line = np.asarray(line)[::4]
    near = np.min((px[:, None] - line[:, 0]) ** 2 + (py[:, None] - line[:, 1]) ** 2, axis=1) < reach ** 2
    return np.c_[px[near], py[near]]


def global_search(grid: Grid, bx, by, points: np.ndarray, top: int = 40):
    """Best (pose, h10, h20) over ``points`` and 18 headings."""
    best = []
    bx2, by2 = bx[::3], by[::3]
    for th in np.arange(0.0, 2.0 * np.pi, np.radians(20.0)):
        c, s = np.cos(th), np.sin(th)
        wx = points[:, 0][:, None] + c * bx2 - s * by2
        wy = points[:, 1][:, None] + s * bx2 + c * by2
        score = (grid.sample(grid.d, wx.ravel(), wy.ravel()).reshape(wx.shape) < 0.25).mean(axis=1)
        for i in np.argsort(score)[-6:]:
            best.append((float(score[i]), points[i, 0], points[i, 1], float(th)))
    best.sort(key=lambda row: -row[0])
    return best_match(match(grid, bx, by, (x, y, th), iters=20, trim=0.6)
                      for _score, x, y, th in best[:top])


def best_match(results):
    """The (pose, h10, h20) with the most beams within 10 cm."""
    return max(results, key=lambda result: result[1])


def scan_points(ranges, angles):
    """Every third valid beam as (x, y) in base_link."""
    r = np.asarray(ranges, dtype=float)
    keep = np.nonzero(np.isfinite(r) & (r > 0.1) & (r < MAX_RANGE))[0][::3]
    return LASER_X + r[keep] * np.cos(angles[keep]), r[keep] * np.sin(angles[keep])


def advance(pose, speed: float, yaw_rate: float, dt: float):
    """Dead-reckon ``pose`` by ``speed`` and ``yaw_rate`` over ``dt``."""
    mid = pose[2] + yaw_rate * dt / 2.0
    return (pose[0] + speed * math.cos(mid) * dt, pose[1] + speed * math.sin(mid) * dt,
            pose[2] + yaw_rate * dt)


def locate(run, grid: Grid, line) -> tuple:
    """Rows of COLUMNS for one run, and how many global searches it took."""
    from run_analysis.bagread import messages
    points = search_points(grid, line)
    v = gz = steer = cmd = 0.0
    lb, poor, count, searches = 0, 0, 0, 0
    last, pose, angles, last_search = None, None, None, -math.inf
    rows = []
    for topic, stamp, msg in messages(run, ["/odom", "/joy", "/drive", "/sensors/imu/raw", "/scan"]):
        if topic == "/odom":
            v = msg.twist.twist.linear.x
        elif topic == "/joy":
            lb = int(len(msg.buttons) > 4 and msg.buttons[4])
        elif topic == "/drive":
            steer, cmd = msg.drive.steering_angle, msg.drive.speed
        elif topic == "/sensors/imu/raw":
            gz = math.radians(msg.angular_velocity.z)   # the VESC reports deg/s
        else:
            if last is not None and pose is not None:
                pose = advance(pose, v, gz, min(stamp - last, 0.2))
            last = stamp
            count += 1
            if count % (2 if (lb or abs(v) > 0.05) else 20):
                continue
            if angles is None:
                angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
            bx, by = scan_points(msg.ranges, angles)
            if len(bx) < 40:
                continue
            if pose is None:
                pose, h10, h20 = global_search(grid, bx, by, points)
                searches += 1
            else:
                pose, h10, h20 = match(grid, bx, by, pose)
            poor = poor + 1 if h10 < GOOD_H10 else 0
            if poor >= POOR_SCANS and stamp - last_search > SEARCH_EVERY:
                last_search = stamp
                searches += 1
                found = global_search(grid, bx, by, points)
                if found[1] > h10 + 0.1:      # keep the pose unless clearly better
                    pose, h10, h20 = found
                    poor = 0
            rows.append((stamp, *pose, h10, h20, v, gz, steer, cmd, lb))
    return np.array(rows), searches


def load_line(path) -> np.ndarray:
    """(N, 2) course line from a CSV with a header row and x, y first."""
    return np.loadtxt(Path(path).expanduser(), delimiter=",", skiprows=1, usecols=(0, 1))


def result_path(run, out=DEFAULT_OUT) -> Path:
    return Path(out).expanduser() / f"{Path(run).expanduser().resolve().name}.npz"


def load_track(run, out=DEFAULT_OUT) -> np.ndarray:
    """The rows loc.py saved for ``run``. Raises FileNotFoundError with the fix."""
    path = result_path(run, out)
    if not path.exists():
        raise FileNotFoundError(f"{path}: run tools/run_analysis/loc.py on {run} first")
    return np.load(path)["a"]


def _one(job):
    run, map_dir, line_path, out, refresh = job
    path = result_path(run, out)
    if path.exists() and not refresh:
        return Path(run).name, "already located"
    rows, searches = locate(run, Grid.from_map(map_dir), load_line(line_path))
    if not len(rows):
        return Path(run).name, "no usable scans"
    np.savez_compressed(path, a=rows)
    return Path(run).name, (f"{len(rows)} scans, {rows[:, COL['h10']].mean():.2f} of beams within "
                            f"10 cm, {searches} global searches")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="run directories")
    ap.add_argument("--map", required=True, help="directory holding map.yaml and the course line")
    ap.add_argument("--line", help="course line CSV (default: <map>/raceline_raw.csv)")
    ap.add_argument("--out", default=str(DEFAULT_OUT), help="where results go (default %(default)s)")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--refresh", action="store_true", help="redo runs already located")
    args = ap.parse_args(argv)
    Path(args.out).expanduser().mkdir(parents=True, exist_ok=True)
    line = args.line or os.path.join(args.map, "raceline_raw.csv")
    jobs = [(run, args.map, line, args.out, args.refresh) for run in args.runs]
    with Pool(args.workers) as pool:
        for name, text in pool.imap_unordered(_one, jobs):
            print(name, text, flush=True)
    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    raise SystemExit(main())
