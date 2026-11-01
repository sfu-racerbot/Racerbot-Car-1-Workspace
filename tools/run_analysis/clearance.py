#!/usr/bin/env python3
"""What each run was launched with, and how close it really came to things, from raw LiDAR.

    python3 tools/run_analysis/clearance.py data/rb2/racerbot_runs/20261030-183134

Clearance is from the car's body box to each beam, with no map involved. A padded
box against a map's thick walls reads zero with air still left, which once wrongly
implied contact. The box's overall length is unmeasured, so a small reading here is
"close", not "touched": look at the peak acceleration and at the car for that.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np

LASER_X = 0.26       # m, measured
HALF_WIDTH = 0.15    # m, measured: 0.30 m outer tire to outer tire
FRONT = 0.46         # m ahead of base_link: the front axle plus 0.10 m, unmeasured
REAR = -0.12         # m, unmeasured
MIN_RANGE = 0.08     # m, nearer returns are the car itself
FIELD = math.radians(120.0)   # the rear wedge sees the car's own parts


def body_clearance(ranges, angles, laser_x: float = LASER_X, half_width: float = HALF_WIDTH,
                   front: float = FRONT, rear: float = REAR, field: float = FIELD) -> float:
    """Metres from the body box to the nearest beam within +/-``field``; inf if none."""
    r, a = np.asarray(ranges, dtype=float), np.asarray(angles, dtype=float)
    keep = np.isfinite(r) & (r > MIN_RANGE) & (np.abs(a) < field)
    if not keep.any():
        return math.inf
    x = r[keep] * np.cos(a[keep]) + laser_x
    y = r[keep] * np.sin(a[keep])
    dx = np.maximum(np.maximum(rear - x, x - front), 0.0)
    dy = np.maximum(np.abs(y) - half_width, 0.0)
    return float(np.hypot(dx, dy).min())


def launch_command(run) -> str:
    """The control layer's launch line from the run's metadata, or empty."""
    path = Path(run).expanduser() / "metadata" / "processes.txt"
    if not path.exists():
        return ""
    for line in path.read_text().splitlines():
        found = re.search(r"ros2 (?:launch|run) (.*)", line)
        if found and not re.search("dashboard|foxglove|bringup_launch|record_run|system_log|safety_node", line):
            return found.group(1)
    return ""


def main(argv=None) -> int:
    from run_analysis.bagread import messages
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="run directories")
    ap.add_argument("--min-speed", type=float, default=0.5, help="only count samples above this, m/s")
    args = ap.parse_args(argv)
    for run in args.runs:
        lb, v, angles = 0, 0.0, None
        clear, accel = [], []
        for topic, _stamp, msg in messages(run, ["/joy", "/odom", "/scan", "/sensors/imu/raw"]):
            if topic == "/odom":
                v = msg.twist.twist.linear.x
            elif topic == "/joy":
                lb = int(len(msg.buttons) > 4 and msg.buttons[4])
            elif topic == "/sensors/imu/raw":
                if lb:
                    accel.append(math.hypot(msg.linear_acceleration.x, msg.linear_acceleration.y))
            elif lb and v > args.min_speed:
                if angles is None:
                    angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
                clear.append(body_clearance(msg.ranges, angles))
        if not clear:
            print(f"{Path(run).name}: no driving with LB held")
            continue
        c, a = np.array(clear), np.array(accel) if accel else np.zeros(1)
        print(f"{Path(run).name}: {launch_command(run)[:70]}\n    clearance min {c.min():.3f} p1 "
              f"{np.percentile(c, 1):.3f} p5 {np.percentile(c, 5):.3f} m; peak acceleration {a.max():.2f} g")
    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    raise SystemExit(main())
