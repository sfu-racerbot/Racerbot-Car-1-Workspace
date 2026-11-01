#!/usr/bin/env python3
"""What happened in each run, at a glance: what drove, for how long, how fast, how hard.

    python3 tools/run_analysis/run_summary.py --date 20261030
    python3 tools/run_analysis/run_summary.py data/racerbot_runs/20261030-185036 --json /tmp/s.json

For a day's runs, first lists each one from its own files (which algorithm auto_record
saw, how it was launched, a few gap_follow settings). Then, per run, only while LB was
held: distance, speeds from /odom, steering use, lateral acceleration from /odom speed
times the gyro, the closest return ahead, the hardest jolt on the IMU, corridor width
from the side beams, whether /drive reached /ackermann_cmd, and how evenly /joy arrived.

/odom over-reads true speed by about 3% (car_measure.py); use loc.py and laps.py for
true speed and lap times. Reads the bag; writes only where --json says.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def run_roots() -> list:
    """data/racerbot_runs here, then in the main checkout if this is a worktree: a worktree
    holds only the runs that were committed when it was made."""
    import subprocess
    roots = [ROOT / "data" / "racerbot_runs"]
    try:
        listing = subprocess.run(["git", "worktree", "list", "--porcelain"], cwd=ROOT, capture_output=True,
                                 text=True, check=True).stdout
        roots.append(Path(listing.splitlines()[0].split(" ", 1)[1]) / "data" / "racerbot_runs")
    except (OSError, subprocess.CalledProcessError, IndexError):
        pass
    return [r for i, r in enumerate(roots) if r.is_dir() and r not in roots[:i]]


def find_runs(prefix: str, roots) -> list:
    """Run folders starting with ``prefix``, the first root's copy winning, in name order."""
    found = {}
    for root in roots:
        for p in Path(root).glob(f"{prefix}*"):
            if p.is_dir():
                found.setdefault(p.name, p)
    return [found[k] for k in sorted(found)]
LOCK = 0.34             # rad: |steering| at or past this counts as full lock (0.349 limit)
MIN_HOLD = 3.0          # s: shorter LB holds are a tap, not a run
SIDE = 0.25             # rad either side of +-90 degrees for the corridor width
FRONT = math.radians(45.0)
G = 9.81
QUIET = re.compile(r"web_dashboard|dashboard_launch|bringup_launch|record_run|system_log|snapshot")
SETTINGS = ("max_speed", "max_lateral_accel", "max_steering_angle", "enable_opponent_awareness",
            "enable_launch_boost", "aim_depth_bias")


def hold_windows(t, lb, min_s: float = MIN_HOLD) -> list:
    """(start, end) times of each LB hold at least ``min_s`` long; an open hold ends at the last sample."""
    t, lb = np.asarray(t, float), np.asarray(lb, bool)
    out, start = [], None
    for ti, held in zip(t, lb):
        if held and start is None:
            start = ti
        elif not held and start is not None:
            out.append((start, ti))
            start = None
    if start is not None and len(t):
        out.append((start, t[-1]))
    return [(a, b) for a, b in out if b - a >= min_s]


def in_windows(t, windows) -> np.ndarray:
    t = np.asarray(t, float)
    mask = np.zeros(t.shape, bool)
    for a, b in windows:
        mask |= (t >= a) & (t <= b)
    return mask


def gaps(t) -> dict:
    """How evenly a topic arrived: mean, p95 and worst gap in ms, and the share over 50 ms."""
    d = np.diff(np.asarray(t, float))
    if not d.size:
        return dict(mean_ms=math.nan, p95_ms=math.nan, max_ms=math.nan, over_50ms=math.nan)
    return dict(mean_ms=float(1e3 * d.mean()), p95_ms=float(1e3 * np.percentile(d, 95)),
                max_ms=float(1e3 * d.max()), over_50ms=float(np.mean(d > 0.05)))


def width_from_scan(ranges, angles) -> float:
    """Corridor width: the nearest wall at the left side plus the nearest at the right."""
    r, a = np.asarray(ranges, float), np.asarray(angles, float)
    ok = np.isfinite(r) & (r > 0.05)
    left, right = ok & (np.abs(a - math.pi / 2) < SIDE), ok & (np.abs(a + math.pi / 2) < SIDE)
    if not (left.any() and right.any()):
        return math.nan
    return float((r[left] * np.abs(np.sin(a[left]))).min() + (r[right] * np.abs(np.sin(a[right]))).min())


def passed_through(drive_t, drive_v, ack_t, ack_v, tolerance: float = 0.05) -> float:
    """Share of /drive commands that /ackermann_cmd repeated within 0.1 s: did the mux pass them?"""
    if not len(drive_t) or not len(ack_t):
        return math.nan
    ack_t, ack_v = np.asarray(ack_t), np.asarray(ack_v)
    hit = 0
    for t, v in zip(drive_t, drive_v):
        k = (ack_t >= t) & (ack_t <= t + 0.1)
        hit += bool(np.any(np.abs(ack_v[k] - v) <= tolerance))
    return hit / len(drive_t)


def hold_stats(series: dict, windows) -> dict:
    """The numbers for one run, over its LB holds. ``series`` holds (N, k) arrays by topic."""
    odom, drive, gyro, imu = series["odom"], series["drive"], series["gyro"], series["accel"]
    o = odom[in_windows(odom[:, 0], windows)]
    d = drive[in_windows(drive[:, 0], windows)] if len(drive) else drive
    dt = np.clip(np.diff(o[:, 0], prepend=o[0, 0]), 0.0, 0.1)
    distance = float(np.sum(np.maximum(o[:, 1], 0.0) * dt))
    held = float(dt.sum())
    out = dict(holds=len(windows), held_s=held, odom_distance_m=distance,
               odom_mean=distance / held if held else math.nan,
               odom_p50=float(np.median(o[:, 1])), odom_p90=float(np.percentile(o[:, 1], 90)),
               odom_max=float(o[:, 1].max()), slow_share=float(np.mean(o[:, 1] < 1.0)))
    if len(d):
        out.update(cmd_mean=float(d[:, 1].mean()), steer_p50=float(np.median(np.abs(d[:, 2]))),
                   steer_p90=float(np.percentile(np.abs(d[:, 2]), 90)), lock_share=float(np.mean(np.abs(d[:, 2]) >= LOCK)),
                   drive_zero_share=float(np.mean(d[:, 1] <= 0.01)))
    if len(gyro):
        g = gyro[in_windows(gyro[:, 0], windows)]
        lateral = np.abs(np.interp(g[:, 0], odom[:, 0], odom[:, 1]) * g[:, 1])
        out.update(gyro_p90=float(np.percentile(np.abs(g[:, 1]), 90)),
                   lateral_p50=float(np.percentile(lateral, 50)), lateral_p90=float(np.percentile(lateral, 90)),
                   lateral_p99=float(np.percentile(lateral, 99)))
    if len(imu):
        a = imu[in_windows(imu[:, 0], windows)]
        jolt = np.hypot(a[:, 1], a[:, 2]) / G
        out.update(jolt_p999_g=float(np.percentile(jolt, 99.9)), jolt_max_g=float(jolt.max()))
    for key, name in (("front", "front_min"), ("width", "width")):
        s = series[key]
        if len(s):
            v = s[in_windows(s[:, 0], windows), 1]
            v = v[np.isfinite(v)]
            if v.size:
                out[f"{name}_p5"], out[f"{name}_p50"] = float(np.percentile(v, 5)), float(np.percentile(v, 50))
    ack = series["ack"]
    if len(ack) and len(d):
        out["drive_reached_ackermann"] = passed_through(d[:, 0], d[:, 1], ack[:, 0], ack[:, 1])
    return out


def inventory(run) -> dict:
    """What a run's own files say: duration, algorithms auto_record saw, launches, settings."""
    import yaml
    run = Path(run)
    out = dict(run=run.name)
    try:
        record = yaml.safe_load((run / "auto_record.yaml").read_text()) or {}
        out["duration_s"] = record.get("duration_sec")
        out["algorithms"] = sorted({n for v in (record.get("algorithms") or {}).values() for n in v})
    except (OSError, yaml.YAMLError):
        pass
    try:
        lines = (run / "metadata" / "processes.txt").read_text().splitlines()
        out["launches"] = [m.group(2)[:100] for m in (re.search(r"ros2 (launch|run) (.*)", l) for l in lines)
                           if m and not QUIET.search(m.group(0))]
    except OSError:
        pass
    params = run / "metadata" / "params" / "gap_follow_node.yaml"
    if params.exists():
        raw = yaml.safe_load(params.read_text())
        values = next(iter(raw.values()))["ros__parameters"]
        out["gap_follow"] = {k: values.get(k) for k in SETTINGS}
    return out


def read_run(run) -> dict:
    """Bag arrays, times relative to the first message."""
    from run_analysis.bagread import messages
    rows = {k: [] for k in ("joy", "odom", "drive", "gyro", "accel", "front", "width", "ack")}
    t0, angles = None, None
    for topic, stamp, m in messages(run, ["/joy", "/odom", "/drive", "/ackermann_cmd", "/sensors/imu/raw", "/scan"]):
        t0 = stamp if t0 is None else t0
        t = stamp - t0
        if topic == "/joy":
            rows["joy"].append((t, float(len(m.buttons) > 4 and m.buttons[4])))
        elif topic == "/odom":
            rows["odom"].append((t, m.twist.twist.linear.x))
        elif topic == "/drive":
            rows["drive"].append((t, m.drive.speed, m.drive.steering_angle))
        elif topic == "/ackermann_cmd":
            rows["ack"].append((t, m.drive.speed))
        elif topic == "/sensors/imu/raw":
            rows["gyro"].append((t, math.radians(m.angular_velocity.z)))
            rows["accel"].append((t, m.linear_acceleration.x, m.linear_acceleration.y))
        else:
            r = np.array(m.ranges, float)
            if angles is None or angles.size != r.size:
                angles = m.angle_min + np.arange(r.size) * m.angle_increment
            ahead = np.isfinite(r) & (r > 0.05) & (np.abs(angles) < FRONT)
            rows["front"].append((t, r[ahead].min() if ahead.any() else math.nan))
            rows["width"].append((t, width_from_scan(r, angles)))
    return {k: np.array(v, float).reshape(len(v), -1) if v else np.zeros((0, 3)) for k, v in rows.items()}


def summarise(run) -> dict:
    series = read_run(run)
    out = inventory(run)
    joy = series["joy"]
    if not len(joy) or not len(series["odom"]):
        out["note"] = "no /joy or no /odom: nothing was driven"
        return out
    out["joy"] = gaps(joy[:, 0])
    windows = hold_windows(joy[:, 0], joy[:, 1] > 0)
    if not windows:
        out["note"] = f"no LB hold of {MIN_HOLD:.0f} s or more"
        return out
    out.update(hold_stats(series, windows))
    return out


def _line(s: dict) -> str:
    keys = [("held_s", "held {:.0f} s"), ("odom_distance_m", "{:.0f} m"), ("odom_mean", "mean {:.2f}"),
            ("odom_p90", "p90 {:.2f}"), ("odom_max", "max {:.2f} m/s"), ("steer_p90", "|steer| p90 {:.3f}"),
            ("lock_share", "at lock {:.0%}"), ("lateral_p99", "lateral p99 {:.2f}"), ("front_min_p5", "ahead p5 {:.2f} m"),
            ("width_p50", "width {:.2f} m"), ("jolt_max_g", "jolt {:.2f} g"), ("drive_reached_ackermann", "/drive passed {:.0%}")]
    return ", ".join(fmt.format(s[k]) for k, fmt in keys if k in s and s[k] is not None and s[k] == s[k])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="*", help="run directories")
    ap.add_argument("--date", help="every run under --root starting with this, e.g. 20261030")
    ap.add_argument("--root", action="append",
                    help="where runs live (default: data/racerbot_runs here and in the main checkout)")
    ap.add_argument("--list", action="store_true", help="only the inventory; do not read the bags")
    ap.add_argument("--json", help="also write everything here")
    args = ap.parse_args(argv)
    runs = [Path(r) for r in args.runs]
    if args.date:
        runs += find_runs(args.date, args.root or run_roots())
    if not runs:
        ap.error("give run directories or --date")
    results = []
    for run in runs:
        s = inventory(run) if args.list else summarise(run)
        results.append(s)
        head = f"{s['run']}: {s.get('duration_s', '?')} s, {', '.join(s.get('algorithms', [])) or 'no algorithm seen'}"
        print(head)
        if s.get("launches"):
            print("   launched: " + "; ".join(s["launches"])[:200])
        if s.get("gap_follow"):
            print("   gap_follow: " + ", ".join(f"{k}={v}" for k, v in s["gap_follow"].items()))
        if s.get("note"):
            print("   " + s["note"])
        elif not args.list:
            print("   " + _line(s))
    if args.json:
        Path(args.json).write_text(json.dumps(results, indent=1))
    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(ROOT / "tools"))
    raise SystemExit(main())
