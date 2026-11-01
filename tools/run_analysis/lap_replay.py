#!/usr/bin/env python3
"""Count a run's laps offline: its recorded SLAM pose through the real lap counter, on each odometry frame.

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 tools/run_analysis/lap_replay.py data/racerbot_runs/<time> [more runs]

Needs a run with /slam_pose: a map_course run, or a recording made by replay_map_course.py. LapRecorder is
auto_map_race_node's own class, set up from the shipped auto_map_race.yaml, fed only while LB was held, with
odometry read at or before each pose's time, as the node reads it. It prints the laps each odometry frame closes
and how many SLAM corrections it absorbed.

Why: on 2026-10-31 this found in seconds why run 20261030-180355 never closed a lap. Counted in `odom` while SLAM
mapped on `odom_imu`, hard turns read as corrections: on its replay, 0 laps and 36 corrections, against 3 laps and
5 in `odom_imu` (docs/superpowers/specs/2026-10-30-slam-reliability.md).
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'src' / 'pure_pursuit'))
CONFIG = ROOT / 'src' / 'pure_pursuit' / 'config' / 'auto_map_race.yaml'


def shipped_recorder(config_path=CONFIG):
    """A LapRecorder set up as auto_map_race_node sets up its own, from the shipped parameter file."""
    from pure_pursuit.auto_map_race_node import LapRecorder
    value = next(iter(yaml.safe_load(Path(config_path).read_text()).values()))['ros__parameters']
    return LapRecorder(
        spacing=float(value['waypoint_spacing']), min_distance=float(value['minimum_lap_distance']),
        departure_distance=float(value['departure_distance']), closure_distance=float(value['closure_distance']),
        closure_heading_rad=math.radians(float(value['closure_heading_deg'])),
        min_duration_sec=float(value['minimum_lap_duration_sec']),
        min_turn_rad=math.radians(float(value['minimum_lap_turn_deg'])),
        max_pose_jump=float(value['max_pose_jump_m']),
        closure_widen_after_revolutions=float(value['closure_widen_after_revolutions']),
        max_closure_distance=float(value['max_closure_distance']),
        close_at_closest_approach=bool(value['close_at_closest_approach']),
        closest_approach_margin=float(value['closest_approach_margin_m']),
        min_duration_fraction=float(value['minimum_lap_duration_fraction']))


def odom_at(odom, t):
    """The newest odometry row (t, x, y, yaw) at or before t, or None before the first."""
    index = int(np.searchsorted(odom[:, 0], t, side='right')) - 1
    return None if index < 0 else odom[index]


def held(windows, t):
    return any(start <= t <= end for start, end in windows)


def count_laps(recorder, poses, odom, windows):
    """([lap length m, ...], SLAM corrections absorbed) from map poses (t, x, y, yaw) and one odometry."""
    laps, corrections = [], 0
    odom = np.asarray(odom, dtype=float).reshape(-1, 4)
    for t, x, y, yaw in np.asarray(poses, dtype=float).reshape(-1, 4):
        if not held(windows, t):
            continue
        row = odom_at(odom, t) if len(odom) else None
        if row is None:
            continue
        if recorder.update(x, y, yaw, t, odom_yaw=row[3], odom_xy=(row[1], row[2])):
            laps.append(recorder.distance)
            corrections += recorder.reanchor_count
            recorder.reset()
    return laps, corrections + recorder.reanchor_count


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_run(run_dir):
    """(slam_pose, {frame: odometry}, LB windows), every pose on its header stamp.

    Header stamps, not arrival times, because the node reads odometry at the map pose's stamp: on arrival times
    the two are a few ms apart and the gap reads as SLAM corrections (23 instead of 2 on one run).
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from run_analysis import runlog
    bag = Path(run_dir) / 'bag'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap' if any(bag.glob('*.mcap')) else 'sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    topics = {'/slam_pose': 'slam_pose', '/odom': 'odom', '/odom/imu': 'odom_imu'}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    rows = {name: [] for name in topics.values()}
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        msg = deserialize_message(raw, get_message(types[topic]))
        pose = msg.pose.pose if hasattr(msg.pose, 'pose') else msg.pose
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        rows[topics[topic]].append((stamp, pose.position.x, pose.position.y, _yaw(pose.orientation)))
    arrays = {name: np.array(sorted(value)).reshape(-1, 4) for name, value in rows.items()}
    return arrays.pop('slam_pose'), arrays, runlog.load_run(run_dir).lb_windows()


def replay(run_dir):
    poses, odometry, windows = read_run(run_dir)
    if len(poses) == 0:
        raise ValueError(f'{run_dir} has no /slam_pose: only map_course runs and their replays do')
    return {frame: count_laps(shipped_recorder(), poses, rows, windows)
            for frame, rows in odometry.items() if len(rows)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('runs', type=Path, nargs='+')
    args = parser.parse_args(argv)
    for run in args.runs:
        try:
            result = replay(run)
        except (ValueError, OSError) as exc:
            print(f'{run.name}: {exc}')
            continue
        print(f'{run.name}:')
        for frame, (laps, corrections) in result.items():
            lengths = ', '.join(f'{lap:.1f}' for lap in laps) or 'none'
            print(f'  counted in {frame:8s} {len(laps)} lap(s) [{lengths}] m, {corrections} SLAM corrections')
    return 0


if __name__ == '__main__':
    sys.exit(main())
