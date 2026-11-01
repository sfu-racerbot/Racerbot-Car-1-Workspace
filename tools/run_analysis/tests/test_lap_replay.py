"""lap_replay.py: recorded poses through the real lap counter.

Runner: python3 -m pytest tools/run_analysis/tests/ with ROS sourced (LapRecorder is in auto_map_race_node,
which imports rclpy; nothing here joins a ROS graph).
Oracles: closed forms. A car driven three times round a 6 m circle with LB held closes 3 laps of 2*pi*6 m; with
LB never held it closes none; odometry is read at or before a pose's time, never after. The recorded case is
pinned in src/pure_pursuit/test/test_lap_recorder_corrections.py.
"""
import math
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from run_analysis import lap_replay as lr  # noqa: E402

RADIUS = 6.0


def _circle(laps=3.2, speed=2.0, rate=40.0):
    """(t, x, y, yaw) rows of a car driving anticlockwise round a circle, starting at the origin heading +x."""
    t = np.arange(0.0, laps * 2 * math.pi * RADIUS / speed, 1.0 / rate)
    angle = speed * t / RADIUS
    return np.c_[t, RADIUS * np.sin(angle), RADIUS - RADIUS * np.cos(angle), angle]


def test_odometry_is_read_at_or_before_the_pose():
    odom = np.array([[1.0, 0, 0, 0], [2.0, 1, 0, 0], [3.0, 2, 0, 0]])
    assert lr.odom_at(odom, 0.99) is None
    assert lr.odom_at(odom, 1.0)[1] == 0
    assert lr.odom_at(odom, 2.5)[1] == 1  # not the later row
    assert lr.odom_at(odom, 9.0)[1] == 2


def test_lb_windows_include_their_ends():
    assert lr.held([(1.0, 2.0)], 1.0) and lr.held([(1.0, 2.0)], 2.0)
    assert not lr.held([(1.0, 2.0)], 2.0001)
    assert not lr.held([], 1.5)


def test_three_laps_of_a_circle_close_three_laps_of_its_length():
    poses = _circle()
    laps, corrections = lr.count_laps(lr.shipped_recorder(), poses, poses, [(0.0, poses[-1, 0])])
    assert len(laps) == 3
    for lap in laps:
        assert lap == pytest.approx(2 * math.pi * RADIUS, rel=0.03)  # sampled at 0.15 m spacing
    assert corrections == 0


def test_no_lap_closes_while_lb_was_never_held():
    poses = _circle()
    assert lr.count_laps(lr.shipped_recorder(), poses, poses, []) == ([], 0)


def test_poses_before_any_odometry_are_skipped():
    poses = _circle()
    late = poses[poses[:, 0] > poses[-1, 0] / 2]  # odometry only for the second half
    laps, _ = lr.count_laps(lr.shipped_recorder(), poses, late, [(0.0, poses[-1, 0])])
    assert len(laps) == 1


def test_the_recorder_is_set_up_from_the_shipped_file():
    recorder = lr.shipped_recorder()
    value = next(iter(yaml.safe_load(lr.CONFIG.read_text()).values()))['ros__parameters']
    assert recorder.closure_distance == float(value['closure_distance'])
    assert recorder.min_turn_rad == pytest.approx(math.radians(float(value['minimum_lap_turn_deg'])), abs=1e-12)
    assert recorder.max_pose_jump == float(value['max_pose_jump_m'])


def test_corrections_of_every_lap_are_counted_not_only_the_last():
    # oracle: closed form. SLAM moves the map pose 0.5 m sideways once, in the first lap, where odometry saw
    # nothing: one correction, still counted after that lap has closed and the recorder has been reset.
    poses = _circle()
    odom = poses.copy()
    poses[(poses[:, 0] > 5.0), 2] += 0.5  # m, from 5 s on: one jump
    laps, corrections = lr.count_laps(lr.shipped_recorder(), poses, odom, [(0.0, poses[-1, 0])])
    assert len(laps) == 3
    assert corrections == 1
