"""Tests for the real DisparityTrajectoryNode: the deadman gate and every stop path.

Needs ROS sourced and an isolated domain (CLAUDE.md):
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 -m pytest src/disparity_extender/test/ -q

Callbacks are driven directly; the assertion is always the published command.
Every node here publishes on /test_only/drive, never the real /drive: the remap
goes LAST, after the params files, or the YAML's drive_topic: /drive wins.
"""
import math
import os
from pathlib import Path

import numpy as np
import pytest
import rclpy
import yaml
from rclpy.duration import Duration
from sensor_msgs.msg import Joy, LaserScan

from disparity_trajectory import trajectory_logic as tl
from disparity_trajectory.disparity_trajectory_node import DisparityTrajectoryNode

PKG = Path(__file__).resolve().parents[1]
SHARED = PKG / 'config' / 'disparity_trajectory.yaml'
RB1 = PKG.parents[1] / 'src' / 'racerbot_car' / 'cars' / 'rb1' / 'calibration' / 'disparity_trajectory.yaml'
TEST_ONLY_DRIVE = ['-p', 'drive_topic:=/test_only/drive']
LB = 4


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args', '--params-file', str(SHARED), '--params-file', str(RB1), *TEST_ONLY_DRIVE])
    n = DisparityTrajectoryNode()
    assert n.drive_pub.topic_name == '/test_only/drive'
    yield n
    n.destroy_node()
    rclpy.shutdown()


def scan(ranges):
    m = LaserScan()
    m.angle_min, m.angle_increment = -3 * math.pi / 4, 1.5 * math.pi / 1080
    m.angle_max = m.angle_min + 1080 * m.angle_increment
    m.range_min, m.range_max = 0.05, 30.0
    m.ranges = [float(r) for r in ranges]
    return m


def corridor(half=0.8):
    a = -3 * math.pi / 4 + 1.5 * math.pi / 1080 * np.arange(1081)
    s = np.sin(a)
    with np.errstate(divide='ignore'):
        r = np.where(np.abs(s) > 1e-9, half / np.abs(s), np.inf)
    return np.where(r < 30.0, r, np.inf)


def joy(n, held=True, buttons=LB + 1):
    m = Joy()
    m.buttons = [0] * buttons
    if buttons > LB:
        m.buttons[LB] = 1 if held else 0
    n.joy_callback(m)


def capture(n):
    out = []
    original = n.drive_pub.publish
    n.drive_pub.publish = lambda msg: (out.append((msg.drive.steering_angle, msg.drive.speed)), original(msg))
    return out


def test_shipped_config_drives_the_real_topic_and_keeps_the_deadman_on():
    params = yaml.safe_load(SHARED.read_text())['disparity_trajectory_node']['ros__parameters']
    assert params['drive_topic'] == '/drive' and params['scan_topic'] == '/scan' and params['joy_topic'] == '/joy'
    assert params['enable_deadman'] is True and params['deadman_button'] == LB


# --- the four deadman states (CLAUDE.md standing requirement) ----------------
def test_lb_held_drives(node):
    out = capture(node)
    joy(node, held=True)
    node.scan_callback(scan(corridor()))
    assert out[-1][1] > 0.0 and node.last_decision_state == 'driving'


def test_lb_released_publishes_zero(node):
    out = capture(node)
    joy(node, held=True)
    joy(node, held=False)
    node.scan_callback(scan(corridor()))
    assert out[-1] == (0.0, 0.0) and node.last_decision_state == 'deadman_released'


def test_no_joy_ever_publishes_zero(node):
    out = capture(node)
    node.scan_callback(scan(corridor()))
    assert out[-1] == (0.0, 0.0) and node.last_decision_state == 'waiting_for_joy'


def test_stale_joy_publishes_zero(node):
    out = capture(node)
    joy(node, held=True)
    node.last_joy_time = node.get_clock().now() - Duration(seconds=node.joy_timeout_sec + 0.01)
    node.scan_callback(scan(corridor()))
    assert out[-1] == (0.0, 0.0) and node.last_decision_state == 'joy_stale'


def test_joy_just_inside_the_timeout_still_drives(node):
    out = capture(node)
    joy(node, held=True)
    node.last_joy_time = node.get_clock().now() - Duration(seconds=node.joy_timeout_sec - 0.05)
    node.scan_callback(scan(corridor()))
    assert out[-1][1] > 0.0


def test_joy_without_the_lb_button_publishes_zero(node):
    out = capture(node)
    joy(node, buttons=LB)                      # buttons 0..3 only
    node.scan_callback(scan(corridor()))
    assert out[-1] == (0.0, 0.0) and node.last_decision_state == 'deadman_button_missing'


def test_joy_from_before_a_backward_clock_step_is_stale(node):
    out = capture(node)
    joy(node, held=True)
    node.last_joy_time = node.get_clock().now() + Duration(seconds=2.7)   # measured WSL step size
    node.scan_callback(scan(corridor()))
    assert out[-1] == (0.0, 0.0) and node.last_decision_state == 'joy_stale'


# --- scan edge cases: a blind or broken LiDAR stops the car -----------------------
@pytest.mark.parametrize('ranges', [np.full(1081, np.inf), np.zeros(1081), np.full(1081, np.nan), np.array([])],
                         ids=['all-inf', 'all-zero', 'all-nan', 'empty'])
def test_an_invalid_scan_publishes_zero_even_with_lb_held(node, ranges):
    out = capture(node)
    joy(node, held=True)
    node.scan_callback(scan(ranges))
    assert out[-1] == (0.0, 0.0) and node.last_decision_state == 'scan_invalid'


def test_valid_fraction_boundary(node):
    out = capture(node)
    joy(node, held=True)
    r = corridor()
    finite = np.nonzero(np.isfinite(r))[0]
    keep = int(math.ceil(node.min_valid_fraction * r.size))          # exactly the minimum fraction
    r2 = np.full(r.size, np.inf)
    r2[finite[:keep]] = r[finite[:keep]]
    node.scan_callback(scan(r2))
    assert node.last_decision_state != 'scan_invalid'
    r2[finite[keep - 1]] = np.inf                                     # one beam short
    node.scan_callback(scan(r2))
    assert out[-1] == (0.0, 0.0) and node.last_decision_state == 'scan_invalid'


def test_no_clear_arc_publishes_zero(node):
    out = capture(node)
    joy(node, held=True)
    node.scan_callback(scan(np.full(1081, 0.12)))                     # everything 12 cm away
    assert out[-1] == (0.0, 0.0) and node.last_decision_state == 'no_clear_arc'


def test_silent_lidar_stops_via_the_watchdog(node):
    out = capture(node)
    joy(node, held=True)
    node.scan_callback(scan(corridor()))
    assert out[-1][1] > 0.0
    node.last_scan_time = node.get_clock().now() - Duration(seconds=node.scan_timeout_sec + 0.01)
    node._watchdog()
    assert out[-1] == (0.0, 0.0) and node.last_decision_state == 'scan_stale'


def test_watchdog_leaves_a_fresh_scan_alone(node):
    out = capture(node)
    joy(node, held=True)
    node.scan_callback(scan(corridor()))
    n = len(out)
    node._watchdog()
    assert len(out) == n


def test_an_exception_stops_the_car_before_propagating(node, monkeypatch):
    out = capture(node)
    joy(node, held=True)

    def broken(*_a, **_k):
        raise RuntimeError('injected fault')
    monkeypatch.setattr(tl, 'trajectory_target', broken)
    with pytest.raises(RuntimeError, match='injected fault'):
        node.scan_callback(scan(corridor()))
    assert out[-1] == (0.0, 0.0) and node.last_decision_state == 'exception'


def test_every_command_respects_the_car_steering_and_speed_limits(node):
    out = capture(node)
    joy(node, held=True)
    for box_y in (-0.9, -0.5, 0.0, 0.5, 0.9):
        r = corridor(1.2)
        a = -3 * math.pi / 4 + 1.5 * math.pi / 1080 * np.arange(1081)
        hit = np.abs(np.arctan2(box_y, 1.5) - a) < 0.15
        r[hit] = np.minimum(r[hit], 1.5)
        node.scan_callback(scan(r))
    assert all(abs(s) <= 0.33 + 1e-9 and 0.0 <= v <= 2.5 + 1e-9 for s, v in out)   # rb1 limit, max_speed


def test_the_car_calibration_overlay_sets_the_steering_limit(node):
    assert node.max_steering == 0.33                                  # rb1 overlay (gap_follow's value)
