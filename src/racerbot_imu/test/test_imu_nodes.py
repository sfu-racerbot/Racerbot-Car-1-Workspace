"""The real imu_conditioner_node and imu_select_node.

Needs ROS 2 sourced, and its own domain:

    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 -m pytest src/racerbot_imu/test/test_imu_nodes.py -v

Neither node can move the car. Tests drive the callbacks directly.
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest  # noqa: E402
import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from racerbot_imu.imu_conditioner_node import ImuConditionerNode  # noqa: E402
from racerbot_imu.imu_select_node import ImuSelectNode  # noqa: E402


def _raw(gyro=(0.0, 0.0, 5.0), accel=(0.0, 0.0, 1.0)):
    m = Imu()
    m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = gyro
    m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = accel
    return m


def _still(node):
    o = Odometry()
    o.twist.twist.linear.x = 0.0
    node.odom_callback(o)


@pytest.fixture
def cond():
    # window 0: one still sample completes the bias window -- the window's own
    # length rules are tested in test_imu_math.py.
    rclpy.init(args=['--ros-args', '-p', 'stationary_window_sec:=0.0', '-p', 'imu_name:=test'])
    n = ImuConditionerNode()
    out = []
    n.data_pub.publish = out.append
    yield n, out
    n.destroy_node()
    rclpy.shutdown()


def test_publishes_nothing_until_ready(cond):
    n, out = cond
    o = Odometry()
    o.twist.twist.linear.x = 1.0                  # moving: no stationary window, no bias
    n.odom_callback(o)
    for _ in range(5):
        n.raw_callback(_raw())
    assert out == [] and n.state == 'waiting'


def test_output_is_si_rotated_and_bias_removed(cond):
    """Oracle: 5 deg/s yaw bias measured still, then a 95 deg/s raw yaw reads
    90 deg/s = pi/2 rad/s; 1 g on z reads 9.80665 m/s^2."""
    n, out = cond
    _still(n)
    n.raw_callback(_raw(gyro=(0.0, 0.0, 5.0)))     # still: becomes the bias
    o = Odometry()
    o.twist.twist.linear.x = 1.0
    n.odom_callback(o)
    n.raw_callback(_raw(gyro=(0.0, 0.0, 95.0)))
    assert n.state == 'ok'
    m = out[-1]
    assert m.angular_velocity.z == pytest.approx(math.pi / 2, abs=1e-9)
    assert m.linear_acceleration.z == pytest.approx(9.80665, abs=1e-9)
    assert m.header.frame_id == 'base_link' and m.orientation_covariance[0] == -1.0


def test_all_zero_stops_publishing(cond):
    n, out = cond
    _still(n)
    n.raw_callback(_raw())
    count = len(out)
    n.raw_callback(_raw(gyro=(0.0, 0.0, 0.0), accel=(0.0, 0.0, 0.0)))
    assert len(out) == count and n.state == 'fault' and 'all zero' in n.reason


def test_saturated_sample_dropped(cond):
    n, out = cond
    _still(n)
    n.raw_callback(_raw())
    count = len(out)
    n.raw_callback(_raw(gyro=(0.0, 0.0, 2001.0)))      # deg/s, past 34.9 rad/s
    assert len(out) == count and n.saturated_count == 1
    n.raw_callback(_raw(gyro=(0.0, 0.0, 1999.0)))      # within range: published
    assert len(out) == count + 1


def test_gravity_fault_latches():
    """Raw accel in m/s^2 read as g (scale 9.81 applied twice) fails gravity,
    and stays failed even after good samples arrive."""
    rclpy.init(args=['--ros-args', '-p', 'stationary_window_sec:=0.0'])
    n = ImuConditionerNode()
    try:
        out = []
        n.data_pub.publish = out.append
        _still(n)
        n.raw_callback(_raw(accel=(0.0, 0.0, 9.81)))
        assert n.state == 'fault' and 'gravity' in n.reason
        n.raw_callback(_raw(accel=(0.0, 0.0, 1.0)))
        assert n.state == 'fault' and out == []
    finally:
        n.destroy_node()
        rclpy.shutdown()


def test_bad_rotation_refused():
    rclpy.init(args=['--ros-args', '-p', 'r_base_imu:=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0]'])
    n = ImuConditionerNode()
    try:
        out = []
        n.data_pub.publish = out.append
        _still(n)
        n.raw_callback(_raw())
        assert n.state == 'fault' and 'proper' in n.reason and out == []
    finally:
        n.destroy_node()
        rclpy.shutdown()


# --- selector -------------------------------------------------------------------------

@pytest.fixture
def sel():
    rclpy.init(args=['--ros-args', '-p', 'disagreement_sec:=0.5'])
    n = ImuSelectNode()
    out = []
    n.data_pub.publish = out.append
    yield n, out
    n.destroy_node()
    rclpy.shutdown()


def _ok(n, name, state='ok'):
    n.status_callback(name, String(data=json.dumps({'state': state})))


def _fill(n, name, w, t_end, dt=0.02, seconds=0.6):
    k = int(seconds / dt)
    for i in range(k):
        n.buf[name].add(t_end - (k - 1 - i) * dt, w)


def test_primary_passed_through_unchanged(sel):
    n, out = sel
    m = _raw(gyro=(0.1, 0.2, 0.3), accel=(1.0, 2.0, 3.0))
    m.header.frame_id = 'base_link'
    n.data_callback('vesc', m)
    n.data_callback('front', _raw(gyro=(9.0, 9.0, 9.0)))
    assert out == [m]                                 # the very same message, only the primary's


def test_agreeing_imus_ok(sel):
    n, _ = sel
    now = n._now()
    _ok(n, 'vesc'), _ok(n, 'front')
    _fill(n, 'vesc', 1.0, now), _fill(n, 'front', 1.03, now)
    n.evaluate()
    assert n.state == 'ok'


def test_disagreement_faults_after_hold(sel):
    n, _ = sel
    _ok(n, 'vesc'), _ok(n, 'front')
    t = n._now()
    _fill(n, 'vesc', 0.0, t), _fill(n, 'front', 1.0, t)
    n.evaluate()
    assert n.state == 'ok'                            # disagreeing, but not for 0.5 s yet
    n.agreement._since -= 0.5                        # as if it started 0.5 s ago
    n.evaluate()
    assert n.state == 'fault' and 'disagree' in n.reason


def test_secondary_missing_faults_when_required(sel):
    n, _ = sel
    _ok(n, 'vesc'), _ok(n, 'front', 'fault')
    _fill(n, 'vesc', 0.0, n._now())
    n.evaluate()
    assert n.state == 'fault' and 'secondary' in n.reason


def test_primary_fault_faults(sel):
    n, _ = sel
    _ok(n, 'vesc', 'waiting'), _ok(n, 'front')
    _fill(n, 'vesc', 0.0, n._now()), _fill(n, 'front', 0.0, n._now())
    n.evaluate()
    assert n.state == 'fault' and 'primary' in n.reason


def test_stale_primary_data_faults(sel):
    n, _ = sel
    _ok(n, 'vesc'), _ok(n, 'front')
    _fill(n, 'vesc', 0.0, n._now() - 0.5), _fill(n, 'front', 0.0, n._now())
    n.evaluate()
    assert n.state == 'fault'


def test_single_imu_mode():
    rclpy.init(args=['--ros-args', '-p', "secondary:=''"])
    n = ImuSelectNode()
    try:
        _ok(n, 'vesc')
        _fill(n, 'vesc', 0.4, n._now())
        n.evaluate()
        assert n.state == 'ok' and 'front' not in n.buf
    finally:
        n.destroy_node()
        rclpy.shutdown()
