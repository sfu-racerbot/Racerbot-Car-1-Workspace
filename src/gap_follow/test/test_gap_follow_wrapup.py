"""Wrap-up fixes (Part B-2/3); run with ROS-sourced package pytest."""
from rclpy.duration import Duration

from test_gap_follow_fused_gate_node import FUSED
from test_gap_follow_traction_node import _node, _odom, _tick
from test_gap_follow_steering_damping_node import (
    _capture, _feed_imu, _feed_odom, _hold_deadman)


def _stale_case(wheel_stale, fused_stale):
    with _node(False, FUSED) as n:
        commands = _capture(n)
        _feed_imu(n, yaw_rate=0.0)
        _feed_odom(n, 1.8)
        _odom(n.wheel_odom_callback, 1.8)
        if wheel_stale:
            n.last_wheel_time -= Duration(seconds=n.odom_timeout_sec)
        if fused_stale:
            n.last_odom_time -= Duration(seconds=n.odom_timeout_sec)
        _hold_deadman(n)
        n.last_commanded_speed = 1.5
        _tick(n)
        return (commands[-1].drive.speed, n.last_decision_state,
                n._stale_odom_detail(), n.wheel_odom_topic, n.odom_topic)


def test_stale_wheel_odometry_names_the_wheel_topic():
    speed, state, detail, wheel, fused = _stale_case(True, False)
    # oracle: spec (wrap-up M1): the stop names the stream that went stale.
    assert (speed, state) == (0.0, 'odometry_stale')
    assert f"'{wheel}'" in detail and f"'{fused}'" not in detail


def test_stale_fused_odometry_names_the_fused_topic():
    speed, state, detail, wheel, fused = _stale_case(False, True)
    # oracle: spec (wrap-up M1).
    assert (speed, state) == (0.0, 'odometry_stale')
    assert f"'{fused}'" in detail and f"'{wheel}'" not in detail


def test_both_stale_names_both_topics():
    speed, state, detail, wheel, fused = _stale_case(True, True)
    # oracle: spec (wrap-up M1).
    assert (speed, state) == (0.0, 'odometry_stale')
    assert f"'{wheel}'" in detail and f"'{fused}'" in detail
