"""R1 conservative safety inputs; run with ROS-sourced package pytest."""
import pytest
from rclpy.duration import Duration

from test_gap_follow_fused_gate_node import FUSED, _boundary_command
from test_gap_follow_traction_node import _node, _odom, _tick
from test_gap_follow_steering_damping_node import (
    _capture, _feed_imu, _feed_odom, _hold_deadman, _scan)


def test_fused_below_wheels_keeps_the_feature_off_published_stop():
    # oracle: invariant R1, same wheel speed and history must retain the stop.
    assert _boundary_command('ok') == _boundary_command('ok', fused=False)
    assert _boundary_command('ok')[0] == 0.0


def _higher_boundary(fused):
    with _node(False, FUSED if fused else ()) as n:
        commands = _capture(n)
        _feed_imu(n, yaw_rate=0.0)
        _feed_odom(n, 2.2 if fused else 2.0)
        _odom(n.wheel_odom_callback, 2.0)
        _hold_deadman(n)
        n.last_commanded_speed = 1.5
        n.last_command_time = n.get_clock().now() - Duration(seconds=0.025)
        # oracle: closed form: TTC(2.2) < threshold < TTC(2.0).
        clearance = 2.1 * n.ttc_threshold_sec
        front = (n.wheelbase + n.car_length) / 2 - n.laser_offset_x
        ranges = [8.0] * 541
        ranges[270] = front + clearance
        ranges[0] = n.car_width / 2 + clearance * 0.75
        n.scan_callback(_scan(ranges))
        return commands[-1].drive.speed, n.last_decision_state


def test_fused_above_wheels_brakes_at_the_higher_speed_boundary():
    # oracle: closed form above; the higher estimate must brake earlier.
    assert _higher_boundary(True) == (0.0, 'ttc_brake')
    assert _higher_boundary(False)[0] > 0.0


def test_fresh_fused_stream_cannot_mask_stale_wheel_odometry():
    with _node(False, FUSED) as n:
        commands = _capture(n)
        _feed_imu(n, yaw_rate=0.0)
        _feed_odom(n, 1.8)
        _odom(n.wheel_odom_callback, 2.0)
        n.last_wheel_time -= Duration(seconds=n.odom_timeout_sec)
        _hold_deadman(n)
        n.last_commanded_speed = 1.5
        _tick(n)
        # oracle: spec R1: /odom freshness remains mandatory at the boundary.
        assert commands[-1].drive.speed == 0.0
        assert n.last_decision_state == 'odometry_stale'


@pytest.mark.parametrize('wheel,fused', [(2.0, 1.8), (2.0, 2.2),
                                          (-2.0, 2.2), (0.0, 2.2)])
def test_safety_speed_retains_wheel_direction_and_larger_magnitude(wheel, fused):
    import math
    with _node(False, FUSED) as n:
        _feed_imu(n, yaw_rate=0.0)
        _feed_odom(n, fused)
        _odom(n.wheel_odom_callback, wheel)
        # oracle: closed form R1, including standstill and reverse sign.
        assert n._safety_speed() == math.copysign(max(abs(wheel), abs(fused)), wheel)
