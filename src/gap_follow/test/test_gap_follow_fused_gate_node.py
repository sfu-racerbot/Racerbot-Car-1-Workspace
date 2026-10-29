"""Fused odometry trust switching; ROS-sourced package pytest runner."""
import pytest
from rclpy.duration import Duration

from test_gap_follow_traction_node import _node, _odom, _tick
from test_gap_follow_steering_damping_node import (
    _capture, _capture_intent, _feed_imu, _feed_odom, _hold_deadman)

FUSED = ['-p', 'odom_topic:=/odom/fused']


def _run(state, fused=True, stale_wheel=False, deadman='held'):
    with _node(False, FUSED if fused else ()) as n:
        commands, intents = _capture(n), _capture_intent(n)
        if state != 'no imu':
            _feed_imu(n, yaw_rate=0.0,
                      state=state if state in ('waiting', 'fault') else 'ok',
                      age_sec=0.2 if state == 'stale' else 0.0)
        _feed_odom(n, 1.8 if fused or state == 'ok' else 2.0)
        _odom(n.wheel_odom_callback, 2.0)
        if stale_wheel:
            if fused:
                n.last_wheel_time -= Duration(seconds=0.6)
            else:
                n.last_odom_time -= Duration(seconds=0.6)
        n.last_commanded_speed = 1.5
        if deadman != 'never':
            _hold_deadman(n, held=deadman != 'released')
        if deadman == 'stale':
            n.last_joy_time -= Duration(seconds=0.6)
        if deadman == 'missing-button':
            from sensor_msgs.msg import Joy
            n.joy_callback(Joy())
        _tick(n)
        return (commands[-1].drive.speed, commands[-1].drive.steering_angle,
                intents[-1]['reason'], n.last_decision_state)


@pytest.mark.parametrize('state', ['ok', 'waiting', 'fault', 'stale', 'no imu'])
def test_fused_gate_matches_selected_raw_speed_and_names_imu_state(state):
    on = _run(state)
    raw = _run(state, False)
    # oracle: invariant: selected speed is 1.8 when trusted, otherwise raw 2.0.
    assert on[:2] == raw[:2]
    assert on[0] == (1.8 if state == 'ok' else 2.0) + 5.0 * 0.025
    assert f'fused_speed_imu: {state}' in on[2]
    assert 'fused_speed_imu:' not in raw[2]


def test_untrusted_imu_and_stale_raw_wheels_stop_exactly_as_raw_node():
    on = _run('fault', stale_wheel=True)
    raw = _run('fault', False, stale_wheel=True)
    # oracle: spec: fallback includes freshness and stale-odometry stop.
    assert on[:2] == raw[:2]
    assert on[0] == 0.0
    assert on[3] == raw[3] == 'odometry_stale'


@pytest.mark.parametrize('state', ['released', 'never', 'stale', 'missing-button'])
def test_fused_gate_deadman_deny_publishes_zero(state):
    # oracle: spec CLAUDE.md: fused speed never authorizes motion without LB.
    assert _run('ok', deadman=state)[0] == 0.0


def test_fused_gate_held_lb_drives():
    # oracle: closed form: permissive scene commands fused speed plus ramp.
    assert _run('ok')[0] == 1.8 + 5.0 * 0.025


def _ttc_boundary_scan(n):
    from test_gap_follow_steering_damping_node import _scan

    # oracle: closed form from shipped TTC parameters: clearance = 1.9*T;
    # at T=.30 s this is .57 m, TTC(2)=.285 < .30 < TTC(1.8)=.31667 s.
    # Front of body in laser coordinates = (wheelbase+length)/2 - offset_x.
    clearance = 1.9 * n.ttc_threshold_sec
    front = (n.wheelbase + n.car_length) / 2 - n.laser_offset_x
    ranges = [8.0] * 541
    ranges[270] = front + clearance
    # A closer flank return outside the TTC corridor makes the minimum body
    # clearance non-forward: no directional escape can mask the TTC verdict.
    # It is above emergency clearance and outside the forward reserve cone.
    ranges[0] = n.car_width / 2 + clearance * 0.75
    assert clearance * 0.75 > n.emergency_stop_clearance
    return _scan(ranges)


def _boundary_command(state, fused=True):
    with _node(False, FUSED if fused else ()) as n:
        commands = _capture(n)
        _feed_imu(n, yaw_rate=0.0, state=state)
        _feed_odom(n, 1.8 if fused else 2.0)
        _odom(n.wheel_odom_callback, 2.0)
        _hold_deadman(n)
        n.last_commanded_speed = 1.5
        n.last_command_time = n.get_clock().now() - Duration(seconds=0.025)
        n.scan_callback(_ttc_boundary_scan(n))
        return (commands[-1].drive.speed, commands[-1].drive.steering_angle,
                n.last_decision_state)


def test_fused_gate_changes_published_stop_at_ttc_boundary():
    trusted = _boundary_command('ok')
    fault = _boundary_command('fault')
    raw = _boundary_command('fault', fused=False)
    # oracle: closed form above and I5 fallback invariant: untrusted fused
    # odometry must produce exactly the raw-wheel node's TTC stop command.
    assert trusted[0] > 0.0
    assert trusted[2] != 'ttc_brake'
    assert fault == raw
    assert fault[0] == 0.0
    assert fault[2] == 'ttc_brake'


def _dropout_history(fused):
    from test_gap_follow_steering_damping_node import _scan

    with _node(False, FUSED if fused else ()) as n:
        commands = _capture(n)
        _feed_imu(n, yaw_rate=0.0)
        _feed_odom(n, 1.8)
        _odom(n.wheel_odom_callback, 2.0)
        _hold_deadman(n)
        n.last_commanded_speed = 1.5
        _tick(n)
        first = (commands[-1].drive.speed, commands[-1].drive.steering_angle)
        # No IMU or fused odometry callback between ticks: time alone makes
        # the gyro stale, while both odometry samples are still fresh.
        n.get_clock().set_ros_time_override(
            n.get_clock().now() + Duration(seconds=n.imu_timeout_sec))
        assert n.imu_timeout_sec + 0.025 < n.odom_timeout_sec
        if not fused:
            _feed_odom(n, 2.0)
        _hold_deadman(n)
        n.last_command_time = n.get_clock().now() - Duration(seconds=0.025)
        n.scan_callback(_scan())
        second = (commands[-1].drive.speed, commands[-1].drive.steering_angle)
        return first, second


def test_fused_gate_rechecks_imu_dropout_between_ticks_without_odom_callback():
    on = _dropout_history(True)
    raw = _dropout_history(False)
    # oracle: invariant: identical tick-k command/history; tick k+1 must use
    # wheel 2.0 immediately on IMU timeout, exactly as the raw node does.
    assert on[0] == raw[0]
    assert on[0][0] == 1.8 + 5.0 * 0.025
    assert on[1] == raw[1]
    assert on[1][0] == 2.0 + 5.0 * 0.025
