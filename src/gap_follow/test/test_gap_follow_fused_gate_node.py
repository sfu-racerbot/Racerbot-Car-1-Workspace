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
