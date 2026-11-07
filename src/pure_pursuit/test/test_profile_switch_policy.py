"""Runtime switch refusal and preservation; isolated test drive topic."""
from test_tracking_law_integration import node_factory, profiled_csv, _hold_lb, _release_lb
from rclpy.parameter import Parameter
import pytest


@pytest.mark.parametrize('state', ['stopped', 'held', 'moving', 'reverse', 'missing_odom', 'stale_odom', 'missing_joy', 'stale_joy', 'nan_speed', 'boundary', 'bad_file'])
def test_switch_policy(node_factory, profiled_csv, state):
    node = node_factory('pure_pursuit')
    node.enable_deadman = True
    _release_lb(node)
    node.current_speed = 0.0
    node.last_odom_time = node.get_clock().now()
    old = node.xy
    if state == 'held':
        _hold_lb(node)
    if state == 'moving':
        node.current_speed = .051
    if state == 'missing_odom':
        node.last_odom_time = None
    from rclpy.duration import Duration
    if state == 'reverse':
        node.current_speed = -.051
    if state == 'boundary':
        node.current_speed = .05
    if state == 'stale_odom':
        node.last_odom_time -= Duration(seconds=1)
    if state == 'missing_joy':
        node.last_joy_time = None
    if state == 'stale_joy':
        node.last_joy_time -= Duration(seconds=1)
    if state == 'nan_speed':
        node.current_speed = float('nan')
    path = '/missing/profile.csv' if state == 'bad_file' else profiled_csv
    result = node.set_parameters([Parameter('waypoints_file', value=path)])[0]
    # oracle: specified switch policy; denied updates preserve the active arrays.
    assert result.successful == (state in ('stopped', 'boundary'))
    if state not in ('stopped', 'boundary'):
        assert node.xy is old
        assert result.reason
    else:
        assert node.prev_nearest_index is None


def test_failed_load_keeps_old_line_driving(node_factory, profiled_csv):
    from test_tracking_law_integration import _pose_slightly_left, _one_command_interval, _capture_published
    node = node_factory('pure_pursuit')
    node.enable_deadman = True
    _release_lb(node)
    node.current_speed = 0.0
    node.last_odom_time = node.get_clock().now()
    old = (node.xy, node.speed_profile, node.cumulative_arc_length, node.path_context)
    result = node.set_parameters([Parameter('waypoints_file', value='/missing/file.csv')])[0]
    assert not result.successful
    assert all(a is b for a, b in zip(old, (node.xy, node.speed_profile, node.cumulative_arc_length, node.path_context)))
    _hold_lb(node)
    _pose_slightly_left(node)
    _one_command_interval(node)
    published = _capture_published(node)
    node.control_loop()
    # oracle: invariant the rejected file does not disable the original drive path.
    assert published[-1].drive.speed > 0


def test_switch_republishes_new_points(node_factory, profiled_csv, tmp_path):
    import json
    import numpy as np
    from pure_pursuit import racing_math
    node = node_factory('pure_pursuit')
    _release_lb(node)
    node.current_speed = 0.0
    node.last_odom_time = node.get_clock().now()
    xy, speed = racing_math.load_profiled_csv(profiled_csv)
    xy = xy + [2.0, 1.0]
    new = str(tmp_path / 'new.csv')
    racing_math.save_profiled_csv(new, xy, speed)
    sent = []
    original = node.racing_line_pub.publish
    node.racing_line_pub.publish = lambda msg: (sent.append(json.loads(msg.data)), original(msg))
    result = node.set_parameters([Parameter('waypoints_file', value=new)])[0]
    assert result.successful, result.reason
    # oracle: translated input coordinates appear in the new published profile.
    np.testing.assert_allclose(node.xy, xy, atol=1e-4, rtol=0)
    np.testing.assert_allclose(np.array(sent[-1]['points'])[0, :2], xy[0], atol=.001, rtol=0)
    assert sent[-1]['source'] == new


def test_pending_line_waits_for_next_lb_press(node_factory, profiled_csv, tmp_path):
    import numpy as np
    from pure_pursuit import racing_math
    node = node_factory('pure_pursuit')
    node.enable_deadman = True
    _release_lb(node)
    node.current_speed = 0.0
    node.last_odom_time = node.get_clock().now()
    xy, speed = racing_math.load_profiled_csv(profiled_csv)
    new = str(tmp_path / 'pending.csv')
    racing_math.save_profiled_csv(new, xy + [1., 0.], speed)
    old = node.xy
    result = node.set_parameters([Parameter('pending_waypoints_file', value=new)])[0]
    assert result.successful, result.reason
    # oracle: user policy, preparation must not change the active line.
    assert node.xy is old
    _release_lb(node)
    assert node.xy is old
    # Activation consumes the validated in-memory snapshot, never a changed file.
    from pathlib import Path
    Path(new).write_text('corrupt after validation')
    _hold_lb(node)
    np.testing.assert_allclose(node.xy, xy + [1., 0.], atol=1e-4, rtol=0)
    assert node._pending_profile is None


@pytest.mark.parametrize('state', ['moving', 'stale', 'cancel'])
def test_pending_line_refuses_unsafe_press_or_cancellation(node_factory, profiled_csv, state):
    from rclpy.duration import Duration
    node = node_factory('pure_pursuit')
    node.enable_deadman = True
    _release_lb(node)
    node.current_speed = 0.0
    node.last_odom_time = node.get_clock().now()
    old = node.xy
    assert node.set_parameters([Parameter('pending_waypoints_file', value=profiled_csv)])[0].successful
    if state == 'moving':
        node.current_speed = .051
    elif state == 'stale':
        node.last_odom_time -= Duration(seconds=1)
    else:
        assert node.set_parameters([Parameter('pending_waypoints_file', value='')])[0].successful
    _hold_lb(node)
    # oracle: no unsafe or cancelled activation.
    assert node.xy is old
