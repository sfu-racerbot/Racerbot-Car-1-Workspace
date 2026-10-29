"""Exact replay contracts. Runner: rb python3 -m pytest tools/run_analysis/tests/."""
import math
import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from run_analysis import slam_replay as replay


def test_plan_preserves_pose_order():
    # oracle: invariant - only processed stamps, in recorded pose order.
    assert replay.exact_scan_plan([10, 20, 30], [30, 10]) == [30, 10]


@pytest.mark.parametrize('poses', [[10, 99], [99, 10]])
def test_missing_scan_refused_including_first_pose(poses):
    # oracle: spec S4 controller ruling - every processed pose has its scan.
    with pytest.raises(replay.StampMismatch, match=r'1.*99'):
        replay.exact_scan_plan([10, 20], poses)


def test_duplicate_scan_refused():
    # oracle: invariant - a stamp must identify exactly one scan.
    with pytest.raises(replay.StampMismatch, match='duplicate.*10'):
        replay.exact_scan_plan([10, 10], [10])


def test_empty_poses():
    # oracle: closed form - no processed poses implies no scans to feed.
    assert replay.exact_scan_plan([10], []) == []


def test_identical_pose_sequence():
    # oracle: closed form - subtraction of identical poses is zero.
    assert replay.pose_sequence_error([(10, 1, -2, 0)], [(10, 1, -2, 0)]) == {
        'matched': 1, 'missing_in_replay': 0, 'extra_in_replay': 0,
        'max_position_m': 0.0, 'max_yaw_deg': 0.0, 'first_over_bar': None}


def test_position_over_bar():
    # oracle: closed form - displacement sqrt(.002**2) = 2 mm exceeds 1 mm.
    result = replay.pose_sequence_error([(10, 0, 0, 0)], [(10, .002, 0, 0)])
    assert result['max_position_m'] == pytest.approx(.002, abs=1e-12)
    assert result['first_over_bar'] == 10


def test_yaw_wrap():
    # oracle: closed form - these angles differ by .02 degrees on the circle.
    result = replay.pose_sequence_error([(10, 0, 0, math.radians(179.99))],
                                        [(10, 0, 0, math.radians(-179.99))])
    assert result['max_yaw_deg'] == pytest.approx(.02, abs=1e-10)  # degrees
    assert result['first_over_bar'] is None


def test_missing_extra():
    # oracle: set difference {10,20} vs {20,30} has one member each way.
    result = replay.pose_sequence_error([(10, 0, 0, 0), (20, 0, 0, 0)],
                                        [(20, 0, 0, 0), (30, 0, 0, 0)])
    assert (result['matched'], result['missing_in_replay'], result['extra_in_replay']) == (1, 1, 1)


def test_map_bytes_and_yaml(tmp_path):
    # oracle: spec nav2_map_server/src/map_io.cpp saveMapToFile trinary, y flipped.
    pgm, meta = replay.map_from_bag_files([[0, 100, -1], [-1, 0, 100]], .05, (-2, 3), tmp_path)
    assert pgm.read_bytes() == b'P5\n3 2\n255\n' + bytes([205, 254, 0, 254, 0, 205])
    assert yaml.safe_load(meta.read_text()) == {
        'image': 'map.pgm', 'mode': 'trinary', 'resolution': .05,
        'origin': [-2, 3, 0], 'negate': 0, 'occupied_thresh': .65, 'free_thresh': .25}


def test_exact_rate_refused(capsys):
    # oracle: spec - lockstep has no playback rate.
    with pytest.raises(SystemExit) as exc:
        replay.main(['unused', '--exact', '--rate', '2'])
    assert exc.value.code == 2
    assert 'not allowed' in capsys.readouterr().err


def test_exact_driver_refusal(tmp_path, monkeypatch, capsys):
    # oracle: spec - same fake /proc boundary as test_slam_replay.py.
    proc = tmp_path / '123'
    proc.mkdir()
    (proc / 'cmdline').write_bytes(b'/opt/ros/urg_node')
    monkeypatch.setattr(replay, 'PROC_ROOT', tmp_path)
    assert replay.main(['unused', '--exact']) == 2
    assert 'driver processes' in capsys.readouterr().err


def test_exact_children_isolated():
    # oracle: spec - every planned process, including feeder, domain 77 localhost.
    children = replay.exact_children('params.yaml', 'bag', '/odom/imu', {'ROS_DOMAIN_ID': '79'})
    assert {name for name, argv, env in children} == {'slam_toolbox', 'exact_feed'}
    for name, argv, env in children:
        assert env['ROS_DOMAIN_ID'] == '77', name
        assert env['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST', name
        assert env['ROS_STATIC_PEERS'] == '', name


def test_odometry_brackets():
    # oracle: closed form - first sample >= scan, including equality, no later sample.
    assert replay.odometry_brackets([10, 20, 30, 40], [15, 20, 31]) == [2, 2, 4]


@pytest.mark.parametrize('odoms', [[], [10, 20]])
def test_missing_odometry_bracket_refused(odoms):
    # oracle: spec controller ruling - no future bracket refuses with scan stamp.
    with pytest.raises(replay.StampMismatch, match='bracket.*21'):
        replay.odometry_brackets(odoms, [21])


@pytest.mark.parametrize('distance,first', [(0, None), (-.001, None), (.0010001, 10)])
def test_position_bar_boundary(distance, first):
    # oracle: spec 1 mm, strict exceedance; sign does not change distance.
    result = replay.pose_sequence_error([(10, 0, 0, 0)], [(10, distance, 0, 0)])
    assert result['max_position_m'] == abs(distance)
    assert result['first_over_bar'] == first


def test_empty_pose_sequences():
    # oracle: closed form - empty sets have no matches, differences or errors.
    assert replay.pose_sequence_error([], []) == {
        'matched': 0, 'missing_in_replay': 0, 'extra_in_replay': 0,
        'max_position_m': 0.0, 'max_yaw_deg': 0.0, 'first_over_bar': None}


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_nonfinite_pose_refused(value):
    # oracle: invariant - undefined error must not appear as zero error.
    with pytest.raises(ValueError, match='non-finite.*10'):
        replay.pose_sequence_error([(10, 0, 0, 0)], [(10, value, 0, 0)])


def _write_map_bag(path, grids):
    import rosbag2_py
    from nav_msgs.msg import OccupancyGrid
    from rclpy.serialization import serialize_message
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=0, name='/map', type='nav_msgs/msg/OccupancyGrid', serialization_format='cdr'))
    for index, grid in enumerate(grids):
        msg = OccupancyGrid()
        msg.info.width, msg.info.height = 3, 2
        msg.info.resolution = .5
        msg.info.origin.position.x, msg.info.origin.position.y = -2., 3.
        msg.info.origin.orientation.w = 1.
        msg.data = grid
        writer.write('/map', serialize_message(msg), index + 1)
    del writer  # close and flush before the real reader opens it


def test_fallback_uses_last_map(tmp_path):
    # oracle: spec - use the LAST replay map, not the first; pixel conversion closed form.
    bag = tmp_path / 'bag'
    _write_map_bag(bag, [[0] * 6, [0, 100, -1, -1, 0, 100]])
    out = tmp_path / 'out'
    assert replay.save_map_from_bag(bag, out) == 'from_bag'
    assert (out / 'map.pgm').read_bytes() == b'P5\n3 2\n255\n' + bytes([205, 254, 0, 254, 0, 205])
    assert yaml.safe_load((out / 'map.yaml').read_text())['origin'] == [-2, 3, 0]


def test_fallback_without_map(tmp_path):
    # oracle: invariant - no recorded map cannot be reported as saved.
    bag = tmp_path / 'bag'
    _write_map_bag(bag, [])
    out = tmp_path / 'out'
    assert replay.save_map_from_bag(bag, out) is False
    assert not out.exists()
