"""Tests for tools/run_analysis/replay_map_course.py's parts with no ROS in them.

Runner (TEST_QUALITY_STANDARDS.md A9):
    python3 -m pytest tools/run_analysis/tests/test_replay_map_course.py -v
The replay itself starts the real launch, so it is verified by running it; those results are in
docs/superpowers/specs/2026-10-30-slam-reliability.md.

Oracles: the log lines are auto_map_race_node's own wording (map_course's _LAP_CLOSED reads the same);
the rebuild.json shape is what tools/run_analysis/rebuild_run.py writes; the rest are closed forms.
"""
import json
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # insert tools/
from run_analysis import replay_map_course as rmc  # noqa: E402


def _rows(*stretches, dt=0.02):
    """(seconds, speed) stretches as (time, speed) rows at 50 Hz."""
    rows, t = [], 100.0
    for seconds, speed in stretches:
        for _ in range(round(seconds / dt)):
            rows.append((t, speed))
            t += dt
    return rows


def test_a_run_that_starts_parked_starts_at_the_beginning():
    window = rmc.still_window(_rows((3.0, 0.0), (5.0, 2.0)))
    assert window[0] == pytest.approx(100.0, abs=1e-6)  # s
    assert window[1] == pytest.approx(103.0, abs=0.03)  # s, one sample


def test_a_run_that_starts_moving_starts_at_the_first_stop():
    window = rmc.still_window(_rows((2.0, 2.0), (3.0, 0.0), (5.0, 2.0)))
    assert window[0] == pytest.approx(102.0, abs=0.03)  # s


def test_a_pause_too_short_to_settle_the_gyro_does_not_count():
    rows = _rows((0.5, 0.0), (2.0, 2.0), (2.0, 0.0), (1.0, 2.0))
    assert rmc.still_window(rows)[0] == pytest.approx(102.5, abs=0.03)  # s


def test_a_car_that_never_stops_has_no_start():
    assert rmc.still_window(_rows((5.0, 1.0))) is None
    assert rmc.still_window([]) is None


def test_exactly_the_still_speed_counts_as_parked_and_just_over_does_not():
    assert rmc.still_window(_rows((2.0, rmc.STILL_SPEED))) is not None
    assert rmc.still_window(_rows((2.0, rmc.STILL_SPEED + 1e-6))) is None
    assert rmc.still_window(_rows((2.0, -rmc.STILL_SPEED - 1e-6))) is None


def test_nan_speed_is_not_parked():
    assert rmc.still_window(_rows((2.0, float('nan')))) is None


def _transform(parent, child):
    return SimpleNamespace(header=SimpleNamespace(frame_id=parent), child_frame_id=child)


def test_only_the_wheel_odometry_transform_is_played():
    # map -> odom is the recording's SLAM, odom_imu -> odom its IMU stack: the frames would have two parents.
    wheel = _transform('odom', 'base_link')
    kept = rmc.wheel_transforms([_transform('map', 'odom'), wheel, _transform('odom_imu', 'odom'),
                                 _transform('map', 'odom_imu')])
    assert kept == [wheel]
    assert rmc.wheel_transforms([_transform('map', 'odom')]) == []


def test_laps_are_read_from_the_nodes_own_words():
    text = ('x: Closed mapping lap 1/3 detected (51.2m, 18.0s)\n'
            'noise\nx: Closed mapping lap 2/3 detected (50.1m, 17.5s)\n')
    assert rmc.laps_from_log(text) == ({1: 51.2, 2: 50.1}, 3)


def test_no_closed_lap_reads_as_none():
    assert rmc.laps_from_log('Lap 2/3 closed; keep holding LB.') == ({}, None)
    assert rmc.laps_from_log('') == ({}, None)


def test_the_recording_folder_comes_from_the_log():
    text = 'stuff\nRecording run to: /some/where/20261030-185036\nmore\n'
    assert rmc.recording_from_log(text) == Path('/some/where/20261030-185036')
    assert rmc.recording_from_log('nothing here') is None


def _rebuild_json(tmp_path, **fields):
    path = tmp_path / 'rebuild.json'
    data = dict(verdict={'ok': True, 'reason': 'map OK'}, laps=3, raceline_ok=True, message='')
    data.update(fields)
    path.write_text(json.dumps(data))
    return path


def test_a_good_map_with_a_line_is_go(tmp_path):
    assert rmc.verdict_of(_rebuild_json(tmp_path)) == (True, 'map OK', 3)


def test_a_bad_map_is_not_go(tmp_path):
    path = _rebuild_json(tmp_path, verdict={'ok': False, 'reason': 'see-through 40%'}, laps=0,
                         raceline_ok=False, message='see-through 40%')
    assert rmc.verdict_of(path) == (False, 'see-through 40%', 0)


def test_a_bad_verdict_is_not_go_even_with_a_line(tmp_path):
    path = _rebuild_json(tmp_path, verdict={'ok': False, 'reason': 'see-through 40%'})
    assert rmc.verdict_of(path)[0] is False


def test_a_good_map_with_no_line_is_not_go(tmp_path):
    path = _rebuild_json(tmp_path, raceline_ok=False, laps=0, message='less than one whole lap')
    assert rmc.verdict_of(path) == (False, 'less than one whole lap', 0)


def test_no_rebuild_file_is_a_failure(tmp_path):
    go, why, laps = rmc.verdict_of(tmp_path / 'missing.json')
    assert go is False and laps == 0
    assert 'rebuild.json' in why


@pytest.mark.parametrize('marker', ['vesc_driver_node', 'urg_node_driver', 'gym_bridge_node',
                                    'async_slam_toolbox_node'])
def test_it_refuses_beside_anything_that_could_take_its_forged_lb(marker):
    # This publishes a held LB, so the refusal is the half that matters (A8).
    reason = rmc.refusal(['/usr/bin/python3 other', f'/opt/ros/jazzy/lib/x/{marker} --ros-args'])
    assert marker in reason
    assert 'refusing' in reason


def test_it_runs_when_nothing_is_up():
    assert rmc.refusal(['/usr/bin/python3 web_dashboard', 'foxglove_bridge', '']) is None


@pytest.mark.parametrize('argv', [
    ['run', '--out', 'o', '--', 'imu:=true', 'gyro_odom:=true'],
    ['--out', 'o', 'run', 'imu:=true', 'gyro_odom:=true'],
    ['run', '--out', 'o', 'imu:=true', 'gyro_odom:=true'],
])
def test_launch_arguments_reach_the_launch_wherever_they_are_written(argv):
    args = rmc.parse_arguments(argv)
    assert args.run_dir == Path('run')
    assert args.launch_args == ['imu:=true', 'gyro_odom:=true']


@pytest.mark.parametrize('argv', [['run', '--out', 'o', '--'], ['run', '--out', 'o']])
def test_no_launch_arguments_is_fine(argv):
    assert rmc.parse_arguments(argv).launch_args == []


def test_its_ros_domain_is_not_the_cars_or_the_test_suites():
    assert rmc.ROS_DOMAIN not in ('0', '79')


def test_the_environment_stays_on_this_machine_and_this_domain(tmp_path, monkeypatch):
    # The caller's shell set the other way, so the result cannot be inherited from it.
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET')
    monkeypatch.setenv('ROS_STATIC_PEERS', '192.168.0.99')
    env = rmc._environment(tmp_path, tmp_path / 'ws')
    assert env['ROS_DOMAIN_ID'] == rmc.ROS_DOMAIN
    assert env['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
    assert env['ROS_STATIC_PEERS'] == ''
    assert env['HOME'] == str(tmp_path)  # recordings land under the replay's folder, not data/
