"""Tests for tools.run_analysis.slam_replay. ROS-free, so this runs unsourced.

Runner (TEST_QUALITY_STANDARDS.md A9):
    python3 -m pytest tools/run_analysis/tests/test_slam_replay.py -v -p no:cacheprovider
slam_replay._run starts ROS processes and needs a live graph, so it is not run here. It is
verified by a smoke replay, recorded in the task report.

Oracle markers (A3): "closed form" = recomputed from the spec in the test; "spec" = the task
brief's values or a file named by path; "run" = a value read from the recorded run
~/.ros/racerbot_runs/20261026-194514. The fixtures copy that run's file shapes, not its folder.

The process table is a fake /proc under tmp_path (PROC_ROOT) and nothing is written under the
real ~/.ros: RUNS_ROOT is redirected too. Popen is replaced with a trap wherever a test must
prove that nothing was started.
"""
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # insert tools/
from run_analysis import slam_replay  # noqa: E402
from run_analysis.slam_replay import ReplayRefused  # noqa: E402

RUN_NAME = '20261026-194514'
ALL_TOPICS = ['/scan', '/tf', '/tf_static', '/odom', '/odom/imu', '/map']
URG = b'/opt/ros/jazzy/lib/urg_node/urg_node_driver\x00--ros-args\x00-r\x00__node:=urg_node\x00'
VESC_PATH_ONLY = b'/home/x/install/vesc_driver/lib/vesc_driver/vesc_driver_node\x00'

# spec: the shape of ~/.ros/racerbot_runs/20261026-194514/metadata/params/slam_toolbox.yaml
# (a top key with a leading slash, ros__parameters below it), trimmed to the keys these tests read.
# The values are that run's: loop_search_maximum_distance 8.0, mode mapping, use_sim_time false.
SNAPSHOT = """\
/slam_toolbox:
  ros__parameters:
    loop_search_maximum_distance: 8.0
    mode: mapping
    odom_frame: odom_imu
    qos_overrides:
      /tf:
        publisher:
          depth: 100
    start_type_description_service: true
    use_sim_time: false
"""


def _bag_metadata(topics):
    """The layout of a rosbag2 metadata.yaml, as in <run>/bag/metadata.yaml of the recorded run."""
    return {'rosbag2_bagfile_information': {
        'version': 8,
        'storage_identifier': 'mcap',
        'topics_with_message_count': [
            {'topic_metadata': {'name': t, 'type': 'sensor_msgs/msg/LaserScan',
                                'serialization_format': 'cdr'},
             'message_count': 1}
            for t in topics],
    }}


def make_run(root: Path, topics=ALL_TOPICS, snapshot=SNAPSHOT) -> Path:
    """A complete fake run: a bag listing `topics`, and the recorded SLAM settings."""
    run = root / RUN_NAME
    (run / 'bag').mkdir(parents=True)
    (run / 'bag' / 'metadata.yaml').write_text(yaml.safe_dump(_bag_metadata(topics)), encoding='utf-8')
    params = run / 'metadata' / 'params'
    params.mkdir(parents=True)
    (params / 'slam_toolbox.yaml').write_text(snapshot, encoding='utf-8')
    return run


def _proc(root: Path, name: str, cmdline: bytes) -> None:
    """A fake /proc/<name>/cmdline."""
    (root / name).mkdir()
    (root / name / 'cmdline').write_bytes(cmdline)


# --- child_env ---------------------------------------------------------------------------------

def test_child_env_always_isolates():
    # spec: the task brief's values for the replay's own graph: domain 77, LOCALHOST discovery,
    # no static peers, whatever the caller had.
    base = {'ROS_DOMAIN_ID': '0', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET',
            'ROS_STATIC_PEERS': '10.0.0.5', 'PATH': '/x'}
    before = dict(base)
    env = slam_replay.child_env(base)
    assert slam_replay.REPLAY_DOMAIN_ID == 77  # spec: the task brief
    assert env['ROS_DOMAIN_ID'] == '77'
    assert env['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
    assert env['ROS_STATIC_PEERS'] == ''
    assert env['PATH'] == '/x'  # unrelated variables pass through
    assert base == before  # the caller's environment is not modified


def test_child_env_adds_the_isolation_to_an_empty_environment():
    # closed form - with no ROS variables at all, exactly the three are set.
    assert slam_replay.child_env({}) == {
        'ROS_DOMAIN_ID': '77', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST', 'ROS_STATIC_PEERS': ''}


# --- running_drivers (the interlock) -----------------------------------------------------------

@pytest.mark.parametrize('marker, cmdline', [
    ('vesc_driver_node', b'/home/x/install/vesc_driver/lib/vesc_driver/vesc_driver_node\x00'
                         b'--ros-args\x00-r\x00__node:=vesc_driver_node\x00'),
    ('urg_node', URG),
    ('joy_node', b'/opt/ros/jazzy/lib/joy/joy_node\x00'),
    ('__node:=joy', b'/opt/ros/jazzy/lib/joy/joy\x00--ros-args\x00-r\x00__node:=joy\x00'),
])
def test_running_drivers_finds_each_marker(tmp_path, marker, cmdline):
    # oracle: closed form - each cmdline names exactly one marker, so the answer is that marker.
    _proc(tmp_path, '4242', cmdline)
    assert slam_replay.running_drivers(tmp_path) == [marker]


def test_running_drivers_matches_executable_path_without_node_remap(tmp_path):
    # Review Focus 1: a driver started under another name (no __node:= remap) still trips the
    # interlock, because the match is on the whole command line.
    _proc(tmp_path, '4242', VESC_PATH_ONLY)
    assert slam_replay.running_drivers(tmp_path) == ['vesc_driver_node']


def test_running_drivers_skips_unreadable_and_vanished_pids(tmp_path):
    _proc(tmp_path, '7', URG)  # a live driver: it must still be found
    (tmp_path / '8').mkdir()  # no cmdline file: the process exited after it was listed
    _proc(tmp_path, '9', b'')  # empty cmdline: a kernel thread
    (tmp_path / 'sys').mkdir()  # not a pid directory, so it must not be read at all
    (tmp_path / 'sys' / 'cmdline').write_bytes(VESC_PATH_ONLY)
    assert slam_replay.running_drivers(tmp_path) == ['urg_node']


def test_running_drivers_reads_a_command_line_that_is_not_utf8(tmp_path):
    # A4 parser edge: an invalid byte in one argument must not hide a driver.
    _proc(tmp_path, '7', b'/x/\xff\xfe/urg_node\x00--ros-args\x00')
    assert slam_replay.running_drivers(tmp_path) == ['urg_node']


def test_running_drivers_ignores_processes_that_are_not_drivers(tmp_path):
    # invariant - a recorder that names /joy is not a driver, and neither is a shell or systemd.
    _proc(tmp_path, '10', b'/usr/bin/python3\x00-m\x00pytest\x00')
    _proc(tmp_path, '11', b'/usr/lib/systemd/systemd\x00--user\x00')
    _proc(tmp_path, '12', b'/opt/ros/jazzy/bin/ros2\x00bag\x00record\x00-o\x00/tmp/x/bag\x00/joy\x00/tf\x00')
    assert slam_replay.running_drivers(tmp_path) == []


def test_running_drivers_with_no_processes_is_empty(tmp_path):
    (tmp_path / 'proc').mkdir()
    assert slam_replay.running_drivers(tmp_path / 'proc') == []


# --- slam_parameters ---------------------------------------------------------------------------

SNAP = {'loop_search_maximum_distance': 8.0, 'odom_frame': 'odom', 'use_sim_time': False,
        'qos_overrides./tf.publisher.depth': 10}


def test_slam_parameters_layer_order():
    # closed form - the profile lays over the snapshot (3.0 beats 8.0), the forced keys lay over
    # both (odom_imu, true), and the qos_overrides key is dropped.
    params = slam_replay.slam_parameters(SNAP, 'odom_imu', {'loop_search_maximum_distance': 3.0})
    assert params == {'slam_toolbox': {'ros__parameters': {
        'loop_search_maximum_distance': 3.0, 'odom_frame': 'odom_imu', 'use_sim_time': True}}}
    assert params['slam_toolbox']['ros__parameters']['use_sim_time'] is True


def test_slam_parameters_forced_keys_beat_the_profile():
    # invariant - a profile cannot turn the replay's sim time off or move it to another frame.
    ros = slam_replay.slam_parameters({}, 'odom_imu', {'use_sim_time': False, 'odom_frame': 'odom'})
    ros = ros['slam_toolbox']['ros__parameters']
    assert ros['use_sim_time'] is True
    assert ros['odom_frame'] == 'odom_imu'


def test_slam_parameters_drops_the_live_graph_keys_from_both_layers():
    # closed form - of the snapshot and the profile, only `mode` survives, plus the forced keys.
    snap = {'start_type_description_service': True, 'mode': 'mapping',
            'qos_overrides': {'/tf': {'publisher': {'depth': 100}}}}
    prof = {'qos_overrides./tf.publisher.depth': 5, 'start_type_description_service': False}
    ros = slam_replay.slam_parameters(snap, 'odom', prof)['slam_toolbox']['ros__parameters']
    assert ros == {'mode': 'mapping', 'odom_frame': 'odom', 'use_sim_time': True}


def test_slam_parameters_without_a_profile_keeps_the_recorded_values():
    ros = slam_replay.slam_parameters(SNAP, 'odom', None)['slam_toolbox']['ros__parameters']
    assert ros['loop_search_maximum_distance'] == 8.0  # the recorded value, unchanged


def test_slam_parameters_does_not_modify_its_inputs():
    snap, prof = dict(SNAP), {'loop_search_maximum_distance': 3.0}
    slam_replay.slam_parameters(snap, 'odom_imu', prof)
    assert snap == SNAP
    assert prof == {'loop_search_maximum_distance': 3.0}


def test_params_file_leaves_out_only_nulls_and_empty_lists():
    # The smoke replay of 20261026-194514 aborted on map_start_pose: [] ("parameter_value_from failed
    # for parameter 'map_start_pose': No parameter value set", in its slam_toolbox.log). Values that
    # look empty or false are valid parameters and must stay: '', 0 and False.
    ros = {'map_start_pose': [], 'map_file_name': '', 'debug_logging': False, 'throttle_scans': 0,
           'start_pose': [0.0, 0.0, 0.0], 'enable_interactive_mode': None}
    kept = slam_replay.loadable({'slam_toolbox': {'ros__parameters': ros}})
    assert kept == {'slam_toolbox': {'ros__parameters': {
        'map_file_name': '', 'debug_logging': False, 'throttle_scans': 0, 'start_pose': [0.0, 0.0, 0.0]}}}


# --- bag_topics ----------------------------------------------------------------------------------

def test_bag_topics_reads_the_names_from_a_rosbag2_metadata_file(tmp_path):
    # spec: the layout of <run>/bag/metadata.yaml. The names are the ones the fixture wrote.
    bag = tmp_path / 'bag'
    bag.mkdir()
    (bag / 'metadata.yaml').write_text(yaml.safe_dump(_bag_metadata(['/scan', '/odom/imu', '/tf_static'])))
    assert slam_replay.bag_topics(bag) == {'/scan', '/odom/imu', '/tf_static'}


def test_bag_topics_refuses_a_truncated_metadata_file(tmp_path):
    bag = tmp_path / 'bag'
    bag.mkdir()
    (bag / 'metadata.yaml').write_text('rosbag2_bagfile_information: {topics_with_message_count: [\n')
    with pytest.raises(ReplayRefused, match='unreadable'):
        slam_replay.bag_topics(bag)


def test_bag_topics_refuses_bytes_that_are_not_utf8(tmp_path):
    bag = tmp_path / 'bag'
    bag.mkdir()
    (bag / 'metadata.yaml').write_bytes(b'\xff\xfe\x00rosbag2')
    with pytest.raises(ReplayRefused, match='unreadable'):
        slam_replay.bag_topics(bag)


def test_bag_topics_refuses_a_file_with_no_topic_list(tmp_path):
    bag = tmp_path / 'bag'
    bag.mkdir()
    (bag / 'metadata.yaml').write_text('rosbag2_bagfile_information:\n  version: 8\n')
    with pytest.raises(ReplayRefused, match='topic list'):
        slam_replay.bag_topics(bag)


# --- plan_replay ----------------------------------------------------------------------------------

def test_plan_never_plays_recorded_tf(tmp_path):
    # spec: the task brief - /scan, /tf_static and the odometry topic; the recorded /tf is never played.
    run = make_run(tmp_path, topics=['/scan', '/tf', '/tf_static', '/odom/imu'])
    plan = slam_replay.plan_replay(run, 'odom_imu', None, tmp_path / 'out')
    assert plan.play_topics == ['/scan', '/tf_static', '/odom/imu']
    assert '/tf' not in plan.play_topics
    assert plan.odom_topic == '/odom/imu'


def test_plan_refuses_a_run_without_the_odometry_topic(tmp_path):
    # Review Focus 2: a gyro-off run has no /odom/imu. Refuse before anything is started, and
    # name the missing topic.
    run = make_run(tmp_path, topics=['/scan', '/odom', '/tf_static'])
    with pytest.raises(ReplayRefused, match='/odom/imu'):
        slam_replay.plan_replay(run, 'odom_imu', None, tmp_path / 'out')
    assert not (tmp_path / 'out').exists()  # nothing was created


@pytest.mark.parametrize('missing', ['/scan', '/tf_static'])
def test_plan_refuses_a_run_without_scans_or_static_transforms(tmp_path, missing):
    # The replay cannot localise without the scans, or without base_link -> laser in /tf_static.
    run = make_run(tmp_path, topics=[t for t in ALL_TOPICS if t != missing])
    with pytest.raises(ReplayRefused, match=missing):
        slam_replay.plan_replay(run, 'odom', None, tmp_path / 'out')


def test_plan_refuses_a_run_with_no_bag(tmp_path):
    run = make_run(tmp_path)
    shutil.rmtree(run / 'bag')
    with pytest.raises(ReplayRefused, match='has no bag/metadata.yaml'):
        slam_replay.plan_replay(run, 'odom', None, tmp_path / 'out')


def test_plan_refuses_a_run_without_its_recorded_slam_settings(tmp_path):
    run = make_run(tmp_path)
    (run / 'metadata' / 'params' / 'slam_toolbox.yaml').unlink()
    with pytest.raises(ReplayRefused, match='has no metadata/params/slam_toolbox.yaml'):
        slam_replay.plan_replay(run, 'odom', None, tmp_path / 'out')


def test_plan_names_the_output_folder_after_the_run_odometry_and_label(tmp_path):
    # spec: the task brief - <run name>-replay-<odom>-<label>, with the label defaulting to "default".
    run = make_run(tmp_path)
    out_root = tmp_path / 'runs'
    plan = slam_replay.plan_replay(run, 'odom_imu', None, out_root, label='smoke')
    assert plan.out_dir == out_root / f'{RUN_NAME}-replay-odom_imu-smoke'
    default = slam_replay.plan_replay(run, 'odom', None, out_root)
    assert default.out_dir == out_root / f'{RUN_NAME}-replay-odom-default'


def test_plan_refuses_an_output_folder_that_already_exists(tmp_path):
    run = make_run(tmp_path)
    out_root = tmp_path / 'runs'
    (out_root / f'{RUN_NAME}-replay-odom-smoke').mkdir(parents=True)
    with pytest.raises(ReplayRefused, match='already exists'):
        slam_replay.plan_replay(run, 'odom', None, out_root, label='smoke')


@pytest.mark.parametrize('label', ['', 'a/b', '../up', 'two words'])
def test_plan_refuses_a_label_that_is_not_a_plain_folder_name(tmp_path, label):
    run = make_run(tmp_path)
    with pytest.raises(ReplayRefused, match='label'):
        slam_replay.plan_replay(run, 'odom', None, tmp_path / 'out', label=label)


def test_plan_refuses_an_unknown_odometry_frame(tmp_path):
    run = make_run(tmp_path)
    with pytest.raises(ReplayRefused, match='unknown odometry frame'):
        slam_replay.plan_replay(run, 'odom_gps', None, tmp_path / 'out')


def test_plan_reads_the_recorded_settings_and_forces_sim_time(tmp_path):
    # oracle: run - loop_search_maximum_distance 8.0 is the value in 20261026-194514's snapshot.
    # The replay forces use_sim_time and the odometry frame, whatever was recorded.
    run = make_run(tmp_path)
    plan = slam_replay.plan_replay(run, 'odom_imu', None, tmp_path / 'out')
    ros = plan.params['slam_toolbox']['ros__parameters']
    assert ros['loop_search_maximum_distance'] == 8.0
    assert ros['mode'] == 'mapping'
    assert ros['use_sim_time'] is True
    assert ros['odom_frame'] == 'odom_imu'
    assert 'qos_overrides' not in ros
    assert 'start_type_description_service' not in ros


PROFILE_FLAT = 'loop_search_maximum_distance: 3.0\n'
PROFILE_WRAPPED = '/slam_toolbox:\n  ros__parameters:\n    loop_search_maximum_distance: 3.0\n'


@pytest.mark.parametrize('text', [PROFILE_FLAT, PROFILE_WRAPPED])
def test_profile_flat_and_recorded_shapes_give_the_same_overrides(tmp_path, text):
    # oracle: closed form - the 3.0 written in the profile is the value the plan must carry.
    run = make_run(tmp_path)
    profile = tmp_path / 'profile.yaml'
    profile.write_text(text, encoding='utf-8')
    plan = slam_replay.plan_replay(run, 'odom_imu', profile, tmp_path / 'out')
    assert plan.profile == {'loop_search_maximum_distance': 3.0}
    assert plan.params['slam_toolbox']['ros__parameters']['loop_search_maximum_distance'] == 3.0


@pytest.mark.parametrize('text, message', [
    ('- 1\n- 2\n', 'profile .* is not a parameter mapping'),  # a list
    ('just a string\n', 'profile .* is not a parameter mapping'),  # a scalar
    ('loop_search_maximum_distance: [3.0\n', 'profile .* is unreadable'),  # truncated YAML
])
def test_profile_that_is_not_a_parameter_mapping_is_refused(tmp_path, text, message):
    run = make_run(tmp_path)
    profile = tmp_path / 'profile.yaml'
    profile.write_text(text, encoding='utf-8')
    with pytest.raises(ReplayRefused, match=message):
        slam_replay.plan_replay(run, 'odom', profile, tmp_path / 'out')


# --- main: the refusals (the orchestration needs a live ROS graph and is smoke-tested) ----------

def _trap_popen(monkeypatch):
    """Any child process started is a failure: this stands in for the process boundary."""
    def trap(*args, **kwargs):
        raise AssertionError(f'a child process was started: {args[0]!r}')
    monkeypatch.setattr(subprocess, 'Popen', trap)


def test_main_refuses_while_drivers_run(tmp_path, monkeypatch, capsys):
    # The brief's test, on a fake /proc that holds a live urg_node. The run itself is valid, so
    # without the interlock main would go on to start processes, and the trap would fail the test.
    proc = tmp_path / 'proc'
    proc.mkdir()
    _proc(proc, '4242', URG)
    monkeypatch.setattr(slam_replay, 'PROC_ROOT', proc)
    monkeypatch.setattr(slam_replay, 'RUNS_ROOT', tmp_path / 'racerbot_runs')
    _trap_popen(monkeypatch)
    run = make_run(tmp_path / 'src')
    assert slam_replay.main([str(run), '--odom', 'odom_imu']) == 2
    assert 'urg_node' in capsys.readouterr().err
    assert not (tmp_path / 'racerbot_runs').exists()


def test_main_refuses_a_run_that_cannot_be_replayed(tmp_path, monkeypatch, capsys):
    # With no driver running, a run without /odom/imu is refused with code 2, and no folder is made.
    monkeypatch.setattr(slam_replay, 'PROC_ROOT', tmp_path / 'empty_proc')
    (tmp_path / 'empty_proc').mkdir()
    monkeypatch.setattr(slam_replay, 'RUNS_ROOT', tmp_path / 'racerbot_runs')
    _trap_popen(monkeypatch)
    run = make_run(tmp_path / 'src', topics=['/scan', '/tf_static', '/odom'])
    assert slam_replay.main([str(run), '--odom', 'odom_imu']) == 2
    assert '/odom/imu' in capsys.readouterr().err
    assert not (tmp_path / 'racerbot_runs').exists()


@pytest.mark.parametrize('rate', ['0', '-1', 'nan', 'fast'])
def test_main_rejects_a_rate_that_is_not_a_positive_number(rate):
    with pytest.raises(SystemExit) as exc:
        slam_replay.main(['unused-run-dir', '--rate', rate])
    assert exc.value.code == 2
