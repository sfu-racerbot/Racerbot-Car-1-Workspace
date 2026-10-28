"""The two-car race launch, run for real: sim_race_launch.py on its own ROS domain.

Every test starts the actual launch file and watches the live graph,
/sim/status and the referee's race.json. Run it sourced and on an isolated
domain (the car's graph is domain 0, and this must never see it):

    RB_WS="$(git rev-parse --show-toplevel)" ~/bin/rb env ROS_DOMAIN_ID=78 \\
        python3 -m pytest src/racerbot_sim/test/test_sim_race_launch.py -v

Each launch is stopped through tools/racerbot_sim/harness_procs.py, which only
signals the process group this test started. The launch-level refusals (bad
opponent, bad track) run in a second or two; the two races take ~30 s each.
"""

import contextlib
import importlib.util
import json
import math
import os
import re
import subprocess
import sys
import time

import pytest
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'tools', 'racerbot_sim'))
import harness_procs  # noqa: E402

SHIPPED_GAP_FOLLOW_YAML = os.path.join(ROOT, 'src', 'gap_follow', 'config', 'gap_follow.yaml')
# Our own nodes only: a leftover gym bridge or referee on this domain would
# publish a second /scan, /odom or /sim/status beside the race being run.
STRAY_PATTERNS = ('gym_bridge_node', 'race_referee_node', 'sim_joy_node')
# The referee's graph footprint. Anything else on its publishers is a bug.
REFEREE_TOPICS = {'/sim/race_result', '/rosout', '/parameter_events'}
STATUS_QOS = QoSProfile(depth=5000, reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def _isolated_domain():
    domain = os.environ.get('ROS_DOMAIN_ID')
    if domain in (None, '', '0') or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        pytest.fail('run with ROS_DOMAIN_ID=78 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST: domain 0 is '
                    "the car's graph, and a simulated race must never join it")


def _refuse_strays():
    try:
        harness_procs.refuse_if_strays(STRAY_PATTERNS)
    except SystemExit as refusal:
        pytest.fail(str(refusal))


def _tail(path, lines=60):
    with open(path, encoding='utf-8', errors='replace') as handle:
        return ''.join(handle.readlines()[-lines:])


@contextlib.contextmanager
def _race_launch(tmp_path, *arguments):
    """Start sim_race_launch.py, yield its process, and stop its group on exit."""
    _isolated_domain()
    _refuse_strays()
    log_path = tmp_path / 'launch.log'
    with open(log_path, 'w', encoding='utf-8') as log:
        proc = subprocess.Popen(
            ['ros2', 'launch', 'racerbot_sim', 'sim_race_launch.py', *arguments],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            yield proc
        finally:
            harness_procs.stop_group(proc)


class _Probe(Node):
    """Watches the graph and /sim/status. Subscribes only; publishes nothing."""

    def __init__(self):
        super().__init__('race_test_probe')
        self.status = []
        self.create_subscription(String, '/sim/status', self._on_status, STATUS_QOS)

    def _on_status(self, message):
        self.status.append(json.loads(message.data))


@pytest.fixture
def probe():
    _isolated_domain()
    rclpy.init()
    node = _Probe()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


class Graph:
    def __init__(self, node):
        self.node = node

    def spin_until(self, predicate, timeout_s):
        deadline = time.monotonic() + timeout_s
        while not predicate():
            if time.monotonic() >= deadline:
                return False
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return True

    def spin_for(self, seconds):
        self.spin_until(lambda: False, seconds)

    def publishers(self, topic):
        return sorted((e.node_namespace, e.node_name)
                      for e in self.node.get_publishers_info_by_topic(topic))

    def subscribers(self, topic):
        return sorted((e.node_namespace, e.node_name)
                      for e in self.node.get_subscriptions_info_by_topic(topic))


def _ros_param(node_name, parameter):
    done = subprocess.run(['ros2', 'param', 'get', node_name, parameter],
                          capture_output=True, text=True, timeout=60)
    match = re.search(r'value is: (.*)$', done.stdout, re.MULTILINE)
    assert match, f'ros2 param get {node_name} {parameter} said: {done.stdout!r} {done.stderr!r}'
    return match.group(1).strip()


def _parameters(probe, names):
    from rclpy.parameter_client import AsyncParameterClient
    client = AsyncParameterClient(probe, '/car2/gap_follow_node')
    assert client.wait_for_service(timeout_sec=10)
    future = client.get_parameters(names)
    rclpy.spin_until_future_complete(probe, future, timeout_sec=10)
    assert future.done(), 'namespaced node did not answer parameter request'
    return future.result().values


def _shipped_value(key):
    with open(SHIPPED_GAP_FOLLOW_YAML, encoding='utf-8') as handle:
        return yaml.safe_load(handle)['gap_follow_node']['ros__parameters'][key]


def _wired(graph):
    return (('/', 'gap_follow_node') in graph.publishers('/drive')
            and ('/car2', 'gap_follow_node') in graph.publishers('/car2/drive')
            and ('/car2', 'gap_follow_node') in graph.subscribers('/car2/joy')
            and ('/', 'race_referee_node') in graph.publishers('/sim/race_result'))


def test_short_race_keeps_each_car_in_its_own_namespace_and_finishes(tmp_path, probe):
    with _race_launch(tmp_path, 'track:=indoor_oval', 'opponent:=ours', 'laps:=0.2',
                      'record:=false', 'timeout_s:=60', f'output_dir:={tmp_path}') as proc:
        graph = Graph(probe)
        if not graph.spin_until(lambda: _wired(graph), 90):
            pytest.fail('the race never wired up. /drive publishers: '
                        f'{graph.publishers("/drive")}; /car2/drive publishers: '
                        f'{graph.publishers("/car2/drive")}\n{_tail(tmp_path / "launch.log")}')
        graph.spin_for(2.0)  # let any late publisher appear before judging the graph

        # Car 1 owns the root names; car 2 owns only its own namespace.
        assert graph.publishers('/drive') == [('/', 'gap_follow_node')]
        assert graph.publishers('/car2/drive') == [('/car2', 'gap_follow_node')]
        for topic in ('/drive', '/ackermann_cmd', '/joy'):
            assert all(namespace != '/car2' for namespace, _ in graph.publishers(topic)), (
                f'a /car2 node publishes {topic}: {graph.publishers(topic)}')
        # Car 2's control layer hears car 2's sensors and car 2's deadman, and nothing of car 1's.
        assert ('/car2', 'gap_follow_node') in graph.subscribers('/car2/scan')
        assert ('/car2', 'gap_follow_node') in graph.subscribers('/car2/joy')
        assert ('/car2', 'gap_follow_node') not in graph.subscribers('/scan')
        assert ('/car2', 'gap_follow_node') not in graph.subscribers('/joy')
        assert ('/', 'gap_follow_node') in graph.subscribers('/scan')
        assert ('/', 'gap_follow_node') in graph.subscribers('/joy')

        # The shipped YAML must reach the namespaced node, and its drive topic must be car 2's.
        drive, speed = _parameters(probe, ['drive_topic', 'max_speed'])
        assert drive.string_value == '/car2/drive'
        assert speed.double_value == _shipped_value('max_speed')

        # The referee's footprint on the graph is its result topic and nothing that drives or senses.
        referee_topics = {topic for topic, _types in
                          graph.node.get_publisher_names_and_types_by_node('race_referee_node', '/')}
        assert referee_topics <= REFEREE_TOPICS, referee_topics

        assert proc.wait(timeout=150) == 0, _tail(tmp_path / 'launch.log')

    race = json.loads((tmp_path / 'race.json').read_text(encoding='utf-8'))
    assert race['track'] == 'indoor_oval'
    assert race['race_laps'] == 0.2
    assert len(race['laps']) == 2
    assert all(isinstance(lap, float) and math.isfinite(lap) for lap in race['laps'])
    assert sorted(race['finish_order']) == [1, 2]
    assert all(t is not None for t in race['finish_time_s']), race['finish_time_s']


def test_car2_without_lb_stays_still_while_car1_drives(tmp_path, probe):
    with _race_launch(tmp_path, 'track:=indoor_oval', 'laps:=0.2', 'record:=false',
                      'car2_hold_deadman:=false', 'timeout_s:=20', f'output_dir:={tmp_path}') as proc:
        graph = Graph(probe)
        deadline = time.monotonic() + 150
        while proc.poll() is None and time.monotonic() < deadline:
            graph.spin_for(0.2)
        assert proc.poll() is not None, 'the race did not end within its time limit'
        assert proc.returncode == 0, _tail(tmp_path / 'launch.log')

    statuses = [s for s in probe.status if len(s.get('cars', [])) == 2]
    assert statuses, 'no /sim/status with both cars was seen'
    car1_speeds = [s['cars'][0]['speed'] for s in statuses]
    car2_speeds = [s['cars'][1]['speed'] for s in statuses]
    # Car 2 has no LB held, so its deadman refuses every drive command: it never moves.
    assert max(car2_speeds) < 0.05, f'car 2 moved without LB: peak {max(car2_speeds):.3f} m/s'
    # Car 1 holds LB and drives, so the car 2 result above is not a stalled simulator.
    assert max(car1_speeds) > 0.5, f'car 1 never got above 0.5 m/s: peak {max(car1_speeds):.3f}'
    race = json.loads((tmp_path / 'race.json').read_text(encoding='utf-8'))
    assert race['finish_time_s'][0] is not None, 'car 1 should have finished 0.2 laps'
    assert race['finish_time_s'][1] is None, 'car 2 cannot finish 0.2 laps while it is stopped'
    assert race['timed_out'] is True


@pytest.mark.parametrize('argument, expected', [
    ('opponent:=team_a', 'opponent:=team_a is added in Task R7'),
    ('opponent:=rival', "unknown opponent:='rival'"),
    ('track:=no_such_track', "track 'no_such_track'"),
])
def test_bad_arguments_fail_the_launch_with_a_clear_message(argument, expected):
    _isolated_domain()
    done = subprocess.run(['ros2', 'launch', 'racerbot_sim', 'sim_race_launch.py', argument],
                          capture_output=True, text=True, timeout=120)
    output = done.stdout + done.stderr
    assert done.returncode != 0, output[-2000:]
    assert expected in output, output[-2000:]
    if argument.startswith('track:='):
        assert os.path.join('tools', 'f1tenth_sim', 'maps') in output, output[-2000:]


def test_referee_creates_only_the_result_publisher():
    # Static companion to the runtime check above. The brief asks for this grep.
    path = os.path.join(os.path.dirname(__file__), '..', 'racerbot_sim', 'race_referee_node.py')
    with open(path, encoding='utf-8') as handle:
        source = handle.read()
    assert source.count('create_publisher(') == 1
    assert "'/sim/race_result'" in source
    for forbidden in ('/drive', '/ackermann_cmd', '/joy', '/scan', '/odom'):
        assert forbidden not in source


@pytest.mark.parametrize('check', ['no_writes', 'status_clock'])
def test_referee_scores_each_new_status_immediately_without_writing_tracks(tmp_path, check):
    from geometry_msgs.msg import PoseStamped
    from racerbot_sim.race_referee_node import RaceRefereeNode
    from racerbot_sim import tracks

    _isolated_domain()
    rclpy.init(args=['--ros-args', '-p', f'track_directory:={tmp_path}',
                     '-p', f'output_dir:={tmp_path / "results"}'])
    node = None
    try:
        node = RaceRefereeNode()
        # oracle: ruling C, referee startup never creates track files.
        if check == 'no_writes':
            assert sorted(p.name for p in tmp_path.iterdir()) == ['results']
            return
        line = tracks.rounded_rectangle_centerline(5.0, 3.2, 1.8)
        for i in range(2):
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = map(float, line[i * 10])
            node._pose(pose, i)
        def offer(t):
            node._status(String(data=json.dumps(dict(
                sim_time_s=t, cars=[dict(speed=1.0, wall_contact_now=False)] * 2,
                car_contact_now=False))))
        offer(0.025)
        # oracle: ruling B, the callback ticks immediately, even faster than wall time.
        assert node.referee.sim_time_s == 0.025
        offer(0.05)
        offer(0.05)
        offer(0.025)
        assert node.referee.sim_time_s == 0.05
        assert [len(t) for t in node.referee.progress_m] == [2, 2]
        deadline = time.monotonic() + 0.15
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert [len(t) for t in node.referee.progress_m] == [2, 2]
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def _load_launch(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('check', ['arguments', 'recorder'])
def test_recorder_inherits_default_topics_and_launch_exposes_timeout(check):
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    race = _load_launch(os.path.join(ROOT, 'src/racerbot_sim/launch/sim_race_launch.py'), 'race_launch_test')
    record = _load_launch(os.path.join(get_package_share_directory('race_diagnostics'),
                                    'launch/record_run.py'), 'record_launch_test')
    context = LaunchContext()
    declarations = [a for a in race.generate_launch_description().entities
                    if isinstance(a, DeclareLaunchArgument)]
    for action in declarations:
        action.execute(context)
    # oracle: amended launch interface includes timeout, excludes unsolicited offsets.
    if check == 'arguments':
        assert 'timeout_s' in context.launch_configurations
        assert 'start_offsets' not in context.launch_configurations
        return
    actions = race._setup(context)
    includes = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    assert len(includes) == 2  # bringup and recorder, as required by the launch contract
    recorder = includes[-1]
    topics = perform_substitutions(context, normalize_to_list_of_substitutions(
        dict(recorder.launch_arguments)['topics'])).split()
    # oracle: spec, every recorder default plus the specified second-car and truth topics.
    assert set(record.DEFAULT_TOPICS) <= set(topics)
    assert set('/car2/scan /car2/odom /car2/drive /car2/ackermann_cmd /car2/joy '
               '/car2/drive_intent /car2/teleop /car2/safety/brake /sim/ground_truth_pose '
               '/sim/car2/ground_truth_pose /sim/status /sim/race_result'.split()) <= set(topics)
