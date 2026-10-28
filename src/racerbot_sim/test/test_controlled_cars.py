"""Multi-car bridge behavior; run with pytest in an isolated ROS domain."""

import json
import math
import os
import subprocess
import sys
import time

import numpy as np
import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from racerbot_sim import sim_bridge as sim, tracks  # noqa: E402


def test_car_names_and_invalid_numbers():
    # oracle: specified ROS names, including frames and scoring topic.
    assert sim.car_names(1) == dict(namespace='', scan='/scan', odom='/odom',
        drive='/ackermann_cmd', joy='/joy', odom_frame='odom',
        base_frame='base_link', laser_frame='laser', truth='/sim/ground_truth_pose')
    assert sim.car_names(2) == dict(namespace='car2', scan='/car2/scan',
        odom='/car2/odom', drive='/car2/ackermann_cmd', joy='/car2/joy',
        odom_frame='car2/odom', base_frame='car2/base_link',
        laser_frame='car2/laser', truth='/sim/car2/ground_truth_pose')
    for value in (0, -1, 1.5, '2'):
        with pytest.raises(ValueError):
            sim.car_names(value)


def test_command_freshness_boundary():
    # oracle: timeout is inclusive, and an absent command is never fresh.
    assert not sim.command_is_fresh(None, 1.0, 0.2)
    assert sim.command_is_fresh(0.0, 0.2, 0.2)
    assert not sim.command_is_fresh(0.0, 0.201, 0.2)
    assert sim.command_is_fresh(1.0, 1.0, 0.2)


@pytest.fixture
def make_bridge(tmp_path):
    sim.add_sim_python_path()
    try:
        import f1tenth_gym  # noqa: F401
    except ModuleNotFoundError as exc:
        if exc.name != 'f1tenth_gym':
            raise
        pytest.skip('needs the gym checkout at .sim/ (docs/ros-simulator.md); runs wherever racerbot_sim runs')
    track = tracks.build('indoor_oval', str(tmp_path))
    centerline = tracks.load_centerline(str(tmp_path), 'indoor_oval')
    bridges = []

    def make(**kwargs):
        config = sim.SimConfig(track_path=str(track), centerline=centerline,
                               odom_speed_noise_std=0.0, **kwargs)
        bridge = sim.SimBridge(config)
        bridges.append(bridge)
        return bridge

    yield make
    for bridge in bridges:
        bridge.close()


@pytest.mark.parametrize('moving', [0, 1])
def test_independent_motion_and_odometry(make_bridge, moving):
    b = make_bridge(controlled_cars=2)
    before = [b.agent_state(i) for i in range(2)]
    odom = [(b.odometry(i).x, b.odometry(i).y) for i in range(2)]
    # oracle: the starting offsets differ by 3 m along the straight.
    assert math.hypot(before[0].x-before[1].x, before[0].y-before[1].y) == pytest.approx(3.0, abs=0.05)  # m
    commands = [(0.0, 0.0), (0.0, 0.0)]
    commands[moving] = (0.0, 1.0)
    for _ in range(40):
        b.step_cars(commands)
    distances = [math.hypot(b.agent_state(i).x-before[i].x,
                            b.agent_state(i).y-before[i].y) for i in range(2)]
    odo_distances = [math.hypot(b.odometry(i).x-odom[i][0],
                                b.odometry(i).y-odom[i][1]) for i in range(2)]
    # oracle: zero speed stays put; a forward command moves its own car.
    assert distances[moving] > 0.3  # m
    assert distances[1-moving] < 0.001  # m
    assert odo_distances[moving] > 0.3  # m
    assert odo_distances[1-moving] < 0.001  # m


def test_car_scan_raycast_sees_other_car(make_bridge):
    b = make_bridge(controlled_cars=2)
    # oracle: bottom straight of indoor_oval runs along +x at y=-3.2 m.
    b.reset(poses=[[2.0, -3.2, 0.0], [0.0, -3.2, 0.0]])
    beam = sim.LIDAR_BEAMS // 2
    # oracle: ray meets the centred other box's rear face, measured from LiDAR.
    assert b.scan(1)[beam] == pytest.approx(2.0-sim.LIDAR_OFFSET_X-sim.CAR_LENGTH/2, abs=0.05)  # m, 5 sigma
    b.reset(poses=[[-2.0, -3.2, 0.0], [0.0, -3.2, 0.0]])
    assert b.scan(1)[beam] > 3.0  # m, wall beyond former obstacle


def test_gym_collision_response_control_and_disabled(make_bridge):
    from f1tenth_gym.envs.simulator import F110Simulator
    b = make_bridge(controlled_cars=2)
    yaw = 0.001
    poses = [[0.0, -3.2, yaw], [-0.6*math.cos(yaw), -3.2-0.6*math.sin(yaw), yaw]]
    # oracle: gym's own class default activates its collision response.
    b.reset(poses=poses)
    b.env.unwrapped.sim.ttc_threshold = F110Simulator.ttc_threshold
    flagged = snapped = False
    for _ in range(16):
        b.step_cars([(0.0, 0.0), (0.0, 0.1)])
        assert not b.car_contact()
        flagged |= b.agent_state(1).collision
        snapped |= abs(b.agent_state(1).yaw) < 1e-6
    assert flagged and snapped
    b.reset(poses=poses)
    for _ in range(16):
        b.step_cars([(0.0, 0.0), (0.0, 0.1)])
        # oracle: straight steering gives zero yaw rate; no collision response.
        assert not b.car_contact()
        for i in range(2):
            assert b.agent_state(i).yaw == pytest.approx(yaw, abs=1e-6)  # rad
            assert not b.agent_state(i).collision


def test_single_car_step_compatibility_and_multi_car_rejection(make_bridge):
    a = make_bridge(controlled_cars=1)
    b = make_bridge(controlled_cars=1)
    for _ in range(40):
        a.step(0.1, 1.0)
        b.step_cars([(0.1, 1.0)])
        # oracle: both APIs apply identical commands to the same plant.
        assert vars(a.agent_state(0)) == vars(b.agent_state(0))
    two = make_bridge(controlled_cars=2)
    with pytest.raises(ValueError, match='step_cars'):
        two.step(0.0, 1.0)


def _spin_until(node, predicate, seconds, tick=None):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if tick:
            tick()
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return True
    return False


@pytest.fixture
def ros_context():
    rclpy.init()
    node = rclpy.create_node('controlled_cars_test')
    processes = []
    def start(executable, *args):
        p = subprocess.Popen(['ros2', 'run', 'racerbot_sim', executable,
                              '--ros-args', *args], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL, start_new_session=True)
        processes.append(p)
        return p
    yield node, start
    for process in processes:
        process.terminate()
    for process in processes:
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=10)
    node.destroy_node()
    rclpy.shutdown()


def test_interlock_suppresses_both_cars_and_namespaced_joy(ros_context):
    node, start = ros_context
    hardware = rclpy.create_node('urg_node')
    received = {topic: 0 for topic in ['/scan', '/car2/scan', '/odom', '/car2/odom', '/car2/joy']}
    def callback(topic):
        def record(_):
            received[topic] += 1
        return record
    for topic in received:
        kind = Joy if topic.endswith('joy') else Odometry if topic.endswith('odom') else LaserScan
        node.create_subscription(kind, topic, callback(topic), 10)
    try:
        start('gym_bridge_node', '-p', 'controlled_cars:=2')
        start('sim_joy_node', '-r', '__ns:=/car2', '-p', 'joy_topic:=/car2/joy')
        # oracle: active hardware suppresses all synthetic sensor and joy output.
        assert not _spin_until(node, lambda: any(received.values()), 3.0)
    finally:
        hardware.destroy_node()
    # oracle: after the hardware disappears both scans resume.
    assert _spin_until(node, lambda: received['/scan'] > 0 and received['/car2/scan'] > 0, 10.0)


def test_silent_car_stops_while_other_car_drives(ros_context):
    node, start = ros_context
    status = []
    node.create_subscription(String, '/sim/status', lambda m: status.append(json.loads(m.data)), 10)
    publishers = [node.create_publisher(AckermannDriveStamped, topic, 10)
                  for topic in ('/ackermann_cmd', '/car2/ackermann_cmd')]
    start('gym_bridge_node', '-p', 'controlled_cars:=2')
    assert _spin_until(node, lambda: bool(status), 20.0)
    command = AckermannDriveStamped()
    command.drive.speed = 1.0
    def both():
        for pub in publishers:
            pub.publish(command)
    # oracle: each input drives its own plant when refreshed.
    assert _spin_until(node, lambda: bool(status and len(status[-1].get('cars', [])) == 2 and status[-1]['cars'][1]['speed'] > 0.2), 5.0, both)
    assert not _spin_until(node, lambda: False, 1.0, both)
    def first():
        publishers[0].publish(command)
    # oracle: stale car 2 decelerates while the fresh car 1 keeps moving.
    assert _spin_until(node, lambda: len(status[-1].get('cars', [])) == 2 and not status[-1]['cars'][1]['commanded']['fresh'] and abs(status[-1]['cars'][1]['speed']) < 0.05 and status[-1]['cars'][0]['speed'] > 0.2, 3.0, first)
