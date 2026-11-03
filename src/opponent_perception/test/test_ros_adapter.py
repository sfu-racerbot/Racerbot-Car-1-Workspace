"""Real ROS transport and simulated-clock checks; no mocked ROS/vision modules.

Full suite: source Jazzy, set ROS_DOMAIN_ID=79 and
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST, then pytest this package's test/.
Offline-only: pytest test/ -m 'not ros'. ROS imports are delayed until fixture
setup so marker selection works without ROS; missing ROS never becomes a skip.
"""

import json
import os
import time
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import yaml

from opponent_perception.offline import make_demo

pytestmark = pytest.mark.ros


@pytest.fixture
def graph(tmp_path):
    if (os.environ.get('ROS_DOMAIN_ID') != '79'
            or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'):
        pytest.fail('ROS tests require isolated domain 79 and LOCALHOST discovery')
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import TransformStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster
    from visualization_msgs.msg import MarkerArray
    from opponent_perception.perception_node import PerceptionNode

    root = tmp_path / 'synthetic'
    make_demo(root)
    rclpy.init(args=['--ros-args', '-p', f'config_file:={root / "synthetic-config.yaml"}',
                     '-p', 'use_sim_time:=true',
                     '-r', '/perception/opponents:=/test_only/perception/opponents',
                     '-r', '/perception/status:=/test_only/perception/status',
                     '-r', '/perception/opponent_markers:=/test_only/perception/markers'])
    node = harness = executor = None
    try:
        node = PerceptionNode()
        harness = Node('perception_test_fixture')
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(harness)
        snapshots, health, markers = [], [], []
        harness.create_subscription(String, '/test_only/perception/opponents',
                                    lambda msg: snapshots.append(json.loads(msg.data)), 10)
        harness.create_subscription(String, '/test_only/perception/status',
                                    lambda msg: health.append(json.loads(msg.data)), 10)
        harness.create_subscription(MarkerArray, '/test_only/perception/markers',
                                    markers.append, 10)
        images = harness.create_publisher(Image, '/camera/camera/color/image_raw',
                                          qos_profile_sensor_data)
        infos = harness.create_publisher(CameraInfo, '/camera/camera/color/camera_info',
                                         qos_profile_sensor_data)
        clocks = harness.create_publisher(Clock, '/clock', 10)
        transforms = TransformBroadcaster(harness)
        bridge = CvBridge()
        frame = 'synthetic_camera_optical_frame'

        def pump(seconds=.1):
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=.01)

        def wait(predicate):
            until = time.monotonic() + 5
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=.01)
                if predicate():
                    return
            pytest.fail(f'ROS condition timed out; health={health[-2:]}')

        def advance(seconds):
            clocks.publish(Clock(clock=Time(seconds=seconds).to_msg()))
            wait(lambda: node.get_clock().now().nanoseconds == round(seconds*1e9))
            pump()

        manifest = [json.loads(line) for line in (root / 'frames.jsonl').read_text().splitlines()]
        calibration = yaml.safe_load((root / 'camera.yaml').read_text())

        def send(index=0, stamp=1.0, info_first=True, with_tf=True,
                 info_frame=None, static_info=False, transform_stamp=None):
            image = bridge.cv2_to_imgmsg(cv2.imread(str(root / manifest[index]['image']), 0),
                                         encoding='mono8')
            image.header.frame_id = frame
            image.header.stamp = Time(seconds=stamp).to_msg()
            info = CameraInfo()
            info.header.frame_id = info_frame or frame
            info.header.stamp = Time(seconds=0.0 if static_info else stamp).to_msg()
            info.width, info.height = calibration['width'], calibration['height']
            info.k = np.array(calibration['matrix'], dtype=float).ravel().tolist()
            info.d = calibration['distortion']
            info.distortion_model = 'plumb_bob'
            if with_tf:
                matrix = np.array(manifest[index]['tracking_from_camera'])
                # Derive quaternion via Rodrigues axis/angle, independent of our pose helper.
                rvec = cv2.Rodrigues(matrix[:3, :3])[0].ravel()
                angle = float(np.linalg.norm(rvec))
                xyz = rvec / angle * np.sin(angle/2)
                tf = TransformStamped()
                tf.header.frame_id, tf.child_frame_id = 'odom', frame
                tf.header.stamp = Time(
                    seconds=stamp if transform_stamp is None else transform_stamp).to_msg()
                tf.transform.translation.x, tf.transform.translation.y, \
                    tf.transform.translation.z = matrix[:3, 3].tolist()
                tf.transform.rotation.x, tf.transform.rotation.y, \
                    tf.transform.rotation.z = xyz.tolist()
                tf.transform.rotation.w = float(np.cos(angle/2))
                transforms.sendTransform(tf)
            for publisher, message in ((infos, info), (images, image)) if info_first else (
                    (images, image), (infos, info)):
                publisher.publish(message)
                pump(.05)

        wait(lambda: images.get_subscription_count() == 1
             and infos.get_subscription_count() == 1
             and clocks.get_subscription_count() >= 1)
        advance(1.0)
        yield SimpleNamespace(node=node, snapshots=snapshots, health=health,
                              markers=markers, send=send, advance=advance, wait=wait)
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if harness is not None:
            harness.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('info_first,static_info', [(True, False), (False, False), (True, True)])
def test_actual_image_info_tf_produce_world_pose(graph, info_first, static_info):
    # oracle: closed form renderer truth: stationary car at [3,0] in odom.
    graph.send(info_first=info_first, static_info=static_info)
    graph.advance(1.0625)
    graph.wait(lambda: bool(graph.snapshots[-1]['tracks']))
    track = graph.snapshots[-1]['tracks'][0]
    assert track['position_m'] == pytest.approx([3, 0], abs=.1)  # m rasterized tag
    assert track['last_seen_sec'] == 1.0
    assert graph.snapshots[-1]['frame_id'] == 'odom'
    assert track['velocity_valid'] is False


def test_missing_tf_never_falls_back_to_identity(graph):
    # oracle: spec README.md: missing timestamped transform produces no measurement.
    graph.send(with_tf=False)
    graph.advance(1.0625)
    graph.wait(lambda: graph.health[-1]['status'] == 'waiting_for_transform_at_image_time')
    assert graph.snapshots[-1]['tracks'] == []
    graph.advance(1.3125)
    graph.wait(lambda: graph.health[-1]['status'] == 'image_timestamp_invalid_or_stale')
    assert graph.snapshots[-1]['tracks'] == []


def test_mismatched_camera_frame_is_refused(graph):
    # oracle: spec README.md: calibration frame must match the image optical frame.
    graph.send(info_frame='wrong_optical_frame')
    graph.advance(1.0625)
    graph.wait(lambda: graph.health[-1]['status'] == 'unsupported_or_mismatched_camera_info')
    assert graph.snapshots[-1]['tracks'] == []


def test_old_tf_cannot_substitute_for_acquisition_time(graph):
    # oracle: spec README.md: older dynamic TF is not a transform at acquisition time.
    graph.send(transform_stamp=.5)
    graph.advance(1.0625)
    graph.wait(lambda: graph.health[-1]['status'] == 'waiting_for_transform_at_image_time')
    assert graph.snapshots[-1]['tracks'] == []


def test_stale_image_is_refused_even_with_matching_info_and_tf(graph):
    # oracle: spec README.md: images older than .25 s cannot add tracks.
    graph.send(stamp=.5)
    graph.advance(1.0625)
    graph.wait(lambda: graph.health[-1]['status'] == 'image_timestamp_invalid_or_stale')
    assert graph.snapshots[-1]['tracks'] == []


def test_aging_publications_and_marker_expiry_without_more_images(graph):
    # oracle: spec README.md: last_seen held; stale then expired, marker cleared.
    graph.send()
    graph.advance(1.0625)
    graph.wait(lambda: bool(graph.snapshots[-1]['tracks']))
    graph.advance(1.3125)
    graph.wait(lambda: graph.snapshots[-1]['stamp_sec'] == 1.3125)
    track = graph.snapshots[-1]['tracks'][0]
    assert track['fresh'] is False and track['age_sec'] == .3125
    graph.advance(1.8125)
    graph.wait(lambda: graph.snapshots[-1]['stamp_sec'] == 1.8125)
    assert graph.snapshots[-1]['tracks'] == []
    graph.wait(lambda: len(graph.markers[-1].markers) == 1)
    assert graph.markers[-1].markers[0].action == 3  # Marker.DELETEALL


def test_ros_clock_rewind_clears_tracks(graph):
    # oracle: invariant: a replay epoch cannot inherit an older epoch's objects.
    graph.send()
    graph.advance(1.0625)
    graph.wait(lambda: bool(graph.snapshots[-1]['tracks']))
    graph.advance(.5)
    graph.advance(.5625)
    graph.wait(lambda: graph.snapshots[-1]['stamp_sec'] == .5625)
    assert graph.snapshots[-1]['tracks'] == []
    assert graph.health[-1]['status'] == 'clock_reset'


def test_real_tf_ego_motion_preserves_stationary_track(graph):
    # oracle: independent synthetic renderer: car is stationary while camera moves.
    graph.send(index=0, stamp=1.0)
    graph.advance(1.0625)
    graph.wait(lambda: bool(graph.snapshots[-1]['tracks']))
    graph.send(index=10, stamp=1.5)
    graph.advance(1.5625)
    graph.wait(lambda: graph.snapshots[-1]['tracks'][0]['last_seen_sec'] == 1.5)
    track = graph.snapshots[-1]['tracks'][0]
    assert track['velocity_mps'] == pytest.approx([0, 0], abs=.3)  # m/s raster noise
    assert len(track['prediction']) == 11
    graph.wait(lambda: len(graph.markers[-1].markers) == 3)
    assert len(graph.markers[-1].markers[2].points) == 11


def test_process_shuts_down_cleanly_after_sigint(tmp_path):
    # oracle: spec: normal ROS shutdown after readiness must exit 0 without a traceback.
    import selectors
    import signal
    import subprocess
    import sys

    if (os.environ.get('ROS_DOMAIN_ID') != '79'
            or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'):
        pytest.fail('ROS tests require isolated domain 79 and LOCALHOST discovery')
    root = tmp_path / 'synthetic'
    make_demo(root)
    process = subprocess.Popen([
        sys.executable, '-c', 'from opponent_perception.perception_node import main; main()',
        '--ros-args', '-p', f'config_file:={root / "synthetic-config.yaml"}'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        env=dict(os.environ, PYTHONPATH=os.pathsep.join([
            str(Path(__file__).resolve().parents[1]), os.environ.get('PYTHONPATH', '')])))
    try:
        ready = False
        startup_lines = []
        with selectors.DefaultSelector() as selector:
            selector.register(process.stdout, selectors.EVENT_READ)
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline and process.poll() is None:
                if selector.select(timeout=.1):
                    line = process.stdout.readline()
                    startup_lines.append(line)
                    if 'opponent perception ready' in line:
                        ready = True
                        break
        assert ready, 'processing node never became ready: ' + ''.join(startup_lines)
        process.send_signal(signal.SIGINT)
        output = process.communicate(timeout=10)[0]
        assert process.returncode == 0, output
        assert 'Traceback' not in output
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5)
