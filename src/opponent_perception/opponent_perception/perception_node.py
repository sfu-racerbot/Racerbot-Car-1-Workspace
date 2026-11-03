"""ROS 2 adapter. Only /perception outputs; no driver or controller services."""

import json

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .configuration import load_config
from .detector import Camera
from .geometry import transform_from_pose
from .pipeline import Pipeline


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('opponent_perception_node')
        self.declare_parameter('config_file', '')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.pipeline = Pipeline(load_config(self.get_parameter('config_file').value))
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pending = None
        self.info = {}
        self.last_clock = None
        self.status = 'waiting_for_image_and_camera_info'
        self.publisher = self.create_publisher(String, '/perception/opponents', 10)
        self.markers = self.create_publisher(MarkerArray, '/perception/opponent_markers', 10)
        self.health = self.create_publisher(String, '/perception/status', 10)
        self.create_subscription(Image, self.get_parameter('image_topic').value,
                                 self._image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value,
                                 self._info, qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self._tick)

    @staticmethod
    def _stamp(header):
        return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec

    def _image(self, message):
        self.pending = message  # bounded: newest frame wins

    def _info(self, message):
        self.info[self._stamp(message.header)] = message
        # Bounded matching cache handles either image/CameraInfo arrival order.
        while len(self.info) > 8:
            del self.info[next(iter(self.info))]

    def _process_pending(self, now):
        image = self.pending
        if image is None:
            return
        stamp_ns = self._stamp(image.header)
        age = now.nanoseconds / 1e9 - stamp_ns / 1e9
        if age < 0 or age > self.pipeline.config.tracking.fresh_sec or stamp_ns <= 0:
            self.pending = None
            self.status = 'image_timestamp_invalid_or_stale'
            return
        info = self.info.get(stamp_ns, self.info.get(0))
        if info is None:
            self.status = 'waiting_for_matching_camera_info'
            return
        if (not image.header.frame_id or info.header.frame_id != image.header.frame_id
                or info.distortion_model not in ('plumb_bob', 'rational_polynomial')
                or (image.width, image.height) != (info.width, info.height)
                or info.binning_x not in (0, 1) or info.binning_y not in (0, 1)
                or info.roi.x_offset or info.roi.y_offset
                or info.roi.width not in (0, info.width)
                or info.roi.height not in (0, info.height)):
            self.pending = None
            self.status = 'unsupported_or_mismatched_camera_info'
            return
        try:
            tf = self.buffer.lookup_transform(
                self.pipeline.config.tracking_frame, image.header.frame_id,
                Time.from_msg(image.header.stamp), timeout=Duration(seconds=0.0))
        except TransformException:
            self.status = 'waiting_for_transform_at_image_time'
            return
        self.pending = None
        try:
            t, q = tf.transform.translation, tf.transform.rotation
            transform = transform_from_pose([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
            camera = Camera(info.width, info.height, np.array(info.k).reshape(3, 3), info.d)
            pixels = self.bridge.imgmsg_to_cv2(image, desired_encoding='mono8')
            self.pipeline.process(pixels, camera, stamp_ns / 1e9, transform)
            self.status = 'ok'
        except (ValueError, CvBridgeError, cv2.error) as error:
            self.status = f'rejected_image: {error}'

    def _tick(self):
        now = self.get_clock().now()
        if self.last_clock is not None and now.nanoseconds < self.last_clock:
            self.pipeline.tracker.clear()
            self.pending, self.info = None, {}
            self.status = 'clock_reset'
        self.last_clock = now.nanoseconds
        self._process_pending(now)
        now = self.get_clock().now()
        last_stamp = self.pipeline.tracker.last_stamp
        if last_stamp is not None and now.nanoseconds / 1e9 < last_stamp:
            self.pipeline.tracker.clear()
            self.status = 'clock_reset'
        snapshot = self.pipeline.tracker.snapshot(now.nanoseconds / 1e9)
        self.publisher.publish(String(data=json.dumps(snapshot, allow_nan=False)))
        self.health.publish(String(data=json.dumps({
            'schema_version': 1, 'stamp_sec': snapshot['stamp_sec'],
            'status': self.status, 'tracks': len(snapshot['tracks']),
            'last_image_age_sec': None if last_stamp is None else max(
                0.0, snapshot['stamp_sec'] - last_stamp)})))
        self.markers.publish(self._marker_array(snapshot, now))

    def _marker_array(self, snapshot, now):
        output = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        output.markers.append(clear)
        for index, track in enumerate(snapshot['tracks']):
            marker = Marker()
            marker.header.frame_id = snapshot['frame_id']
            marker.header.stamp = now.to_msg()
            marker.ns, marker.id = 'opponent', index
            marker.type, marker.action = Marker.SPHERE, Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.pose.position.x, marker.pose.position.y = track['position_m']
            marker.scale.x = marker.scale.y = marker.scale.z = 0.12
            marker.color.g = 1.0 if track['fresh'] else 0.3
            marker.color.r = 0.2 if track['fresh'] else 1.0
            marker.color.a = 1.0
            marker.lifetime = Duration(seconds=0.2).to_msg()
            output.markers.append(marker)
            path = Marker()
            path.header = marker.header
            path.ns, path.id = 'prediction', index
            path.type, path.action = Marker.LINE_STRIP, Marker.ADD
            path.pose.orientation.w = 1.0
            path.scale.x = 0.025
            path.color = marker.color
            path.lifetime = marker.lifetime
            path.points = [Point(x=p['position_m'][0], y=p['position_m'][1], z=0.0)
                           for p in track['prediction']]
            if len(path.points) >= 2:
                output.markers.append(path)
        return output


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        executor = SingleThreadedExecutor()
        node = PerceptionNode()
        executor.add_node(node)
        node.get_logger().info(
            f'opponent perception ready; fixed frame={node.pipeline.config.tracking_frame}')
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
