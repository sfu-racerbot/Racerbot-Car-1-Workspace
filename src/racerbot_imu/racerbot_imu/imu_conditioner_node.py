"""imu_conditioner_node: one raw IMU stream -> SI, base_link axes, bias removed.

Run one instance per IMU (imu_name: vesc, front). Publishes

    /imu/<imu_name>/data     sensor_msgs/Imu, SI units, frame base_link
    /imu/<imu_name>/status   std_msgs/String JSON {state, reason, bias, rate_hz, saturated}

and publishes NO data until a stationary window has given a gyro bias and
passed the gravity check. On any fault it stops publishing data rather than
publishing zeros: a consumer then sees a stale stream and refuses it, instead
of reading "not turning". Publishes nothing that can move the car.
"""
import json
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from racerbot_imu import imu_math


class ImuConditionerNode(Node):

    def __init__(self):
        super().__init__('imu_conditioner_node')
        p = self.declare_parameter
        p('imu_name', 'vesc')
        p('input_topic', '/sensors/imu/raw')
        p('odom_topic', '/odom')
        p('input_gyro_scale', imu_math.VESC_GYRO_SCALE)
        p('input_accel_scale', imu_math.VESC_ACCEL_SCALE)
        p('r_base_imu', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        p('stationary_window_sec', 2.0)
        p('gravity_tolerance', 0.5)
        p('stale_timeout_sec', 0.1)
        p('odom_timeout_sec', 0.2)
        p('gyro_saturation', 34.9)          # rad/s (2000 deg/s); TODO(verify) per chip
        p('accel_saturation', 156.9)        # m/s^2 (16 g); TODO(verify) per chip
        v = lambda n: self.get_parameter(n).value   # noqa: E731

        self.imu_name = v('imu_name')
        self.gyro_scale = float(v('input_gyro_scale'))
        self.accel_scale = float(v('input_accel_scale'))
        self.gravity_tol = float(v('gravity_tolerance'))
        self.stale_timeout = float(v('stale_timeout_sec'))
        self.odom_timeout = float(v('odom_timeout_sec'))
        self.gyro_limit = float(v('gyro_saturation'))
        self.accel_limit = float(v('accel_saturation'))
        self.bias = imu_math.GyroBiasEstimator(window_sec=float(v('stationary_window_sec')))
        try:
            self.r = imu_math.check_rotation(v('r_base_imu'))
            self.config_error = None
        except imu_math.ImuSampleError as exc:
            self.r = np.eye(3)
            self.config_error = str(exc)

        self.state, self.reason = ('fault', self.config_error) if self.config_error else ('waiting', 'no bias yet')
        self.gravity_ok = False
        self.gravity_failed = False       # latched until restart
        self.saturated_count = 0
        self.odom_speed = math.nan
        self.last_odom_time = None
        self.last_raw_time = None
        self.sample_times = []

        base = f'/imu/{self.imu_name}'
        self.data_pub = self.create_publisher(Imu, f'{base}/data', 20)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, f'{base}/status', latched)
        self.create_subscription(Imu, v('input_topic'), self.raw_callback, 20)
        self.create_subscription(Odometry, v('odom_topic'), self.odom_callback, 10)
        self.create_timer(0.05, self.watchdog)
        self._published_status = None
        self.publish_status(force=True)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def odom_callback(self, msg: Odometry):
        self.odom_speed = float(msg.twist.twist.linear.x)
        self.last_odom_time = self._now()

    def _set(self, state: str, reason):
        if (state, reason) != (self.state, self.reason):
            self.state, self.reason = state, reason
            self.publish_status(force=True)

    def raw_callback(self, msg: Imu):
        if self.config_error or self.gravity_failed:
            return
        now = self._now()
        self.last_raw_time = now
        self.sample_times = [t for t in self.sample_times if t > now - 1.0] + [now]
        g_raw = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        a_raw = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        try:
            imu_math.validate(g_raw, a_raw)
        except imu_math.ImuSampleError as exc:
            return self._set('fault', str(exc))
        g, a = imu_math.to_si(g_raw, a_raw, self.gyro_scale, self.accel_scale)
        if imu_math.saturated(g, a, self.gyro_limit, self.accel_limit):
            self.saturated_count += 1
            return                                   # a gap, exactly like a dropout
        g, a = self.r @ g, self.r @ a
        odom_fresh = self.last_odom_time is not None and now - self.last_odom_time < self.odom_timeout
        if self.bias.update(now, g, a, self.odom_speed if odom_fresh else math.nan):
            try:
                imu_math.gravity_check(self.bias.last_window_accel, self.gravity_tol)
                self.gravity_ok = True
            except imu_math.ImuSampleError as exc:
                # Wrong units or a wrong mount rotation: nothing later can fix
                # that, so the fault latches until the node is restarted.
                self.gravity_ok, self.gravity_failed = False, True
                return self._set('fault', str(exc))
        if self.bias.bias is None or not self.gravity_ok:
            return self._set('waiting', 'no stationary window yet (keep the car still for 2 s)')
        self._set('ok', None)
        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'base_link'
        gb = g - self.bias.bias
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = map(float, gb)
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = map(float, a)
        out.orientation_covariance[0] = -1.0         # orientation not provided (REP-145)
        self.data_pub.publish(out)

    def watchdog(self):
        if self.config_error or self.gravity_failed:
            return
        now = self._now()
        if self.last_raw_time is None or now - self.last_raw_time > self.stale_timeout:
            self._set('fault', f"input stale (nothing on the raw topic for {self.stale_timeout:.2f} s)")
        self.publish_status()

    def publish_status(self, force: bool = False):
        rate = len(self.sample_times)
        status = {'state': self.state, 'reason': self.reason, 'imu': self.imu_name,
                  'bias': None if self.bias.bias is None else [round(float(b), 5) for b in self.bias.bias],
                  'rate_hz': rate, 'saturated': self.saturated_count}
        text = json.dumps(status)
        if force or text != self._published_status:
            self.status_pub.publish(String(data=text))
            self._published_status = text


def main(args=None):
    rclpy.init(args=args)
    node = ImuConditionerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
