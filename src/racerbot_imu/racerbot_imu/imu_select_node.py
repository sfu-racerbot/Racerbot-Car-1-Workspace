"""imu_select_node: the primary IMU on /imu/data, cross-checked by the secondary.

    /imu/data     the primary's conditioned samples, republished UNCHANGED
    /imu/status   {state: ok|fault, reason, disagreement, primary, secondary}

/imu/status is ok only while the primary is ok and -- with a secondary
configured -- the secondary is ok and the two yaw rates agree. A sustained
disagreement is a fault: with two sensors there is no majority to say which
one is wrong. No failover in v1: the two accelerometers sit at different
points and the network was trained on the primary's.

Single-IMU mode (secondary: ''): for a front IMU wired through the VESC,
which reports one IMU at a time. Publishes nothing that can move the car.
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from racerbot_imu.imu_crosscheck import GyroAgreement, SampleBuffer, aligned_window_mean


class ImuSelectNode(Node):

    def __init__(self):
        super().__init__('imu_select_node')
        p = self.declare_parameter
        p('primary', 'vesc')
        p('secondary', 'front')
        p('secondary_required', True)
        p('delay_primary_sec', 0.0)
        p('delay_secondary_sec', 0.0)
        p('gyro_base_tolerance', 0.05)     # rad/s; P0.5 sets max(0.05, 5 sigma measured)
        p('gyro_relative_tolerance', 0.05)
        p('disagreement_sec', 0.5)
        p('window_sec', 0.2)
        p('data_timeout_sec', 0.1)
        v = lambda n: self.get_parameter(n).value   # noqa: E731
        self.primary = v('primary')
        self.secondary = v('secondary') or ''
        self.secondary_required = bool(v('secondary_required'))
        self.delay_p = float(v('delay_primary_sec'))
        self.delay_s = float(v('delay_secondary_sec'))
        self.window = float(v('window_sec'))
        self.data_timeout = float(v('data_timeout_sec'))
        self.agreement = GyroAgreement(float(v('gyro_base_tolerance')),
                                       float(v('gyro_relative_tolerance')),
                                       float(v('disagreement_sec')))
        self.buf = {self.primary: SampleBuffer()}
        self.status = {self.primary: None}
        self.status_time = {self.primary: None}
        if self.secondary:
            self.buf[self.secondary] = SampleBuffer()
            self.status[self.secondary] = None
            self.status_time[self.secondary] = None

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.data_pub = self.create_publisher(Imu, '/imu/data', 20)
        self.status_pub = self.create_publisher(String, '/imu/status', latched)
        for name in self.buf:
            self.create_subscription(Imu, f'/imu/{name}/data',
                                     lambda m, n=name: self.data_callback(n, m), 20)
            self.create_subscription(String, f'/imu/{name}/status',
                                     lambda m, n=name: self.status_callback(n, m), latched)
        self.create_timer(0.05, self.evaluate)
        self.state, self.reason = 'fault', 'waiting for IMU status'
        self._published = None

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def data_callback(self, name: str, msg: Imu):
        self.buf[name].add(self._now(), float(msg.angular_velocity.z))
        if name == self.primary:
            self.data_pub.publish(msg)                 # unchanged, field for field

    def status_callback(self, name: str, msg: String):
        try:
            self.status[name] = json.loads(msg.data).get('state')
        except (ValueError, AttributeError):
            self.status[name] = None
        self.status_time[name] = self._now()

    def _ok(self, name: str, now: float) -> bool:
        """Its conditioner says ok (status is latched, published on change)
        and its data is still arriving."""
        samples = self.buf[name].samples
        return (self.status[name] == 'ok' and bool(samples)
                and now - samples[-1][0] < self.data_timeout)

    def evaluate(self):
        now = self._now()
        if not self._ok(self.primary, now):
            state, reason = 'fault', f'primary IMU {self.primary!r} not ok'
        elif not self.secondary:
            state, reason = 'ok', None
        elif not self._ok(self.secondary, now):
            state, reason = (('fault', f'secondary IMU {self.secondary!r} not ok (secondary_required)')
                             if self.secondary_required else ('ok', 'secondary unavailable'))
        else:
            t_truth = now - max(self.delay_p, self.delay_s) - 0.02    # both streams have arrived
            wp = aligned_window_mean(self.buf[self.primary].samples, t_truth, self.window, self.delay_p)
            ws = aligned_window_mean(self.buf[self.secondary].samples, t_truth, self.window, self.delay_s)
            agree, d = self.agreement.update(now, wp, ws)
            state, reason = ('ok', None) if agree else (
                'fault', f'IMUs disagree on yaw rate by {d:.3f} rad/s: a loose mount, or wrong axes on one')
        self.state, self.reason = state, reason
        text = json.dumps({'state': state, 'reason': reason, 'primary': self.primary,
                           'secondary': self.secondary or None,
                           'disagreement': (None if not math.isfinite(self.agreement.disagreement)
                                            else round(self.agreement.disagreement, 4))})
        if text != self._published:
            self.status_pub.publish(String(data=text))
            self._published = text


def main(args=None):
    rclpy.init(args=args)
    node = ImuSelectNode()
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
