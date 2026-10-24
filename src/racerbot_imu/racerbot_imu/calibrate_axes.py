"""calibrate_axes: find one IMU's mount rotation on the car (plan, P0.4).

    ros2 run racerbot_imu calibrate_axes --imu front --topic /sensors/imu_front/raw \
        --gyro-scale 1.0 --accel-scale 1.0

With the drivers up and NO control layer running, it asks for three things:

  1. 10 s with the car still and level        -> gravity gives "up"
  2. a gentle straight push forward, by hand   -> gives "forward"
  3. a counter-clockwise spin by hand          -> checks the result: yaw must read positive

then prints the r_base_imu block to paste into config/imu_rb1.yaml. It
subscribes only; it never publishes and never opens a device.
"""
import argparse
import sys
import time

import numpy as np

from racerbot_imu import imu_math


def _recorder(topic: str):
    """A subscribe-only node that records raw IMU samples (rclpy imported here,
    so the pure helpers below stay testable unsourced)."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu

    class Recorder(Node):
        def __init__(self):
            super().__init__('imu_calibrate_axes')
            self.samples = []
            self.create_subscription(Imu, topic, self._cb, 50)

        def _cb(self, msg):
            self.samples.append((
                (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
                (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)))

        def record(self, seconds: float):
            self.samples = []
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
            return list(self.samples)

    return rclpy, Recorder()


def push_samples(accel, up, min_horizontal: float = 0.3):
    """The samples of a push where horizontal acceleration is clearly present."""
    a = np.asarray(accel, dtype=np.float64)
    horizontal = a - up
    horizontal -= np.outer(horizontal @ (up / np.linalg.norm(up)), up / np.linalg.norm(up))
    keep = np.linalg.norm(horizontal, axis=1) > min_horizontal
    return a[keep]


def spin_check(gyro_base, min_rate: float = 0.2) -> float:
    """Mean base-frame yaw rate during a counter-clockwise hand spin; must be positive."""
    w = float(np.mean(np.asarray(gyro_base)[:, 2]))
    if w < min_rate:
        raise imu_math.ImuSampleError(
            f'CCW spin read yaw {w:+.2f} rad/s; expected above +{min_rate}: axes are wrong')
    return w


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--imu', required=True)
    ap.add_argument('--topic', required=True)
    ap.add_argument('--gyro-scale', type=float, default=imu_math.VESC_GYRO_SCALE)
    ap.add_argument('--accel-scale', type=float, default=imu_math.VESC_ACCEL_SCALE)
    args = ap.parse_args(argv)
    import rclpy as _rclpy
    _rclpy.init()
    rclpy, rec = _recorder(args.topic)
    try:
        def si(samples):
            g = np.array([s[0] for s in samples]) * args.gyro_scale
            a = np.array([s[1] for s in samples]) * args.accel_scale
            return g, a
        input('1/3  Car still and level on the floor. Press Enter, then do not touch it for 10 s.')
        _, still = si(rec.record(10.0))
        if len(still) < 100:
            print(f'only {len(still)} samples on {args.topic}: is the IMU publishing?', file=sys.stderr)
            return 1
        imu_math.gravity_check(still, tol=0.5, max_tilt_deg=180.0)    # units check only, any mount
        up = still.mean(axis=0)
        input('2/3  Press Enter, then push the car gently STRAIGHT FORWARD for about a second.')
        _, push = si(rec.record(3.0))
        r = imu_math.axes_from_gravity_and_push(still, push_samples(push, up))
        input('3/3  Press Enter, then spin the car COUNTER-CLOCKWISE (seen from above) by hand.')
        spin_g, _ = si(rec.record(3.0))
        w = spin_check(spin_g @ r.T)
    except imu_math.ImuSampleError as exc:
        print(f'FAILED: {exc}', file=sys.stderr)
        return 1
    finally:
        rec.destroy_node()
        rclpy.shutdown()
    print(f'spin check passed: yaw {w:+.2f} rad/s\n')
    print(f'# {args.imu}: paste into config/imu_rb1.yaml, imu_conditioner_{args.imu} block')
    print('r_base_imu: [' + ', '.join(f'{x:.5f}' for x in r.reshape(-1)) + ']')
    return 0


if __name__ == '__main__':
    sys.exit(main())
