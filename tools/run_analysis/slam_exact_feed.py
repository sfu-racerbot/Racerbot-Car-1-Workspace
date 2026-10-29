#!/usr/bin/env python3
"""Feed only scans answered by recorded SLAM, waiting for each new pose.

Before each scan, send odometry through the first sample at or after its
stamp. On the car, SLAM's message filter likewise holds a scan until this
sample arrives and TF can interpolate at the scan stamp. Advance /clock to
the scan stamp, never to its future bracket; then wait for that stamp's pose.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from slam_replay import bag_messages, exact_scan_plan, odometry_brackets, stamp_ns


class ExactFeed(Node):
    def __init__(self, odom_topic):
        super().__init__('slam_exact_feed')
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.static_pub = self.create_publisher(
            TFMessage, '/tf_static', QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 100)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.answered = set()
        self.tf_stamps = set()
        self.lookahead_samples = 0
        self.completed_scans = 0
        self.create_subscription(PoseWithCovarianceStamped, '/pose',
                                 lambda msg: self.answered.add(stamp_ns(msg)), 100)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)

    def on_tf(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == 'base_link':
                self.tf_stamps.add(stamp_ns(transform))

    def wait(self, predicate, description, timeout=10.0, tick=None):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise RuntimeError(f'timeout waiting for {description}')
            if tick is not None:
                tick()
            rclpy.spin_once(self, timeout_sec=.02)

    def clock(self, stamp):
        msg = Clock()
        msg.clock.sec, msg.clock.nanosec = divmod(stamp, 1_000_000_000)
        self.clock_pub.publish(msg)

    def feed(self, bag, odom_topic):
        scans, poses, odoms, statics = [], [], [], []
        for topic, msg in bag_messages(bag, ['/scan', '/pose', '/tf_static', odom_topic]):
            if topic == '/scan':
                scans.append(msg)
            elif topic == '/pose':
                poses.append(stamp_ns(msg))
            elif topic == odom_topic:
                odoms.append(msg)
            else:
                statics.extend(msg.transforms)
        plan = exact_scan_plan([stamp_ns(msg) for msg in scans], poses)
        by_stamp = {stamp_ns(msg): msg for msg in scans}
        odoms.sort(key=stamp_ns)
        ends = odometry_brackets([stamp_ns(msg) for msg in odoms], plan)
        self.wait(lambda: self.scan_pub.get_subscription_count() > 0
                  and self.odom_pub.get_subscription_count() > 0
                  and self.clock_pub.get_subscription_count() > 0,
                  'SLAM, relay and clock subscribers')
        self.static_pub.publish(TFMessage(transforms=statics))
        if plan:
            # rosbag2 --use-sim-time defers topic discovery until its first clock.
            # Repeat that same time until the recorder can capture the first pose.
            self.wait(lambda: any(info.node_name.startswith('rosbag2_recorder')
                                  for info in self.get_subscriptions_info_by_topic('/pose')),
                      'recorder /pose subscription', tick=lambda: self.clock(plan[0]))
        index = 0
        for stamp, end in zip(plan, ends):
            while index < end:
                odom = odoms[index]
                self.odom_pub.publish(odom)
                self.wait(lambda: stamp_ns(odom) in self.tf_stamps,
                          f'relay TF at {stamp_ns(odom)}')
                self.lookahead_samples += stamp_ns(odom) > stamp
                index += 1
            self.clock(stamp)
            self.scan_pub.publish(by_stamp[stamp])
            self.wait(lambda: stamp in self.answered, f'/pose at scan stamp {stamp}')
            self.completed_scans += 1
            print(f'answered {stamp}', flush=True)
        print(f'finished {len(plan)} planned scans', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--odom-topic', required=True)
    parser.add_argument('--result', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = ExactFeed(args.odom_topic)
    try:
        node.feed(args.bag, args.odom_topic)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr, flush=True)
        return 1
    finally:
        args.result.write_text(json.dumps({'lookahead_samples': node.lookahead_samples,
                                          'completed_scans': node.completed_scans}) + '\n')
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
