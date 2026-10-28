"""ROS node that scores a multi-car simulator race from ground truth."""

from __future__ import annotations

import json
import math
import os
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from racerbot_sim import tracks
from racerbot_sim.referee import Referee
from racerbot_sim.sim_bridge import car_names


class RaceRefereeNode(Node):
    def __init__(self):
        super().__init__('race_referee_node')
        self.declare_parameter('track', 'indoor_oval')
        self.declare_parameter('track_directory', '~/.ros/racerbot_sim/tracks')
        self.declare_parameter('laps', 3.0)
        self.declare_parameter('controlled_cars', 2)
        self.declare_parameter('rate_hz', 40.0)
        self.declare_parameter('output_dir', '')
        self.declare_parameter('reverse', False)
        self.declare_parameter('timeout_s', 300.0)
        value = lambda name: self.get_parameter(name).value
        self.track = str(value('track'))
        self.track_directory = Path(os.path.expanduser(str(value('track_directory'))))
        self.cars = int(value('controlled_cars'))
        if self.cars < 1:
            raise ValueError('controlled_cars must be positive')
        track_path = tracks.resolve(self.track, self.track_directory)
        del track_path
        centerline = tracks.load_centerline(self.track_directory, self.track)
        self.output_dir = Path(os.path.expanduser(str(value('output_dir')) or
            f'~/.ros/racerbot_sim/races/{datetime.now(timezone.utc):%Y%m%d-%H%M%S}'))
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.reverse = bool(value('reverse'))
        self.laps = float(value('laps'))
        self.dt_s = 1.0 / float(value('rate_hz'))
        self.timeout_s = float(value('timeout_s'))
        self.poses = [None] * self.cars
        self.status = None
        self.referee = None
        self.done = False
        for index in range(self.cars):
            self.create_subscription(PoseStamped, car_names(index + 1)['truth'],
                                     lambda msg, i=index: self._pose(msg, i), 10)
        self.create_subscription(String, '/sim/status', self._status, 10)
        self.result_pub = self.create_publisher(String, '/sim/race_result', 1)
        self.create_timer(self.dt_s, self._tick)

    def _pose(self, message, index):
        self.poses[index] = (float(message.pose.position.x), float(message.pose.position.y))

    def _status(self, message):
        try:
            status = json.loads(message.data)
            if isinstance(status, dict) and isinstance(status.get('cars'), list):
                self.status = status
        except (TypeError, ValueError):
            return

    def _tick(self):
        if self.done or self.status is None or any(p is None for p in self.poses):
            return
        cars = self.status.get('cars', [])
        if len(cars) != self.cars:
            return
        if self.referee is None:
            self.referee = Referee(
                tracks.load_centerline(self.track_directory, self.track), self.poses,
                self.laps, self.dt_s, self.reverse)
        wall = [bool(car.get('wall_contact_now', False)) for car in cars]
        speeds = [float(car.get('speed', math.nan)) for car in cars]
        car_contact = bool(self.status.get('car_contact_now', False))
        try:
            self.referee.tick(float(self.status['sim_time_s']), self.poses, speeds,
                              wall, car_contact)
        except (KeyError, TypeError, ValueError) as error:
            self.get_logger().error(f'cannot score simulator status: {error}')
            return
        if self.referee.finished or self.referee._sim_time >= self.timeout_s:
            self._finish()

    def _finish(self):
        self.done = True
        result = self.referee.result()
        result.update(track=self.track, laps=self.laps, controlled_cars=self.cars,
                      parameters={})
        payload = json.dumps(result, indent=2) + '\n'
        (self.output_dir / 'race.json').write_text(payload)
        message = String()
        message.data = payload
        self.result_pub.publish(message)
        self.get_logger().info(
            f"race result: order={result['finish_order']} winner={result['winner']} "
            f"times={result['finish_time_s']} contacts={result['contacts']}")
        self.destroy_node()
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = RaceRefereeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
