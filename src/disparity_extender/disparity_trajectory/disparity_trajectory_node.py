"""disparity_trajectory_node: method D on the car's own topics.

    /scan  (sensor_msgs/LaserScan)       -- the car's LiDAR
    /joy   (sensor_msgs/Joy)             -- the F710; LB (button 4) is the deadman
    /drive (ackermann_msgs/AckermannDriveStamped) -> ackermann_mux, priority 10

A control layer, run on top of bringup_launch.py like gap_follow
(launch/disparity_trajectory_launch.py). The steering choice lives in
trajectory_logic.py (no rclpy); this file is the ROS wiring and the safety gate.

Safety (CLAUDE.md, docs/writing-your-own-node.md): a non-zero command is
published only while LB is held on a live /joy stream. Every other case
publishes an explicit 0.0 / 0.0 stop: no /joy yet, /joy stale, LB missing or
released, no arc clear for min_score_m, a scan with too few valid returns
(all inf / zero / NaN: a dead LiDAR), the LiDAR gone quiet (watchdog timer),
and any exception in the scan callback (stop first, then re-raise).
"""
from __future__ import annotations

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy, LaserScan

from disparity_trajectory import trajectory_logic as tl


class DisparityTrajectoryNode(Node):
    def __init__(self):
        super().__init__('disparity_trajectory_node')
        d = tl.TrajectoryParams()
        s = tl.SpeedParams()
        for name, default in (
            ('scan_topic', '/scan'), ('joy_topic', '/joy'), ('drive_topic', '/drive'),
            ('deadman_button', 4), ('joy_timeout_sec', 0.5), ('enable_deadman', True),
            ('scan_timeout_sec', 0.5), ('min_valid_fraction', 0.1),
            ('wheelbase', d.wheelbase), ('max_steering_angle', d.max_steering),
            ('alpha', 0.8),
            ('max_speed', s.max_speed), ('min_speed', s.min_speed),
            ('max_lateral_acc', s.max_lateral_acc),
            ('trajectory_n_curvatures', d.n_curvatures), ('trajectory_step_m', d.step_m),
            ('trajectory_horizon_m', d.horizon_m), ('trajectory_margin_m', d.margin_m),
            ('trajectory_strip_half_m', d.strip_half_m),
            ('trajectory_continuation_cap_m', d.continuation_cap_m),
            ('trajectory_point_range_m', d.point_range_m), ('trajectory_smooth_w', d.smooth_w),
            ('trajectory_min_score_m', d.min_score_m), ('lidar_x_m', d.lidar_x_m),
            ('enable_obstacle_slowdown', s.enable_obstacle_slowdown),
            ('obstacle_count_threshold', s.obstacle_count_threshold),
            ('obstacle_count_range', 3.0), ('obstacle_cluster_gap', 0.3), ('obstacle_min_beams', 5),
            ('obstacle_fov_half_angle_deg', 90.0),
            ('obstacle_slowdown', s.obstacle_slowdown), ('obstacle_min_speed', s.obstacle_min_speed),
        ):
            self.declare_parameter(name, default)
        v = lambda name: self.get_parameter(name).value  # noqa: E731

        self.scan_topic, self.joy_topic = str(v('scan_topic')), str(v('joy_topic'))
        self.deadman_button = int(v('deadman_button'))
        self.joy_timeout_sec = float(v('joy_timeout_sec'))
        self.enable_deadman = bool(v('enable_deadman'))
        self.scan_timeout_sec = max(0.05, float(v('scan_timeout_sec')))
        self.min_valid_fraction = float(v('min_valid_fraction'))
        self.alpha = float(v('alpha'))
        self.max_steering = float(v('max_steering_angle'))
        if not (0.0 < self.alpha <= 1.0):
            raise ValueError(f'alpha must be in (0, 1], got {self.alpha}')
        self.trajectory = tl.TrajectoryParams(
            wheelbase=float(v('wheelbase')), max_steering=self.max_steering,
            n_curvatures=int(v('trajectory_n_curvatures')), step_m=float(v('trajectory_step_m')),
            horizon_m=float(v('trajectory_horizon_m')), margin_m=float(v('trajectory_margin_m')),
            strip_half_m=float(v('trajectory_strip_half_m')),
            continuation_cap_m=float(v('trajectory_continuation_cap_m')),
            point_range_m=float(v('trajectory_point_range_m')), smooth_w=float(v('trajectory_smooth_w')),
            min_score_m=float(v('trajectory_min_score_m')), lidar_x_m=float(v('lidar_x_m')))
        self.speed = tl.SpeedParams(
            max_speed=float(v('max_speed')), min_speed=float(v('min_speed')),
            max_lateral_acc=float(v('max_lateral_acc')), wheelbase=float(v('wheelbase')),
            enable_obstacle_slowdown=bool(v('enable_obstacle_slowdown')),
            obstacle_count_threshold=int(v('obstacle_count_threshold')),
            obstacle_slowdown=float(v('obstacle_slowdown')),
            obstacle_min_speed=float(v('obstacle_min_speed')))
        if not (0.0 <= self.speed.min_speed <= self.speed.max_speed):
            raise ValueError('need 0 <= min_speed <= max_speed')
        self.count = dict(fov=math.radians(float(v('obstacle_fov_half_angle_deg'))),
                          range=float(v('obstacle_count_range')),
                          gap=float(v('obstacle_cluster_gap')), beams=int(v('obstacle_min_beams')))

        self.deadman_held = False
        self.joy_button_available = False
        self.last_joy_time = None
        self.last_scan_time = None
        self.previous_steering = 0.0
        self.last_decision_state = 'starting'

        self.drive_pub = self.create_publisher(AckermannDriveStamped, str(v('drive_topic')), 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(Joy, self.joy_topic, self.joy_callback, 10)
        self.watchdog = self.create_timer(min(0.1, self.scan_timeout_sec / 2.0), self._watchdog)
        self.get_logger().info(
            f"disparity_trajectory_node: {self.scan_topic} -> {self.drive_pub.topic_name}, "
            f"deadman {'ENABLED (LB must be held)' if self.enable_deadman else 'DISABLED'}, "
            f"max_steering {self.max_steering:.3f} rad, max_speed {self.speed.max_speed:.2f} m/s")

    # --- the deadman gate (gap_follow_node's pattern) ---------------------
    def joy_callback(self, msg: Joy):
        self.last_joy_time = self.get_clock().now()
        self.joy_button_available = len(msg.buttons) > self.deadman_button
        self.deadman_held = bool(msg.buttons[self.deadman_button]) if self.joy_button_available else False

    def _age_sec(self, stamp) -> float:
        """Seconds since stamp; inf if the clock stepped backwards since (unknown age reads as stale)."""
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return age if age >= 0.0 else math.inf

    def _deadman_status(self):
        if not self.enable_deadman:
            return True, None
        if self.last_joy_time is None:
            return False, 'waiting_for_joy'
        if self._age_sec(self.last_joy_time) >= self.joy_timeout_sec:
            return False, 'joy_stale'
        if not self.joy_button_available:
            return False, 'deadman_button_missing'
        if not self.deadman_held:
            return False, 'deadman_released'
        return True, None

    # --- commands ---------------------------------------------------------
    def _publish(self, steering: float, speed: float):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(steering)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

    def _stop(self, state: str):
        self.last_decision_state = state
        self.previous_steering = 0.0
        self._publish(0.0, 0.0)

    def scan_callback(self, scan: LaserScan):
        try:
            self._scan_step(scan)
        except Exception:
            self._stop('exception')
            raise

    def _scan_step(self, scan: LaserScan):
        self.last_scan_time = self.get_clock().now()
        engaged, why = self._deadman_status()
        if not engaged:
            self._stop(why)
            return
        if tl.valid_fraction(scan.ranges, scan.range_min, scan.range_max) < self.min_valid_fraction:
            # All inf / zero / NaN reads as "nothing anywhere" to method D, which
            # would drive straight at full speed: a dead or blinded LiDAR. Stop.
            self._stop('scan_invalid')
            return
        points = tl.scan_points(scan.ranges, scan.angle_min, scan.angle_increment,
                                scan.range_min, self.trajectory)
        target = tl.trajectory_target(points, self.previous_steering, self.trajectory)
        if target is None:
            self._stop('no_clear_arc')
            return
        obstacles = tl.count_obstacles(scan.ranges, scan.angle_min, scan.angle_increment,
                                       scan.range_min, self.count['fov'], self.count['range'],
                                       self.count['gap'], self.count['beams'])
        clear = math.inf if target.no_points else target.score
        steering, self.previous_steering = tl.smooth(
            target.steering, self.previous_steering, self.alpha, self.max_steering)
        speed = tl.speed_for(steering, clear, obstacles, self.speed)
        self.last_decision_state = 'driving'
        self._publish(steering, speed)

    def _watchdog(self):
        """The LiDAR going quiet stops the car: an explicit zero, not silence."""
        if self.last_scan_time is None or self._age_sec(self.last_scan_time) >= self.scan_timeout_sec:
            self._stop('scan_stale' if self.last_scan_time is not None else 'waiting_for_scan')


def main(args=None):
    rclpy.init(args=args)
    node = DisparityTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish(0.0, 0.0)
        except Exception:  # noqa: BLE001 - shutting down; the mux times out regardless
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
