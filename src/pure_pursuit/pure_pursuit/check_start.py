"""check_start: is the car really on a mapping run's start pose?

    ros2 run pure_pursuit check_start --ros-args -p run_dir:=<the run's folder>

race_launch.py tells the particle filter the car is on start_pose.yaml, and only searches about 0.5 m and 20 deg
around it. seed_start_pose reports success as soon as the filter echoes that seed back, which it does wherever
the car really is. So a car in the wrong place would start racing on a confident, wrong position.

This compares one live /scan with the saved map and answers GO or NO-GO, with how far off the car is. Support
code (docs/adding-your-own-code.md): it subscribes to /scan and publishes nothing, so it cannot move the car.

Exit 0 for GO, 1 for NO-GO.
"""

from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np

from pure_pursuit import map_localize, occupancy_map, run_commands

MAX_OFFSET_M = 0.30  # well inside the filter's 0.5 m seed spread (seed_start_pose.py)
MAX_OFFSET_DEG = 12.0  # well inside its 23 deg
MIN_FIT = 0.80
NEAR_RADIUS_M = 1.0
NEAR_FIT_MARGIN = 0.02  # how much worse than the best place anywhere the start pose may fit and still stand


@dataclass
class StartCheck:
    go: bool
    fit: float
    offset_m: float
    offset_deg: float
    pose: tuple
    start: tuple
    see_through: float = 0.0


def verdict(ranges, angle_min, angle_increment, run_dir) -> StartCheck:
    """Where this scan puts the car on run_dir's map, and whether that is the start pose.

    Always searches the whole map as well as round the start pose. A course repeats itself, so a pose near the
    start can fit quite well while the car is somewhere else entirely: the near pose stands only if it fits as
    well as the best place anywhere. Beams are also checked for passing through mapped walls, because in a boxy
    room a scan from somewhere else entirely can still end mostly on some wall.
    """
    run_dir = Path(run_dir)
    start = run_commands.read_start_pose(run_dir)
    if start is None:
        raise ValueError(f'{run_dir} has no usable {run_commands.START_POSE_FILE}')
    field = map_localize.WallField.from_occupancy(
        occupancy_map.OccupancyMap.from_yaml(str(run_dir / 'map.yaml')))
    bx, by = map_localize.scan_points(ranges, angle_min, angle_increment)
    pose, fit = map_localize.find_start(field, bx, by)
    near_pose, near_fit = map_localize.find_start(field, bx, by, near=start, near_radius=NEAR_RADIUS_M)
    if near_fit >= fit - NEAR_FIT_MARGIN:
        pose, fit = near_pose, near_fit
    through = float(map_localize.see_through_rate(field, np.array([pose]), np.array([ranges]), angle_min,
                                                  angle_increment)[0])
    offset_m = math.hypot(pose[0] - start[0], pose[1] - start[1])
    offset_deg = abs(math.degrees(math.remainder(pose[2] - start[2], 2.0 * math.pi)))
    go = (fit >= MIN_FIT and through <= map_localize.MAX_SEE_THROUGH
          and offset_m <= MAX_OFFSET_M and offset_deg <= MAX_OFFSET_DEG)
    return StartCheck(go, fit, offset_m, offset_deg, pose, start, through)


def describe(check: StartCheck) -> str:
    """What an operator needs to read, with how to fix a NO-GO in the car's own terms."""
    if check.fit < MIN_FIT or check.see_through > map_localize.MAX_SEE_THROUGH:
        return (f'NO-GO: this scan fits nowhere on the map (best fit {check.fit * 100:.0f}%, needs '
                f'{MIN_FIT * 100:.0f}%; {check.see_through * 100:.0f}% of beams pass through mapped walls). '
                'Wrong map, or the course has changed. Nothing here will localize.')
    text = (f'car is {check.offset_m:.2f} m and {check.offset_deg:.0f} deg from the start pose '
            f'(limits {MAX_OFFSET_M:.2f} m, {MAX_OFFSET_DEG:.0f} deg), scan fits {check.fit * 100:.0f}%.')
    if check.go:
        return 'GO: ' + text
    dx, dy = check.start[0] - check.pose[0], check.start[1] - check.pose[1]
    c, s = math.cos(check.pose[2]), math.sin(check.pose[2])
    return (f'NO-GO: {text}\n  Move the car about {c * dx + s * dy:+.2f} m forward and '
            f'{-s * dx + c * dy:+.2f} m to its left, then turn it to '
            f'{math.degrees(check.start[2]):.0f} deg on the map.')


def main(args=None):
    import rclpy  # here, so verdict() and its tests need no ROS
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init(args=args)
    node = rclpy.create_node('check_start')
    node.declare_parameter('run_dir', '')
    run_dir = node.get_parameter('run_dir').value
    scans = []
    node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
    try:
        if not run_dir:
            print('NO-GO: give the run folder: --ros-args -p run_dir:=<folder>')
            return 1
        for _ in range(100):
            rclpy.spin_once(node, timeout_sec=0.1)
            if scans:
                break
        if not scans:
            print('NO-GO: no /scan in 10 s. Is bringup_launch.py running, and this terminal sourced?')
            return 1
        scan = scans[0]
        try:
            check = verdict(np.array(scan.ranges), scan.angle_min, scan.angle_increment, run_dir)
        except (OSError, ValueError) as exc:
            print(f'NO-GO: {exc}')
            return 1
        print(describe(check))
        return 0 if check.go else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
