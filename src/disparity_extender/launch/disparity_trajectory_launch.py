"""Control layer: method D on the car (run on top of f1tenth_stack bringup_launch.py).

    ros2 launch f1tenth_stack bringup_launch.py                     # terminal 1
    ros2 launch disparity_trajectory disparity_trajectory_launch.py # terminal 2, hold LB to drive

Loads the shared config, then this car's calibration overlay (its steering
limit) through racerbot_car.layers, like gap_follow_launch.py. Do not run it
beside teleop_launch.py: /teleop would always win the mux.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from racerbot_car.layers import launch_car_id, layered


def generate_launch_description():
    car_id = launch_car_id()
    shared = os.path.join(get_package_share_directory('disparity_trajectory'),
                          'config', 'disparity_trajectory.yaml')
    return LaunchDescription([
        Node(package='disparity_trajectory', executable='disparity_trajectory_node',
             name='disparity_trajectory_node', output='screen',
             parameters=layered(shared, 'disparity_trajectory.yaml', car_id)),
    ])
