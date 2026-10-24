"""Car 1's IMU chain: one conditioner per IMU, then the selector.

    ros2 launch racerbot_imu imu_launch.py                 # both IMUs, cross-checked
    ros2 launch racerbot_imu imu_launch.py front:=false    # VESC IMU only (single-IMU mode)

Working when: ros2 topic echo /imu/status --once shows "state": "ok"
(keep the car still for 2 s after starting, for the gyro bias).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('racerbot_imu'), 'config', 'imu_rb1.yaml')
    front = LaunchConfiguration('front')
    return LaunchDescription([
        DeclareLaunchArgument('front', default_value='true',
                              description='also run the front IMU and the cross-check'),
        Node(package='racerbot_imu', executable='imu_conditioner_node',
             name='imu_conditioner_vesc', parameters=[config], output='screen'),
        Node(package='racerbot_imu', executable='imu_conditioner_node',
             name='imu_conditioner_front', parameters=[config], output='screen',
             condition=IfCondition(front)),
        Node(package='racerbot_imu', executable='imu_select_node',
             name='imu_select_node', parameters=[config], output='screen',
             condition=IfCondition(front)),
        Node(package='racerbot_imu', executable='imu_select_node',
             name='imu_select_node', parameters=[config, {'secondary': ''}], output='screen',
             condition=UnlessCondition(front)),
    ])
