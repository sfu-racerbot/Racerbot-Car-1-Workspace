"""Processing only: starts no camera, IMU, TF publisher or driving node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file',
                              description='Required measured tag-mount config YAML'),
        DeclareLaunchArgument('image_topic', default_value='/camera/camera/color/image_raw'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value='/camera/camera/color/camera_info'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use /clock for bag replay and synthetic tests'),
        Node(package='opponent_perception', executable='opponent_perception_node',
             name='opponent_perception_node', output='screen', parameters=[{
                 key: LaunchConfiguration(key)
                 for key in ('config_file', 'image_topic', 'camera_info_topic', 'use_sim_time')}]),
    ])
