import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: os._exit(3))),
        Node(package='demo_pkg', executable='exe', name='safe'),
    ])
