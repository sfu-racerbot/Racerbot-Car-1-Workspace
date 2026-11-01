from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(
        package='demo_pkg', executable='inc_exe', name='named_node',
        parameters=[str(Path(__file__).parent / 'overlay.yaml')])])
