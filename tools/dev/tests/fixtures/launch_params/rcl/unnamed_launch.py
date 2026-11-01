from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    directory = Path(__file__).parent
    return LaunchDescription([
        Node(package='demo_pkg', executable='exe', parameters=[
            str(directory / 'case_3_a.yaml'), str(directory / 'case_3_b.yaml')]),
        Node(package='demo_pkg', executable='exe', parameters=[
            str(directory / 'case_1_a.yaml'), str(directory / 'case_1_b.yaml')]),
    ])
