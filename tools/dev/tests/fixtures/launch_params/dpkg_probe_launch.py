import subprocess

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def actions(context):
    result = subprocess.run(
        ['dpkg-query', '-W', 'ros-jazzy-*'], capture_output=True, text=True)
    return [Node(package='demo_pkg', executable='exe', name=f'n{len(result.stdout)}')]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=actions)])
