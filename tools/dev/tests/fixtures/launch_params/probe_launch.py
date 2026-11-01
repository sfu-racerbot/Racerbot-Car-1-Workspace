import subprocess

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def actions(context):
    result = subprocess.run(['pgrep', '-f', 'some_pattern'], capture_output=True, text=True)
    if result.returncode:
        return [Node(package='demo_pkg', executable='bridge_exe', name='bridge_node')]
    return []


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=actions)])
