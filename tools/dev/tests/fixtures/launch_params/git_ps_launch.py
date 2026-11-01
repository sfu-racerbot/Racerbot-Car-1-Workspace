import subprocess
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def actions(context):
    git = subprocess.run(['git', '-C', '/', 'status'], capture_output=True, text=True)
    subprocess.run(['ps', '-e'], capture_output=True, text=True)
    return [Node(package='demo_pkg', executable='exe', name='n' + str(git.returncode))]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=actions)])
