import subprocess

from launch import LaunchDescription
from launch.actions import OpaqueFunction


def actions(context):
    subprocess.run(['ls'])
    return []


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=actions)])
