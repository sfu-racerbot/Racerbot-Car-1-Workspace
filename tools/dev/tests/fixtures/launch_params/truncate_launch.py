import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def truncate(context):
    os.truncate(path=LaunchConfiguration('target').perform(context), length=0)
    return []


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target'), OpaqueFunction(function=truncate)])
