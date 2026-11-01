import io
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def read_device(context):
    target = LaunchConfiguration('target').perform(context)
    api = LaunchConfiguration('api').perform(context)
    if api == 'os':
        fd = os.open(target, os.O_RDONLY)
        try:
            data = os.read(fd, 0)
        finally:
            os.close(fd)
    else:
        opener = io.open if api == 'io' else open
        with opener(target, 'rb') as stream:
            data = stream.read(0)
    return [Node(package='demo_pkg', executable='exe', name='device_read',
                 parameters=[{'bytes_read': len(data)}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target'), DeclareLaunchArgument('api'),
        OpaqueFunction(function=read_device)])
