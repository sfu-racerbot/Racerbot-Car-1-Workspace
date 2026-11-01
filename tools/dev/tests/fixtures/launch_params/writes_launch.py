import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction


def write(context):
    directory = os.path.expanduser('~/x')
    os.makedirs(directory, exist_ok=True)
    with open(os.path.join(directory, 'out.txt'), 'w') as stream:
        stream.write('sandboxed')
    return []


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=write)])
