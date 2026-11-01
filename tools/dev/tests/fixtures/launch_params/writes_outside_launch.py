from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def write(context):
    with open(LaunchConfiguration('target').perform(context), 'w') as stream:
        stream.write('outside')
    return []


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('target'), OpaqueFunction(function=write)])
