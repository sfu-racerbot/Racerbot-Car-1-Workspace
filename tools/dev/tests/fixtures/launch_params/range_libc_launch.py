from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    import range_libc
    return LaunchDescription([Node(package='demo_pkg', executable='exe',
                                   name='range_node' if range_libc.SHOULD_USE_CUDA else 'cpu')])
