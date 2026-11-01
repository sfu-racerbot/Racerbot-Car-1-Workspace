from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    return LaunchDescription([LifecycleNode(
        package='demo_pkg', executable='lifecycle_exe', name='lifecycle_node', namespace='')])
