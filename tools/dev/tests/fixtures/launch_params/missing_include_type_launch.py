from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription, LaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


class MissingPackageSource(LaunchDescriptionSource):
    def get_launch_description(self, context):
        raise PackageNotFoundError('no_such_pkg_abc')


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(MissingPackageSource()),
        Node(package='demo_pkg', executable='exe', name='after_include'),
    ])
