from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    here = Path(__file__).parent
    return LaunchDescription([
        DeclareLaunchArgument('marker'),
        Node(package='demo_pkg', executable='does_not_exist_exe', name='demo_node',
             parameters=[str(here / 'demo.yaml'), str(here / 'overlay.yaml'), {'inline': 3}]),
        Node(package='demo_pkg', executable='never_runs', name='hidden',
             condition=IfCondition('false')),
        ExecuteProcess(cmd=['touch', LaunchConfiguration('marker')]),
        GroupAction([PushRosNamespace('ns2'), IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(here / 'included_launch.py')))]),
    ])
