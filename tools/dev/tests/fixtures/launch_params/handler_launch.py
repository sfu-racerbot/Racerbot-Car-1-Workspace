from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    first = ExecuteProcess(cmd=['true'], name='first')
    return LaunchDescription([first, RegisterEventHandler(OnProcessExit(
        target_action=first, on_exit=[Node(package='demo_pkg', executable='after_exe',
                                           name='after_node')]))])
