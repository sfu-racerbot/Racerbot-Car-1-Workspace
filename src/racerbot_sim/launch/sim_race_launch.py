"""Launch two real gap-follow nodes, the simulator, referee, and recorder."""

from __future__ import annotations

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from racerbot_sim import tracks


def _params(path):
    with open(path, encoding='utf-8') as handle:
        return yaml.safe_load(handle)['gap_follow_node']['ros__parameters']


def _setup(context):
    opponent = LaunchConfiguration('opponent').perform(context)
    if opponent == 'team_a':
        raise RuntimeError('opponent:=team_a is added in Task R7')
    if opponent != 'ours':
        raise RuntimeError(f"unknown opponent:={opponent!r}; expected ours")
    track = LaunchConfiguration('track').perform(context)
    if track in tracks.LAYOUTS:
        track_directory = '~/.ros/racerbot_sim/tracks'
    else:
        here = Path(__file__).resolve()
        try:
            from race_diagnostics.run_metadata import find_repo_root
            root = find_repo_root(here)
        except ImportError:
            root = None
        candidate = root / 'tools' / 'f1tenth_sim' / 'maps' if root else None
        if candidate is None or not (candidate / f'{track}_map.yaml').is_file() or not (candidate / f'{track}_centerline.csv').is_file():
            raise RuntimeError(f"track '{track}' was not found in {candidate or '<no repository root>'}")
        track_directory = str(candidate)
    sim_share = get_package_share_directory('racerbot_sim')
    gap_share = get_package_share_directory('gap_follow')
    default_yaml = os.path.join(gap_share, 'config', 'gap_follow.yaml')
    car1_yaml = LaunchConfiguration('car1_params').perform(context) or default_yaml
    car2_yaml = LaunchConfiguration('car2_params').perform(context) or default_yaml
    car2 = _params(car2_yaml)
    car2.update(scan_topic='/car2/scan', odom_topic='/car2/odom', drive_topic='/car2/drive',
                joy_topic='/car2/joy', intent_topic='/car2/drive_intent',
                imu_topic='/car2/imu/data', imu_status_topic='/car2/imu/status')
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'sim_bringup_launch.py')),
        launch_arguments={
            'track': track, 'track_directory': track_directory,
            'seed': LaunchConfiguration('seed'), 'controlled_cars': '2',
            'controlled_start_offsets_m': LaunchConfiguration('start_offsets'),
            'hold_deadman': 'true', 'car2_hold_deadman': LaunchConfiguration('car2_hold_deadman'),
        }.items())
    car1 = Node(package='gap_follow', executable='gap_follow_node', name='gap_follow_node',
                parameters=[car1_yaml], output='screen')
    car2_node = Node(package='gap_follow', executable='gap_follow_node', name='gap_follow_node',
                     namespace='car2', parameters=[car2], output='screen')
    referee = Node(package='racerbot_sim', executable='race_referee_node', name='race_referee_node',
                   parameters=[{'track': track, 'track_directory': track_directory,
                                'laps': LaunchConfiguration('laps'), 'controlled_cars': 2,
                                'reverse': LaunchConfiguration('reverse'),
                                'output_dir': LaunchConfiguration('output_dir')}], output='screen')
    if LaunchConfiguration('swap_starts').perform(context).lower() == 'true':
        bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'sim_bringup_launch.py')),
            launch_arguments={
                'track': track, 'track_directory': track_directory,
                'seed': LaunchConfiguration('seed'), 'controlled_cars': '2',
                'controlled_start_offsets_m': '-3.0,0.0',
                'hold_deadman': 'true', 'car2_hold_deadman': LaunchConfiguration('car2_hold_deadman'),
            }.items())
    actions = [bringup, car1, car2_node, referee]
    if LaunchConfiguration('record').perform(context).lower() == 'true':
        topics = ('/scan /odom /slam_pose /drive /ackermann_cmd /teleop /tf /tf_static /map /joy '
                  '/drive_intent /car2/scan /car2/odom /car2/drive /car2/ackermann_cmd /car2/joy '
                  '/car2/drive_intent /car2/teleop /car2/safety/brake /sim/ground_truth_pose '
                  '/sim/car2/ground_truth_pose /sim/status /sim/race_result')
        recorder = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('race_diagnostics'), 'launch', 'record_run.py')),
            launch_arguments={'topics': topics, 'run_directory': LaunchConfiguration('record_dir')}.items())
        actions.append(recorder)
    actions.append(RegisterEventHandler(OnProcessExit(target_action=referee, on_exit=[Shutdown()])))
    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument('track', default_value='indoor_oval'),
        DeclareLaunchArgument('opponent', default_value='ours'),
        DeclareLaunchArgument('car1_params', default_value=''),
        DeclareLaunchArgument('car2_params', default_value=''),
        DeclareLaunchArgument('laps', default_value='3.0'),
        DeclareLaunchArgument('swap_starts', default_value='false'),
        DeclareLaunchArgument('seed', default_value='12345'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('record_dir', default_value=''),
        DeclareLaunchArgument('output_dir', default_value=''),
        DeclareLaunchArgument('reverse', default_value='false'),
        DeclareLaunchArgument('car2_hold_deadman', default_value='true'),
        DeclareLaunchArgument('start_offsets', default_value='0.0,-3.0'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
