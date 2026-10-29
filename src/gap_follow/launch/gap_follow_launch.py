import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from gap_follow.launch_speed import speed_cap_overrides, yaml_parameters
from gap_follow.launch_imu import select_imu_hardware


FUSED_ODOM = '/odom/fused'   # vehicle_state.yaml's fused_odom_topic


def _flag(name, value):
    """A launch boolean, read the way racerbot_launch's map_course_launch.py reads its own."""
    text = str(value).strip().lower()
    if text in ('true', '1'):
        return True
    if text in ('false', '0'):
        return False
    raise RuntimeError(f"{name}:={value!r}: expected 'true' or 'false'")


def generate_launch_description(imu_probe=None):
    gap_follow_config = os.path.join(
        get_package_share_directory('gap_follow'),
        'config',
        'gap_follow.yaml'
    )

    opponent_arg = DeclareLaunchArgument(
        'opponent_awareness', default_value='true',
        description='on by default since 2026-10-28; false turns the whole layer off. '
                    'When on: track moving cars, follow them (the follow cap is held '
                    'for opponent_follow_memory_sec after one is lost), and pass only '
                    'with room on a straight (docs/gap-follow.md#several-cars-at-once). '
                    'Simulator-validated only.')
    max_speed_arg = DeclareLaunchArgument(
        'max_speed', default_value='',
        description='Top speed for this run (m/s), up to 5.0: e.g. 1.0 for '
                    'a first floor test, 5.0 for a fast straight. Empty: '
                    'gap_follow.yaml\'s max_speed (5.0 on car 1). Refused above 5.0. Below '
                    'the YAML value, min_speed, corner_speed and '
                    'corner_speed_wide are lowered with it; above it, only '
                    'max_speed rises.')

    imu_hardware_arg = DeclareLaunchArgument(
        'imu_hardware', default_value='auto',
        description='IMU source: auto uses simulated topics when gym_bridge_node is running, '
                    'otherwise real hardware. true/false explicitly select real/simulated inputs.')
    steering_damping_arg = DeclareLaunchArgument(
        'steering_damping', default_value='true',
        description='true (the default since 2026-10-27): steer against the car\'s own '
                    'rotation, using the gyro yaw rate in /imu/data. Starts imu_bridge\'s '
                    'vehicle_state_launch.py (the IMU stack). false turns it off. '
                    'Simulator-validated only, not yet run on the floor '
                    '(docs/gap-follow.md#steering-damping-gyro).')
    adaptive_grip_arg = DeclareLaunchArgument(
        'adaptive_grip', default_value='false',
        description='true: raise or cut the lateral limit behind the corner speed '
                    'caps from the gyro\'s grip ratio, starting at max_lateral_accel. '
                    'Starts imu_bridge\'s vehicle_state_launch.py (the IMU stack), '
                    'once, even when steering_damping:=true as well. Ships off; '
                    'simulator-validated only, not yet run on the floor '
                    '(docs/gap-follow.md#adaptive-grip-imu).')

    fused_speed_arg = DeclareLaunchArgument(
        'fused_speed', default_value='false',
        description='true: read speed from /odom/fused (wheels plus BNO forward acceleration). '
                    'Starts the shared IMU stack once, including with either gyro feature on. '
                    'Ships off; floor trial needed (docs/gap-follow.md#fused-speed-imu).')

    understeer_arg = DeclareLaunchArgument(
        'imu_understeer', default_value='false',
        description='true: cap speed when measured steering shows sustained understeer. '
                    'Starts the shared IMU stack once. Ships off; floor trial needed '
                    '(docs/gap-follow.md#understeer-check-imu).')

    traction_arg = DeclareLaunchArgument(
        'imu_traction', default_value='false',
        description='true: hold acceleration briefly when wheel speed exceeds fused speed. '
                    'Starts the shared IMU stack once. Ships off; floor trial needed '
                    '(docs/gap-follow.md#traction-control-imu).')

    def node(context):
        enabled = LaunchConfiguration('opponent_awareness').perform(context).lower()
        if enabled not in ('true', 'false'):
            raise RuntimeError(
                f"opponent_awareness:={enabled!r}: use true or false")
        damping = _flag('steering_damping',
                        LaunchConfiguration('steering_damping').perform(context))
        grip = _flag('adaptive_grip',
                     LaunchConfiguration('adaptive_grip').perform(context))
        fused = _flag('fused_speed',
                      LaunchConfiguration('fused_speed').perform(context))
        understeer = _flag('imu_understeer',
                          LaunchConfiguration('imu_understeer').perform(context))
        traction = _flag('imu_traction', LaunchConfiguration('imu_traction').perform(context))
        speed = speed_cap_overrides(
            yaml_parameters(gap_follow_config),
            LaunchConfiguration('max_speed').perform(context))
        actions = [Node(
            package='gap_follow',
            executable='gap_follow_node',
            name='gap_follow_node',
            output='screen',
            parameters=[gap_follow_config,
                        {'enable_opponent_awareness': enabled == 'true',
                         'enable_steering_damping': damping,
                         'enable_adaptive_grip': grip,
                         'enable_imu_understeer': understeer,
                         'enable_imu_traction': traction},
                        speed,
                        {'odom_topic': FUSED_ODOM} if fused else {}],
        )]
        if damping or grip or fused or understeer or traction:
            # All IMU features share one estimator, but simulated IMUs
            # already come from gym_bridge_node. Start real readers only
            # when selected; any combination of features still includes once.
            hardware, reason = select_imu_hardware(
                LaunchConfiguration('imu_hardware').perform(context), probe=imu_probe)
            source = 'real IMU hardware' if hardware == 'true' else 'simulated IMU topics'
            actions.append(LogInfo(msg=f'IMU source: {source} ({reason}).'))
            actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('imu_bridge'), 'launch',
                             'vehicle_state_launch.py')),
                launch_arguments={'hardware': hardware}.items()))
        return actions

    return LaunchDescription([opponent_arg, max_speed_arg, steering_damping_arg,
                              adaptive_grip_arg, fused_speed_arg, understeer_arg,
                              traction_arg, imu_hardware_arg,
                              OpaqueFunction(function=node)])
