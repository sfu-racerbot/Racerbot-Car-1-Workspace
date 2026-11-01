import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from racerbot_car.layers import car_params, launch_car_id


def generate_launch_description():
    car_id = launch_car_id()
    config_path = os.path.join(
        get_package_share_directory('odom_calibration'),
        'config',
        'odom_calibration.yaml',
    )
    return LaunchDescription([
        # Another car, or the end-to-end test, passes its own copy:
        #   ros2 launch odom_calibration odom_calibration_launch.py config:=/path.yaml
        DeclareLaunchArgument(
            'config', default_value=config_path,
            description='Parameter file for odom_calibration_node.'),
        Node(
            package='odom_calibration',
            executable='calibration_node',
            name='odom_calibration_node',
            output='screen',
            parameters=car_params('config', config_path, 'odom_calibration.yaml', car_id),
            # The wizard may run under a different Linux account from
            # bringup. Fast DDS shared memory cannot cross that account
            # boundary; UDP keeps discovery and telemetry working for both.
            additional_env={'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4'},
        ),
    ])
