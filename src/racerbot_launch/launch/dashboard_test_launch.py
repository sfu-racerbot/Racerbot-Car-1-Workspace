import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from racerbot_car.layers import layered, launch_car_id


def generate_launch_description():
    """Everything the web dashboard can show, minus anything that can move
    the car: LiDAR + the car's camera stream + the
    dashboard itself, all in one launch, for testing the dashboard/sensors
    without bringing up the driving stack (VESC/joy/ackermann_mux from
    f1tenth_stack's bringup_launch.py).

    None of these nodes touch /drive, /ackermann_cmd, or
    /commands/motor|servo/* -- support/tooling per
    docs/adding-your-own-code.md, same category as web_dashboard/
    usb_cam_stream already is -- so there's no
    LB-deadman check and no bringup ordering to worry about here, unlike
    the driving-code procedures in docs/operations.md.

    The dashboard itself is NOT started by default: on both cars it already
    runs at boot (car 1: web-dashboard.service; car 2: racerbot-dashboard.service),
    and a second copy would join the
    graph and then die on the busy port 8080. Add `dashboard:=true` only on
    a machine where that service isn't running.

    For actual driving/racing, don't use this file -- launch
    bringup_launch.py + a control layer as usual (see docs/operations.md),
    and the dashboard service (or dashboard_launch.py by hand) alongside.

        ros2 launch racerbot_launch dashboard_test_launch.py
        ros2 launch racerbot_launch dashboard_test_launch.py dashboard:=true   # no boot service

    Then open https://dashboard.sfuracerbot.ca and pick your car -- the car
    serves no pages itself any more.
    """
    car_id = launch_car_id()
    sensors_config = os.path.join(
        get_package_share_directory('f1tenth_stack'), 'config', 'sensors.yaml')

    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=layered(sensors_config, 'sensors.yaml', car_id),
    )
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        # Same measured mount as f1tenth_stack's bringup_launch.py.
        arguments=['0.26', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser'],
    )

    camera_stream_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('racerbot_launch'), 'launch',
                'camera_launch.py')
        )
    )
    web_dashboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('racerbot_launch'), 'launch',
                'dashboard_launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('dashboard')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'dashboard', default_value='false',
            description='Also start dashboard_launch.py. Leave false on the cars: '
                        'their boot service already runs it.'),
        urg_node,
        static_tf_node,
        camera_stream_launch,
        web_dashboard_launch,
    ])
