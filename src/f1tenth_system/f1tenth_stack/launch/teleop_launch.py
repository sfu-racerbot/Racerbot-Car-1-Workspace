import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
# racerbot: car overlays (docs/car1-vs-car2.md)
from racerbot_car.layers import car_params, launch_car_id


def generate_launch_description():
    """
    Start the manual-driving control layer: just `joy_teleop`.

    This is the teleop half of what used to be bundled into
    bringup_launch.py. Run bringup_launch.py first (in its own terminal --
    it owns joy_node, the VESC, the LiDAR, and the mux, none of which this
    file touches) and this on top of it, in a second terminal, whenever you
    want manual stick control. Leave it out entirely and launch an autonomy
    node instead (gap_follow_launch.py, pure_pursuit_launch.py) if you want
    that to drive instead -- see docs/operations.md.
    """
    car_id = launch_car_id()
    joy_teleop_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'joy_teleop.yaml'
    )

    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Description for joy_teleop config')

    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=car_params('joy_config', joy_teleop_config, 'joy_teleop.yaml', car_id)
    )

    return LaunchDescription([joy_la, joy_teleop_node])
