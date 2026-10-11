"""Car 1's USB webcam as the dashboard's MJPEG camera stream, port 9090.

    ros2 launch racerbot_launch webcam_launch.py

Exactly `ros2 launch usb_cam_stream usb_cam_stream_launch.py` with
car_config:=config/web_dashboard_rb1.yaml from this package: the package
(in the src/web_dashboards submodule) ships generic defaults, and this
car's camera device lives in that YAML. Car 1 has a UVC webcam, not a
RealSense, so there is no camera driver node to start first.

Support/tooling, not a control layer: publishes to no topic, so it is safe
to start alongside anything, at any time. Started by hand; it does not run
at boot. See docs/usb-camera-livestream.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    car_config = os.path.join(
        get_package_share_directory('racerbot_launch'), 'config', 'web_dashboard_rb1.yaml')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('usb_cam_stream'), 'launch',
                'usb_cam_stream_launch.py')),
            launch_arguments={'car_config': car_config}.items(),
        ),
    ])
