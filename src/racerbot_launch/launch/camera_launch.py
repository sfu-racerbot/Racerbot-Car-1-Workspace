"""Start the running car's camera stream, if it has a camera."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from racerbot_car.layers import launch_car_id
from racerbot_car.profile import CameraRealSense, CameraUVC, load_profile


def generate_launch_description():
    car_id = launch_car_id()
    camera = load_profile(car_id).camera
    if camera is None:
        return LaunchDescription([LogInfo(msg=f'Car {car_id} has no camera')])
    if isinstance(camera, CameraUVC):
        launch_file = 'webcam_launch.py'
    elif isinstance(camera, CameraRealSense):
        launch_file = 'realsense_camera_launch.py'
    else:
        raise ValueError(f'Unsupported camera for car {car_id}: {camera!r}')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('racerbot_launch'), 'launch', launch_file))),
    ])
