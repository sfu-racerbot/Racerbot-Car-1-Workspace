"""The web dashboard's car-side server, with this car's settings.

    ros2 launch racerbot_launch dashboard_launch.py

Exactly `ros2 launch web_dashboard web_dashboard_launch.py` with
car_config:=config/web_dashboard_<car>.yaml from this package, where <car> is
the car this machine is (racerbot_car: its serial, or RACERBOT_CAR; an unknown
or conflicting machine refuses to launch). The web_dashboard package (in the
src/web_dashboards submodule) ships generic defaults, and each car's site,
geometry, tunable nodes, stoppable processes and map folders live in its
YAML: web_dashboard_rb1.yaml for car 1, web_dashboard_rb2.yaml for car 2.
Open it through https://dashboard.sfuracerbot.ca.

Support/tooling, not a control layer: publishes to no topic, so it is safe
to start alongside anything, at any time. Both cars run it at boot (car 1:
web-dashboard.service in tools/systemd/; car 2: racerbot-dashboard.service in
this package's systemd/). Stop that before starting this by hand, or the second
copy can't get port 8080. See docs/web-dashboard.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from racerbot_car.layers import launch_car_id


def generate_launch_description():
    car_id = launch_car_id()
    car_config = os.path.join(
        get_package_share_directory('racerbot_launch'), 'config', f'web_dashboard_{car_id}.yaml')
    if not os.path.isfile(car_config):
        raise FileNotFoundError(
            f'no dashboard settings for car {car_id}: expected {car_config}')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('web_dashboard'), 'launch',
                'web_dashboard_launch.py')),
            launch_arguments={'car_config': car_config}.items(),
        ),
    ])
