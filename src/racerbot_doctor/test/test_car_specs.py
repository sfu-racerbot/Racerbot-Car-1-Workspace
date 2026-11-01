"""Hardware spec oracles from each car's original doctor configuration."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'racerbot_car'))

from racerbot_car.profile import load_profile  # noqa: E402
from racerbot_doctor import car  # noqa: E402


def test_spec_for_rb1_is_car_1s_constants():
    spec = car.spec_for(load_profile('rb1'))
    assert spec.lidar == car.SerialLidarSpec(
        ('15d1', '0000'), 'sensors/hokuyo', 'cdc_acm')  # from car 1's car.py
    assert spec.camera == car.UvcCameraSpec(
        'video0', 'uvcvideo', 'camera_stream_node')  # from car 1's car.py


def test_spec_for_rb2_is_car_2s_constants():
    spec = car.spec_for(load_profile('rb2'))
    assert spec.lidar == car.EthernetLidarSpec(
        'enP8p1s0', '192.168.0.15/24', '192.168.0.10', 10940)  # from car 2's car.py
    assert spec.camera == car.RealSenseSpec(
        ('8086', '0b3a'), 'uvcvideo', 5000.0,
        '/camera/camera/color/image_raw', 8.0,
        'realsense2_camera_node')  # from car 2's car.py


def test_lb_button_is_the_teleop_deadman():
    # Oracle: joy_teleop.yaml's human_control deadman_buttons.
    import yaml
    path = os.path.join(os.path.dirname(__file__), '..', '..', 'f1tenth_system',
                        'f1tenth_stack', 'config', 'joy_teleop.yaml')
    with open(path) as f:
        params = yaml.safe_load(f)['joy_teleop']['ros__parameters']
    assert params['human_control']['deadman_buttons'] == [car.JOY_LB_BUTTON]
