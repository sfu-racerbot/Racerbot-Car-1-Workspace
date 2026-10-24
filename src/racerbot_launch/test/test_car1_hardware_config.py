"""
Car 1's hardware differences from car 2 survive a port from the car 2 repo.

The two cars share almost all code, and car 2's workspace is ported into
this one by hand (docs/car1-vs-car2.md). A port that copies car 2's files
over car 1's fails *silently*: urg_node starts and waits on an Ethernet
address car 1 does not have, the doctor reports a RealSense missing, and
nothing errors at build time. This file makes those mistakes fail in pytest
instead of on the car.

Spec (the oracle for every assertion): docs/car1-vs-car2.md, which cites
docs/hardware-reference.md and the udev rules in /etc/udev/rules.d/.
Where two files must agree, the test reads both and compares them, so it
checks consistency rather than memorising a value.

No ROS needed:

    python3 -m pytest src/racerbot_launch/test/ -v
"""
import glob
import importlib.util
import os
import re

import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_HERE))
_STACK = os.path.join(_SRC, 'f1tenth_system', 'f1tenth_stack')
_RB1 = os.path.join(_SRC, 'racerbot_launch', 'config', 'web_dashboard_rb1.yaml')
_DOCTOR_CAR = os.path.join(_SRC, 'racerbot_doctor', 'racerbot_doctor', 'car.py')
_HW_DOC = os.path.join(_SRC, '..', 'docs', 'hardware-reference.md')


def _yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _doctor_car():
    spec = importlib.util.spec_from_file_location('doctor_car', _DOCTOR_CAR)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _urg_params():
    return _yaml(os.path.join(_STACK, 'config', 'sensors.yaml'))['urg_node'][
        'ros__parameters']


# ------------------------------------------------------------------ LiDAR

def test_urg_node_uses_the_usb_hokuyo_not_ethernet():
    # Car 1: USB UST-30LX at /dev/sensors/hokuyo. Car 2: Ethernet UST-10LX.
    # urg_node prefers ip_address when it is set, so it must be absent.
    params = _urg_params()
    assert params.get('serial_port') == '/dev/sensors/hokuyo'
    assert 'ip_address' not in params
    assert 'ip_port' not in params


def test_hokuyo_usb_id_agrees_between_doctor_and_hardware_doc():
    # hardware-reference.md quotes the udev rule's idVendor/idProduct.
    with open(_HW_DOC) as f:
        doc = f.read()
    m = re.search(r'99-hokuyo\.rules` .{0,40}idVendor `([0-9a-f]{4})`, '
                  r'idProduct `([0-9a-f]{4})`', doc)
    assert m, 'hardware-reference.md no longer states the Hokuyo USB id'
    assert _doctor_car().LIDAR_USB == (m.group(1), m.group(2))


def test_doctor_lidar_symlink_is_the_one_urg_node_opens():
    assert '/dev/' + _doctor_car().LIDAR_SYMLINK == _urg_params()['serial_port']


def test_doctor_has_no_ethernet_lidar_settings():
    car = _doctor_car()
    for car2_only in ('LIDAR_IFACE', 'LIDAR_IP', 'LIDAR_PORT', 'LIDAR_HOST_IP'):
        assert not hasattr(car, car2_only), car2_only


# ----------------------------------------------------------------- camera

def test_doctor_camera_device_is_the_dashboard_webcam_device():
    device = _yaml(_RB1)['usb_cam_stream_node']['ros__parameters']['device']
    assert '/dev/' + _doctor_car().CAMERA_DEVICE == device


def test_no_realsense_anywhere_in_car_1_launch_or_doctor():
    # Car 1 has no RealSense (docs/hardware-reference.md#camera--uvc-usb-webcam).
    car = _doctor_car()
    assert not hasattr(car, 'EXE_REALSENSE')
    assert not hasattr(car, 'CAMERA_USB')
    launches = (glob.glob(os.path.join(_SRC, 'racerbot_launch', 'launch', '*.py'))
                + glob.glob(os.path.join(_STACK, 'launch', '*.py')))
    assert launches, 'found no launch files: the paths in this test are wrong'
    hits = []
    for path in launches:
        with open(path) as f:
            # the driver, its launch file or package -- not prose saying
            # "car 1 has no RealSense"
            if re.search(r'realsense2_camera|realsense_camera_launch|'
                         r'realsense-ros', f.read()):
                hits.append(os.path.basename(path))
    assert hits == []
    assert not os.path.exists(os.path.join(_SRC, 'realsense-ros'))


# -------------------------------------------------------- dashboard site id

def test_dashboard_launch_loads_car_1s_settings():
    # Car 1 is dashboard id rb1; a ported car 2 launch file would load rb2.
    path = os.path.join(_SRC, 'racerbot_launch', 'launch', 'dashboard_launch.py')
    with open(path) as f:
        configs = set(re.findall(r"'(web_dashboard_rb\d+\.yaml)'", f.read()))
    assert configs == {'web_dashboard_rb1.yaml'}


# ------------------------------------------------------------- Steering

def _hw_doc():
    with open(_HW_DOC) as f:
        return f.read()


def _doc_number(pattern):
    m = re.search(pattern, _hw_doc())
    assert m, f'hardware-reference.md no longer matches {pattern!r}'
    return float(m.group(1))


def test_vesc_steering_calibration_is_car_1s_measured_one():
    # A port that takes car 2's vesc.yaml (its own circles, different servo)
    # breaks this. Oracle: the vesc.yaml block quoted in hardware-reference.md.
    shared = _yaml(os.path.join(_STACK, 'config', 'vesc.yaml'))['/**'][
        'ros__parameters']
    for key in ('steering_angle_to_servo_gain', 'steering_angle_to_servo_offset',
                'servo_min', 'servo_max'):
        documented = _doc_number(r'(?m)^' + key + r':\s*(-?[0-9.]+)')
        assert shared[key] == documented, key


def test_full_stick_and_autonomy_limits_are_car_1s():
    # hardware-reference.md's "Steering limits" bullets are car 1's numbers;
    # car 2's 0.436 full stick or 0.349 limit fail here.
    joy = _yaml(os.path.join(_STACK, 'config', 'joy_teleop.yaml'))['joy_teleop'][
        'ros__parameters']
    scale = joy['human_control']['axis_mappings']['drive-steering_angle']['scale']
    assert scale == _doc_number(r'Full stick \(`joy_teleop\.yaml` scale\) is \*\*([0-9.]+) rad')
    controller = _doc_number(r'`max_steering_angle` \*\*([0-9.]+) rad')
    for pkg, cfg, node in (('gap_follow', 'gap_follow.yaml', 'gap_follow_node'),
                           ('pure_pursuit', 'pure_pursuit.yaml', 'pure_pursuit_node')):
        params = _yaml(os.path.join(_SRC, pkg, 'config', cfg))[node]['ros__parameters']
        assert params['max_steering_angle'] == controller, cfg
    planner = _yaml(os.path.join(_SRC, 'pure_pursuit', 'config', 'auto_map_race.yaml'))[
        'auto_map_race_node']['ros__parameters']['profile_max_steering_angle']
    assert planner == _doc_number(r'`profile_max_steering_angle` is \*\*([0-9.]+) rad')


# -------------------------------------------------------------------- IMU

_IMU_BRIDGE = os.path.join(_SRC, 'imu_bridge')


def _read(*parts):
    with open(os.path.join(*parts)) as f:
        return f.read()


def test_bno_is_read_through_the_jetson_i2c_bridge():
    # Car 1's BNO085 is on the Jetson's I2C bus (bus 7, 0x4A), read through
    # imu-calibration's tools/bno_i2c_bridge.py onto a pty; car 2's sits
    # behind a microcontroller on a USB serial port. A ported car 2 launch
    # would point imu_bridge at /dev/ttyACM1, which car 1 does not have.
    launch = _read(_IMU_BRIDGE, 'launch', 'vehicle_state_launch.py')
    assert "BRIDGE_SCRIPT = 'bno_i2c_bridge.py'" in launch
    # Dynamic calibration off: it shifted offsets mid-run and spoiled car 1's
    # first BNO calibration (imu-calibration calib/bno_calib_run1_dyncal_on.json).
    assert "'--no-dyncal'" in launch
    assert "'bno_pty', default_value='/tmp/bno085'" in launch
    assert "'bno_port': pty if bno else ''" in launch


def test_imu_bridge_restarts_the_bno_driver():
    # Car 1 only: the bridge's pty appears after imu_bridge starts, and
    # racer_imu's BnoDriver gives up for good on a missing port. Car 2's
    # imu_bridge_node (a USB port that exists at boot) has no watchdog.
    node = _read(_IMU_BRIDGE, 'imu_bridge', 'imu_bridge_node.py')
    assert 'BNO_RESTART_AFTER_S' in node
    assert 'self.create_timer(1.0, self._watch_bno)' in node


def test_map_course_runs_car_1s_imu_stack():
    launch = _read(_SRC, 'racerbot_launch', 'launch', 'map_course_launch.py')
    assert "'launch', 'vehicle_state_launch.py'" in launch


def test_vehicle_state_geometry_is_car_1s():
    # Same oracle as the rest of this file's geometry: vesc.yaml, which
    # hardware-reference.md quotes. The BNO sits on the front axle.
    wheelbase = _yaml(os.path.join(_STACK, 'config', 'vesc.yaml'))[
        'vesc_to_odom_node']['ros__parameters']['wheelbase']
    params = _yaml(os.path.join(_IMU_BRIDGE, 'config', 'vehicle_state.yaml'))[
        'vehicle_state_node']['ros__parameters']
    assert params['wheelbase'] == wheelbase
    assert params['bno_x_offset'] == wheelbase
