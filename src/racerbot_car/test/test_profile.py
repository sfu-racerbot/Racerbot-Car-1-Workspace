import os
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from racerbot_car import profile

REAL_CARS = Path(__file__).resolve().parents[1] / 'cars'
FIXTURES = Path(__file__).resolve().parent / 'fixtures' / 'cars_ok'


def write_profile(tmp_path, data, car='rb1'):
    folder = tmp_path / car
    folder.mkdir(exist_ok=True)
    (folder / 'car.yaml').write_text(yaml.safe_dump(data))


@pytest.fixture
def rb1_data():
    return yaml.safe_load((REAL_CARS / 'rb1/car.yaml').read_text())


def test_profile_rb1_matches_car1_facts():
    # Oracle: docs/hardware-reference.md and docs/car1-vs-car2.md.
    p = profile.load_profile('rb1', REAL_CARS)
    assert p.id == 'rb1'
    assert p.name == 'car 1'
    assert p.lidar == profile.LidarSerial('/dev/sensors/hokuyo', '15d1:0000')
    assert p.camera == profile.CameraUVC('/dev/video0')
    assert p.dashboard_site == 'rb1'
    assert p.hostnames == {'remote_desktop': 'rd-rb1', 'ssh': 'ssh-rb1'}
    assert p.distinctive_usb_ids == ('15d1:0000',)


def test_profile_rb2_fixture_ethernet_realsense():
    # Oracle: task's car 2 hardware facts.
    p = profile.load_profile('rb2', FIXTURES)
    assert p.id == 'rb2'
    assert p.name == 'car 2'
    assert p.lidar == profile.LidarEthernet('192.168.0.10', 10940, 'enP8p1s0', '192.168.0.15/24')
    assert p.camera == profile.CameraRealSense('8086:0b3a', '/camera/camera/color/image_raw')
    assert p.dashboard_site == 'rb2'
    assert p.hostnames == {}
    assert p.distinctive_usb_ids == ('8086:0b3a',)


def test_profile_unknown_top_key_rejected(tmp_path, rb1_data):
    rb1_data['surprise'] = 'x'
    write_profile(tmp_path, rb1_data)
    with pytest.raises(profile.ProfileError, match='car.yaml.*surprise'):
        profile.load_profile('rb1', tmp_path)


def test_profile_missing_lidar_rejected(tmp_path, rb1_data):
    del rb1_data['lidar']
    write_profile(tmp_path, rb1_data)
    with pytest.raises(profile.ProfileError, match='car.yaml.*lidar'):
        profile.load_profile('rb1', tmp_path)


def test_profile_wrong_lidar_kind_rejected(tmp_path, rb1_data):
    rb1_data['lidar']['kind'] = 'laser'
    write_profile(tmp_path, rb1_data)
    with pytest.raises(profile.ProfileError, match='car.yaml.*lidar.kind'):
        profile.load_profile('rb1', tmp_path)


def test_profile_extra_key_for_kind_rejected(tmp_path, rb1_data):
    rb1_data['lidar']['ip'] = '192.168.0.10'
    write_profile(tmp_path, rb1_data)
    with pytest.raises(profile.ProfileError, match='car.yaml.*lidar.ip'):
        profile.load_profile('rb1', tmp_path)


def test_profile_ethernet_port_must_be_int(tmp_path, rb1_data):
    rb1_data['lidar'] = {'kind': 'ethernet', 'ip': 'x', 'port': '10940', 'iface': 'x', 'host_ip': 'x'}
    write_profile(tmp_path, rb1_data)
    with pytest.raises(profile.ProfileError, match='car.yaml.*lidar.port'):
        profile.load_profile('rb1', tmp_path)


def test_profile_bad_usb_id_rejected(tmp_path, rb1_data):
    rb1_data['lidar']['usb_id'] = '15D1:0000'
    write_profile(tmp_path, rb1_data)
    with pytest.raises(profile.ProfileError, match='car.yaml.*lidar.usb_id'):
        profile.load_profile('rb1', tmp_path)


def test_profile_camera_none(tmp_path, rb1_data):
    rb1_data['camera'] = {'kind': 'none'}
    write_profile(tmp_path, rb1_data)
    assert profile.load_profile('rb1', tmp_path).camera is None


def test_profile_missing_car_dir_rejected(tmp_path):
    with pytest.raises(profile.ProfileError, match='car.yaml'):
        profile.load_profile('rb9', tmp_path)


def test_all_profiles_sorted(tmp_path, rb1_data):
    write_profile(tmp_path, rb1_data, 'rb2')
    write_profile(tmp_path, rb1_data, 'rb1')
    assert [p.id for p in profile.all_profiles(tmp_path)] == ['rb1', 'rb2']


@pytest.mark.parametrize('key', ['name', 'camera', 'dashboard_site', 'distinctive_usb_ids'])
def test_other_required_fields_rejected(tmp_path, rb1_data, key):
    del rb1_data[key]
    write_profile(tmp_path, rb1_data)
    with pytest.raises(profile.ProfileError, match='car.yaml.*' + key):
        profile.load_profile('rb1', tmp_path)


@pytest.mark.parametrize(('section', 'value', 'key'), [
    ('lidar', 'serial', 'lidar'),
    ('lidar', {'kind': 'serial', 'usb_id': '15d1:0000'}, 'lidar.port'),
    ('lidar', {'kind': 'serial', 'port': 3, 'usb_id': '15d1:0000'}, 'lidar.port'),
    ('lidar', {'kind': 'ethernet', 'ip': 'x', 'port': True, 'iface': 'x', 'host_ip': 'x'}, 'lidar.port'),
    ('camera', {'kind': 'unknown'}, 'camera.kind'),
    ('camera', {'kind': 'none', 'device': '/dev/video0'}, 'camera.device'),
    ('camera', {'kind': 'uvc'}, 'camera.device'),
    ('camera', {'kind': 'realsense', 'usb_id': 'BAD', 'topic': '/image'}, 'camera.usb_id'),
    ('camera', {'kind': 'realsense', 'usb_id': '8086:0b3a', 'topic': 3}, 'camera.topic'),
    ('hostnames', {'ssh': 3}, 'hostnames.ssh'),
    ('distinctive_usb_ids', ['invalid'], 'distinctive_usb_ids'),
    ('distinctive_usb_ids', '15d1:0000', 'distinctive_usb_ids'),
    ('name', 3, 'name'),
    ('dashboard_site', 3, 'dashboard_site'),
])
def test_field_types_and_camera_refusals(tmp_path, rb1_data, section, value, key):
    rb1_data[section] = value
    write_profile(tmp_path, rb1_data)
    with pytest.raises(profile.ProfileError, match='car.yaml.*' + key):
        profile.load_profile('rb1', tmp_path)
