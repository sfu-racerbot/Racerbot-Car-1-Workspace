from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from racerbot_car.guards import CalibrationOverlayError, check_calibration_overlay, overlay_blocks
from racerbot_car import paths


with pytest.MonkeyPatch.context() as patch:
    patch.delenv('RACERBOT_CARS_DIR', raising=False)
    REAL_OVERLAYS = sorted(paths.cars_dir().glob('*/calibration/*.yaml'))


@pytest.fixture
def vesc():
    return {'/**': {'ros__parameters': {
        'speed_to_erpm_gain': 1, 'steering_angle_to_servo_gain': 2,
        'steering_angle_to_servo_offset': 3, 'servo_min': 4, 'servo_max': 5}},
        'vesc_to_odom_node': {'ros__parameters': {'speed_to_erpm_gain': -1}}}


def write(tmp_path, data):
    path = tmp_path / 'overlay.yaml'
    path.write_text(yaml.safe_dump(data), encoding='utf-8')
    return path


def test_check_clean_overlay_has_no_problems(tmp_path, vesc):
    assert check_calibration_overlay(write(tmp_path, vesc), 'vesc.yaml') == []


def test_check_rejects_non_calibration_key(tmp_path, vesc):
    vesc['/**']['ros__parameters']['max_speed'] = 4
    problems = check_calibration_overlay(write(tmp_path, vesc), 'vesc.yaml')
    assert len(problems) == 1 and 'max_speed' in problems[0]


def test_check_rejects_unknown_block(tmp_path, vesc):
    vesc['vesc_driver'] = {'ros__parameters': {'max_speed': 1}}
    assert any('vesc_driver' in p for p in check_calibration_overlay(write(tmp_path, vesc), 'vesc.yaml'))


def test_check_reports_missing_required_key(tmp_path, vesc):
    del vesc['/**']['ros__parameters']['servo_min']
    assert any('servo_min' in p for p in check_calibration_overlay(write(tmp_path, vesc), 'vesc.yaml'))


def test_check_unknown_config_name(tmp_path, vesc):
    problems = check_calibration_overlay(write(tmp_path, vesc), 'mux.yaml')
    assert len(problems) == 1 and 'mux.yaml' in problems[0]


def test_check_nested_joy_key_recognised(tmp_path):
    data = {'joy_teleop': {'ros__parameters': {'human_control': {'axis_mappings': {
        'drive-steering_angle': {'scale': 0.38}}}}}}
    assert check_calibration_overlay(write(tmp_path, data), 'joy_teleop.yaml') == []


def test_overlay_blocks_rejects_missing_ros_parameters(tmp_path):
    with pytest.raises(CalibrationOverlayError, match='ros__parameters'):
        overlay_blocks(write(tmp_path, {'node': {}}))


@pytest.mark.parametrize('path', REAL_OVERLAYS)
def test_every_real_car_calibration_overlay_is_clean(path):
    assert check_calibration_overlay(path, path.name) == []


def test_real_calibration_scan_finds_the_cars_dir(monkeypatch):
    monkeypatch.delenv('RACERBOT_CARS_DIR', raising=False)
    cars = paths.cars_dir()
    assert cars.is_dir() and (cars / 'rb1').is_dir()


def test_speed_to_erpm_offset_optional_under_vesc_to_odom(tmp_path, vesc):
    assert check_calibration_overlay(write(tmp_path, vesc), 'vesc.yaml') == []
    vesc['vesc_to_odom_node']['ros__parameters']['speed_to_erpm_offset'] = 12.0
    assert check_calibration_overlay(write(tmp_path, vesc), 'vesc.yaml') == []
    vesc['/**']['ros__parameters']['speed_to_erpm_offset'] = 12.0
    assert any('speed_to_erpm_offset' in p for p in
               check_calibration_overlay(write(tmp_path, vesc), 'vesc.yaml'))
