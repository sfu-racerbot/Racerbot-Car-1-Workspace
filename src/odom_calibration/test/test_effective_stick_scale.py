"""The wizard displays the selected car's validated effective stick scale."""

from types import SimpleNamespace

import pytest
import yaml

from odom_calibration.calibration_node import OdomCalibrationNode


@pytest.fixture
def scale_files(tmp_path, monkeypatch):
    cars = tmp_path / 'cars'
    car = cars / 'rb1'
    (car / 'calibration').mkdir(parents=True)
    (car / 'car.yaml').write_text('{}\n')
    (cars / 'machines.yaml').write_text('{}\n')
    monkeypatch.setenv('RACERBOT_CARS_DIR', str(cars))
    monkeypatch.setenv('RACERBOT_CAR', 'rb1')
    shared = tmp_path / 'joy_teleop.yaml'
    overlay = car / 'calibration' / 'joy_teleop.yaml'
    warnings = []
    node = SimpleNamespace(joy_teleop_yaml_path=shared,
                           get_logger=lambda: SimpleNamespace(warning=warnings.append))

    def write(path, value):
        path.write_text(yaml.safe_dump({'joy_teleop': {'ros__parameters': {
            'human_control': {'axis_mappings': {'drive-steering_angle': value}}}}}))

    write(shared, {'scale': 0.1})
    return node, overlay, warnings, write


def test_effective_scale_uses_overlay(scale_files):
    node, overlay, warnings, write = scale_files
    write(overlay, {'scale': 0.4})
    assert OdomCalibrationNode._file_stick_scale(node) == 0.4
    assert warnings == []


@pytest.mark.parametrize('value', [float('nan'), float('inf'), True, '0.4'],
                         ids=['nan', 'inf', 'bool', 'string'])
def test_invalid_effective_scale_is_rejected(scale_files, value):
    node, overlay, warnings, write = scale_files
    write(overlay, {'scale': value})
    assert OdomCalibrationNode._file_stick_scale(node) is None
    assert len(warnings) == 1 and 'Cannot read the steering scale' in warnings[0]


def test_missing_scale_is_rejected(scale_files):
    node, overlay, warnings, write = scale_files
    write(overlay, {})
    assert OdomCalibrationNode._file_stick_scale(node) is None
    assert len(warnings) == 1 and 'Cannot read the steering scale' in warnings[0]


def test_unresolvable_car_is_rejected(scale_files, monkeypatch):
    node, overlay, warnings, write = scale_files
    write(overlay, {'scale': 0.4})
    monkeypatch.setenv('RACERBOT_CAR', 'rb999')
    assert OdomCalibrationNode._file_stick_scale(node) is None
    assert len(warnings) == 1 and 'Cannot read the steering scale' in warnings[0]
