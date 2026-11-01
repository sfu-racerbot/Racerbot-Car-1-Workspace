from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from odom_calibration.calibration_node import OdomCalibrationNode
from odom_calibration import vesc_yaml
from racerbot_car import identity


@pytest.fixture
def cars(tmp_path, monkeypatch):
    root = tmp_path / 'cars'
    calibration = root / 'rb1' / 'calibration'
    calibration.mkdir(parents=True)
    (root / 'machines.yaml').write_text('{}\n')
    (root / 'rb1' / 'car.yaml').write_text('{}\n')
    for name in ('vesc.yaml', 'joy_teleop.yaml'):
        (calibration / name).write_text('{}\n')
    monkeypatch.setenv('RACERBOT_CARS_DIR', str(root))
    monkeypatch.setenv('RACERBOT_CAR', 'rb1')
    monkeypatch.setattr(identity, 'read_serial', lambda path: None)
    return calibration


def test_default_vesc_target_is_car_overlay(cars):
    assert OdomCalibrationNode._resolve_config('', 'vesc.yaml') == (cars / 'vesc.yaml').resolve()


def test_default_teleop_target_is_car_overlay(cars):
    assert OdomCalibrationNode._resolve_config('', 'joy_teleop.yaml') == (cars / 'joy_teleop.yaml').resolve()


def test_unknown_car_makes_apply_unavailable(cars, monkeypatch):
    monkeypatch.delenv('RACERBOT_CAR')
    warnings = []
    assert OdomCalibrationNode._resolve_config('', 'vesc.yaml', warnings.append) is None
    assert len(warnings) == 1 and 'RACERBOT_CAR' in warnings[0]


def test_missing_overlay_makes_apply_unavailable(cars):
    (cars / 'vesc.yaml').unlink()
    warnings = []
    assert OdomCalibrationNode._resolve_config('', 'vesc.yaml', warnings.append) is None
    assert len(warnings) == 1 and 'vesc.yaml' in warnings[0]


def test_explicit_path_still_wins(cars, tmp_path, monkeypatch):
    monkeypatch.delenv('RACERBOT_CAR')
    path = tmp_path / 'custom.yaml'
    assert OdomCalibrationNode._resolve_config(str(path), 'vesc.yaml') == path.resolve()


def test_mismatched_serial_makes_apply_unavailable(tmp_path, monkeypatch):
    root = tmp_path / 'cars'
    for car in ('rb1', 'rb2'):
        (root / car).mkdir(parents=True)
        (root / car / 'car.yaml').write_text('{}\n')
    (root / 'machines.yaml').write_text('"fake-serial": rb2\n')
    serial = tmp_path / 'serial'
    serial.write_text('fake-serial')
    monkeypatch.setattr(identity, 'SERIAL_PATH', str(serial))
    monkeypatch.setenv('RACERBOT_CARS_DIR', str(root))
    monkeypatch.setenv('RACERBOT_CAR', 'rb1')
    warnings = []
    assert OdomCalibrationNode._resolve_config('', 'vesc.yaml', warnings.append) is None
    assert len(warnings) == 1
    assert 'serial fake-serial identifies rb2, but RACERBOT_CAR is rb1' in warnings[0]


def test_nested_overlay_write_is_guarded_and_rolled_back(cars, tmp_path):
    nested = cars / 'nested'
    nested.mkdir()
    target = nested / 'vesc.yaml'
    original = (Path(__file__).resolve().parents[2] / 'racerbot_car' / 'cars' / 'rb1'
                / 'calibration' / 'vesc.yaml').read_text()
    original = original.replace('    servo_min: 0.05\n', '    servo_min: 0.05\n    port: bad\n')
    target.write_text(original)
    node = SimpleNamespace(calibration_paths_are_overlays=False)
    is_overlay = OdomCalibrationNode._is_calibration_overlay(node, target)
    with pytest.raises(ValueError, match='port'):
        vesc_yaml.apply(target, {'servo_min': 0.06}, tmp_path / 'backup',
                        target_is_overlay=is_overlay)
    assert target.read_text() == original
