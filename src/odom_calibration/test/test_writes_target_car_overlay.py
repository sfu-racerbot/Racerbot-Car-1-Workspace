from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from odom_calibration.calibration_node import OdomCalibrationNode
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
