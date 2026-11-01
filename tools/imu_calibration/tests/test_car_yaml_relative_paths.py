"""Oracles: Task 8 path rules and the actual car calibration JSON."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from racer_imu import frames
from racer_imu.calibration import ImuCalibration

CAR = Path(__file__).resolve().parents[3] / 'src/racerbot_car/cars/rb1/imu/car.yaml'


def test_calib_resolves_next_to_car_yaml(tmp_path):
    config = tmp_path / 'profile/car.yaml'
    config.parent.mkdir()
    config.write_text('imus: {bno: {calib: calib/x.json}}')
    expected = config.parent / 'calib/x.json'
    expected.parent.mkdir()
    expected.write_text('{}')
    cfg = frames.load_config(config)
    assert cfg.resolve(cfg.imus['bno'].extra['calib']) == expected


def test_calib_falls_back_to_root(tmp_path, monkeypatch):
    root = tmp_path / 'root'
    (root / 'calib').mkdir(parents=True)
    expected = root / 'calib/x.json'
    expected.write_text('{}')
    monkeypatch.setattr(frames, 'ROOT', root)
    config = tmp_path / 'profile/car.yaml'
    config.parent.mkdir()
    config.write_text('imus: {bno: {calib: calib/x.json}}')
    cfg = frames.load_config(config)
    assert cfg.resolve(cfg.imus['bno'].extra['calib']) == expected


def test_loaded_bno_calibration_values_match_file():
    cfg = frames.load_config(CAR)
    cal = ImuCalibration.load_or_identity(cfg.resolve(cfg.imus['bno'].extra['calib']))
    data = json.loads((CAR.parent / 'calib/bno_calib.json').read_text())
    for key in ('A_inv', 'acc_bias', 'gyro_bias'):
        np.testing.assert_array_equal(getattr(cal, key), data[key])


def test_default_config_is_car_profile(monkeypatch):
    monkeypatch.setenv('RACERBOT_CAR', 'rb1')
    assert frames.default_config_path() == CAR


def test_default_config_refuses_unknown_car(monkeypatch):
    monkeypatch.setenv('RACERBOT_CAR', 'rbX')
    with pytest.raises(FileNotFoundError, match='RACERBOT_CAR'):
        frames.default_config_path()
