"""Oracles: Task 8 path rules and the actual car calibration JSON."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from racer_imu import frames
from racer_imu.calibration import ImuCalibration
from racerbot_car import identity

CAR = Path(__file__).resolve().parents[3] / 'src/racerbot_car/cars/rb1/imu/car.yaml'


def test_calib_resolves_next_to_car_yaml(tmp_path, monkeypatch):
    config = tmp_path / 'profile/car.yaml'
    config.parent.mkdir()
    config.write_text('imus: {bno: {calib: calib/x.json}}')
    expected = config.parent / 'calib/x.json'
    expected.parent.mkdir()
    expected.write_text('{}')
    root = tmp_path / 'root'
    (root / 'calib').mkdir(parents=True)
    (root / 'calib/x.json').write_text('{"other": true}')
    monkeypatch.setattr(frames, 'ROOT', root)
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


def test_default_config_refuses_conflicting_identity(monkeypatch, tmp_path):
    machines = yaml.safe_load((CAR.parents[2] / 'machines.yaml').read_text())
    serial = tmp_path / 'serial'
    serial.write_text(next(key for key, car in machines.items() if car == 'rb2'))
    monkeypatch.setattr(identity, 'SERIAL_PATH', str(serial))
    monkeypatch.setenv('RACERBOT_CARS_DIR', str(CAR.parents[2]))
    monkeypatch.setenv('RACERBOT_CAR', 'rb1')
    with pytest.raises(FileNotFoundError, match='identifies rb2, but RACERBOT_CAR is rb1') as exc:
        frames.default_config_path()
    assert isinstance(exc.value.__cause__, identity.CarMismatchError)
    assert '--config' in str(exc.value)


@pytest.mark.parametrize('car_id', ['../rb1', 'rb9'])
def test_default_config_refuses_invalid_car_id(monkeypatch, tmp_path, car_id):
    monkeypatch.setattr(identity, 'SERIAL_PATH', str(tmp_path / 'no-serial'))
    monkeypatch.setenv('RACERBOT_CARS_DIR', str(CAR.parents[2]))
    monkeypatch.setenv('RACERBOT_CAR', car_id)
    with pytest.raises(FileNotFoundError, match='unknown RACERBOT_CAR') as exc:
        frames.default_config_path()
    # Identity must reject the value before a calibration path is constructed.
    assert isinstance(exc.value.__cause__, identity.UnknownCarError)
    assert '--config' in str(exc.value)


def test_calibration_output_stays_with_config(tmp_path, monkeypatch):
    root = tmp_path / 'root'
    (root / 'calib').mkdir(parents=True)
    (root / 'calib/x.json').write_text('{}')
    monkeypatch.setattr(frames, 'ROOT', root)
    config = tmp_path / 'car.yaml'
    config.write_text('imus: {}')
    cfg = frames.load_config(config)
    assert cfg.resolve('calib/x.json', for_write=True) == tmp_path / 'calib/x.json'
    assert cfg.resolve('calib/new.json', for_write=True) == tmp_path / 'calib/new.json'


def test_sim_calibration_stays_at_root(tmp_path, monkeypatch):
    root = tmp_path / 'root'
    monkeypatch.setattr(frames, 'ROOT', root)
    config = tmp_path / 'car.yaml'
    config.write_text('imus: {bno: {calib: calib/bno_calib.json}}')
    cfg = frames.load_config(config)
    from racer_imu.cli import use_sim_paths
    use_sim_paths(cfg)
    for writing in (False, True):
        assert cfg.resolve(cfg.imus['bno'].extra['calib'], for_write=writing) == root / 'calib/sim/bno_calib.json'
        assert cfg.resolve(cfg.section('sync')['file'], for_write=writing) == root / 'calib/sim/sync.json'
