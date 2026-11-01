"""Partial CLI updates retain untouched data from the normal read path.

Oracles: the seed JSON and unchanged root file; real calibration code runs
against the existing simulator's virtual serial ports, without physical hardware.
"""
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest

from racer_imu import frames
from racer_imu.calibration import ImuCalibration
from racer_imu.sim import SimServer
from racerbot_car import identity

TOOLS = Path(__file__).resolve().parents[1] / 'tools'


def tool(name):
    spec = importlib.util.spec_from_file_location(name, TOOLS / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def fallback(tmp_path, monkeypatch):
    root = tmp_path / 'root'
    (root / 'calib').mkdir(parents=True)
    profile = tmp_path / 'profile'
    profile.mkdir()
    config = profile / 'car.yaml'
    config.write_text('imus:\n  vesc: {calib: calib/vesc.json}\n'
                      '  bno: {calib: calib/bno.json}\n'
                      'sync: {reference: vesc, file: calib/sync.json}\n')
    monkeypatch.setattr(frames, 'ROOT', root)
    monkeypatch.setattr(identity, 'SERIAL_PATH', str(tmp_path / 'no-serial'))
    monkeypatch.setenv('RACERBOT_CAR', 'rb1')
    monkeypatch.setenv('RACERBOT_WS', str(tmp_path))
    calibration = ImuCalibration(
        A_inv=np.diag([1.04, 0.97, 1.02]), acc_bias=np.array([0.2, -0.3, 0.4]),
        gyro_bias=np.array([0.1, -0.2, 0.3]),
        dev_acc_offset=np.array([0.01, 0.02, 0.03]),
        dev_gyro_offset=np.array([0.001, 0.002, 0.003]),
        meta={'accel': {'measurement': 'seed poses'}, 'mount': {'rpy_deg': [1, 2, 3]}})
    for imu in ('bno', 'vesc'):
        calibration.save(root / f'calib/{imu}.json')
    return config, root, profile


def test_partial_gyro_keeps_root_fallback_fields(fallback, monkeypatch):
    config, root, profile = fallback
    source = root / 'calib/bno.json'
    original = source.read_text()
    seed = json.loads(original)
    module = tool('calibrate_imu')
    sim = SimServer(frames.load_config(config), 'still').start()
    try:
        monkeypatch.setattr(sys, 'argv', ['calibrate_imu', '--config', str(config),
                            '--imu', 'bno', '--step', 'gyro', '--gyro-seconds', '0.2',
                            '--bno', sim.ports['bno']])
        module.main()
    finally:
        sim.stop()
    result = json.loads((profile / 'calib/bno.json').read_text())
    for key in ('A_inv', 'acc_bias', 'device_offsets'):
        assert result[key] == seed[key]
    assert result['meta']['accel'] == seed['meta']['accel']
    assert result['meta']['mount'] == seed['meta']['mount']
    assert result['gyro_bias'] != seed['gyro_bias']
    assert result['meta']['gyro']['n'] > 0
    assert source.read_text() == original


def test_sync_keeps_root_fallback_other_offsets(fallback, monkeypatch):
    config, root, profile = fallback
    source = root / 'calib/sync.json'
    seed = {'offsets': {'vesc': 0.0, 'bno': -0.02, 'third': 0.035}}
    source.write_text(json.dumps(seed))
    original = source.read_text()
    module = tool('sync_calibrate')
    sim = SimServer(frames.load_config(config), 'shake').start()
    try:
        monkeypatch.setattr(sys, 'argv', ['sync_calibrate', '--config', str(config),
                            '--seconds', '0.2', '--vesc', sim.ports['vesc'],
                            '--bno', sim.ports['bno']])
        module.main()
    finally:
        sim.stop()
    result = json.loads((profile / 'calib/sync.json').read_text())
    assert result['offsets']['third'] == seed['offsets']['third']
    assert result['offsets']['vesc'] == 0.0
    assert result['offsets']['bno'] != seed['offsets']['bno']
    assert source.read_text() == original


@pytest.mark.parametrize('operation', ['--written', '--cleared'])
def test_vesc_offsets_keep_root_fallback_fields(fallback, monkeypatch, operation):
    config, root, profile = fallback
    source = root / 'calib/vesc.json'
    original = source.read_text()
    seed = json.loads(original)
    module = tool('vesc_offsets')
    monkeypatch.setattr(sys, 'argv', ['vesc_offsets', '--config', str(config), operation])
    module.main()
    result = json.loads((profile / 'calib/vesc.json').read_text())
    for key in ('A_inv', 'acc_bias', 'gyro_bias', 'meta'):
        assert result[key] == seed[key]
    if operation == '--written':
        np.testing.assert_allclose(result['device_offsets']['accel_g'],
                                   np.round(np.array(seed['acc_bias']) / 9.80665, 3),
                                   rtol=0, atol=1e-15)  # g, JSON round-trip precision
        np.testing.assert_allclose(result['device_offsets']['gyro_deg_s'],
                                   np.round(np.degrees(seed['gyro_bias']), 3),
                                   rtol=0, atol=1e-12)  # deg/s, radian round-trip precision
    else:
        assert result['device_offsets']['accel_g'] == [0.0, 0.0, 0.0]
        assert result['device_offsets']['gyro_deg_s'] == [0.0, 0.0, 0.0]
    assert source.read_text() == original
