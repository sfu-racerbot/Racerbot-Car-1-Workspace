"""Task 8: selection/refusal and byte-source calibration oracles."""
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/imu_bridge'))
sys.path.insert(0, str(ROOT / 'src/racerbot_car'))
from imu_bridge.convert import load_mount, load_calibration
from racerbot_car import identity, hardware

CAR = ROOT / 'src/racerbot_car/cars'


@pytest.fixture(params=['rb1', 'rb2'])
def car(request, monkeypatch, tmp_path):
    monkeypatch.setenv('RACERBOT_CAR', request.param)
    monkeypatch.setenv('RACERBOT_CARS_DIR', str(CAR))
    monkeypatch.setattr(identity, 'SERIAL_PATH', str(tmp_path / 'no-serial'))
    monkeypatch.setattr(hardware, 'USB_DEVICES_ROOT', tmp_path / 'usb')
    return request.param


def test_default_car_yaml_is_car_profile(car):
    from imu_bridge.imu_bridge_node import resolve_car_yaml
    assert resolve_car_yaml('') == CAR / car / 'imu/car.yaml'


def test_missing_car_imu_refuses(monkeypatch, tmp_path):
    from imu_bridge.imu_bridge_node import resolve_car_yaml
    (tmp_path / 'rb1').mkdir()
    (tmp_path / 'rb1/car.yaml').write_text('{}')
    (tmp_path / 'machines.yaml').write_text('{}')
    monkeypatch.setenv('RACERBOT_CARS_DIR', str(tmp_path))
    expected = tmp_path / 'rb1/imu/car.yaml'
    with pytest.raises(FileNotFoundError, match=str(expected)):
        resolve_car_yaml('')


def capture(filename):
    spec = importlib.util.spec_from_file_location('imu_launch_capture', ROOT / 'tools/dev/launch_params.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, lambda: module.capture(str(ROOT / 'src/imu_bridge/launch' / filename), [], str(ROOT))


def test_launch_passes_car_yaml(car):
    _, run = capture('vehicle_state_launch.py')
    result = run()
    node = next(n for n in result['nodes'] if n['name'] == '/imu_bridge_node')
    assert node['params']['car_yaml'].endswith(f'cars/{car}/imu/car.yaml')


@pytest.mark.parametrize('filename', ['vehicle_state_launch.py', 'imu_bridge_launch.py'])
def test_imu_launch_refuses_unknown_machine(car, monkeypatch, filename):
    monkeypatch.delenv('RACERBOT_CAR')
    module, run = capture(filename)
    with pytest.raises(module.CaptureError, match='RACERBOT_CAR'):
        run()


@pytest.mark.parametrize('car_id', ['rb1', 'rb2'])
def test_bridge_loads_car_bno_calibration(car_id):
    car_yaml = CAR / car_id / 'imu/car.yaml'
    mount, path = load_mount(car_yaml, 'bno')
    cal = load_calibration(path, mount)
    data = json.loads((car_yaml.parent / 'calib/bno_calib.json').read_text())
    assert cal.source == str(car_yaml.parent / 'calib/bno_calib.json')
    for key in ('A_inv', 'acc_bias', 'gyro_bias'):
        np.testing.assert_array_equal(getattr(cal, key), data[key])


def test_rb2_imu_car_yaml_agrees_with_rb2_calibration():
    import yaml

    car_yaml = yaml.safe_load((CAR / 'rb2/imu/car.yaml').read_text())
    vesc = yaml.safe_load((CAR / 'rb2/calibration/vesc.yaml').read_text())
    assert car_yaml['car']['wheelbase'] == 0.36
    assert car_yaml['car']['speed_to_erpm_gain'] == vesc['/**']['ros__parameters']['speed_to_erpm_gain']
