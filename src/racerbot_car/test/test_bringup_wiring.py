import importlib.util
import shutil
import sys
from pathlib import Path

import pytest

pytest.importorskip('launch', reason='ROS launch is needed; source the workspace to run wiring tests')
from launch.launch_description_sources import get_launch_description_from_python_launch_file

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/racerbot_car'))
from racerbot_car import hardware, identity, layers

CARS = ROOT / 'src/racerbot_car/cars'
LAUNCH = ROOT / 'src/f1tenth_system/f1tenth_stack/launch'


@pytest.fixture
def setup_car(tmp_path, monkeypatch):
    cars = tmp_path / 'cars'
    shutil.copytree(CARS, cars)
    monkeypatch.setenv('RACERBOT_CARS_DIR', str(cars))
    monkeypatch.setattr(identity, 'SERIAL_PATH', str(tmp_path / 'missing-serial'))
    monkeypatch.setattr(hardware, 'USB_DEVICES_ROOT', tmp_path / 'usb')
    monkeypatch.delenv('RACERBOT_CAR', raising=False)
    return cars


def load(name):
    return get_launch_description_from_python_launch_file(str(LAUNCH / name))


def test_bringup_refuses_unknown_machine(setup_car):
    with pytest.raises(identity.UnknownCarError, match='RACERBOT_CAR'):
        load('bringup_launch.py')


@pytest.mark.parametrize('real', [False, True])
def test_bringup_refuses_contradicting_hardware(setup_car, monkeypatch, tmp_path, real):
    if real:
        monkeypatch.setenv('RACERBOT_CARS_DIR', str(CARS))
    monkeypatch.setenv('RACERBOT_CAR', 'rb1')
    device = tmp_path / 'usb' / '2-1'
    device.mkdir(parents=True)
    (device / 'idVendor').write_text('8086')
    (device / 'idProduct').write_text('0b3a')
    with pytest.raises(hardware.CarHardwareError, match='rb2'):
        load('bringup_launch.py')


def test_bringup_refuses_missing_calibration_key(setup_car, monkeypatch):
    monkeypatch.setenv('RACERBOT_CAR', 'rb1')
    overlay = setup_car / 'rb1/calibration/vesc.yaml'
    overlay.write_text(overlay.read_text().replace('    servo_min: 0.05\n', ''))
    with pytest.raises(layers.MissingCalibrationError, match='servo_min'):
        load('bringup_launch.py')


def test_teleop_refuses_unknown_machine(setup_car):
    with pytest.raises(identity.UnknownCarError, match='RACERBOT_CAR'):
        load('teleop_launch.py')


def test_bringup_loads_rb1_overlays(setup_car, monkeypatch):
    monkeypatch.setenv('RACERBOT_CAR', 'rb1')
    monkeypatch.setenv('RACERBOT_CARS_DIR', str(CARS))
    spec = importlib.util.spec_from_file_location('launch_params', ROOT / 'tools/dev/launch_params.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    result = module.capture(str(LAUNCH / 'bringup_launch.py'), [], str(ROOT))
    node = next(n for n in result['nodes'] if n['name'] == '/vesc_driver_node')
    files = node['param_files']
    overlay = next(i for i, name in enumerate(files) if 'rb1/calibration/vesc.yaml' in name)
    shared = next(i for i, name in enumerate(files) if 'f1tenth_stack/config/vesc.yaml' in name)
    assert shared < overlay
    # Oracle: car 1 servo clipping, docs/hardware-reference.md.
    assert node['params']['servo_min'] == 0.05
    assert node['params']['servo_max'] == 0.75
