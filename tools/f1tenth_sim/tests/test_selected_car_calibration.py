"""Selected-car VESC values must reach the simulator without import-time I/O."""

import os
from pathlib import Path
import subprocess
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools/f1tenth_sim'))
sys.path.insert(0, str(ROOT / 'src/racerbot_car'))

from racerbot_car.layers import effective_config  # noqa: E402
from racerbot_car import identity  # noqa: E402
from racerbot_car.identity import known_car_ids  # noqa: E402
from racerbot_car.paths import cars_dir  # noqa: E402
from sim_fidelity import calibration  # noqa: E402

CARS = sorted(car for car in known_car_ids(cars_dir())
              if (cars_dir() / car / 'calibration/vesc.yaml').is_file())


@pytest.mark.parametrize('car', CARS)
def test_calibration_matches_effective_vesc(car):
    # Oracle: eRPM / abs(gain), and the affine servo map in
    # docs/hardware-reference.md (car 1's tape/circle calibration, 2026-10-21).
    vesc = effective_config('vesc.yaml', '/**', car)
    throttle = effective_config('vesc.yaml', 'throttle_interpolator', car)
    cal = calibration.calibration_for_car(car)
    for key in ('speed_to_erpm_gain', 'steering_angle_to_servo_gain',
                'steering_angle_to_servo_offset', 'servo_min', 'servo_max'):
        assert getattr(cal, key) == vesc[key]
    assert cal.speed_max == pytest.approx(vesc['speed_max'] / abs(vesc['speed_to_erpm_gain']), abs=1e-12)
    assert cal.speed_min == -cal.speed_max
    assert cal.steering_rate_max == throttle['max_servo_speed']


def test_default_calibration_resolves_at_call_time(monkeypatch):
    monkeypatch.delenv('RACERBOT_CAR', raising=False)
    from racerbot_car.identity import UnknownCarError
    with pytest.raises(UnknownCarError, match='RACERBOT_CAR'):
        calibration.CarCalibration()
    monkeypatch.setenv('RACERBOT_CAR', 'rb1')
    assert calibration.CarCalibration() == calibration.calibration_for_car('rb1')


def test_selected_car_calibration_uses_overlay(tmp_path, monkeypatch):
    # Oracle: VESC conversion and clipping from the selected car's parameters;
    # car 1's measured VESC calibration is documented in docs/hardware-reference.md.
    cars = tmp_path / 'cars'
    overlay = cars / 'rb1/calibration/vesc.yaml'
    overlay.parent.mkdir(parents=True)
    (cars / 'rb1/car.yaml').write_text('{}\n')
    overlay.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'speed_to_erpm_gain': 2000.0, 'steering_angle_to_servo_gain': -0.5,
        'steering_angle_to_servo_offset': 0.4, 'servo_min': 0.1, 'servo_max': 0.9}},
        'vesc_to_odom_node': {'ros__parameters': {'speed_to_erpm_gain': -2000.0}}}))
    monkeypatch.setenv('RACERBOT_CARS_DIR', str(cars))
    vesc = effective_config('vesc.yaml', '/**', 'rb1')
    throttle = effective_config('vesc.yaml', 'throttle_interpolator', 'rb1')
    cal = calibration.calibration_for_car('rb1')
    assert cal.speed_to_erpm_gain == vesc['speed_to_erpm_gain'] == 2000.0
    assert cal.steering_angle_to_servo_gain == vesc['steering_angle_to_servo_gain'] == -0.5
    assert cal.steering_angle_to_servo_offset == vesc['steering_angle_to_servo_offset'] == 0.4
    assert cal.servo_min == vesc['servo_min'] == 0.1
    assert cal.servo_max == vesc['servo_max'] == 0.9
    assert cal.speed_max == pytest.approx(vesc['speed_max'] / abs(vesc['speed_to_erpm_gain']), abs=1e-12)
    assert cal.speed_min == pytest.approx(-cal.speed_max, abs=1e-12)
    assert cal.steering_rate_max == throttle['max_servo_speed']


def test_import_without_car_and_entry_point_refuses():
    env = dict(os.environ)
    env.pop('RACERBOT_CAR', None)
    code = ('import sim_fidelity.calibration as c; '
            'print("imported"); c.main([])')
    result = subprocess.run([sys.executable, '-c', code], cwd=ROOT / 'tools/f1tenth_sim',
                            env=env, text=True, capture_output=True, check=False)
    assert 'imported' in result.stdout
    assert result.returncode != 0
    assert 'RACERBOT_CAR' in result.stderr


def _set_machine_identity(tmp_path, monkeypatch, car):
    machines = yaml.safe_load((cars_dir() / 'machines.yaml').read_text())
    serial = tmp_path / 'serial'
    serial.write_text(next(key for key, value in machines.items() if value == car))
    monkeypatch.setattr(identity, 'SERIAL_PATH', str(serial))


def test_implicit_selection_refuses_serial_mismatch(tmp_path, monkeypatch):
    _set_machine_identity(tmp_path, monkeypatch, 'rb2')
    monkeypatch.setenv('RACERBOT_CAR', 'rb1')
    with pytest.raises(identity.CarMismatchError, match='identifies rb2, but RACERBOT_CAR is rb1'):
        calibration.selected_car()


def test_explicit_car_may_differ_from_machine(tmp_path, monkeypatch, capsys):
    _set_machine_identity(tmp_path, monkeypatch, 'rb2')
    monkeypatch.delenv('RACERBOT_CAR', raising=False)
    selected = calibration.selected_car('rb1')
    assert selected == 'rb1'
    assert calibration.calibration_for_car(selected).speed_to_erpm_gain == effective_config(
        'vesc.yaml', '/**', 'rb1')['speed_to_erpm_gain']
    assert 'Machine identity is rb2; explicitly selected rb1' in capsys.readouterr().out


def test_explicit_unknown_car_refuses():
    with pytest.raises(identity.UnknownCarError, match="unknown car 'rb9'"):
        calibration.selected_car('rb9')
