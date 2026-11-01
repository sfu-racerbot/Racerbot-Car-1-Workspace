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
from sim_fidelity import calibration  # noqa: E402


def test_selected_car_calibration_uses_overlay(tmp_path, monkeypatch):
    # Oracle: VESC conversion and clipping from the selected car's parameters;
    # car 1's measured VESC calibration is documented in docs/hardware-reference.md.
    cars = tmp_path / 'cars'
    overlay = cars / 'rb1/calibration/vesc.yaml'
    overlay.parent.mkdir(parents=True)
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
