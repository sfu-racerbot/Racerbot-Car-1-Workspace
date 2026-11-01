"""Run the tool's controller checks with each calibrated car selected."""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/racerbot_car'))
from racerbot_car.identity import known_car_ids  # noqa: E402
from racerbot_car.paths import cars_dir  # noqa: E402

CARS = sorted(car for car in known_car_ids(cars_dir())
              if (cars_dir() / car / 'calibration/vesc.yaml').is_file())


@pytest.fixture(autouse=True, params=CARS)
def selected_test_car(request, monkeypatch):
    monkeypatch.setenv('RACERBOT_CAR', request.param)


@pytest.fixture
def car_profiles(tmp_path, monkeypatch):
    """Two real layered profiles with distinguishable calibration and tuning."""
    from copy import deepcopy
    from types import SimpleNamespace
    import yaml
    from racerbot_car.layers import effective_config

    base = effective_config('vesc.yaml', '/**', 'rb1')
    directory = tmp_path / 'cars'
    directory.mkdir(parents=True)
    (directory / 'machines.yaml').write_text('{}\n')
    values = {}
    for car, gain, angle, accel in (('rb1', 2000.0, 0.2, 3.0),
                                     ('rb2', 4000.0, 0.3, 1.0)):
        profile = directory / car
        (profile / 'calibration').mkdir(parents=True)
        (profile / 'tuning').mkdir()
        (profile / 'car.yaml').write_text('{}\n')
        values[car] = {
            'vesc': {key: base[key] for key in (
                'speed_to_erpm_gain', 'steering_angle_to_servo_gain',
                'steering_angle_to_servo_offset', 'servo_min', 'servo_max')},
            'pure': {'max_steering_angle': angle},
            'tuning': {'max_acceleration': accel},
        }
        values[car]['vesc']['speed_to_erpm_gain'] = gain

    def write(car):
        profile = directory / car
        data = values[car]
        (profile / 'calibration/vesc.yaml').write_text(yaml.safe_dump({
            '/**': {'ros__parameters': data['vesc']},
            'vesc_to_odom_node': {'ros__parameters': {
                'speed_to_erpm_gain': -abs(data['vesc']['speed_to_erpm_gain'])}}}))
        (profile / 'calibration/pure_pursuit.yaml').write_text(yaml.safe_dump({
            'pure_pursuit_node': {'ros__parameters': data['pure']}}))
        (profile / 'tuning/pure_pursuit.yaml').write_text(yaml.safe_dump({
            'pure_pursuit_node': {'ros__parameters': data['tuning']}}))

    def set_calibration(car, **changes):
        values[car]['vesc'].update(changes)
        write(car)

    def clone(source, target):
        values[target] = deepcopy(values[source])
        write(target)

    for car in values:
        write(car)
    monkeypatch.setenv('RACERBOT_CARS_DIR', str(directory))
    return SimpleNamespace(set_calibration=set_calibration, clone=clone)
