"""Run the tool's controller checks with each calibrated car selected."""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/racerbot_car'))
from racerbot_car.identity import known_car_ids  # noqa: E402
from racerbot_car.paths import cars_dir  # noqa: E402

CARS = sorted(car for car in known_car_ids(cars_dir())
              if (cars_dir() / car / 'calibration/vesc.yaml').is_file())


def profile_text(car, directory=None):
    return ((directory or cars_dir()) / car / 'car.yaml').read_text()


@pytest.fixture(autouse=True, params=CARS)
def selected_test_car(request, monkeypatch):
    monkeypatch.setenv('RACERBOT_CAR', request.param)


def car_config_args(path):
    """gap_follow's node-test helper (src/gap_follow/test/conftest.py), reached by name.

    test_traction_ceiling_parity imports gap_follow node-test modules, whose
    `from conftest import car_config_args` resolves to the conftest module already
    loaded -- this one, under this suite. Delegate to gap_follow's own, loaded from its
    file, so there is one definition."""
    import importlib.util
    path_to = Path(__file__).resolve().parents[3] / 'src/gap_follow/test/conftest.py'
    spec = importlib.util.spec_from_file_location('gap_follow_test_conftest', path_to)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.car_config_args(path)


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
        (profile / 'car.yaml').write_text(profile_text(car))
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
        # A tuning overlay needs a '# why:' line above each key (racerbot_car.guards,
        # Task 14), which yaml.safe_dump cannot write.
        lines = ['pure_pursuit_node:', '  ros__parameters:']
        for key, value in data['tuning'].items():
            lines += ['    # why: test fixture: tuning the two cars can be told apart by',
                      f'    {key}: {yaml.safe_dump(value).splitlines()[0]}']
        (profile / 'tuning/pure_pursuit.yaml').write_text('\n'.join(lines) + '\n')

    def set_calibration(car, **changes):
        values[car]['vesc'].update(changes)
        write(car)

    def clone(source, target):
        values[target] = deepcopy(values[source])
        (directory / target).mkdir(exist_ok=True)
        (directory / target / 'car.yaml').write_text(
            profile_text(source, directory))
        write(target)

    for car in values:
        write(car)
    monkeypatch.setenv('RACERBOT_CARS_DIR', str(directory))
    return SimpleNamespace(set_calibration=set_calibration, clone=clone)
