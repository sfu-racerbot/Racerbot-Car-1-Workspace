"""
Every top speed a car ships with must be one the VESC will actually command.

A speed command becomes an eRPM command in ackermann_to_vesc.cpp,
``erpm = speed_to_erpm_gain * speed + speed_to_erpm_offset``, and vesc_driver
then clips that to [speed_min, speed_max] (vesc_driver.cpp, ``speed_limit_.clip``)
without a warning. A top speed past the clip is a speed the node believes it
commands and the motor never gets.

The command gain is per-car calibration (src/racerbot_car/cars/<id>/calibration/
vesc.yaml), and it changes when a car is recalibrated: car 1's went from 3650 to
4600 on 2026-11-02 (docs/hardware-reference.md, car 1's VESC section). With the
5.0 m/s top speed, the most a car's gain may be is 23250 / 5.0 = 4650, the shared
speed_max; a recalibration past that clips the top speed, and this test says so.

Oracle: the closed form above, recomputed from each car's effective vesc.yaml,
applied to every shipped top speed. Boundary-inclusive, like vesc_driver's clip.

No ROS needed:

    python3 -m pytest src/racerbot_launch/test/test_speed_limits_fit_vesc.py -v
"""
import os
import sys

import pytest
import yaml

_SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_SRC, 'racerbot_car'))
sys.path.insert(0, os.path.join(_SRC, 'gap_follow'))
from gap_follow import launch_speed  # noqa: E402
from racerbot_car.identity import known_car_ids  # noqa: E402
from racerbot_car.layers import effective_config  # noqa: E402
from racerbot_car.paths import cars_dir  # noqa: E402

CARS = sorted(car for car in known_car_ids(cars_dir())
              if (cars_dir() / car / 'calibration' / 'vesc.yaml').is_file())

TOP_SPEEDS = ['gap_follow.yaml max_speed', 'gap_follow max_speed:= ceiling',
              'joy_teleop.yaml full stick', 'pure_pursuit.yaml max_speed',
              'auto_map_race.yaml profile_max_speed', 'neural_racer.yaml max_speed']


def _params(path, node):
    with open(path, encoding='utf-8') as handle:
        return yaml.safe_load(handle)[node]['ros__parameters']


def _vesc(car):
    vesc = effective_config('vesc.yaml', '/**', car_id=car)
    return (vesc['speed_to_erpm_gain'], vesc['speed_to_erpm_offset'],
            vesc['speed_min'], vesc['speed_max'])


def _top_speeds(car):
    joy = effective_config('joy_teleop.yaml', 'joy_teleop', car_id=car)
    assert joy['human_control.axis_mappings.drive-speed.offset'] == 0.0
    return {
        'gap_follow.yaml max_speed': effective_config(
            'gap_follow.yaml', 'gap_follow_node', car_id=car)['max_speed'],
        'gap_follow max_speed:= ceiling': launch_speed.MAX_SPEED_CEILING,
        # Full stick: the F710 axis reads +/-1.0, so both directions.
        'joy_teleop.yaml full stick': abs(joy['human_control.axis_mappings.drive-speed.scale']),
        'pure_pursuit.yaml max_speed': effective_config(
            'pure_pursuit.yaml', 'pure_pursuit_node', car_id=car)['max_speed'],
        # Shared by every car: auto_map_race.yaml has no per-car overlay.
        'auto_map_race.yaml profile_max_speed': _params(
            os.path.join(_SRC, 'pure_pursuit', 'config', 'auto_map_race.yaml'),
            'auto_map_race_node')['profile_max_speed'],
        'neural_racer.yaml max_speed': effective_config(
            'neural_racer.yaml', 'neural_racer_node', car_id=car)['max_speed'],
    }


def clipped_directions(speed, gain, offset, speed_min, speed_max):
    """The directions ('forward', 'reverse') in which +/-speed m/s is clipped."""
    clipped = []
    for direction, signed in (('forward', speed), ('reverse', -speed)):
        erpm = gain * signed + offset
        if not speed_min <= erpm <= speed_max:
            clipped.append(direction)
    return clipped


@pytest.mark.parametrize('car', CARS)
@pytest.mark.parametrize('name', TOP_SPEEDS)
def test_shipped_top_speed_is_not_clipped_by_the_vesc(name, car):
    speed = _top_speeds(car)[name]
    assert speed > 0.0
    gain, offset, speed_min, speed_max = _vesc(car)
    assert clipped_directions(speed, gain, offset, speed_min, speed_max) == [], (
        f'{car}: {name} = {speed} m/s is {gain * speed + offset:.0f} eRPM at '
        f'speed_to_erpm_gain {gain}, past speed_max {speed_max} in vesc.yaml')


# The check itself, on known cases, so it is not only ever seen passing.

def test_check_boundary_is_inclusive_and_one_step_past_fails():
    # 5.0 m/s at 4650 eRPM per m/s is 23250, exactly speed_max: not clipped.
    # 5.001 m/s is 23254.65: clipped both ways.
    assert clipped_directions(5.0, 4650.0, 0.0, -23250.0, 23250.0) == []
    assert clipped_directions(5.001, 4650.0, 0.0, -23250.0, 23250.0) == ['forward', 'reverse']


def test_check_catches_a_gain_10_percent_too_high():
    # 4650 * 1.1 = 5115 eRPM per m/s puts 5.0 m/s at 25575.
    assert clipped_directions(5.0, 5115.0, 0.0, -23250.0, 23250.0) == ['forward', 'reverse']


def test_check_applies_the_offset_one_direction_at_a_time():
    # An offset of +100 eRPM pushes forward past the clip and keeps reverse inside it.
    assert clipped_directions(5.0, 4650.0, 100.0, -23250.0, 23250.0) == ['forward']
