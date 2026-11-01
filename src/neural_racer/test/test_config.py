"""The shipped config (config/neural_racer.yaml). ROS-free.

Runner: python3 -m pytest src/neural_racer/test/test_config.py -v   (unsourced)
Oracles: the workspace safety policy (CLAUDE.md), and gap_follow.yaml -- the
wrapper's braking and footprint values are gap_follow's, so the two stay in step.
"""
import os
from pathlib import Path
import sys

import nr_testutil
import pytest
import yaml

from neural_racer.safety_wrapper import WrapperConfig

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'racerbot_car'))
from racerbot_car.identity import known_car_ids  # noqa: E402
from racerbot_car.layers import effective_config  # noqa: E402
from racerbot_car.paths import cars_dir  # noqa: E402

CARS = sorted(car for car in known_car_ids(cars_dir())
              if (cars_dir() / car / 'calibration/neural_racer.yaml').is_file())

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, '..', 'config', 'neural_racer.yaml')
GAP_FOLLOW = os.path.join(HERE, '..', '..', 'gap_follow', 'config', 'gap_follow.yaml')


def _params(path, node):
    with open(path) as f:
        return yaml.safe_load(f)[node]['ros__parameters']


def test_deadman_on_and_shadow_by_default():
    p = _params(CONFIG, 'neural_racer_node')
    assert p['enable_deadman'] is True and p['deadman_button'] == 4
    assert p['shadow'] is True
    assert p['max_speed'] == 1.0 and p['mode'] == 'reactive' and p['require_imu'] is True
    assert p['camera_enabled'] is False


@pytest.mark.parametrize('car', CARS)
def test_wrapper_values_match_gap_follow(car):
    p = effective_config('neural_racer.yaml', 'neural_racer_node', car_id=car)
    g = effective_config('gap_follow.yaml', 'gap_follow_node', car_id=car)
    # max_lateral_accel is deliberately NOT paired here, since 2026-10-27:
    # gap_follow raised its cornering limit to 2.5 m/s^2 from car 1's grip
    # data, while neural_racer keeps 2.0 in training and deployment. The cap
    # is never a training-time change (see the 2026-10-23 neural-racer plan,
    # docs/superpowers/specs/2026-10-23-neural-racer-plan.md). Every other
    # pair below is still checked exactly.
    pairs = {'max_steering_angle': 'max_steering_angle', 'car_width': 'car_width',
             'car_length': 'car_length', 'max_braking_decel': 'max_braking_decel',
             'braking_latency_sec': 'braking_latency_sec',
             'max_steering_rate': 'max_steering_rate', 'command_slew_max_dt': 'command_slew_max_dt',
             'reserve_distance': 'forward_stop_clearance', 'wheelbase': 'wheelbase',
             'laser_offset_x': 'laser_offset_x'}
    mismatched = {k: (p[k], g[v]) for k, v in pairs.items() if p[k] != g[v]}
    assert mismatched == {}


@pytest.mark.parametrize('car', CARS)
def test_yaml_matches_wrapper_defaults(car):
    p = effective_config('neural_racer.yaml', 'neural_racer_node', car_id=car)
    d = WrapperConfig()
    assert {k: p[k] for k in vars(d) if k != 'max_speed'} == \
        {k: v for k, v in vars(d).items() if k != 'max_speed'}
