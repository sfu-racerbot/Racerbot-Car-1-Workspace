"""slam_gyro.yaml has to make loop closure possible and give the gyro a say.

Runner: python3 -m pytest src/racerbot_launch/test/test_slam_gyro_profile.py (no sourcing needed), and colcon test.

These read the shipped YAML files and check invariants, not pinned values, so a later edit that makes closure
impossible again fails here however it does it. Reasons and measurements:
docs/superpowers/specs/2026-10-30-slam-reliability.md.
"""
from pathlib import Path

import pytest
import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config'
#: Median spacing of slam_toolbox's nodes while moving, runs 20261030-184556 and -185036 (0.58, 0.57 m). The
#: 0.2 s time gate sets it above 1.25 m/s, not minimum_travel_distance (0.25 m).
NODE_SPACING_M = 0.57
#: slam_toolbox's angle penalty is 1 - gain * dtheta^2 / angle_variance_penalty, floored at minimum_angle_penalty.
ANGLE_PENALTY_GAIN = 0.2
GYRO_DISAGREEMENT_RAD = 0.0873  # 5 deg, which the live pose reached within one lap on shipped settings
PRIOR_KEYS = ('angle_variance_penalty', 'minimum_angle_penalty', 'coarse_search_angle_offset',
              'coarse_angle_resolution')


def _params(name):
    return yaml.safe_load((CONFIG / name).read_text())['slam_toolbox']['ros__parameters']


def _gyro():
    """slam_gyro.yaml laid over slam_tracking.yaml, as slam_launch.py layers them."""
    return {**_params('slam_tracking.yaml'), **_params('slam_gyro.yaml')}


def test_a_closure_chain_fits_inside_the_search_radius():
    # oracle: invariant. A chain is consecutive older nodes inside the radius: a straight through a circle of
    # radius r holds 2r of path, so at most 2r/spacing nodes. Chain 20 at 3 m could never be met when the pose
    # was right (0.0% of nodes, runs 20261030-184556 and -185036).
    params = _gyro()
    reachable_nodes = 2.0 * params['loop_search_maximum_distance'] / NODE_SPACING_M
    assert params['loop_match_minimum_chain_size'] <= reachable_nodes


def test_a_closure_chain_is_still_evidence():
    # oracle: recorded measurement. Chain 7 was worse than chain 10 on run 20261030-185036 (0.67-0.74 against
    # 0.92 of walls on the reference map), so the chain must not be cut to the bare minimum that fits.
    assert _gyro()['loop_match_minimum_chain_size'] >= 10


def test_the_closure_window_cannot_reach_the_next_stretch_of_track():
    # oracle: recorded measurement, docs/localization.md. Neighbouring stretches are 2.2-2.6 m apart on the
    # courses measured, and a window of +/- half its dimension must stay short of that.
    assert _gyro()['loop_search_space_dimension'] / 2.0 < 2.2


def test_disagreeing_with_the_gyro_costs_a_match_something():
    # oracle: closed form of the penalty above. Shipped settings charged 0.15% for 5 deg, which the gyro's
    # 1 deg over 25 m (against SLAM's 5.3 deg) shows was far too little.
    params = _gyro()
    penalty = 1.0 - ANGLE_PENALTY_GAIN * GYRO_DISAGREEMENT_RAD ** 2 / params['angle_variance_penalty']
    assert penalty <= 0.9


def test_the_penalty_floor_does_not_cancel_the_penalty():
    # oracle: closed form. A floor above the penalty's value would simply replace it, which is how 0.9 hid it.
    params = _gyro()
    penalty = 1.0 - ANGLE_PENALTY_GAIN * GYRO_DISAGREEMENT_RAD ** 2 / params['angle_variance_penalty']
    assert params['minimum_angle_penalty'] < penalty


def test_the_coarse_angle_search_still_has_steps_in_it():
    # oracle: invariant. Narrowing the search without refining it would leave only a few angles to try.
    params = _gyro()
    steps = params['coarse_search_angle_offset'] / params['coarse_angle_resolution']
    assert round(steps, 6) >= 10.0  # rounded: 0.175 / 0.0175 is 9.999... in binary


@pytest.mark.parametrize('key', PRIOR_KEYS)
def test_steering_odometry_never_gets_the_tight_prior(key):
    # oracle: recorded measurement. Steering odometry is 15 deg out over 5 m (runs 20261030-*): holding SLAM to
    # that would hold it to a wrong answer. Only the profile that requires the gyro may set these.
    assert key not in _params('slam_tracking.yaml')


@pytest.mark.parametrize('key', PRIOR_KEYS + ('loop_match_minimum_chain_size',))
def test_the_gyro_profile_sets_each_of_these_itself(key):
    # oracle: invariant. Each one layered in slam_gyro.yaml, so it reaches only runs with gyro_odom:=true.
    assert key in _params('slam_gyro.yaml')
