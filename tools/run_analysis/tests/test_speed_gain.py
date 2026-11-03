"""Tests for speed_gain.py: the measurement behind car 1's 2026-11-02 command-gain change.

Pure numpy, no bag and no ROS graph. Each oracle is a closed form recomputed here or a
synthetic hold whose truth the test built; the one file read is a recorded run's metadata.
Run by: python3 -m pytest tools/f1tenth_sim/tests/ tools/racerbot_sim/tests/ tools/run_analysis/tests/ -q
"""
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, os.path.join(ROOT, 'tools'))
from run_analysis import speed_gain  # noqa: E402


# --- merge_short_releases ---------------------------------------------------

def test_short_release_is_filled_and_long_one_kept():
    t = np.arange(0.0, 3.0, 0.1)             # 30 samples, 0.1 s apart
    held = np.ones(30, bool)
    held[10:12] = False                       # released 0.9 -> 1.2 s: pressed again 0.3 s later
    held[20:27] = False                       # released 1.9 -> 2.7 s: 0.8 s
    out = speed_gain.merge_short_releases(t, held)
    assert out[10:12].all()
    assert not out[20:27].any()


def test_release_exactly_at_the_limit_is_kept():
    # Held at 0.0 and again at 0.5 s: a gap of exactly MERGE_GAP is not "shorter than".
    t = np.array([0.0, 0.25, 0.5])
    out = speed_gain.merge_short_releases(t, np.array([True, False, True]), max_gap=0.5)
    assert out.tolist() == [True, False, True]
    out = speed_gain.merge_short_releases(t, np.array([True, False, True]), max_gap=0.5001)
    assert out.tolist() == [True, True, True]


def test_leading_and_trailing_releases_are_not_filled():
    t = np.arange(6) * 0.1
    held = np.array([False, True, True, False, True, False])
    out = speed_gain.merge_short_releases(t, held)
    assert out.tolist() == [False, True, True, True, True, False]


@pytest.mark.parametrize('held', [[], [False, False, False], [True]])
def test_nothing_to_merge(held):
    t = np.arange(len(held)) * 0.1
    assert speed_gain.merge_short_releases(t, np.array(held, bool)).tolist() == held


# --- forward_integral -------------------------------------------------------

def test_integral_of_a_constant_is_value_times_time():
    t = np.linspace(0.0, 10.0, 401)
    assert speed_gain.forward_integral(t, np.full(401, 2.0), 0.0, 10.0) == pytest.approx(20.0, abs=1e-9)  # m


def test_integral_of_a_ramp_is_its_triangle():
    # v = 0.5 t from 2 s to 6 s: (0.5 * 36 - 0.5 * 4) / 2 = 8 m, exact for the trapezoid rule.
    t = np.linspace(0.0, 10.0, 1001)
    assert speed_gain.forward_integral(t, 0.5 * t, 2.0, 6.0) == pytest.approx(8.0, abs=1e-9)   # m


def test_reverse_speed_does_not_count_as_distance():
    # 5 s at +1 m/s then 5 s at -1 m/s: forward distance is 5 m, not 0 m.
    t = np.linspace(0.0, 10.0, 1001)
    v = np.where(t < 5.0, 1.0, -1.0)
    assert speed_gain.forward_integral(t, v, 0.0, 10.0) == pytest.approx(5.0, abs=0.01)   # m, one 10 ms edge


@pytest.mark.parametrize('t0, t1', [(3.0, 3.0), (20.0, 30.0)])
def test_integral_with_under_two_samples_is_nan(t0, t1):
    t = np.arange(11.0)
    assert math.isnan(speed_gain.forward_integral(t, np.ones(11), t0, t1))


# --- map_distance -----------------------------------------------------------

def test_straight_path_length():
    x = np.linspace(0.0, 10.0, 101)
    dist, used = speed_gain.map_distance(x, np.zeros(101), np.ones(101))
    assert dist == pytest.approx(10.0, abs=1e-9)   # m
    assert used == 1.0


def test_a_poor_match_is_left_out_and_scaled_back_up():
    # 101 poses 0.1 m apart; pose 50 fits badly and has jumped 0.4 m sideways. Its two steps
    # are dropped (2 of 100), and the other 98 x 0.1 m are scaled by 100/98 back to 10 m.
    x = np.linspace(0.0, 10.0, 101)
    y = np.zeros(101)
    y[50] = 0.4
    fit = np.ones(101)
    fit[50] = 0.5
    dist, used = speed_gain.map_distance(x, y, fit)
    assert used == pytest.approx(0.98, abs=1e-12)
    assert dist == pytest.approx(10.0, abs=1e-9)   # m
    # Kept, the jump would have added 2 * (hypot(0.1, 0.4) - 0.1) = 0.62 m.
    raw, _ = speed_gain.map_distance(x, y, np.ones(101))
    assert raw == pytest.approx(10.0 + 2 * (math.hypot(0.1, 0.4) - 0.1), abs=1e-9)


def test_fit_exactly_at_the_threshold_counts():
    x = np.linspace(0.0, 1.0, 11)
    dist, used = speed_gain.map_distance(x, np.zeros(11), np.full(11, 0.85), min_fit=0.85)
    assert (dist, used) == (pytest.approx(1.0, abs=1e-9), 1.0)
    dist, used = speed_gain.map_distance(x, np.zeros(11), np.full(11, 0.8499), min_fit=0.85)
    assert math.isnan(dist) and used == 0.0


@pytest.mark.parametrize('n', [0, 1])
def test_too_few_poses_is_nan(n):
    dist, used = speed_gain.map_distance(np.zeros(n), np.zeros(n), np.ones(n))
    assert math.isnan(dist) and used == 0.0


# --- hold_summary and pooled ------------------------------------------------

def test_a_car_needing_4600_driven_at_3650_suggests_4600():
    # Synthetic hold: commanded 100 m; at 3650 eRPM per commanded m/s a car that needs
    # 4600 eRPM per true m/s covers 100 * 3650 / 4600 m, and its wheels turn 3650 * 100 eRPM s.
    true_m = 100.0 * 3650.0 / 4600.0
    row = speed_gain.hold_summary(true_m, 100.0, 1.15 * true_m, 3650.0 * 100.0, 3650.0)
    assert row['realisation'] == pytest.approx(3650.0 / 4600.0, abs=1e-12)
    assert row['suggested_gain'] == pytest.approx(4600.0, abs=1e-9)   # eRPM per m/s
    assert row['effective_gain'] == pytest.approx(4600.0, abs=1e-9)   # eRPM per m/s
    assert row['odom_over_map'] == pytest.approx(1.15, abs=1e-12)


def test_a_car_delivering_its_command_suggests_its_own_gain():
    row = speed_gain.hold_summary(80.0, 80.0, 80.0, 4614.0 * 80.0, 4614.0)
    assert row['suggested_gain'] == pytest.approx(4614.0, abs=1e-9)
    assert row['realisation'] == 1.0


def test_pooled_weights_by_distance_not_by_hold():
    # Holds of (map 10, commanded 20) and (map 90, commanded 100): 100 / 120, where the
    # mean of the two ratios would be (0.5 + 0.9) / 2 = 0.7.
    rows = [speed_gain.hold_summary(10.0, 20.0, 10.0, 1.0, 3650.0),
            speed_gain.hold_summary(90.0, 100.0, 90.0, 1.0, 3650.0)]
    p = speed_gain.pooled(rows, 3650.0)
    assert p['realisation'] == pytest.approx(100.0 / 120.0, abs=1e-12)
    assert p['suggested_gain'] == pytest.approx(3650.0 * 120.0 / 100.0, abs=1e-9)
    assert (p['map_m'], p['commanded_m']) == (100.0, 120.0)


# --- command_gain_of --------------------------------------------------------

def test_command_gain_is_read_from_the_runs_own_parameters():
    # Recorded: car 1's run 20261030-180531 was driven at 3650 (its metadata/params).
    run = ROOT / 'data/rb1/racerbot_runs/20261030-180531'
    assert speed_gain.command_gain_of(run) == 3650.0
    run = ROOT / 'data/rb2/racerbot_runs/20261030-185036'
    assert speed_gain.command_gain_of(run) == 4614.0
