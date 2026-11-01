"""Tests for car_measure.py and scan_map.py: the numbers the simulator is calibrated from.

Pure numpy, no bag and no ROS graph. Each oracle is named: a closed form recomputed
here, an invariant, or a synthetic signal whose truth is known because the test built it.
Run by: python3 -m pytest tools/f1tenth_sim/tests/ tools/racerbot_sim/tests/ tools/run_analysis/tests/ -q
"""
import math
import os
import sys

import numpy as np
import pytest

ROOT = os.path.join(os.path.dirname(__file__), '..', '..', '..')
sys.path[:0] = [os.path.join(ROOT, 'tools'), os.path.join(ROOT, 'src', 'gap_follow')]
from run_analysis import car_measure as cm  # noqa: E402
from run_analysis import scan_map as sm  # noqa: E402

DT = 0.025


# --- lag and steering ------------------------------------------------------------------

def test_best_lag_recovers_a_lag_the_test_built():
    # Oracle: the signal is the command shifted by exactly 0.1375 s.
    t = np.arange(0.0, 20.0, DT)
    cmd = np.sin(1.3 * t) + 0.4 * np.sin(3.1 * t)
    measured = np.interp(t - 0.1375, t, cmd)
    lag, rms = cm.best_lag(t[40:], measured[40:], t, cmd, np.arange(0.0, 0.3, 0.0125))
    assert lag == pytest.approx(0.1375, abs=1e-9)   # s, on the grid
    assert rms < 1e-9


def test_best_lag_a_lead_comes_out_negative():
    t = np.arange(0.0, 20.0, DT)
    cmd = np.sin(1.3 * t)
    measured = np.interp(t + 0.05, t, cmd)
    lag, _ = cm.best_lag(t[40:-40], measured[40:-40], t, cmd, np.arange(-0.2, 0.2, 0.025))
    assert lag == pytest.approx(-0.05, abs=1e-9)    # s


def test_steering_from_motion_inverts_the_bicycle_model():
    # Oracle: yaw rate = v tan(delta) / L, solved back for delta.
    delta = np.array([-0.4, -0.1, 0.0, 0.05, 0.3])
    v = np.array([1.0, 2.0, 3.0, 2.5, 1.5])
    yaw = v * np.tan(delta) / cm.WHEELBASE
    assert cm.steering_from_motion(v, yaw) == pytest.approx(delta, abs=1e-12)   # rad


def test_steering_from_motion_survives_zero_speed():
    out = cm.steering_from_motion(np.array([0.0]), np.array([0.0]))
    assert np.isfinite(out).all() and out[0] == 0.0


def test_linkage_table_takes_medians_per_bin_and_drops_thin_bins():
    cmd = np.r_[np.full(20, 0.1), np.full(20, 0.2), np.full(3, 0.3)]
    actual = np.r_[np.full(19, 0.07), [5.0], np.full(20, 0.15), np.full(3, 0.29)]
    rows = cm.linkage_table(cmd, actual, np.array([0.05, 0.15, 0.25, 0.35]), min_samples=15)
    assert rows == [(pytest.approx(0.1), pytest.approx(0.07), 20), (pytest.approx(0.2), pytest.approx(0.15), 20)]


def test_replay_servo_delays_by_whole_ticks():
    cmd = np.r_[np.zeros(10), np.full(30, 0.2)]
    out = cm.replay_servo(cmd, DT, delay=3, gain=1.0 / DT, rate=1e6)
    # gain * dt = 1 and no slew limit: the rack follows the command exactly, three ticks late.
    assert out[12] == 0.0
    assert out[13] == pytest.approx(0.2, abs=1e-12)


def test_replay_servo_slews_no_faster_than_its_rate():
    cmd = np.r_[0.0, np.full(40, 0.4)]
    out = cm.replay_servo(cmd, DT, delay=0, gain=1.0 / DT, rate=2.0)
    assert np.max(np.diff(out)) == pytest.approx(2.0 * DT, abs=1e-12)   # rad per tick
    assert out[-1] == pytest.approx(0.4, abs=1e-12)


def test_replay_servo_proportional_approach_is_geometric():
    # Oracle: angle_k = target (1 - (1 - g dt)^k) for an unsaturated P loop.
    cmd = np.r_[0.0, np.full(20, 0.1)]
    out = cm.replay_servo(cmd, DT, delay=0, gain=10.0, rate=1e6)
    k = np.arange(1, 21)
    assert out[1:] == pytest.approx(0.1 * (1 - (1 - 10.0 * DT) ** k), abs=1e-12)


# --- speed loop and stops --------------------------------------------------------------

def test_simulate_speed_loop_caps_acceleration_and_braking_separately():
    up = cm.simulate_speed_loop(np.full(200, 5.0), 0.0, DT, 0, 100.0, 3.0, 100.0, 6.0)
    assert np.max(np.diff(np.r_[0.0, up])) == pytest.approx(3.0 * DT, abs=1e-12)   # m/s per tick
    down = cm.simulate_speed_loop(np.zeros(200), 5.0, DT, 0, 100.0, 3.0, 100.0, 6.0)
    assert np.min(np.diff(np.r_[5.0, down])) == pytest.approx(-6.0 * DT, abs=1e-12)
    assert down.min() == 0.0                       # never reverses


def test_simulate_speed_loop_waits_out_its_delay():
    cmd = np.r_[np.zeros(5), np.full(20, 2.0)]
    out = cm.simulate_speed_loop(cmd, 0.0, DT, 4, 5.0, 4.0, 5.0, 4.0)
    assert np.all(out[:9] == 0.0) and out[9] > 0.0


def test_fit_speed_loop_finds_the_loop_that_made_the_data():
    rng = np.random.default_rng(3)
    truth = dict(delay=3, accel_gain=7.0, accel_max=4.0, brake_gain=5.0, brake_max=3.0)
    segments = []
    for _ in range(4):
        # Held at the start speed first: the fitter starts from the first measured sample.
        cmd = np.r_[np.full(40, 1.0), np.repeat(rng.uniform(0.5, 4.5, 12), 40)]
        segments.append((cmd, cm.simulate_speed_loop(cmd, 1.0, DT, **truth)))
    grid = dict(delay=[1, 3, 5], accel_gain=[3.0, 7.0], accel_max=[3.0, 4.0], brake_gain=[5.0, 9.0],
                brake_max=[3.0, 5.0])
    best, rms = cm.fit_speed_loop(segments, DT, grid)
    assert best == truth and rms < 1e-9


def test_hard_stops_finds_a_stop_and_ignores_a_dip():
    t = np.arange(0.0, 10.0, DT)
    cmd = np.full(t.size, 3.0)
    cmd[100:160] = 0.0        # a real stop: the command stays down
    cmd[300:305] = 0.0        # a dip: the command comes back before the car stops
    wheel = cm.simulate_speed_loop(cmd, 3.0, DT, 2, 5.0, 4.0, 5.0, 4.0)
    stops = cm.hard_stops(t, wheel, cmd)
    assert len(stops) == 1
    i, j = stops[0]
    assert i == 100 and wheel[j] <= 0.3 and wheel[j - 1] > 0.3


def test_hard_stops_leaves_out_a_stop_whose_command_came_back_up():
    # The command drops, briefly asks for 0.5 m/s, then drops again: the wheels still reach
    # 0.3 m/s, but the stop was not one clean command, so its distance means nothing.
    t = np.arange(0.0, 6.0, DT)
    cmd = np.full(t.size, 3.0)
    cmd[40:] = 0.0
    cmd[48:52] = 0.5
    wheel = cm.simulate_speed_loop(cmd, 3.0, DT, 0, 5.0, 4.0, 5.0, 4.0)
    assert wheel.min() <= 0.3
    assert cm.hard_stops(t, wheel, cmd) == []


def test_hard_stops_needs_the_minimum_speed():
    t = np.arange(0.0, 5.0, DT)
    cmd = np.r_[np.full(50, 1.5), np.zeros(150)]
    wheel = cm.simulate_speed_loop(cmd, 1.5, DT, 0, 5.0, 4.0, 5.0, 4.0)
    assert cm.hard_stops(t, wheel, cmd, min_speed=2.0) == []


def test_stop_distance_at_constant_deceleration_is_the_closed_form():
    # Gain so high the ceiling always binds: d = v0 * delay + (v0^2 - end^2) / (2 a).
    d = cm.stop_distance(5.0, 0.1, 1e6, 4.0, end_speed=0.3, dt=1e-5)
    assert d == pytest.approx(5.0 * 0.1 + (25.0 - 0.09) / 8.0, abs=2e-3)   # m


def test_stop_distance_longer_with_a_lower_ceiling():
    assert cm.stop_distance(4.0, 0.05, 7.0, 3.0) > cm.stop_distance(4.0, 0.05, 7.0, 5.0)


# --- odometry ---------------------------------------------------------------------------

def test_speed_ratio_recovers_an_over_read():
    true_v = np.linspace(0.0, 5.0, 400)
    assert cm.speed_ratio(1.033 * true_v, true_v) == pytest.approx(1.033, abs=1e-12)


def test_speed_ratio_ignores_slow_samples():
    true_v = np.r_[np.full(50, 0.5), np.full(50, 3.0)]
    wheel = np.r_[np.full(50, 5.0), np.full(50, 3.3)]       # wild while slow
    assert cm.speed_ratio(wheel, true_v, min_speed=1.0) == pytest.approx(1.1, abs=1e-12)


# --- LiDAR ------------------------------------------------------------------------------

def test_scan_statistics_noise_and_dropouts():
    rng = np.random.default_rng(1)
    scans = 2.0 + rng.normal(0.0, 0.01, (2000, 50))
    scans[:, 7] = np.inf                       # one beam never returns
    s = cm.scan_statistics(scans)
    assert s["noise"] == pytest.approx(0.01, abs=5e-4)     # m
    assert s["dropout"] == pytest.approx(1 / 50, abs=1e-12)
    assert s["dropout_by_beam"][7] == 1.0 and s["dropout_by_beam"][6] == 0.0


def test_self_hits_lists_only_near_beams():
    angles = np.array([-2.3, -2.2, 0.0, 1.0])
    hits = cm.self_hits(np.array([0.10, 0.36, 0.2, np.nan]), angles, max_range=0.35)
    assert hits == [(-2.3, 0.10), (0.0, 0.2)]


def test_edge_statistics_counts_jumps_and_lone_returns():
    scan = np.full(20, 2.0)
    scan[5] = 1.0             # a lone return: two jumps around it
    scan[12:] = 3.0           # a step
    s = cm.edge_statistics(scan[None, :])
    assert s["jumps"] == 3.0
    assert s["isolated"] == 1.0


def test_edge_statistics_ignores_no_echo():
    scan = np.array([[2.0, np.inf, 2.0, 0.0, 2.0]])
    assert cm.edge_statistics(scan) == {"jumps": 0.0, "isolated": 0.0}


def test_cast_scan_reads_the_distance_to_a_wall():
    occ = np.zeros((200, 100), bool)           # 10 m tall, so the 45 degree beam stays inside
    occ[:, 80] = True                          # a wall at x = 4.0 m
    r = cm.cast_scan(occ, 0.05, 0.0, 0.0, 1.0, 2.5, 0.0, np.array([0.0, math.pi / 4, math.pi]))
    assert r[0] == pytest.approx(3.0, abs=0.011)                  # m, straight ahead
    assert r[1] == pytest.approx(3.0 * math.sqrt(2.0), abs=0.08)  # m, at 45 degrees (cell size)
    assert r[2] == np.inf                       # nothing behind, inside the range


# --- scan_map ---------------------------------------------------------------------------

def _grid(h=60, w=100):
    return np.zeros((h, w), int), np.zeros((h, w), int)


def test_accumulate_counts_the_end_cell_as_hit_and_the_way_there_as_passed():
    hits, passes = _grid()
    # From (0.025, 1.525) straight along +x, a 2.0 m return: ends in column 40.
    sm.accumulate(hits, passes, 0.05, 0.0, 0.0, 0.025, 1.525, 0.0, np.array([0.0]), np.array([2.0]))
    assert hits[30, 40] == 1 and hits.sum() == 1
    assert passes[30, 10:40].min() == 1 and passes[30, 41:].sum() == 0
    assert passes.max() == 1                   # one beam counts a cell once


def test_accumulate_a_short_beam_stops_where_it_returned():
    # Two beams in one scan, 1.0 m along +y and 2.5 m along +x: the short one must not be
    # carried on to the long one's length.
    hits, passes = _grid()
    sm.accumulate(hits, passes, 0.05, 0.0, 0.0, 0.025, 0.525, 0.0, np.array([math.pi / 2, 0.0]),
                  np.array([1.0, 2.5]))
    assert hits[30, 0] == 1 and hits[10, 50] == 1
    assert passes[31:, 0].sum() == 0


def test_accumulate_drops_near_and_far_returns():
    hits, passes = _grid()
    sm.accumulate(hits, passes, 0.05, 0.0, 0.0, 0.0, 1.5, 0.0, np.array([0.0, 0.1]),
                  np.array([sm.MIN_RANGE - 0.01, sm.MAX_RANGE + 1.0]))
    assert hits.sum() == 0 and passes.sum() == 0


def test_open_walls_keeps_only_cells_hit_often_enough():
    hits = np.array([[5, 5, 1, 0]])
    passes = np.array([[5, 20, 0, 9]])
    assert sm.open_walls(hits, passes, min_hits=3, min_share=0.25).tolist() == [[True, False, False, False]]


def test_rebuild_peels_a_thick_wall_back_to_where_beams_end():
    occ = np.zeros((20, 20), bool)
    occ[:, 8:12] = True                       # a wall four cells thick
    hits, passes = np.zeros((20, 20), int), np.zeros((20, 20), int)
    passes[:, 8:10] = 10                      # beams from the left pass the first two columns
    hits[:, 10] = 10                          # and end in the third
    out = sm.rebuild(occ, hits, passes)
    assert not out[2:18, 8:10].any()          # peeled
    assert out[:, 10].all() and out[:, 11].all()   # the face and the unseen back stay


def test_rebuild_never_opens_a_wall():
    occ = np.zeros((20, 20), bool)
    occ[:, 10] = True                         # one cell thin
    hits, passes = np.zeros((20, 20), int), np.zeros((20, 20), int)
    passes[:, 10] = 10                        # beams claim to pass straight through it
    out = sm.rebuild(occ, hits, passes)
    assert out[1:19, 10].all()                # still closed


def test_rebuild_keeps_what_no_beam_reached():
    occ = np.zeros((10, 10), bool)
    occ[3:6, 3:6] = True
    out = sm.rebuild(occ, np.zeros((10, 10), int), np.zeros((10, 10), int))
    assert np.array_equal(out, occ)


def test_keeps_walls_closed_refuses_the_ends_and_the_middle_of_a_line():
    grid = np.zeros((5, 5), bool)
    grid[2, 1:4] = True
    assert not sm._keeps_walls_closed(grid, 2, 2)    # removing it splits the line
    assert not sm._keeps_walls_closed(grid, 2, 3)    # an end: two or fewer neighbours
    grid[1:4, 1:4] = True
    assert sm._keeps_walls_closed(grid, 1, 2)        # an edge cell of a block
    assert not sm._keeps_walls_closed(grid, 0, 2)    # the border


def test_wall_bias_reads_a_map_whose_wall_stands_proud():
    occ = np.zeros((100, 100), bool)
    occ[:, 70:] = True                        # the map's wall face at x = 3.5 m
    angles = np.array([0.0])
    # A real wall 0.1 m further on: from a laser at x = 1.0 + LASER_X the real range is 2.6 - LASER_X.
    poses = [(1.0, 2.5, 0.0)] * 5
    scans = [np.array([3.6 - 1.0 - sm.LASER_X])] * 5
    b = sm.wall_bias(occ, 0.05, 0.0, 0.0, poses, angles, scans)
    assert b["median"] == pytest.approx(0.1, abs=0.011)    # m, one cast step
    assert b["close"] == 1.0
