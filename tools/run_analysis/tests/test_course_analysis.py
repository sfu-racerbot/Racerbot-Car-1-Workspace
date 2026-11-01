"""Tests for the course-analysis tools: loc, laps, sectors, gripcurve, stops, clearance,
lap_theory and oa_replay. Pure numpy and scipy, no bag and no ROS graph.

Each oracle is named: a closed form recomputed here, an invariant, or a synthetic scene
whose truth is known because the test built it.
Run by: python3 -m pytest tools/f1tenth_sim/tests/ tools/racerbot_sim/tests/ tools/run_analysis/tests/ -q
"""
import math
import os
import sys

import numpy as np
import pytest

ROOT = os.path.join(os.path.dirname(__file__), '..', '..', '..')
sys.path[:0] = [os.path.join(ROOT, 'tools'), os.path.join(ROOT, 'src', 'gap_follow')]
from run_analysis import clearance, gripcurve, lap_theory, laps, loc, oa_replay, sectors, stops  # noqa: E402

RES = 0.05


def room_grid():
    """A 10 x 8 m room with an asymmetric block, so a pose is unique."""
    occ = np.zeros((int(8 / RES), int(10 / RES)), bool)
    occ[:4, :] = occ[-4:, :] = True
    occ[:, :4] = occ[:, -4:] = True
    occ[60:90, 120:130] = True     # 3 m wide-ish block off centre
    occ[110:115, 40:100] = True    # a short wall, for asymmetry
    return loc.Grid(occ, ~occ, RES, -2.0, -1.0)


def cast(grid, pose, angles, max_range=12.0, step=0.01):
    """The scan a LiDAR 0.26 m ahead of base_link at ``pose`` would return: ray march."""
    x, y, th = pose
    lx, ly = x + loc.LASER_X * math.cos(th), y + loc.LASER_X * math.sin(th)
    out = np.full(len(angles), np.inf)
    for i, a in enumerate(angles):
        ux, uy = math.cos(th + a), math.sin(th + a)
        for r in np.arange(0.1, max_range, step):
            col = int((lx + r * ux - grid.ox) / RES)
            row = int((ly + r * uy - grid.oy) / RES)
            if not (0 <= row < grid.occ.shape[0] and 0 <= col < grid.occ.shape[1]) or grid.occ[row, col]:
                out[i] = r
                break
    return out


@pytest.fixture(scope='module')
def scene():
    grid = room_grid()
    angles = np.radians(np.linspace(-135.0, 135.0, 271))
    pose = (3.0, 1.5, 0.4)
    bx, by = loc.scan_points(cast(grid, pose, angles), angles)
    return grid, pose, bx, by


# --- loc ---------------------------------------------------------------------------------

def test_match_recovers_a_pose_it_was_started_near(scene):
    # oracle: synthetic, the scan was cast from the true pose.
    grid, pose, bx, by = scene
    found, h10, _h20 = loc.match(grid, bx, by, (pose[0] + 0.15, pose[1] - 0.1, pose[2] + 0.05))
    assert math.hypot(found[0] - pose[0], found[1] - pose[1]) < 0.03   # m
    assert abs(found[2] - pose[2]) < 0.02                              # rad
    assert h10 > 0.9


def test_match_scores_a_wrong_pose_badly(scene):
    grid, pose, bx, by = scene
    # Inside the room: beams landing off the map read as the wall at its edge.
    _found, h10, _h20 = loc.match(grid, bx, by, (pose[0] + 1.0, pose[1] + 0.8, pose[2] + 1.2), iters=0)
    assert h10 < 0.5
    assert loc.match(grid, bx, by, pose, iters=0)[1] > 0.9   # the same scan at the truth


def test_global_search_finds_the_pose_from_nothing(scene):
    # oracle: synthetic. Only the map and the scan are given.
    grid, pose, bx, by = scene
    line = np.array([[x, 1.5] for x in np.arange(1.0, 7.0, 0.25)])
    found, h10, _h20 = loc.global_search(grid, bx, by, loc.search_points(grid, line))
    assert h10 > 0.9
    assert math.hypot(found[0] - pose[0], found[1] - pose[1]) < 0.05   # m


def test_best_match_is_the_most_beams_not_the_first():
    a, b, c = ((0, 0, 0), 0.5, 0.9), ((1, 1, 1), 0.9, 0.2), ((2, 2, 2), 0.7, 0.9)
    assert loc.best_match([a, b, c]) is b


def test_search_points_are_free_and_near_the_line(scene):
    grid = scene[0]
    line = np.array([[x, 3.0] for x in np.arange(1.0, 7.0, 0.25)])
    points = loc.search_points(grid, line, reach=1.0)
    assert len(points) > 0
    assert np.all(np.abs(points[:, 1] - 3.0) < 1.0 + 1e-9)
    rows = ((points[:, 1] - grid.oy) / RES).astype(int)
    cols = ((points[:, 0] - grid.ox) / RES).astype(int)
    assert not grid.occ[rows, cols].any()


def test_scan_points_drop_invalid_and_far_beams():
    angles = np.zeros(6)
    bx, by = loc.scan_points([np.nan, np.inf, 0.05, 1.0, 50.0, 2.0], angles)
    # oracle: closed form, every third of the valid beams (1.0 m and 2.0 m -> the first).
    assert bx == pytest.approx([loc.LASER_X + 1.0], abs=1e-12)
    assert by == pytest.approx([0.0], abs=1e-12)
    # A beam past MAX_RANGE first in line must not be the one kept.
    bx, _by = loc.scan_points([50.0, 1.0], np.zeros(2))
    assert bx == pytest.approx([loc.LASER_X + 1.0], abs=1e-12)


def test_advance_on_a_straight_and_on_a_turn():
    assert loc.advance((1.0, 2.0, 0.0), 2.0, 0.0, 0.5) == pytest.approx((2.0, 2.0, 0.0), abs=1e-12)
    # oracle: closed form, the midpoint rule for a half-turn step.
    x, y, th = loc.advance((0.0, 0.0, 0.0), 1.0, 2.0, 0.5)
    assert th == pytest.approx(1.0, abs=1e-12)
    assert x == pytest.approx(0.5 * math.cos(0.5), abs=1e-12)
    assert y == pytest.approx(0.5 * math.sin(0.5), abs=1e-12)


def test_results_default_outside_data():
    # The request: no caches in data/ unless asked.
    assert 'data' not in loc.DEFAULT_OUT.parts
    assert loc.result_path('/a/data/racerbot_runs/20261030-185036').parent == loc.DEFAULT_OUT
    assert 'data' not in loc.result_path('/a/data/racerbot_runs/x').parts


def test_load_track_says_how_to_fix_a_missing_run(tmp_path):
    with pytest.raises(FileNotFoundError, match='loc.py'):
        loc.load_track('/no/such/run', tmp_path)


# --- laps --------------------------------------------------------------------------------

def circle(radius=5.0, n=400):
    a = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    return np.c_[radius * np.cos(a), radius * np.sin(a)]


def test_course_length_is_the_circumference():
    # oracle: closed form, within the polygon's chord shortfall.
    assert laps.Course(circle()).length == pytest.approx(2.0 * math.pi * 5.0, abs=0.01)


def test_project_gives_arc_length_and_signed_offset():
    course = laps.Course(circle())
    # Quarter way round, counter-clockwise: left of travel is towards the centre.
    s, lateral = course.project(0.0, 4.8)
    assert s[0] == pytest.approx(course.length / 4.0, abs=0.06)  # m
    assert lateral[0] == pytest.approx(0.2, abs=0.01)            # m
    # At the start the line runs along +y, so both axes of the offset get used.
    assert course.project(4.8, 0.0)[1][0] == pytest.approx(0.2, abs=0.01)   # m, inside
    assert course.project(5.2, 0.0)[1][0] == pytest.approx(-0.2, abs=0.01)  # m, outside


def test_unwrap_carries_over_the_start_line_and_ignores_jumps():
    course = laps.Course(circle())
    length = course.length
    s = np.array([length - 0.2, length - 0.1, 0.0, 0.1])
    assert course.unwrap(s) == pytest.approx([0.0, 0.1, 0.2, 0.3], abs=1e-9)
    # A re-localisation of 5 m counts as no progress at all.
    assert course.unwrap(np.array([1.0, 1.1, 6.1, 6.2])) == pytest.approx([0.0, 0.1, 0.1, 0.2], abs=1e-9)


def test_holds_finds_each_run_of_true():
    assert laps.holds([0, 1, 1, 0, 1]) == [(1, 3), (4, 5)]
    assert laps.holds([0, 0]) == []
    assert laps.holds([1, 1]) == [(0, 2)]


@pytest.mark.parametrize('direction', [1.0, -1.0])
def test_lap_times_at_constant_speed(direction):
    # oracle: closed form, 100 m at 4 m/s over a 25 m course is 6.25 s a lap, 4 laps.
    t = np.arange(0.0, 25.0 + 1e-9, 0.05)
    assert laps.lap_times(t, direction * 4.0 * t, 25.0) == pytest.approx([6.25] * 4, abs=1e-6)


def test_lap_times_with_too_little_progress_is_empty():
    assert laps.lap_times(np.array([0.0, 1.0]), np.array([0.0, 24.9]), 25.0) == []
    assert laps.lap_times(np.array([0.0]), np.array([0.0]), 25.0) == []


def test_best_pace_takes_the_fastest_stretch():
    # 2 m/s for 30 m, then 5 m/s for 30 m: fastest 25 m is at 5 m/s.
    t = np.r_[np.arange(0.0, 15.0, 0.01), 15.0 + np.arange(0.0, 6.0, 0.01)]
    p = np.r_[2.0 * np.arange(0.0, 15.0, 0.01), 30.0 + 5.0 * np.arange(0.0, 6.0, 0.01)]
    assert laps.best_pace(t, p, 50.0, window=25.0) == pytest.approx(10.0, abs=0.05)  # s/lap
    assert math.isnan(laps.best_pace(t, p, 50.0, window=500.0))


# --- sectors -----------------------------------------------------------------------------

def test_true_speed_of_a_straight_line():
    # oracle: closed form, away from the ends the smoothing window spoils.
    t = np.arange(0.0, 5.0, 0.05)
    v = sectors.true_speed(t, 3.0 * t, 4.0 * t)
    assert v[10:-10] == pytest.approx(5.0, abs=1e-6)  # m/s


def test_sector_medians_bin_and_leave_empty_sectors_nan():
    s = np.array([0.1, 0.2, 2.6, 2.7, 2.8])
    v = np.array([1.0, 3.0, 5.0, 6.0, 10.0])     # median 6, mean 7
    out = sectors.sector_medians(s, v, 7.5, 2.5)
    assert out[0] == pytest.approx(2.0, abs=1e-12)
    assert out[1] == pytest.approx(6.0, abs=1e-12)
    assert np.isnan(out[2])


def test_sector_medians_respect_the_mask_and_sample_floor():
    s = np.array([0.1, 0.2, 0.3])
    v = np.array([1.0, 9.0, 3.0])
    assert sectors.sector_medians(s, v, 2.5, 2.5, np.array([1, 0, 1]))[0] == pytest.approx(2.0, abs=1e-12)
    assert np.isnan(sectors.sector_medians(s, v, 2.5, 2.5, np.array([1, 0, 0]))[0])


def test_time_at_speeds_skips_empty_sectors():
    # oracle: closed form, 2.5/2.5 + 2.5/5.
    assert sectors.time_at_speeds([2.5, np.nan, 5.0], 2.5) == pytest.approx(1.5, abs=1e-12)


# --- gripcurve ---------------------------------------------------------------------------

def test_asked_is_the_bicycle_model_with_the_rack_lag():
    t = np.arange(0.0, 2.0, 0.05)
    steer = np.where(t < 1.0, 0.0, 0.2)
    asked, achieved = gripcurve.asked_achieved(t, np.full_like(t, 2.0), np.full_like(t, -0.5), steer,
                                               wheelbase=0.36, lag=0.15)
    # oracle: closed form. Before the lag runs out the step has not reached the rack.
    assert asked[np.searchsorted(t, 1.1)] == pytest.approx(0.0, abs=1e-9)
    assert asked[np.searchsorted(t, 1.5)] == pytest.approx(4.0 * math.tan(0.2) / 0.36, abs=1e-9)
    assert achieved[0] == pytest.approx(1.0, abs=1e-12)   # |v * w|, sign dropped


def test_asked_clips_a_command_past_full_lock():
    t = np.arange(0.0, 1.0, 0.05)
    asked, _ = gripcurve.asked_achieved(t, np.ones_like(t), np.zeros_like(t), np.full_like(t, 0.74),
                                        wheelbase=0.36, lag=0.0, full_lock=0.46)
    assert asked[0] == pytest.approx(math.tan(0.46) / 0.36, abs=1e-9)


def test_curve_bins_and_medians():
    rows = gripcurve.curve([2.1] * 10 + [2.2] * 10, [1.0] * 10 + [3.0] * 10, bins=((2, 2.5), (9, 10)))
    assert len(rows) == 1
    low, high, n, median, _p90 = rows[0]
    assert (low, high, n) == (2, 2.5, 20)
    assert median == pytest.approx(2.0, abs=1e-12)


def test_curve_skips_a_bin_with_too_few_samples():
    assert gripcurve.curve([2.1] * 9, [1.0] * 9, bins=((2, 2.5),)) == []


# --- stops -------------------------------------------------------------------------------

def stop_trace(v0=4.0, decel=4.0, wall=8.0):
    """A straight stop: command drops at 1 s, the car decelerates uniformly."""
    t = np.arange(0.0, 4.0, 0.025)
    v = np.where(t < 1.0, v0, np.maximum(v0 - decel * (t - 1.0), 0.0))
    x = np.r_[0.0, np.cumsum(v[:-1] * 0.025)]
    return t, v, np.where(t < 1.0, v0, 0.0), wall - x, np.zeros_like(t)


def test_a_uniform_stop_measures_the_closed_form():
    stop = stops.find_stops(*stop_trace())[0]
    # oracle: closed form, v^2/2a to the 0.3 m/s end of the stop.
    expected = (4.0 ** 2 - stops.STOPPED_SPEED ** 2) / (2.0 * 4.0)
    assert stop.wall_m == pytest.approx(expected, abs=0.06)   # m
    assert stop.wheel_m == pytest.approx(expected, abs=0.06)  # m
    assert stop.straight


def test_a_turning_stop_is_not_called_straight():
    t, v, c, r, _s = stop_trace()
    assert not stops.find_stops(t, v, c, r, np.full_like(t, 0.2))[0].straight


def test_a_slow_stop_and_a_command_blip_are_not_stops():
    t, v, c, r, s = stop_trace(v0=1.5, decel=1.0)       # long enough to be one, but slow
    assert stops.find_stops(t, v, c, r, s, min_speed=2.0) == []
    assert len(stops.find_stops(t, v, c, r, s, min_speed=1.0)) == 1
    t, v, c, r, s = stop_trace()
    blip = np.where((t > 1.0) & (t < 1.2), 0.0, 4.0)       # shorter than MIN_DURATION
    assert stops.find_stops(t, np.full_like(t, 4.0), blip, r, s) == []


def test_model_distance_is_the_braking_ceilings_formula():
    # oracle: recorded, run 20261030-175124 quoted against this: 5.41 m/s at 3.0 with 0.25 s.
    assert stops.model_distance(5.41, 3.0, 0.25) == pytest.approx(5.41 ** 2 / 6.0 + 1.3525, abs=1e-9)


def test_implied_decel_inverts_the_model_and_refuses_no_room():
    d = stops.model_distance(4.0, 5.0, 0.25)
    assert stops.implied_decel(4.0, d, 0.25) == pytest.approx(5.0, abs=1e-9)
    assert math.isnan(stops.implied_decel(4.0, 0.5, 0.25))


def test_forward_median_ignores_bad_beams_and_says_when_it_has_none():
    r = [9.0] * 10 + [2.0, 2.0, np.inf, np.nan, 2.0] + [9.0] * 10
    assert stops.forward_median(r, half_beams=2) == pytest.approx(2.0, abs=1e-12)
    assert math.isnan(stops.forward_median([np.nan] * 25))


# --- clearance ---------------------------------------------------------------------------

def test_body_clearance_ahead_and_beside():
    # oracle: closed form. Ahead: beam range + laser_x - front. Beside: y - half width.
    assert clearance.body_clearance([1.0], [0.0]) == pytest.approx(1.0 + 0.26 - 0.46, abs=1e-12)
    assert clearance.body_clearance([0.5], [math.pi / 2]) == pytest.approx(0.5 - 0.15, abs=1e-12)


def test_body_clearance_inside_the_box_is_zero():
    assert clearance.body_clearance([0.1], [0.0]) == 0.0


def test_body_clearance_ignores_the_rear_wedge_self_hits_and_bad_beams():
    # Self-hits: closer than MIN_RANGE, or in the rear wedge.
    assert clearance.body_clearance([0.05], [0.0]) == math.inf
    assert clearance.body_clearance([0.2], [math.radians(130.0)]) == math.inf
    assert clearance.body_clearance([np.nan, np.inf], [0.0, 0.1]) == math.inf


def test_launch_command_reads_the_control_layer(tmp_path):
    (tmp_path / 'metadata').mkdir()
    (tmp_path / 'metadata' / 'processes.txt').write_text(
        '1 1 x /usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch f1tenth_stack bringup_launch.py\n'
        '2 1 x /usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch gap_follow gap_follow_launch.py opponent_awareness:=false\n')
    assert clearance.launch_command(tmp_path) == 'gap_follow gap_follow_launch.py opponent_awareness:=false'
    assert clearance.launch_command(tmp_path / 'none') == ''


# --- lap_theory --------------------------------------------------------------------------

def test_lap_time_on_a_circle_at_the_lateral_limit():
    # oracle: closed form. Constant radius 5 at 2 m/s^2: v = sqrt(10); circumference / v.
    seconds, metres = lap_theory.lap_time(circle(), 2.0, 3.0, 3.0, 100.0)
    assert metres == pytest.approx(2.0 * math.pi * 5.0, abs=0.2)
    assert seconds == pytest.approx(2.0 * math.pi * 5.0 / math.sqrt(10.0), abs=0.2)


def test_lap_time_on_a_circle_at_the_top_speed():
    seconds, _m = lap_theory.lap_time(circle(), 100.0, 3.0, 3.0, 2.0)
    assert seconds == pytest.approx(2.0 * math.pi * 5.0 / 2.0, abs=0.2)


def test_speed_profile_respects_the_braking_and_acceleration_limits():
    kappa = np.zeros(200)
    kappa[100:] = 1.0        # a corner at 10 m along an open path
    v = lap_theory.speed_profile(kappa, 0.1, 1.0, 2.0, 2.0, 5.0, closed=False)
    assert v[150] == pytest.approx(1.0, abs=1e-9)
    dv2 = np.diff(v ** 2)
    assert np.all(np.abs(dv2) <= 2.0 * 2.0 * 0.1 + 1e-9)   # invariant: v^2 changes by 2 a ds at most
    assert v[0] > 4.9


def test_relax_line_shortens_but_keeps_the_margin():
    length = lambda p: np.hypot(*(np.roll(p, -1, 0) - p).T).sum()
    # A ring just outside a wall at radius 4.8: it wants to shrink, and may not
    # come nearer the wall than 0.1 m. Unconstrained it would reach 4.86 here.
    start = circle(radius=4.92)
    relaxed = lap_theory.relax_line(start, lambda x, y: np.abs(np.hypot(x, y) - 4.8), 0.1, iterations=200)
    assert length(relaxed) < length(start)
    assert np.hypot(*relaxed.T).min() >= 4.9 - 1e-9


# --- oa_replay ---------------------------------------------------------------------------

def car_frames(moving_box=True, ticks=80):
    """An empty corridor with a box 2 m ahead: it drives on at 2 m/s, or stays put while
    this car, at standstill, sees it as static."""
    angles = np.radians(np.linspace(-135.0, 135.0, 1081))
    for k in range(ticks):
        x = 2.0 + (2.0 * k * 0.025 if moving_box else 0.0)
        r = np.full(1081, 8.0)
        hit = np.abs(np.tan(angles) * x) < 0.15
        r[hit & (np.abs(angles) < 1.0)] = x
        yield r, np.ones(1081, bool), angles, 100.0 + k * 0.025, 0.0, 0.0


def run_replay(frames, **overrides):
    from gap_follow import opponent_tracking as ot
    return oa_replay.replay(frames, ot.OpponentParams(**overrides), 0.36, 0.26, 0.0, 0.58)


def test_a_moving_box_counts_as_a_mover_and_a_static_one_does_not():
    # oracle: synthetic, the test decided which moved.
    assert run_replay(car_frames(True)).movers >= 1
    assert run_replay(car_frames(False)).movers == 0


def test_replay_times_its_frames_at_the_scanner_rate():
    assert run_replay(car_frames(False, ticks=40)).seconds == pytest.approx(1.0, abs=1e-9)  # s


def test_a_stall_needs_the_cap_low_for_long_enough():
    r = oa_replay.Replay(episodes=[oa_replay.EPISODE, oa_replay.EPISODE - 0.01, 2.0])
    assert r.stalls == 2


def test_parse_overrides_refuses_an_unknown_or_malformed_field():
    assert oa_replay.parse_overrides(['follow_floor=0.25']) == {'follow_floor': 0.25}
    assert oa_replay.parse_overrides(None) == {}
    with pytest.raises(SystemExit, match='OpponentParams'):
        oa_replay.parse_overrides(['nonsense=1'])
    with pytest.raises(SystemExit, match='name=value'):
        oa_replay.parse_overrides(['follow_floor'])
