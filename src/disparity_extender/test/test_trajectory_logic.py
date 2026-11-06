"""Tests for disparity_trajectory.trajectory_logic (pure; no rclpy, runs unsourced).

Runner: python3 -m pytest src/disparity_extender/test/ -q (and colcon test).
Oracles: closed-form geometry of synthetic scenes ray-cast here (corridors,
boxes, a dead end), invariants (NaN beams change nothing; mirror symmetry),
and boundaries (thresholds exactly at and one step past).
"""
import math

import numpy as np
import pytest

from disparity_trajectory import trajectory_logic as tl

N, AMIN = 1081, -3 * math.pi / 4             # the cars' Hokuyos: 1081 beams over 270 deg
INC = 1.5 * math.pi / (N - 1)
P = tl.TrajectoryParams()                    # the shipped defaults, car 1's 0.33 rad


def cast(walls_y=(), boxes=(), max_r=30.0):
    """Ranges from a LiDAR at the origin: horizontal walls y = +/-w and axis-aligned boxes (x0, x1, y0, y1)."""
    out = np.full(N, np.inf)
    for i in range(N):
        a = AMIN + INC * i
        dx, dy = math.cos(a), math.sin(a)
        best = max_r
        for w in walls_y:
            if (w > 0 and dy > 1e-12) or (w < 0 and dy < -1e-12):
                best = min(best, w / dy)
        for x0, x1, y0, y1 in boxes:
            t0, t1 = 0.0, 1e9
            for d, lo, hi in ((dx, x0, x1), (dy, y0, y1)):
                if abs(d) < 1e-12:
                    if not (lo <= 0.0 <= hi):
                        t0 = 1e9
                    continue
                ta, tb = sorted((lo / d, hi / d))
                t0, t1 = max(t0, ta), min(t1, tb)
            if t0 <= t1 and t0 > 0:
                best = min(best, t0)
        out[i] = best if best < max_r else np.inf
    return out


def target(ranges, prev=0.0, p=P):
    return tl.trajectory_target(tl.scan_points(ranges, AMIN, INC, 0.05, p), prev, p)


# --- choosing an arc ---------------------------------------------------------
def test_straight_corridor_steers_straight_with_full_score():
    t = target(cast(walls_y=(0.8, -0.8)))
    assert abs(t.steering) < 1e-9                                         # rad
    # closed form: whole 2 m horizon clear + continuation capped at 5 m (walls stay 0.8 m off)
    assert t.score == pytest.approx(P.horizon_m + P.continuation_cap_m, abs=1e-9)  # m


def test_a_slot_too_narrow_for_the_car_is_not_chosen_and_mirror_holds():
    # Box at 3-3.5 m ahead leaves 0.25 m by the left wall (the car is 0.30 wide) and 1 m on the right.
    right = target(cast(walls_y=(1.2, -1.2), boxes=[(2.74, 3.24, -0.2, 0.95)]))
    left = target(cast(walls_y=(1.2, -1.2), boxes=[(2.74, 3.24, -0.95, 0.2)]))
    assert right.steering < -0.05 and left.steering > 0.05                # rad: away from the slot
    assert right.steering == pytest.approx(-left.steering, abs=1e-9)      # mirror symmetry (invariant)


def test_narrow_dead_end_scores_the_closed_form_clear_distance():
    # Walls +/-0.4 (in the LiDAR frame), end wall 2.0 m ahead of the LiDAR = 2.26 m ahead of the
    # rear axle. Straight on, the front circle (0.36 m ahead, radius 0.225) touches it once
    # s + 0.36 + 0.225 > 2.26: the last clear 0.1 m step is s = 1.6 m. Side walls are 0.175 m
    # beyond the circles, and any curving arc reaches them first.
    t = target(cast(walls_y=(0.4, -0.4), boxes=[(2.0, 2.5, -0.4, 0.4)]))
    assert t.steering == pytest.approx(0.0, abs=1e-9)                     # rad
    assert t.score == pytest.approx(1.6, abs=1e-9)                        # m


def test_boxed_in_returns_no_target():
    t = target(cast(boxes=[(0.15, 0.3, -3, 3), (-0.5, 0.3, 0.2, 0.3), (-0.5, 0.3, -0.3, -0.2)]))
    assert t is None


def test_min_score_boundary():
    walls = cast(walls_y=(0.4, -0.4), boxes=[(2.0, 2.5, -0.4, 0.4)])     # best score 1.6 m (above)
    assert target(walls, p=tl.TrajectoryParams(min_score_m=1.6)) is not None   # exactly at: kept
    assert target(walls, p=tl.TrajectoryParams(min_score_m=1.6 + 1e-9)) is None


def test_steering_stays_within_the_car_limit():
    t = target(cast(boxes=[(2.0, 2.2, -3, 0.5)]))                         # wall 2 m ahead, open on the left only
    assert 0.0 < t.steering <= P.max_steering + 1e-12


def test_no_points_in_range_is_flagged_straight_on():
    t = target(np.full(N, np.inf))
    assert t.no_points and t.steering == 0.0


def test_nan_beams_are_skipped_not_obstacles():
    clean = cast(walls_y=(0.8, -0.8), boxes=[(2.74, 3.24, -0.2, 0.95)])
    holey = clean.copy()
    holey[::7] = np.nan
    a, b = target(clean), target(holey)
    assert np.sign(a.steering) == np.sign(b.steering) == -1.0             # same decision (invariant)


def test_scan_points_drop_nan_inf_and_below_range_min():
    pts = tl.scan_points([np.nan, np.inf, 0.01, 1.0, 1.0], 0.0, 0.0, 0.05, P)
    assert pts.shape == (1, 2)                                           # every second of the two valid 1.0s
    assert pts[0, 0] == pytest.approx(P.lidar_x_m + 1.0, abs=1e-12)      # m, rear-axle frame


def test_valid_fraction():
    assert tl.valid_fraction([], 0.05, 30.0) == 0.0
    assert tl.valid_fraction([np.inf] * 10, 0.05, 30.0) == 0.0
    assert tl.valid_fraction([0.0] * 10, 0.05, 30.0) == 0.0
    assert tl.valid_fraction([1.0, np.nan, 0.05, 30.0], 0.05, 30.0) == pytest.approx(0.75, abs=1e-12)


# --- speed and smoothing (closed forms of the speed law) ----------------------
S = tl.SpeedParams()


def test_speed_open_and_straight_is_max_speed():
    assert tl.speed_for(0.0, math.inf, 0, S) == S.max_speed


def test_speed_limited_by_the_turn():
    expected = math.sqrt(S.max_lateral_acc * S.wheelbase / math.tan(0.33))     # 1.621 m/s
    assert tl.speed_for(0.33, 100.0, 0, S) == pytest.approx(expected, abs=1e-12)


def test_speed_limited_by_room_then_clamped_to_min():
    assert tl.speed_for(0.0, 0.5, 0, S) == pytest.approx(math.sqrt(2.5 * 0.5), abs=1e-12)  # 1.118 m/s
    assert tl.speed_for(0.0, 0.1, 0, S) == S.min_speed                 # sqrt(0.25) = 0.5 -> min 1.0


def test_obstacle_slowdown_threshold_and_floor():
    assert tl.speed_for(0.0, math.inf, 2, S) == 2.5                    # one below the threshold
    assert tl.speed_for(0.0, math.inf, 3, S) == pytest.approx(1.5, abs=1e-12)   # at it: -1.0 m/s
    assert tl.speed_for(0.0, 0.1, 3, S) == 1.0                         # never below obstacle_min_speed
    low = tl.SpeedParams(min_speed=0.5)
    assert tl.speed_for(0.0, 0.256, 3, low) == pytest.approx(math.sqrt(2.5 * 0.256), abs=1e-12)  # 0.8: never raised
    off = tl.SpeedParams(enable_obstacle_slowdown=False)
    assert tl.speed_for(0.0, math.inf, 9, off) == 2.5


def test_smooth_blends_then_clamps_keeping_the_unclamped_previous():
    cmd, prev = tl.smooth(0.30, 0.10, 0.8, 0.33)
    assert cmd == pytest.approx(0.8 * 0.30 + 0.2 * 0.10, abs=1e-12) and prev == cmd
    cmd, prev = tl.smooth(0.50, 0.40, 0.8, 0.33)
    assert cmd == 0.33 and prev == pytest.approx(0.48, abs=1e-12)      # clamped command, raw memory
    cmd, _ = tl.smooth(-0.50, -0.40, 0.8, 0.33)
    assert cmd == -0.33


# --- counting obstacles ------------------------------------------------------
def count(ranges):
    return tl.count_obstacles(ranges, AMIN, INC, 0.05, math.pi / 2, 3.0, 0.3, 5)


def test_count_corridor_two_walls_and_a_box_makes_three():
    assert count(cast(walls_y=(0.8, -0.8))) == 2
    assert count(cast(walls_y=(0.8, -0.8), boxes=[(1.5, 1.8, -0.15, 0.15)])) == 3
    assert count(np.full(N, np.inf)) == 0


def test_count_min_beams_boundary():
    r = np.full(N, np.inf)
    mid = N // 2
    r[mid:mid + 4] = 1.0
    assert count(r) == 0                                               # 4 beams: noise
    r[mid + 4] = 1.0
    assert count(r) == 1                                               # exactly 5: an obstacle


def test_params_refuse_nonsense():
    with pytest.raises(ValueError, match='invalid TrajectoryParams'):
        tl.TrajectoryParams(horizon_m=0.05)


def test_count_min_beams_boundary_at_the_end_of_the_scan():
    # A run still open when the scan ends is closed by the final check: exactly 5 counts, 4 does not.
    r = np.full(9, 1.0)
    r[:4] = np.inf
    assert tl.count_obstacles(r, 0.0, 0.01, 0.05, math.pi, 3.0, 0.3, 5) == 1
    r[4] = np.inf
    assert tl.count_obstacles(r, 0.0, 0.01, 0.05, math.pi, 3.0, 0.3, 5) == 0
