"""Tests for run_summary, intent_report, spot, oa_replay's ego and truth helpers, and
car_measure.fit_braking. Pure functions on data the test built, so each answer is known.
Run by: python3 -m pytest tools/f1tenth_sim/tests/ tools/racerbot_sim/tests/ tools/run_analysis/tests/ tools/dev/tests/ -q
"""
import json
import math
import os
import sys

import numpy as np
import pytest

ROOT = os.path.join(os.path.dirname(__file__), '..', '..', '..')
sys.path[:0] = [os.path.join(ROOT, 'tools'), os.path.join(ROOT, 'src', 'gap_follow')]
from run_analysis import car_measure, intent_report as ir, oa_replay, run_summary as rs, spot  # noqa: E402


# --- run_summary ---------------------------------------------------------------------

def test_hold_windows_keeps_long_holds_and_closes_an_open_one():
    t = np.arange(0.0, 20.0, 0.5)
    lb = (t >= 1.0) & (t < 2.0) | (t >= 5.0) & (t < 10.0) | (t >= 16.0)
    assert rs.hold_windows(t, lb, min_s=3.0) == [(5.0, 10.0), (16.0, 19.5)]


def test_in_windows_marks_both_ends():
    assert rs.in_windows([0.9, 1.0, 2.0, 2.1], [(1.0, 2.0)]).tolist() == [False, True, True, False]


def test_gaps_reports_the_uneven_arrival():
    t = np.r_[np.arange(0.0, 1.0, 0.02), 1.1]          # 50 even gaps of 20 ms, then one of 120 ms
    g = rs.gaps(t)
    assert g["max_ms"] == pytest.approx(120.0, abs=1e-6)
    assert g["over_50ms"] == pytest.approx(1 / 50, abs=1e-12)


def test_width_from_scan_adds_the_two_nearest_side_walls():
    angles = np.linspace(-math.pi, math.pi, 721)
    # Walls 0.6 m to the left and 0.9 m to the right, parallel to the car.
    ranges = np.where(angles > 0, 0.6 / np.abs(np.sin(angles)), 0.9 / np.abs(np.sin(angles)))
    assert rs.width_from_scan(ranges, angles) == pytest.approx(1.5, abs=1e-6)     # m


def test_width_from_scan_needs_both_sides():
    angles = np.linspace(-math.pi, math.pi, 721)
    ranges = np.where(angles > 0, 0.6 / np.abs(np.sin(angles)), np.inf)
    assert math.isnan(rs.width_from_scan(ranges, angles))


def test_passed_through_counts_commands_the_mux_repeated():
    drive_t, drive_v = [0.0, 1.0, 2.0, 3.0], [1.0, 2.0, 3.0, 4.0]
    ack_t, ack_v = [0.02, 1.03, 2.5, 3.01], [1.0, 2.0, 3.0, 0.0]   # late, and a zero from teleop
    assert rs.passed_through(drive_t, drive_v, ack_t, ack_v) == pytest.approx(0.5, abs=1e-12)


def test_find_runs_prefers_the_first_root_and_sorts(tmp_path):
    a, b = tmp_path / "a", tmp_path / "b"
    for root, names in ((a, ["20261030-1", "20261030-3"]), (b, ["20261030-1", "20261030-2", "20261031-1"])):
        for n in names:
            (root / n).mkdir(parents=True)
    found = rs.find_runs("20261030", [a, b])
    assert [p.name for p in found] == ["20261030-1", "20261030-2", "20261030-3"]
    assert found[0].parent == a


# --- intent_report -------------------------------------------------------------------

def intent(state="gap_follow", speed=2.0, binding=(), factors=None):
    return ir.parse(json.dumps({"state": state, "commanded_speed": speed, "desired_speed": speed,
                                "commanded_steering": 0.1, "desired_steering": 0.1,
                                "factors": [{"name": n, "value": v, "binding": n in binding}
                                            for n, v in (factors or {}).items()]}))


def test_parse_flattens_an_intent_and_refuses_junk():
    r = intent(binding=("curve cap",), factors={"curve cap": 1.5, "accel ceiling": 3.0})
    assert r["speed"] == 2.0 and r["binding"] == ["curve cap"] and r["factors"]["accel ceiling"] == 3.0
    assert ir.parse("not json") is None and ir.parse("[1, 2]") is None and ir.parse('{"x": 1}') is None


def test_limiter_is_the_binding_factor_else_the_state():
    assert ir.limiter(intent(binding=("follow cap",), factors={"follow cap": 0.0})) == "follow cap"
    assert ir.limiter(intent(state="no_safe_gap")) == "no_safe_gap"


def test_shares_sum_to_one_and_rank():
    rows = [intent(binding=("a",), factors={"a": 1})] * 3 + [intent(state="s")]
    out = ir.shares(rows)
    assert list(out) == ["a", "s"] and out["a"] == pytest.approx(0.75, abs=1e-12)


def _events(speeds_and_cmds, lb_off_at=None, dt=0.025):
    events = [("joy", -0.01, True)]
    for i, (v, cmd) in enumerate(speeds_and_cmds):
        t = i * dt
        if lb_off_at is not None and abs(t - lb_off_at) < dt / 2:
            events.append(("joy", t, False))
        events.append(("odom", t, v))
        events.append(("intent", t, intent(speed=cmd, binding=("follow cap",), factors={"follow cap": cmd})))
    return events


def test_stalls_find_one_that_ends_by_itself():
    seq = [(2.0, 2.0)] * 10 + [(0.1, 0.0)] * 30 + [(1.0, 1.0)] * 10
    out = ir.stalls(_events(seq))
    assert len(out) == 1
    assert out[0]["ended"] == "by itself"
    assert out[0]["end"] - out[0]["start"] == pytest.approx(29 * 0.025, abs=1e-9)
    assert out[0]["limiters"]["follow cap"] == 30


def test_stalls_ended_by_the_driver_and_short_ones_dropped():
    seq = [(0.1, 0.0)] * 30 + [(0.1, 0.0)] * 5
    out = ir.stalls(_events(seq, lb_off_at=30 * 0.025))
    assert [s["ended"] for s in out] == ["LB released"]
    assert ir.stalls(_events([(0.1, 0.0)] * 5 + [(2.0, 2.0)] * 5)) == []      # 0.1 s: too short


def test_stalls_need_the_command_low_too():
    assert ir.stalls(_events([(0.1, 2.0)] * 40)) == []        # slow but asking to go: a launch, not a stall


def test_lb_toggles_reads_the_follow_cap_around_a_release():
    t = np.arange(0.0, 5.0, 0.1)
    cap = np.where((t >= 1.0) & (t < 2.55), 0.0, np.inf)
    out = ir.lb_toggles(t, cap, [(2.05, False), (2.25, True)])
    assert len(out) == 1
    g = out[0]
    assert g["cap_at_release"] == 0.0 and g["low_since"] == pytest.approx(1.0, abs=1e-9)
    assert g["low_after_press"] == pytest.approx(2.6 - 2.25, abs=1e-9)


def test_lb_toggles_ignore_a_press_without_a_release():
    t = np.arange(0.0, 2.0, 0.1)
    assert ir.lb_toggles(t, np.full(t.size, np.inf), [(0.5, True), (1.0, False)]) == []


# --- spot ----------------------------------------------------------------------------

def test_passes_counts_each_visit():
    s = np.linspace(0.0, 4 * math.pi, 400)
    x, y = 2.0 * np.cos(s), 2.0 * np.sin(s)        # two laps of a 2 m circle
    assert len(spot.passes(x, y, 2.0, 0.0, 0.5)) == 3    # the start, then once a lap, cut by the ends


def test_scan_world_puts_returns_where_the_laser_points():
    pts = spot.scan_world((1.0, 2.0, math.pi / 2), [1.0, np.inf, 2.0], [0.0, 0.5, math.pi / 2], laser_x=0.26)
    assert pts == pytest.approx(np.array([[1.0, 3.26], [-1.0, 2.26]]), abs=1e-12)


# --- oa_replay ego and truth -----------------------------------------------------------

def test_ego_motion_modes():
    rec = oa_replay.ego_motion("recorded", 3.1, 0.2, 3.0, 0.5, 0.36)
    assert rec == (3.1, 0.2)
    assert oa_replay.ego_motion("true-speed", 3.1, 0.2, 3.0, 0.5, 0.36) == (3.0, 0.2)
    speed, steering = oa_replay.ego_motion("true", 3.1, 0.2, 3.0, 0.5, 0.36)
    assert speed == 3.0 and steering == pytest.approx(math.atan(0.36 * 0.5 / 3.0), abs=1e-12)
    _s, crawl = oa_replay.ego_motion("gyro", 0.1, 0.2, 0.1, 0.5, 0.36)
    assert crawl == pytest.approx(math.atan(0.36 * 0.5 / 0.5), abs=1e-12)    # floored at 0.5 m/s


def test_ego_motion_refuses_an_unknown_mode():
    with pytest.raises(ValueError, match="ego must be one of"):
        oa_replay.ego_motion("odometry", 1.0, 0.0, 1.0, 0.0, 0.36)


def test_to_map_rotates_then_translates():
    assert oa_replay.to_map((1.0, 2.0, math.pi / 2), 1.0, 0.5) == pytest.approx((0.5, 3.0), abs=1e-12)


def test_track_truth_net_displacement_and_wall_distance():
    points = [(0.0, 0.0), (5.0, 5.0), (0.3, 0.4)]          # wanders, ends 0.5 m from where it began
    moved, wall = oa_replay.track_truth(points, lambda x, y: x)
    assert moved == pytest.approx(0.5, abs=1e-12)
    assert wall == pytest.approx(0.3, abs=1e-12)            # the median of 0, 5, 0.3


# --- car_measure.fit_braking -----------------------------------------------------------

def test_fit_braking_recovers_the_settings_that_made_the_stops():
    truth = (0.075, 7.0, 4.5)
    stops = [(v0, car_measure.stop_distance(v0, *truth)) for v0 in (2.0, 3.0, 5.0)]
    error, delay, gain, top = car_measure.fit_braking(stops, [0.05, 0.075, 0.1], [3.0, 7.0], [4.0, 4.5, 5.0])[0]
    assert (delay, gain, top) == truth and error < 1e-12
