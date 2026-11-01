"""slam_score.py: a map lined up with a reference, and a live pose against the truth.

Runner: python3 -m pytest tools/run_analysis/tests/ (no ROS needed).
Oracles: closed forms. A map that is the reference moved by a known rigid transform must line up with that
transform undone and score about 1; the reference drawn twice, 0.6 m apart, about half; a live pose a fixed
0.3 m or 5 deg off the truth, exactly that. Reading real runs is exercised by running it; results in
docs/superpowers/specs/2026-10-30-slam-reliability.md.
"""
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from run_analysis import slam_score as ss  # noqa: E402

RES = 0.05


def _course():
    """A walled ring with uneven bumps, so it lines up one way only."""
    g = np.zeros((int(10 / RES), int(16 / RES)), dtype=np.int8)
    g[:2, :] = g[-2:, :] = 100
    g[:, :2] = g[:, -2:] = 100
    # The inner block as SLAM draws it: its surface only. A solid block would hold a shifted copy inside it.
    g[int(3 / RES):int(7 / RES), int(3 / RES):int(13 / RES)] = 100
    g[int(3 / RES) + 2:int(7 / RES) - 2, int(3 / RES) + 2:int(13 / RES) - 2] = 0
    for index, x in enumerate(np.arange(1.0, 15.0, 1.7)):
        depth = int((0.3 + 0.2 * (index % 3)) / RES)
        g[2:2 + depth, int(x / RES):int(x / RES) + 6] = 100
    for index, y in enumerate(np.arange(1.0, 9.0, 1.9)):
        g[int(y / RES):int(y / RES) + 5, 2:2 + int((0.3 + 0.25 * (index % 2)) / RES)] = 100
    return g


COURSE = _course()
REFERENCE = ss.field_of(COURSE, RES, 0.0, 0.0)
WALLS = ss.wall_points(COURSE, RES, 0.0, 0.0)


def test_wall_points_are_cell_centres():
    g = np.zeros((4, 5), dtype=np.int8)
    g[1, 3] = 100
    assert np.allclose(ss.wall_points(g, 0.1, -1.0, 2.0), [[-1.0 + 0.35, 2.0 + 0.15]])


def test_inverse_undoes_transform():
    pose = (1.3, -0.7, 0.9)
    xy = np.array([[0.0, 0.0], [2.0, -1.0], [-3.5, 4.0]])
    assert np.allclose(ss.transform(ss.inverse(pose), ss.transform(pose, xy)), xy, atol=1e-12)


def test_a_moved_copy_lines_up_with_the_move_undone():
    moved_by = (2.0, -1.5, math.radians(40.0))
    pose, share = ss.register(REFERENCE, ss.transform(moved_by, WALLS))
    back = ss.transform(pose, ss.transform(moved_by, WALLS))
    assert share > 0.95
    assert np.median(np.hypot(*(back - WALLS).T)) < 0.05  # m


def test_the_score_of_the_reference_against_itself_is_one():
    result, _ = ss.score_map(REFERENCE, COURSE, RES, 0.0, 0.0)
    assert result['walls'] > 0.98
    assert result['recall'] > 0.98
    assert abs(result['rotation_deg']) < 1.0


def test_a_map_drawn_twice_scores_about_half_and_misses_nothing():
    # The doubled-walls failure: every wall twice, the copy 0.42 m across and 0.42 m up, so that no wall of the
    # copy lies along its original (a copy moved along x alone keeps every horizontal wall: that scores 0.87).
    shift = int(0.42 / RES)
    doubled = np.pad(COURSE, ((0, shift), (0, shift)))  # room for the whole copy
    doubled[shift:, shift:] = np.maximum(doubled[shift:, shift:], COURSE)
    result, _ = ss.score_map(REFERENCE, doubled, RES, 0.0, 0.0)
    assert 0.35 < result['walls'] < 0.75
    assert result['recall'] > 0.95


def test_a_map_of_somewhere_else_scores_low():
    rng = np.random.default_rng(7)
    elsewhere = np.zeros_like(COURSE)
    elsewhere[rng.integers(0, COURSE.shape[0], 1500), rng.integers(0, COURSE.shape[1], 1500)] = 100
    result, _ = ss.score_map(REFERENCE, elsewhere, RES, 0.0, 0.0)
    assert result['walls'] < 0.5


def test_a_map_with_no_walls_is_reported_not_scored():
    result, pose = ss.score_map(REFERENCE, np.zeros_like(COURSE), RES, 0.0, 0.0)
    assert pose is None and result['walls'] == 0.0


def _truth(seconds=20.0):
    t = np.arange(0.0, seconds, 0.05)
    return np.c_[t, 0.5 * t, np.sin(t), 0.2 * t]


def test_a_live_pose_off_by_a_fixed_distance_reads_that_distance():
    truth = _truth()
    live = truth.copy()
    live[:, 1] += 0.3
    errors = ss.pose_errors(truth, live)
    assert errors['live_median_m'] == pytest.approx(0.3, abs=1e-6)  # m
    assert errors['live_p95_m'] == pytest.approx(0.3, abs=1e-6)  # m
    assert errors['heading_median_deg'] == pytest.approx(0.0, abs=1e-6)


def test_a_live_heading_off_by_five_degrees_reads_five_across_the_wrap():
    truth = _truth()
    truth[:, 3] = math.pi - 0.01  # heading next to the +-pi wrap
    live = truth.copy()
    live[:, 3] = -math.pi + math.radians(5.0) - 0.01 + 1e-9  # 5 deg on, written past the wrap
    assert ss.pose_errors(truth, live)['heading_median_deg'] == pytest.approx(5.0, abs=1e-3)


def test_the_first_seconds_are_not_counted():
    truth = _truth()
    live = truth.copy()
    live[live[:, 0] < ss.SETTLE_S, 1] += 10.0  # wildly off, but only while settling
    errors = ss.pose_errors(truth, live)
    assert errors['live_p95_m'] == pytest.approx(0.0, abs=1e-9)
    assert errors['compared'] == int(np.sum(truth[:, 0] >= ss.SETTLE_S))


def test_a_live_pose_that_never_overlaps_the_truth_is_refused():
    truth = _truth(5.0)
    with pytest.raises(ValueError, match='does not overlap'):
        ss.pose_errors(truth, truth)
    with pytest.raises(ValueError, match='no truth'):
        ss.pose_errors(truth[:1], truth)
