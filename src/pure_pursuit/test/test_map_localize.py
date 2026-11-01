"""Scan-to-map localization and the map verdict, with no ROS.

Runner: python3 -m pytest src/pure_pursuit/test/test_map_localize.py (no sourcing needed), and colcon test.

The oracle is a world this file builds: a corridor course on a grid, a car on a path whose pose is known
in closed form, and a ray-caster written here that shares nothing with the module under test. Scans it
produces have a known true pose, so the module is judged on recovering that pose.
"""
import math

import numpy as np
import pytest

from pure_pursuit import map_localize as ml

RES = 0.05
ANGLE_MIN = -math.radians(135.0)
ANGLE_INC = math.radians(0.25)
BEAMS = 1081
LASER_X = 0.26


def _course(textured=True):
    """A 24 x 14 m walled ring round an asymmetric centre block.

    Real walls have door frames and clutter, so the outer walls carry a bump of uneven size every few
    metres, clear of the car's path. Without them a straight says nothing about position along it, which
    is its own case: test_a_bare_corridor_stays_on_odometry.
    """
    g = np.zeros((int(14 / RES), int(24 / RES)), dtype=bool)
    g[:2, :] = g[-2:, :] = True
    g[:, :2] = g[:, -2:] = True
    g[int(4 / RES):int(10 / RES), int(5 / RES):int(19 / RES)] = True
    if not textured:
        return g
    for index, x in enumerate(np.arange(3.0, 22.0, 2.5)):
        depth = 0.3 + 0.25 * (index % 3)
        width = int((0.3 + 0.2 * (index % 2)) / RES)
        g[2:int(depth / RES), int(x / RES):int(x / RES) + width] = True          # south wall
        g[-int((depth + 0.2) / RES):, int((x + 1) / RES):int((x + 1) / RES) + width] = True  # north wall
    for index, y in enumerate(np.arange(3.0, 12.0, 2.5)):
        depth = 0.3 + 0.2 * (index % 3)
        g[int(y / RES):int((y + 0.4) / RES), 2:int(depth / RES)] = True           # west wall
        g[int((y + 1) / RES):int((y + 1.4) / RES), -int(depth / RES):] = True     # east wall
    return g


COURSE = _course()


def _field(grid=COURSE):
    free = ~grid
    return ml.WallField(grid, free, RES, 0.0, 0.0)


def _ray_cast(pose, grid=COURSE):
    """Independent oracle scan: march each beam until it hits a wall cell."""
    x, y, yaw = pose
    lx, ly = x + LASER_X * math.cos(yaw), y + LASER_X * math.sin(yaw)
    angles = yaw + ANGLE_MIN + ANGLE_INC * np.arange(BEAMS)
    distances = np.arange(0.1, 12.0, RES / 2)
    cols = ((lx + np.outer(np.cos(angles), distances)) / RES).astype(int)
    rows = ((ly + np.outer(np.sin(angles), distances)) / RES).astype(int)
    inside = (rows >= 0) & (rows < grid.shape[0]) & (cols >= 0) & (cols < grid.shape[1])
    hit = np.zeros(cols.shape, dtype=bool)
    hit[inside] = grid[rows[inside], cols[inside]]
    first = hit.argmax(axis=1)
    return np.where(hit.any(axis=1), distances[first], np.inf).astype(np.float32)


def _path_pose(s):
    """The car on a rounded rectangle centred in the corridor, s metres along it, anticlockwise."""
    west, east, south, north = 3.5, 20.5, 2.0, 12.0
    r = 1.5
    legs = [east - west - 2 * r, north - south - 2 * r, east - west - 2 * r, north - south - 2 * r]
    arc = math.pi / 2 * r
    lap = sum(legs) + 4 * arc
    s = s % lap
    corners = [(west + r, south), (east, south + r), (east - r, north), (west, north - r)]
    headings = [0.0, math.pi / 2, math.pi, 3 * math.pi / 2]
    for leg, (cx, cy), heading in zip(legs, corners, headings):
        if s <= leg:
            return cx + s * math.cos(heading), cy + s * math.sin(heading), heading
        s -= leg
        if s <= arc:
            t = s / r
            ex, ey = cx + leg * math.cos(heading), cy + leg * math.sin(heading)
            # centre of the turn is r to the left of the end of the leg
            ox, oy = ex - r * math.sin(heading), ey + r * math.cos(heading)
            return (ox + r * math.sin(heading + t), oy - r * math.cos(heading + t), heading + t)
        s -= arc
    raise AssertionError('s past the lap')


LAP_M = 2 * (17.0 - 3.0) + 2 * (10.0 - 3.0) + 2 * math.pi * 1.5


def _drive(laps=1.2, spacing=0.5, odom_scale=1.0, odom_yaw_rate_bias=0.0):
    """(times, scans, odom rows, true poses): one scan each `spacing` metres at 2.5 m/s."""
    distances = np.arange(0.0, laps * LAP_M, spacing)
    times = distances / 2.5
    poses = np.array([_path_pose(s) for s in distances])
    poses[:, 2] = np.unwrap(poses[:, 2])
    scans = np.array([_ray_cast(p) for p in poses])
    odom = np.c_[times, odom_scale * poses[:, 0], odom_scale * poses[:, 1], poses[:, 2] + odom_yaw_rate_bias * times]
    return times, scans, odom, poses


@pytest.fixture(scope='module')
def driven():
    return _drive()


def _beams(ranges):
    return ml.scan_points(ranges, ANGLE_MIN, ANGLE_INC, LASER_X)


# --- scan_points ---------------------------------------------------------------------------------

def test_scan_points_puts_a_beam_at_its_closed_form_place():
    ranges = np.full(BEAMS, np.inf)
    ranges[540] = 2.0  # straight ahead: angle_min + 540 * 0.25 deg = 0
    ranges[180] = 1.0  # 90 deg to the right
    x, y = ml.scan_points(ranges, ANGLE_MIN, ANGLE_INC, LASER_X, stride=1)
    assert sorted(zip(np.round(x, 6), np.round(y, 6))) == [(LASER_X, -1.0), (LASER_X + 2.0, 0.0)]


@pytest.mark.parametrize('bad', [np.nan, np.inf, -np.inf, 0.0, 0.05, -1.0, 12.5])
def test_scan_points_drops_unusable_ranges(bad):
    ranges = np.full(BEAMS, bad, dtype=float)
    ranges[10] = 3.0
    x, _ = ml.scan_points(ranges, ANGLE_MIN, ANGLE_INC, LASER_X, stride=1)
    assert len(x) == 1


def test_scan_points_of_an_empty_scan_is_empty():
    x, y = ml.scan_points(np.array([]), ANGLE_MIN, ANGLE_INC, LASER_X)
    assert len(x) == 0 and len(y) == 0


# --- WallField -----------------------------------------------------------------------------------

def test_field_distance_is_the_closed_form_distance_to_a_wall():
    grid = np.zeros((40, 40), dtype=bool)
    grid[:, 20] = True
    field = ml.WallField(grid, ~grid, 0.1, -2.0, -2.0)
    # oracle: closed form. The wall cell column spans x = 0.0..0.1; a point at x = -1 is 1 m from it.
    assert field.distance(np.array([-1.0]), np.array([0.0]))[0] == pytest.approx(1.0, abs=0.1)  # 1 cell


def test_field_refuses_mismatched_grids():
    with pytest.raises(ValueError, match='same shape'):
        ml.WallField(np.zeros((3, 3), bool), np.zeros((3, 4), bool), 0.1, 0, 0)


def test_field_refuses_a_map_with_no_walls():
    with pytest.raises(ValueError, match='no walls'):
        ml.WallField(np.zeros((3, 3), bool), np.ones((3, 3), bool), 0.1, 0, 0)


# --- match_scan ----------------------------------------------------------------------------------

@pytest.mark.parametrize('error', [
    pytest.param((0.2, 0.0, 0.0), id='x'),
    pytest.param((0.0, -0.2, 0.0), id='y'),
    pytest.param((0.0, 0.0, 0.08), id='yaw+'),
    pytest.param((0.0, 0.0, -0.08), id='yaw-'),
    pytest.param((-0.15, 0.15, 0.05), id='all'),
])
def test_match_recovers_a_true_pose_from_a_wrong_guess(error):
    # oracle: the true pose the scan was cast from. Wall texture makes every axis observable.
    truth = (8.5, 2.2, 0.0)
    bx, by = _beams(_ray_cast(truth))
    guess = (truth[0] + error[0], truth[1] + error[1], truth[2] + error[2])
    x, y, yaw = ml.match_scan(_field(), bx, by, guess)
    # 10 cm is two cells: the same limit as ON_WALL_M. A beam ends on a wall surface, the field bottoms out
    # inside the wall, and these fixture walls are solid, the worst case for that pull.
    assert x == pytest.approx(truth[0], abs=0.10)  # 10 cm
    assert y == pytest.approx(truth[1], abs=0.10)  # 10 cm
    assert yaw == pytest.approx(truth[2], abs=math.radians(1.0))  # 1 deg


def test_match_with_too_few_beams_leaves_the_pose_alone():
    # A blank scan says nothing, so odometry's pose stands: it must not become NaN or jump.
    guess = (8.0, 2.0, 0.3)
    assert ml.match_scan(_field(), np.array([1.0]), np.array([0.0]), guess) == guess


def test_fit_is_high_at_the_truth_and_low_when_wrong():
    truth = (8.5, 2.2, 0.0)
    bx, by = _beams(_ray_cast(truth))
    assert ml.scan_fit(_field(), bx, by, truth) > 0.95
    assert ml.scan_fit(_field(), bx, by, (10.5, 2.2, 0.6)) < 0.6


def test_fit_of_no_beams_is_zero():
    assert ml.scan_fit(_field(), np.array([]), np.array([]), (1.0, 1.0, 0.0)) == 0.0


# --- find_start ----------------------------------------------------------------------------------

def test_find_start_locates_the_car_with_no_guess():
    truth = (15.0, 11.6, math.pi)
    bx, by = _beams(_ray_cast(truth))
    pose, fit = ml.find_start(_field(), bx, by)
    assert fit > 0.9
    assert math.hypot(pose[0] - truth[0], pose[1] - truth[1]) < 0.1  # m
    assert abs(math.remainder(pose[2] - truth[2], 2 * math.pi)) < math.radians(2.0)


def test_find_start_can_be_limited_to_a_region():
    truth = (15.0, 11.6, math.pi)
    bx, by = _beams(_ray_cast(truth))
    pose, _ = ml.find_start(_field(), bx, by, near=(15.3, 11.4, math.pi - 0.1))
    assert math.hypot(pose[0] - truth[0], pose[1] - truth[1]) < 0.1  # m


def test_find_start_never_puts_the_car_inside_a_wall():
    # oracle: invariant. Inside a solid block every beam ends on a wall, a perfect score at an impossible
    # place. A scan of something close all round, people round the car, used to land there.
    ranges = np.full(BEAMS, 0.6)
    bx, by = _beams(ranges)
    field = _field()
    pose, fit = ml.find_start(field, bx, by)
    assert fit == 0.0 or field.is_free(pose[0], pose[1])
    assert fit < 0.9


def test_is_free_at_its_boundaries():
    # oracle: closed form. One free cell at row 1, col 2 of a 0.1 m grid at origin (1, 1).
    free = np.zeros((3, 4), dtype=bool)
    free[1, 2] = True
    field = ml.WallField(~free, free, 0.1, 1.0, 1.0)
    assert field.is_free(1.25, 1.15)
    assert not field.is_free(1.15, 1.15)   # next cell left
    assert not field.is_free(1.25, 1.05)   # next cell down
    assert not field.is_free(0.5, 1.15)    # off the map to the left
    assert not field.is_free(5.0, 1.15)    # off the map to the right


def test_find_start_with_no_beams_reports_no_fit():
    assert ml.find_start(_field(), np.array([]), np.array([]))[1] == 0.0


def test_find_start_trusts_no_place_from_fewer_than_min_beams():
    # oracle: spec, MIN_BEAMS's comment: fewer usable beams say too little. A handful of true beams fit perfectly
    # at the truth and at many places besides, so a full fit from them would be a guess dressed as a match.
    bx, by = _beams(_ray_cast((15.0, 11.6, math.pi)))
    few = slice(0, ml.MIN_BEAMS - 1)
    assert ml.find_start(_field(), bx[few], by[few]) == ((0.0, 0.0, 0.0), 0.0)
    _, fit = ml.find_start(_field(), bx[:ml.MIN_BEAMS], by[:ml.MIN_BEAMS])
    assert fit > 0.0  # exactly enough is searched


# --- track_scans ---------------------------------------------------------------------------------

def test_tracking_follows_the_true_path(driven):
    times, scans, odom, truth = driven
    result = ml.track_scans(_field(), times, scans, odom, ANGLE_MIN, ANGLE_INC, truth[0], laser_x=LASER_X)
    error = np.hypot(result.poses[:, 0] - truth[:, 0], result.poses[:, 1] - truth[:, 1])
    assert np.percentile(error, 95) < 0.10  # m
    assert result.fit.mean() > 0.9


def test_tracking_is_not_fooled_by_a_bad_odometry_scale():
    # oracle: the true path. Wheels over-reading 4%, as this car's do (runs 20261030-*): the scans
    # must correct it, so the result stays on the truth where raw odometry would be metres away.
    times, scans, odom, truth = _drive(odom_scale=1.04)
    result = ml.track_scans(_field(), times, scans, odom, ANGLE_MIN, ANGLE_INC, truth[0], laser_x=LASER_X)
    error = np.hypot(result.poses[:, 0] - truth[:, 0], result.poses[:, 1] - truth[:, 1])
    raw = np.hypot(odom[:, 1] - truth[:, 0], odom[:, 2] - truth[:, 1])
    assert raw.max() > 0.5  # the fixture's odometry really is bad
    assert np.percentile(error, 95) < 0.10  # m


def test_a_bare_corridor_stays_on_odometry():
    # oracle: closed form. Two parallel walls say nothing about position along them, so the odometry
    # step is all there is, and a match must not slide the pose along the corridor.
    grid = np.zeros((int(6 / RES), int(40 / RES)), dtype=bool)
    grid[:2, :] = grid[-2:, :] = True
    field = ml.WallField(grid, ~grid, RES, 0.0, 0.0)
    distances = np.arange(0.0, 8.0, 0.5)
    truth = np.c_[10.0 + distances, np.full(len(distances), 3.0), np.zeros(len(distances))]
    scans = np.array([_ray_cast(pose, grid) for pose in truth])
    odom = np.c_[distances / 2.5, truth]
    result = ml.track_scans(field, distances / 2.5, scans, odom, ANGLE_MIN, ANGLE_INC, truth[0], laser_x=LASER_X)
    assert np.abs(result.poses[:, 0] - truth[:, 0]).max() < 0.02  # m
    assert np.abs(result.poses[:, 1] - truth[:, 1]).max() < 0.10  # m, two cells


def test_a_lone_feature_does_not_drag_the_pose_along_a_corridor():
    # oracle: the true path. One bump on an otherwise bare wall: a few beams graze it from far off, and with
    # nothing else observing that direction they slid the pose metres along the corridor until a match was
    # held to its odometry prediction. Odometry here is perfect, so the truth is available to stay on.
    grid = np.zeros((int(6 / RES), int(40 / RES)), dtype=bool)
    grid[:2, :] = grid[-2:, :] = True
    grid[2:int(1.0 / RES), int(20 / RES):int(21 / RES)] = True
    field = ml.WallField(grid, ~grid, RES, 0.0, 0.0)
    distances = np.arange(0.0, 16.0, 0.5)
    truth = np.c_[10.0 + distances, np.full(len(distances), 3.0), np.zeros(len(distances))]
    scans = np.array([_ray_cast(pose, grid) for pose in truth])
    odom = np.c_[distances / 2.5, truth]
    result = ml.track_scans(field, distances / 2.5, scans, odom, ANGLE_MIN, ANGLE_INC, truth[0], laser_x=LASER_X)
    assert np.abs(result.poses[:, 0] - truth[:, 0]).max() < 0.15  # m, three cells


def test_tracking_survives_blank_scans_on_odometry(driven):
    # A scan consumer's edge: all-inf frames in the middle. Pose carries on from odometry, and those
    # frames report zero fit so a verdict can see them.
    times, scans, odom, truth = driven
    scans = scans.copy()
    scans[20:24] = np.inf
    result = ml.track_scans(_field(), times, scans, odom, ANGLE_MIN, ANGLE_INC, truth[0], laser_x=LASER_X)
    assert np.all(result.fit[20:24] == 0.0)
    error = np.hypot(result.poses[:, 0] - truth[:, 0], result.poses[:, 1] - truth[:, 1])
    assert error.max() < 0.3  # m


def test_tracking_with_no_scans_is_empty():
    result = ml.track_scans(_field(), np.zeros(0), np.zeros((0, BEAMS)),
                            np.array([[0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]),
                            ANGLE_MIN, ANGLE_INC, (5.0, 2.0, 0.0))
    assert result.poses.shape == (0, 3) and len(result.fit) == 0


def test_tracking_refuses_empty_odometry(driven):
    times, scans, _, truth = driven
    with pytest.raises(ValueError, match='odometry'):
        ml.track_scans(_field(), times, scans, np.zeros((0, 4)), ANGLE_MIN, ANGLE_INC, truth[0])


# --- see_through ---------------------------------------------------------------------------------

def _doubled(grid=COURSE):
    """The same course with a false copy of the south wall in the open corridor, as a bad SLAM map draws."""
    bad = grid.copy()
    bad[int(2.5 / RES):int(2.6 / RES), int(2 / RES):int(22 / RES)] = True
    return bad


def test_see_through_is_low_on_the_true_map(driven):
    times, scans, _, truth = driven
    rates = ml.see_through_rate(_field(), truth, scans, ANGLE_MIN, ANGLE_INC, laser_x=LASER_X)
    assert rates.mean() < 0.02


def test_see_through_catches_a_doubled_wall(driven):
    # oracle: invariant. The true rays pass straight through the false wall, whatever a fit score says.
    times, scans, _, truth = driven
    rates = ml.see_through_rate(_field(_doubled()), truth, scans, ANGLE_MIN, ANGLE_INC, laser_x=LASER_X)
    assert rates.mean() > 0.10


def test_see_through_does_not_count_a_beam_ending_at_the_wall_it_hit():
    # oracle: closed form. One beam, ending 2 m ahead at a wall: no wall crossed before its end.
    grid = np.zeros((100, 100), dtype=bool)
    grid[:, 50] = True  # a wall at x = 2.5
    field = ml.WallField(grid, ~grid, RES, 0.0, 0.0)
    ranges = np.full(BEAMS, np.inf)
    ranges[540] = 2.5 - 0.26 - 0.1
    poses = np.array([[0.1, 2.5, 0.0]])
    assert ml.see_through_rate(field, poses, ranges[None], ANGLE_MIN, ANGLE_INC, laser_x=LASER_X, stride=1)[0] == 0.0
    # The same beam going 1 m past that wall did cross it.
    ranges[540] = 3.2
    assert ml.see_through_rate(field, poses, ranges[None], ANGLE_MIN, ANGLE_INC, laser_x=LASER_X, stride=1)[0] == 1.0


def test_see_through_of_nothing_is_empty():
    assert len(ml.see_through_rate(_field(), np.zeros((0, 3)), np.zeros((0, BEAMS)), ANGLE_MIN, ANGLE_INC)) == 0


# --- find_laps -----------------------------------------------------------------------------------

def _lap_poses(laps, spacing=0.25):
    distances = np.arange(0.0, laps * LAP_M, spacing)
    poses = np.array([_path_pose(s) for s in distances])
    poses[:, 2] = np.unwrap(poses[:, 2])
    return poses


def test_find_laps_counts_whole_laps_of_the_right_length():
    laps = ml.find_laps(_lap_poses(3.4))
    assert len(laps) == 3
    for lap in laps:
        assert lap.length == pytest.approx(LAP_M, abs=0.5)  # m, two samples
        assert lap.gap < 0.3  # m


def test_find_laps_on_less_than_a_lap_is_empty():
    assert ml.find_laps(_lap_poses(0.8)) == []


def test_find_laps_on_nothing_is_empty():
    assert ml.find_laps(np.zeros((0, 3))) == []
    assert ml.find_laps(np.zeros((1, 3))) == []


def test_find_laps_does_not_count_a_spin_on_the_spot():
    # A full turn with no travel is not a lap: nothing went round a course.
    yaw = np.linspace(0, 3 * math.pi, 200)
    assert ml.find_laps(np.c_[np.zeros(200), np.zeros(200), yaw]) == []


def test_find_laps_does_not_count_a_full_turn_that_ends_elsewhere():
    # oracle: invariant. 2*pi of turning on a spiral ends 5 m from where it started: not closed.
    s = np.linspace(0, 1, 400)
    yaw = 2.2 * math.pi * s
    radius = 3.0 + 6.0 * s
    poses = np.c_[radius * np.sin(yaw), 3.0 - radius * np.cos(yaw), yaw]
    assert ml.find_laps(poses) == []


# --- judge ---------------------------------------------------------------------------------------

def test_a_clean_map_is_accepted():
    assert ml.judge(np.full(100, 0.97), np.full(100, 0.05)).ok


@pytest.mark.parametrize('fit, through, needle', [
    pytest.param(0.97, 0.30, 'see-through', id='doubled-walls'),
    pytest.param(0.50, 0.05, 'fit', id='scans-off-the-map'),
])
def test_a_bad_map_is_refused_and_says_why(fit, through, needle):
    verdict = ml.judge(np.full(100, fit), np.full(100, through))
    assert not verdict.ok
    assert needle in verdict.reason


def test_a_steadily_poor_fit_is_refused_on_its_mean():
    # 0.85 everywhere: never a lost stretch, but under the mean limit all the same.
    verdict = ml.judge(np.full(100, 0.85), np.full(100, 0.05))
    assert not verdict.ok
    assert 'mean fit' in verdict.reason
    assert 'worst' not in verdict.reason


def test_one_lost_stretch_refuses_even_with_a_good_average():
    # 20 scans lost of 200 leaves a 0.92 mean, past the mean limit, but the worst window is at zero.
    fit = np.full(200, 0.97)
    fit[100:120] = 0.1
    verdict = ml.judge(fit, np.full(200, 0.05), window=10)
    assert not verdict.ok
    assert 'worst' in verdict.reason


@pytest.mark.parametrize('through, expected', [
    pytest.param(ml.MAX_SEE_THROUGH, True, id='exactly-at-the-limit'),
    pytest.param(ml.MAX_SEE_THROUGH + 1 / 1024, False, id='just-past'),
])
def test_see_through_limit_boundary(through, expected):
    assert ml.judge(np.full(100, 0.97), np.full(100, through)).ok is expected


def test_nothing_to_judge_is_not_a_pass():
    verdict = ml.judge(np.zeros(0), np.zeros(0))
    assert not verdict.ok
    assert 'no scans' in verdict.reason


def test_nan_in_the_inputs_is_not_a_pass():
    assert not ml.judge(np.full(100, np.nan), np.full(100, 0.05)).ok
    assert not ml.judge(np.full(100, 0.97), np.full(100, np.nan)).ok
