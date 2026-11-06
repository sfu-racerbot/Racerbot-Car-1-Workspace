"""Unit tests for tools/f1tenth_sim/tune_lookahead.py's pure logic (no gym).

Runner: the tool-suites line, python3 -m pytest tools/f1tenth_sim/tests/ -q.
Oracles: synthetic per-run results built so the right answer is known by
construction (ranking), hashlib on the file itself (line provenance), and
closed-form geometry against an analytic box obstacle (footprint clearance).
"""
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import tune_lookahead as tl  # noqa: E402

THRESH = 0.10


def run(la, seed, lap=10.0, coll=False, clr=0.30, cte=0.05, rms=0.02, steer=0.5,
        mode='stress', done=True):
    return dict(min_lookahead=la, speed_mode=mode, seed=seed, lap_completed=done,
                collision=coll, lap_time_s=lap if done else None, min_clearance_m=clr,
                max_abs_cte_m=cte, rms_cte_m=rms, mean_abs_steer_rate_rad_s=steer)


def winner_of(runs):
    mode, ranking = tl.rank(tl.aggregate(runs, THRESH), 'stress', 'cap_1.5')
    return mode, (ranking[0]['min_lookahead'] if ranking else None)


# 1 -- a crash can never be selected, however fast
def test_ranker_rejects_the_fastest_candidate_when_one_seed_collides():
    runs = [run(0.4, s, lap=8.0) for s in (0, 1, 2)] + [run(0.6, s, lap=10.0) for s in (0, 1, 2)]
    runs[1]['collision'] = True                        # 0.4 is fastest, but seed 1 crashes
    aggs = {(a['min_lookahead'], a['speed_mode']): a for a in tl.aggregate(runs, THRESH)}
    assert aggs[(0.4, 'stress')]['passed'] is False
    assert 'seed 1: collision' in aggs[(0.4, 'stress')]['fail_reason']
    assert winner_of(runs) == ('stress', 0.6)


# 2 -- clearance just below the threshold on one seed fails the candidate
def test_ranker_rejects_clearance_just_below_threshold_on_one_seed():
    runs = [run(0.4, s, lap=8.0) for s in (0, 1, 2)] + [run(0.6, s, lap=10.0) for s in (0, 1, 2)]
    runs[2]['min_clearance_m'] = THRESH - 1e-4         # one step past the boundary
    assert winner_of(runs) == ('stress', 0.6)
    # exactly at the threshold passes (>=), so the boundary itself is the edge
    runs[2]['min_clearance_m'] = THRESH
    assert winner_of(runs) == ('stress', 0.4)


def test_an_incomplete_flying_lap_fails_the_candidate():
    runs = [run(0.4, s, lap=8.0) for s in (0, 1, 2)] + [run(0.6, s, lap=10.0) for s in (0, 1, 2)]
    runs[0] = run(0.4, 0, done=False)
    assert winner_of(runs) == ('stress', 0.6)


def test_ranking_falls_back_to_the_baseline_cap_only_when_nothing_passes_at_stress():
    runs = [run(0.4, s, coll=True) for s in (0, 1)] + [run(0.4, s, mode='cap_1.5', lap=20.0) for s in (0, 1)]
    assert winner_of(runs) == ('cap_1.5', 0.4)


# 3 -- no winner: verdict text, and no YAML (a stale one is removed)
def test_no_winner_reports_no_safe_configuration_and_writes_no_yaml(tmp_path):
    runs = [run(la, s, coll=True) for la in (0.4, 0.6) for s in (0, 1)]
    runs += [run(la, s, mode='cap_1.5', clr=0.05) for la in (0.4, 0.6) for s in (0, 1)]
    aggs = tl.aggregate(runs, THRESH)
    mode, ranking = tl.rank(aggs)
    assert mode is None and ranking == []
    (tmp_path / 'recommended_lookahead.yaml').write_text('stale')
    doc = tl.write_outputs(tmp_path, runs, aggs, mode, ranking, {'map_id': 'x'})
    assert doc['verdict'] == 'NO SAFE CONFIGURATION FOUND' and doc['winner'] is None
    assert json.loads((tmp_path / 'lookahead_results.json').read_text())['verdict'] == tl.NO_WINNER
    assert not (tmp_path / 'recommended_lookahead.yaml').exists()


def test_a_winner_writes_yaml_with_the_controller_parameter_names(tmp_path):
    runs = [run(0.5, s) for s in (0, 1)]
    aggs = tl.aggregate(runs, THRESH)
    mode, ranking = tl.rank(aggs)
    tl.write_outputs(tmp_path, runs, aggs, mode, ranking, {'map_id': 'x'})
    import yaml
    params = yaml.safe_load((tmp_path / 'recommended_lookahead.yaml').read_text())
    assert params == {'pure_pursuit_node': {'ros__parameters': {
        'min_lookahead': 0.5, 'max_lookahead': 1.5, 'lookahead_speed_gain': 0.15}}}


# 4 -- lap times within 1% are tied; then lower max CTE wins
def test_tie_within_one_percent_is_broken_by_lower_max_cte():
    runs = [run(0.4, s, lap=10.00, cte=0.20) for s in (0, 1)] + [run(0.6, s, lap=10.09, cte=0.10) for s in (0, 1)]
    assert winner_of(runs) == ('stress', 0.6)          # 0.9% slower, half the CTE


def test_more_than_one_percent_slower_is_not_a_tie():
    runs = [run(0.4, s, lap=10.00, cte=0.20) for s in (0, 1)] + [run(0.6, s, lap=10.11, cte=0.10) for s in (0, 1)]
    assert winner_of(runs) == ('stress', 0.4)          # 1.1% slower loses despite lower CTE


def test_worst_seed_lap_time_decides_not_the_mean():
    runs = [run(0.4, 0, lap=8.0), run(0.4, 1, lap=12.0), run(0.6, 0, lap=10.5), run(0.6, 1, lap=10.5)]
    assert winner_of(runs) == ('stress', 0.6)          # 0.4's worst seed is 12.0


# 5 -- the line is the supplied file, or the run fails
def test_line_sha256_is_the_file_hash_and_columns_are_read(tmp_path):
    p = tmp_path / 'line.csv'
    p.write_text('x,y,speed\n0,0,1.0\n1,0,2.0\n1,1,3.0\n')
    xy, speed, sha = tl.load_line(p)
    assert sha == hashlib.sha256(p.read_bytes()).hexdigest()
    assert xy.tolist() == [[0, 0], [1, 0], [1, 1]] and speed.tolist() == [1.0, 2.0, 3.0]


def test_missing_line_fails_instead_of_falling_back(tmp_path):
    with pytest.raises(tl.LineError, match='not found'):
        tl.load_line(tmp_path / 'nope.csv')


@pytest.mark.parametrize('content', [b'x,y\n', b'x,y\n0,0\nnan,1\n2,2\n', b'x,y\n0,0\n1,1\ngarbage,row\n',
                                     b'\xff\xfe\x00bad', b'x,y\n0\n1\n2\n'])
def test_unreadable_or_degenerate_line_fails(tmp_path, content):
    p = tmp_path / 'bad.csv'
    p.write_bytes(content)
    with pytest.raises(tl.LineError, match='racing-line CSV'):
        tl.load_line(p)


# 6 -- clearance is the oriented chassis footprint, not the reference point
# Grid: 40 x 40 cells at 0.05 m, origin (-1, -1). Cell (row r, column c) is the square
# x in [-1 + 0.05c, + 0.05], y in [-1 + 0.05(39 - r), + 0.05].
def _grid(*cells):
    img = np.full((40, 40), 254, dtype=np.uint8)
    for x0, y0 in cells:                       # occupy the cell whose lower-left corner is (x0, y0)
        img[39 - int(round((y0 + 1) / 0.05)), int(round((x0 + 1) / 0.05))] = 0
    return tl.MapDistance(img, 0.05, (-1.0, -1.0))


def _body(x, y, yaw, back=-0.0875, front=0.4475, half=0.15):
    """The measured chassis (0.535 x 0.30, centred 0.18 m ahead of the rear axle), consecutive corners."""
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(back, -half), (front, -half), (front, half), (back, half)]
    return np.array([(x + a * c - b * s, y + a * s + b * c) for a, b in pts])


def test_corner_overlap_reports_non_positive_clearance_though_the_reference_point_is_clear():
    g = _grid((0.40, 0.10))                    # square [0.40, 0.45] x [0.10, 0.15]: holds the front-left corner
    assert g(0.0, 0.0) > 0.35                   # the rear-axle reference point is well clear
    assert g.body_clearance(_body(0, 0, 0)) <= 0.0


def test_clearance_is_the_closed_form_gap_from_the_nose():
    g = _grid((0.60, 0.0))                      # square [0.60, 0.65] x [0.00, 0.05], dead ahead
    assert g.body_clearance(_body(0, 0, 0)) == pytest.approx(0.60 - 0.4475, abs=1e-9)   # m


def test_clearance_follows_the_heading():
    # Rotated 90 deg left the body spans x in [-0.15, 0.15]: the same square is 0.60 - 0.15 away.
    g = _grid((0.60, 0.0))
    assert g.body_clearance(_body(0, 0, math.pi / 2)) == pytest.approx(0.60 - 0.15, abs=1e-9)  # m


def test_an_obstacle_wholly_inside_the_body_is_caught():
    g = _grid((0.10, -0.05))                    # one 5 cm cell under the car, touching no edge
    assert g.body_clearance(_body(0, 0, 0)) <= 0.0


def test_a_body_corner_inside_a_cell_is_caught():
    # Rotated 30 deg so a body vertex, not a cell vertex, is the overlap: the front-right
    # corner lands at (0.4475 cos30 + 0.15 sin30, 0.4475 sin30 - 0.15 cos30) = (0.4625, 0.0938).
    g = _grid((0.45, 0.05))                     # square [0.45, 0.50] x [0.05, 0.10]
    assert g.body_clearance(_body(0, 0, math.radians(30))) <= 0.0


def test_a_body_corner_nearest_a_cell_face_is_measured_from_the_corner():
    # Rotated 45 deg, the front-right corner (0.4475, -0.15) is the body's most +x point:
    # x = (0.4475 + 0.15) cos45 = 0.42250, y = 0.21037. A square with its left face at x = 0.50
    # spanning y in [0.20, 0.25] is nearest that corner: closed form 0.50 - 0.5975 cos45 (m).
    # Every cell vertex is farther (it is off the corner's y), so only the body-vertex side gives it.
    g = _grid((0.50, 0.20))
    expected = 0.50 - 0.5975 * math.cos(math.pi / 4)
    assert g.body_clearance(_body(0, 0, math.pi / 4)) == pytest.approx(expected, abs=1e-9)  # m


def test_open_space_saturates_at_the_search_radius():
    assert _grid().body_clearance(_body(0, 0, 0), search_m=0.5) == 0.5


def test_effective_lookahead_is_the_documented_clip():
    # L_d = clip(0.15 * 1.5 + min, min, 1.5): +0.225 m for every candidate at 1.5 m/s
    for la in tl.CANDIDATES:
        assert tl.effective_lookahead(1.5, la) == pytest.approx(la + 0.225, abs=1e-12)  # m
    assert tl.effective_lookahead(10.0, 0.8) == pytest.approx(1.5, abs=1e-12)          # the max clips


def test_map_distance_is_continuous_not_snapped_to_the_grid():
    # 10 x 10 cells at 0.05 m, origin (0, 0); one occupied cell: column 5, row 4 from the
    # top = world square x in [0.25, 0.30], y in [0.25, 0.30] (row r covers y from (h-1-r)*res).
    img = np.full((10, 10), 254, dtype=np.uint8)
    img[4, 5] = 0
    d = tl.MapDistance(img, 0.05, (0.0, 0.0))
    assert d(0.20, 0.27) == pytest.approx(0.05, abs=1e-12)            # m, straight left of the square
    assert d(0.23, 0.27) == pytest.approx(0.02, abs=1e-12)            # m, sub-cell: not 0 or 0.05
    assert d(0.20, 0.20) == pytest.approx(math.hypot(0.05, 0.05), abs=1e-12)  # m, to the corner
    assert d(0.27, 0.27) == 0.0                                       # inside the occupied cell
    assert d(-1.0, 0.0) == 0.0                                        # off the map counts as blocked
    img[4, 5] = 205                                                   # SLAM 'unknown' grey: free, as gym
    assert tl.MapDistance(img, 0.05, (0.0, 0.0))(0.27, 0.27) > 0.0
