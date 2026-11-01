"""Tests for tools.run_analysis.clean_map -- a run's SLAM map made race-ready.

Unsourced, no ROS:

    python3 -m pytest tools/run_analysis/tests/test_clean_map.py -v -p no:cacheprovider

Oracles: hand-built grids whose answer is read off the construction (a
straight corridor 1 m wide at 0.05 m/cell, with a speck, a blob and a leak
placed at known cells), and the shipped map that this code made on 2026-10-27.
"""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..'))  # tools/
from run_analysis import clean_map as cm  # noqa: E402

WS = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
RES = 0.05  # m per cell
FREE, OCC, UNKNOWN = 0, 100, -1


def corridor():
    """A 100 x 200 cell world (5 m x 10 m), all unknown except a corridor of
    free cells, rows 40..59 (1 m wide), walled by occupied rows 39 and 60. The
    car drove down its centre line, row 50, so path y = 50.5 cells."""
    grid = np.full((100, 200), UNKNOWN, dtype=np.int8)
    grid[40:60, :] = FREE
    grid[39, :] = OCC
    grid[60, :] = OCC
    xs = (np.arange(5, 195) + 0.5) * RES
    path = np.column_stack([xs, np.full_like(xs, 50.5 * RES)])
    return grid, path


def test_corridor_is_track_and_its_walls_are_walls():
    grid, path = corridor()
    img, track, wall = cm.clean_grid(grid, RES, 0.0, 0.0, path)
    # oracle: construction -- every corridor cell is free, connected to the
    # path and within 0.5 m of it, so all of them are track.
    assert track[40:60, :].all()
    assert img[40:60, :].min() == cm.PGM_FREE
    # oracle: closed form -- the ring holds the cells k = 1, 2, ... away with
    # k * RES <= WALL_M, evaluated in floating point as the code does (at 0.05 m
    # that is 2 cells: 3 * 0.05 = 0.15000000000000002 > 0.15).
    depth = max(k for k in range(1, 10) if k * RES <= cm.WALL_M)
    assert depth == 2
    assert (img[40 - depth:40, :] == cm.PGM_WALL).all()
    assert (img[60:60 + depth, :] == cm.PGM_WALL).all()
    # One cell past the ring is unknown, not wall.
    assert (img[39 - depth, :] == cm.PGM_UNKNOWN).all()
    assert (img[60 + depth, :] == cm.PGM_UNKNOWN).all()


def test_a_speck_on_the_track_is_cleared_and_a_blob_is_kept():
    grid, path = corridor()
    grid[44:46, 100:102] = OCC          # 4 cells: a speck (<= 8)
    grid[44:47, 150:153] = OCC          # 9 cells: one past the speck limit
    img, track, _ = cm.clean_grid(grid, RES, 0.0, 0.0, path)
    # oracle: construction -- the speck becomes driveable track...
    assert track[44:46, 100:102].all()
    # ...but a blob of SPECK_CELLS + 1 is an obstacle and stays wall.
    assert (img[44:47, 150:153] == cm.PGM_WALL).all()


def test_free_space_leaking_through_a_wall_gap_is_not_track():
    grid, path = corridor()
    # A gap in the top wall at columns 100..103, and a "ray" of free cells
    # beyond it out to row 99 (2.45 m to 2.45+ m from the path at row 50.5).
    grid[60, 100:104] = FREE
    grid[61:100, 100:104] = FREE
    img, track, _ = cm.clean_grid(grid, RES, 0.0, 0.0, path)
    # oracle: construction -- the ray is connected to the track, but every
    # cell more than REACH_M = 2.0 m (40 cells) from the path is cut off.
    beyond = (np.arange(100)[:, None] + 0.5 - 50.5) * RES > cm.REACH_M + RES
    assert not track[61:100, 100:104][beyond[61:100, 0]].any()
    assert (img[95:100, 100:104] == cm.PGM_UNKNOWN).all()


def test_free_space_not_connected_to_the_path_is_not_track():
    grid, path = corridor()
    grid[70:80, 20:30] = FREE           # an island, within reach but walled off
    _, track, _ = cm.clean_grid(grid, RES, 0.0, 0.0, path)
    # oracle: construction -- it never touches a driven cell's component.
    assert not track[70:80, 20:30].any()


def test_no_path_means_no_track():
    grid, _ = corridor()
    img, track, wall = cm.clean_grid(grid, RES, 0.0, 0.0, np.empty((0, 2)))
    # oracle: invariant -- track is defined by the driven path; none, none.
    assert not track.any() and not wall.any()
    assert (img == cm.PGM_UNKNOWN).all()


def test_reproduces_the_shipped_car1_course_map():
    # change-detector (not an oracle): the map clean_map.py made on 2026-10-27,
    # which became the simulator's car 1 course. The run's cached SLAM arrays
    # (analysis/slam.npz) load without ROS.
    run = os.path.join(WS, 'data', 'rb1', 'racerbot_runs', '20261026-194514')
    shipped = os.path.join(WS, 'data', 'rb1', 'racerbot_auto', '20261026-194514-clean', 'map.pgm')
    if not os.path.isfile(os.path.join(run, 'analysis', 'slam.npz')):
        pytest.skip('data/rb1/racerbot_runs/20261026-194514/analysis/slam.npz is not in this checkout')
    from PIL import Image
    from run_analysis import slam_bag
    d = slam_bag.load(run)
    res, ox, oy, grid = d['map']
    img, _, _ = cm.clean_grid(grid, res, ox, oy, d['slam_pose'][:, 1:3])
    assert np.array_equal(img[::-1], np.asarray(Image.open(shipped)))


def test_a_speck_exactly_at_the_limit_is_cleared():
    grid, path = corridor()
    grid[44:46, 100:104] = OCC          # 2 x 4 = 8 cells = SPECK_CELLS exactly
    grid[44:47, 150:153] = OCC          # 9 cells, one past it
    assert cm.SPECK_CELLS == 8
    img, track, _ = cm.clean_grid(grid, RES, 0.0, 0.0, path)
    # oracle: spec -- "blobs of <= SPECK_CELLS cells are cleared" (module docstring;
    # auto_map_race's map_despeckle_max_cells is 8). At the limit: cleared.
    assert track[44:46, 100:104].all()
    assert (img[44:47, 150:153] == cm.PGM_WALL).all()


def test_a_cell_exactly_wall_m_from_the_track_is_wall():
    grid, path = corridor()
    # 2 * 0.05 == 0.1 exactly in floating point, so the second cell out sits
    # exactly on the boundary.
    assert 2 * RES == 0.1
    img, _, _ = cm.clean_grid(grid, RES, 0.0, 0.0, path, wall_m=0.1)
    # oracle: spec -- "every non-track cell within WALL_M of the track" (inclusive).
    assert (img[38, :] == cm.PGM_WALL).all() and (img[61, :] == cm.PGM_WALL).all()
    assert (img[37, :] == cm.PGM_UNKNOWN).all() and (img[62, :] == cm.PGM_UNKNOWN).all()
