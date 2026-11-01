#!/usr/bin/env python3
"""Clean a run's final SLAM map into a race-ready map_server map.

    python3 tools/run_analysis/clean_map.py <run_dir> <out_dir>

Made data/racerbot_auto/20261026-194514-clean/, which became the simulator's
car 1 course (tools/f1tenth_sim/maps/car1_course_2026_10_26_*). Written in a
2026-10-27 session for that one run; generalised here to any run with a /map
and a SLAM pose in its bag (read through slam_bag.load, cached in analysis/).

Steps:
1. free = cells SLAM marked free (0 <= p < 25), occupied = p > 65, the rest unknown.
2. specks: occupied blobs of <= SPECK_CELLS cells touching free space are cleared
   (auto_map_race's own map_despeckle_max_cells is 8).
3. track = free cells connected to the driven path (every SLAM pose sample) and within
   REACH_M of it. That drops free-space "rays" leaking through wall gaps.
4. walls = every non-track cell within WALL_M of the track (centre to centre, in
   floating point: at 0.05 m cells 3 x 0.05 > 0.15, so the ring is 2 cells, 0.10 m,
   deep -- as in the shipped 2026-10-27 map); everything else unknown.

Writes map.pgm (0 wall, 254 free, 205 unknown), map.yaml, raw_vs_clean.png and
SOURCE.txt. Needs numpy, scipy, Pillow, PyYAML; reading an uncached bag needs ROS sourced.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REACH_M = 2.0
WALL_M = 0.15
SPECK_CELLS = 8
FREE_BELOW = 25
OCCUPIED_ABOVE = 65
PGM_WALL, PGM_FREE, PGM_UNKNOWN = 0, 254, 205


def clean_grid(grid, res, ox, oy, path_xy, reach_m=REACH_M, wall_m=WALL_M,
               speck_cells=SPECK_CELLS):
    """The cleaned map as a uint8 image in grid order (row 0 = origin y), plus
    the track and wall masks. grid: OccupancyGrid values (-1 unknown, 0..100).
    path_xy: (N, 2) driven positions in the map frame [m]."""
    from scipy import ndimage

    grid = np.asarray(grid).astype(int)
    free = (grid >= 0) & (grid < FREE_BELOW)
    occ = grid > OCCUPIED_ABOVE

    lab, n = ndimage.label(occ, structure=np.ones((3, 3)))
    sizes = ndimage.sum(occ, lab, index=np.arange(1, n + 1))
    small = np.isin(lab, np.nonzero(sizes <= speck_cells)[0] + 1)
    cleared = small & ndimage.binary_dilation(free, iterations=1)
    occ &= ~cleared
    free |= cleared

    path_xy = np.asarray(path_xy, dtype=float).reshape(-1, 2)
    if not len(path_xy):
        # No driven path, no track: unknown everywhere, never an all-wall map.
        none = np.zeros(grid.shape, dtype=bool)
        return np.full(grid.shape, PGM_UNKNOWN, dtype=np.uint8), none, none.copy()
    cols = np.clip(((path_xy[:, 0] - ox) / res).astype(int), 0, grid.shape[1] - 1)
    rows = np.clip(((path_xy[:, 1] - oy) / res).astype(int), 0, grid.shape[0] - 1)
    path = np.zeros_like(free)
    path[rows, cols] = True
    dist_to_path = ndimage.distance_transform_edt(~path) * res
    candidate = free & (dist_to_path <= reach_m)
    lab, _ = ndimage.label(candidate)
    keep = np.unique(lab[path & candidate])
    track = np.isin(lab, keep[keep > 0])

    near_track = ndimage.distance_transform_edt(~track) * res <= wall_m
    wall = (near_track & ~track) | (occ & near_track)

    img = np.full(grid.shape, PGM_UNKNOWN, dtype=np.uint8)
    img[track] = PGM_FREE
    img[wall] = PGM_WALL
    return img, track, wall


def main(argv=None):
    import yaml
    from PIL import Image

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from run_analysis import slam_bag

    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 2:
        print(__doc__.split('\n\n')[1], file=sys.stderr)
        return 2
    run, out = Path(argv[0]).expanduser(), Path(argv[1]).expanduser()
    d = slam_bag.load(run)
    if d['map'] is None or not len(d['slam_pose']):
        print(f'{run}: the bag has no /map or no SLAM pose', file=sys.stderr)
        return 1
    res, ox, oy, grid = d['map']
    sp = d['slam_pose']
    img, track, wall = clean_grid(grid, res, ox, oy, sp[:, 1:3])

    out.mkdir(parents=True, exist_ok=False)
    Image.fromarray(img[::-1]).save(out / 'map.pgm')  # pgm row 0 = top = max y
    with open(out / 'map.yaml', 'w') as f:
        yaml.safe_dump({'image': 'map.pgm', 'mode': 'trinary', 'resolution': res,
                        'origin': [ox, oy, 0.0], 'negate': 0,
                        'occupied_thresh': 0.65, 'free_thresh': 0.25}, f, sort_keys=False)
    g = np.asarray(grid).astype(int)
    raw = np.full(g.shape, PGM_UNKNOWN, dtype=np.uint8)
    raw[(g >= 0) & (g < FREE_BELOW)] = PGM_FREE
    raw[g > OCCUPIED_ABOVE] = PGM_WALL
    both = np.concatenate([raw[::-1], np.full((g.shape[0], 10), 128, np.uint8), img[::-1]], axis=1)
    Image.fromarray(both).save(out / 'raw_vs_clean.png')
    (out / 'SOURCE.txt').write_text(
        f'Cleaned final SLAM map of run {run.name}.\n'
        f'Made by tools/run_analysis/clean_map.py: specks <= {SPECK_CELLS} cells cleared; track = free\n'
        f'space connected to the driven SLAM path and within {REACH_M} m of it; walls = {WALL_M} m\n'
        f'ring around the track. Resolution {res} m, origin ({ox:.3f}, {oy:.3f}).\n')
    print(f'track cells {int(track.sum())}, wall cells {int(wall.sum())} -> {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
