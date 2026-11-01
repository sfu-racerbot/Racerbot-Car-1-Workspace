#!/usr/bin/env python3
"""Score a SLAM map, and SLAM's live pose, against a reference map of the same course.

    python3 tools/run_analysis/slam_score.py --reference data/<car>/racerbot_auto/<good run>/map.yaml TARGET...

TARGET is a run folder with a /map and /slam_pose (a map_course run, or a replay_map_course.py recording),
or a map.yaml (walls only). This is how the replay tables in docs/superpowers/specs/2026-10-30-slam-reliability.md were measured.
It is independent of the SLAM under test: the truth is the run's own scans tracked on the reference map, and the
map is compared wall to wall.

- walls: the share of the map's walls within 10 cm of a reference wall once the two are lined up, and the 95th
  percentile distance. A doubled map scores about half. Lining up is tried from many headings, because one start
  can settle in a wrong minimum: that once made a 0.86 map read 0.22.
- recall: the share of the reference's walls within 10 cm of one of the map's: what the map missed.
- live: how far SLAM's live pose was from the truth, after its first 6 s, median and 95th percentile. A live pose
  can be wrong while the final map is fine, and the lap counter steers by it.

Judge a setting over repeats on more than one run: identical settings on identical data gave 0.32 and 0.64.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from run_analysis import rebuild_run  # noqa: E402  (also puts src/pure_pursuit on the path)
from pure_pursuit import map_localize, occupancy_map  # noqa: E402

WITHIN_M = 0.10
COARSE_HEADING_DEG = 15.0
# Each heading is also tried from these shifts. A doubled map has two equally good fits, and from their middle
# Gauss-Newton settles between the copies, on neither: a doubled test map read 0.13 instead of about half.
START_OFFSETS_M = [(0.0, 0.0)] + [(0.4 * math.cos(a), 0.4 * math.sin(a)) for a in np.radians(np.arange(0, 360, 60))]
SETTLE_S = 6.0  # the live pose's first seconds are the start pose, right by construction


def wall_points(grid, resolution, origin_x, origin_y):
    """(N, 2) centres of the occupied cells, row 0 at the lowest y."""
    rows, cols = np.nonzero(np.asarray(grid) >= 50)
    return np.c_[origin_x + (cols + 0.5) * resolution, origin_y + (rows + 0.5) * resolution]


def transform(pose, xy):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    return np.c_[pose[0] + c * xy[:, 0] - s * xy[:, 1], pose[1] + s * xy[:, 0] + c * xy[:, 1]]


def inverse(pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (-(c * pose[0] + s * pose[1]), s * pose[0] - c * pose[1], -pose[2])


def agreement(field, xy):
    """(share within WITHIN_M of a wall, 95th percentile distance) of points already in field's frame."""
    if len(xy) == 0:
        return 0.0, math.inf
    d = field.distance(xy[:, 0], xy[:, 1])
    return float((d < WITHIN_M).mean()), float(np.percentile(d, 95))


def register(reference, xy, stride=3):
    """(pose taking xy into the reference's frame, share of xy then on its walls), from many headings."""
    points = np.asarray(xy, dtype=float)[::stride]
    rows, cols = np.nonzero(reference.walls)
    centre = np.array([reference.origin_x + (cols.mean() + 0.5) * reference.resolution,
                       reference.origin_y + (rows.mean() + 0.5) * reference.resolution])
    mean = points.mean(axis=0)
    best = ((0.0, 0.0, 0.0), -1.0)
    for heading in np.radians(np.arange(0.0, 360.0, COARSE_HEADING_DEG)):
        c, s = math.cos(heading), math.sin(heading)
        for dx, dy in START_OFFSETS_M:
            pose = (centre[0] + dx - (c * mean[0] - s * mean[1]), centre[1] + dy - (s * mean[0] + c * mean[1]),
                    heading)
            pose = map_localize.match_scan(reference, points[:, 0], points[:, 1], pose, iterations=40,
                                           start_trim=1.5, min_trim=0.10)
            share, _ = agreement(reference, transform(pose, points))
            if share > best[1]:
                best = (pose, share)
    return best


def pose_errors(truth, live, settle_s=SETTLE_S):
    """Median and p95 of position (m) and heading (deg) error of live rows (t, x, y, yaw) against truth rows."""
    truth, live = np.asarray(truth, float).reshape(-1, 4), np.asarray(live, float).reshape(-1, 4)
    if len(truth) < 2 or len(live) == 0:
        raise ValueError('no truth or no live pose to compare')
    keep = (live[:, 0] >= truth[0, 0] + settle_s) & (live[:, 0] <= truth[-1, 0])
    if not keep.any():
        raise ValueError('the live pose does not overlap the truth after its first seconds')
    t = live[keep, 0]
    yaw = np.unwrap(truth[:, 3])
    tx, ty, tyaw = (np.interp(t, truth[:, 0], column) for column in (truth[:, 1], truth[:, 2], yaw))
    position = np.hypot(live[keep, 1] - tx, live[keep, 2] - ty)
    heading = np.degrees(np.abs(np.angle(np.exp(1j * (live[keep, 3] - tyaw)))))
    return {'live_median_m': float(np.median(position)), 'live_p95_m': float(np.percentile(position, 95)),
            'heading_median_deg': float(np.median(heading)), 'heading_p95_deg': float(np.percentile(heading, 95)),
            'compared': int(keep.sum())}


def field_of(grid, resolution, origin_x, origin_y):
    grid = np.asarray(grid)
    return map_localize.WallField(grid >= 50, grid == 0, resolution, origin_x, origin_y)


def score_map(reference, grid, resolution, origin_x, origin_y):
    """Walls and recall of one map against the reference, and the pose that lines them up."""
    xy = wall_points(grid, resolution, origin_x, origin_y)
    if len(xy) < map_localize.MIN_BEAMS:
        return {'walls': 0.0, 'error': 'the map has almost no walls'}, None
    pose, _ = register(reference, xy)
    walls, p95 = agreement(reference, transform(pose, xy))
    ref_xy = wall_points(reference.walls.astype(np.int8) * 100, reference.resolution, reference.origin_x,
                         reference.origin_y)
    recall, _ = agreement(field_of(grid, resolution, origin_x, origin_y), transform(inverse(pose), ref_xy))
    return {'walls': walls, 'walls_p95_m': p95, 'recall': recall,
            'rotation_deg': math.degrees(pose[2])}, pose


def score_run(reference, run_dir):
    """score_map on the run's last /map, plus its live pose against its own scans tracked on the reference."""
    inputs = rebuild_run.load_inputs(run_dir)
    result, pose = score_map(reference, inputs.grid, inputs.resolution, inputs.origin_x, inputs.origin_y)
    if pose is None:
        return result
    live = np.asarray(inputs.slam_pose, float).reshape(-1, 4)
    if len(live) == 0:
        result['live'] = 'no /slam_pose in this run'
        return result
    xy = transform(pose, live[:, 1:3])
    live = np.c_[live[:, 0], xy, live[:, 3] + pose[2]]
    # The truth starts where the reference search puts the car; SLAM's pose only breaks a tie (rebuild_run).
    inputs.slam_pose = live
    placed = rebuild_run._place_car(reference, inputs)
    if placed is None:
        result['live'] = 'the reference has no place for these scans: another course?'
        return result
    start, first, _ = placed
    picked = np.arange(first, len(inputs.times), rebuild_run.SCAN_STRIDE)
    track = map_localize.track_scans(reference, inputs.times[picked], inputs.scans[picked], inputs.odom,
                                     inputs.angle_min, inputs.angle_increment, start)
    result['truth_fit'] = float(np.mean(track.fit))
    if result['truth_fit'] < map_localize.MIN_MEAN_FIT:
        result['live'] = f"the truth is unreliable here (fit {result['truth_fit']:.2f} on the reference)"
        return result
    result.update(pose_errors(np.c_[track.times, track.poses], live))
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--reference', type=Path, required=True, help="a good map's map.yaml")
    parser.add_argument('targets', type=Path, nargs='+', help='run folders or map.yaml files')
    args = parser.parse_args(argv)
    reference = map_localize.WallField.from_occupancy(occupancy_map.OccupancyMap.from_yaml(str(args.reference)))
    for target in args.targets:
        try:
            if target.suffix == '.yaml':
                loaded = occupancy_map.OccupancyMap.from_yaml(str(target))
                # OccupancyMap marks walls 1 and free 0; the bag's grids, which score_map takes, use 100 and 0.
                grid = np.where(loaded.grid == occupancy_map.OCCUPIED, 100,
                                np.where(loaded.grid == occupancy_map.FREE, 0, -1))
                result, _ = score_map(reference, grid, loaded.resolution, loaded.origin_x, loaded.origin_y)
            else:
                result = score_run(reference, target)
        except (rebuild_run.NothingToRebuild, ValueError, OSError) as exc:
            result = {'error': str(exc)}
        print(json.dumps({'target': str(target), **{k: (round(v, 3) if isinstance(v, float) else v)
                                                     for k, v in result.items()}}), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
