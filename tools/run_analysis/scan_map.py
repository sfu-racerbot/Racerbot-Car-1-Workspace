#!/usr/bin/env python3
"""Rebuild a saved map's walls from where real scans actually landed.

    python3 tools/run_analysis/scan_map.py data/racerbot_runs/20261030-185036 ... \\
        --map data/racerbot_auto/20261030-185036-fixed --out /tmp/walls

SLAM's walls are a few cells thick and grow into free space, so a simulator using them
drives a narrower corridor than the floor has. This counts, per cell, the beams that
ended there and the beams that passed through, and writes two maps:

    <out>.png          what beams see: cells beams end in, with the gaps the floor has
    <out>_contact.png  what a car hits: the saved walls with free cells peeled away,
                       never opened, and a cell no beam reached keeps what the map said

The gaps matter. They make the scan fragments that the opponent tracker reads as cars,
and a closed map has almost none. Needs each run located with loc.py first.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

LASER_X = 0.26          # m ahead of base_link, docs/hardware-reference.md
MIN_MATCH = 0.90        # share of beams within 10 cm for a pose to be trusted
MIN_RANGE = 0.45        # m; nearer returns are the car itself or noise
MAX_RANGE = 10.0        # m; further returns land on too coarse a footprint
END_MARGIN = 0.0        # m short of a return before a beam counts as passing through


def accumulate(hits, passes, resolution: float, ox: float, oy: float, x: float, y: float, yaw: float,
               angles, ranges, step: float = 0.025) -> None:
    """Add one scan's beams to the ``hits`` and ``passes`` count grids, in place."""
    ranges, angles = np.asarray(ranges, float), np.asarray(angles, float)
    keep = np.isfinite(ranges) & (ranges > MIN_RANGE) & (ranges < MAX_RANGE)
    r, a = ranges[keep], yaw + angles[keep]
    if not r.size:
        return
    height, width = hits.shape
    ex, ey = x + r * np.cos(a), y + r * np.sin(a)
    col, row = ((ex - ox) / resolution).astype(int), ((ey - oy) / resolution).astype(int)
    inside = (row >= 0) & (row < height) & (col >= 0) & (col < width)
    np.add.at(hits, (row[inside], col[inside]), 1)
    along = np.arange(step, r.max(), step)
    d = np.outer(along, np.ones(r.size))
    mask = d < (r - END_MARGIN)
    px = x + (d * np.cos(a))[mask]
    py = y + (d * np.sin(a))[mask]
    col, row = ((px - ox) / resolution).astype(int), ((py - oy) / resolution).astype(int)
    inside = (row >= 0) & (row < height) & (col >= 0) & (col < width)
    # Once per beam per cell, like the hits: the fine step visits a cell several times.
    beam = (np.broadcast_to(np.arange(r.size), d.shape))[mask][inside]
    keys = np.unique(beam.astype(np.int64) * (height * width) + row[inside] * width + col[inside])
    cells = keys % (height * width)
    np.add.at(passes, (cells // width, cells % width), 1)


def _keeps_walls_closed(grid, row: int, col: int) -> bool:
    """Whether a wall cell can go without splitting or opening the wall through it."""
    h, w = grid.shape
    if not (0 < row < h - 1 and 0 < col < w - 1):
        return False
    ring = [grid[row - 1, col], grid[row - 1, col + 1], grid[row, col + 1], grid[row + 1, col + 1],
            grid[row + 1, col], grid[row + 1, col - 1], grid[row, col - 1], grid[row - 1, col - 1]]
    # Wall runs around the eight neighbours. One run means removing this cell joins or
    # splits nothing. Two or fewer neighbours means it is the end of a wall, which stays.
    runs = sum(1 for a, b in zip(ring, ring[1:] + ring[:1]) if a and not b)
    return runs == 1 and sum(map(bool, ring)) >= 3


def rebuild(occupied, hits, passes, min_hits: int = 3, min_share: float = 0.35) -> np.ndarray:
    """Walls from the counts: ``occupied`` with cells beams pass through peeled away
    and cells beams end in added. Peeling never opens a wall, so a sparse stretch
    stays closed; cells no beam reached keep what the map said."""
    hits, passes = np.asarray(hits), np.asarray(passes)
    total = hits + passes
    with np.errstate(invalid="ignore", divide="ignore"):
        share = np.where(total > 0, hits / total, 0.0)
    strong = (hits >= min_hits) & (share >= min_share)
    out = np.asarray(occupied, bool) | strong
    changed = True
    while changed:
        changed = False
        rows, cols = np.nonzero(out & ~strong & (passes > 0))
        for i in np.argsort(share[rows, cols], kind="stable"):
            if _keeps_walls_closed(out, int(rows[i]), int(cols[i])):
                out[rows[i], cols[i]] = False
                changed = True
    return out


def open_walls(hits, passes, min_hits: int = 3, min_share: float = 0.25) -> np.ndarray:
    """Cells beams end in often enough, with nothing added to close the gaps."""
    hits, passes = np.asarray(hits), np.asarray(passes)
    total = hits + passes
    with np.errstate(invalid="ignore", divide="ignore"):
        share = np.where(total > 0, hits / total, 0.0)
    return (hits >= min_hits) & (share >= min_share)


def wall_bias(occupied, resolution: float, ox: float, oy: float, poses, angles, scans, near: float = 0.4) -> dict:
    """Real range minus the range ray cast on ``occupied``, over located scans."""
    from run_analysis.car_measure import cast_scan
    diffs = []
    for (x, y, yaw), ranges in zip(poses, scans):
        lx, ly = x + LASER_X * math.cos(yaw), y + LASER_X * math.sin(yaw)
        diffs.append(np.asarray(ranges) - cast_scan(occupied, resolution, ox, oy, lx, ly, yaw, angles))
    d = np.concatenate(diffs) if diffs else np.zeros(0)
    d = d[np.isfinite(d)]
    close = d[np.abs(d) < near]
    if not close.size:
        return dict(median=math.nan, p10=math.nan, p90=math.nan, close=0.0, shorter=0.0, longer=0.0)
    return dict(median=float(np.median(close)), p10=float(np.percentile(close, 10)),
                p90=float(np.percentile(close, 90)), close=close.size / d.size,
                shorter=float(np.mean(d <= -near)), longer=float(np.mean(d >= near)))


def located_scans(run, loc_dir, every: int = 3):
    """(x, y, yaw), beam angles and ranges of a run's well matched scans while moving."""
    from run_analysis.bagread import messages
    from run_analysis.loc import COL, load_track
    track = load_track(run, loc_dir)
    t = track[:, COL["t"]]
    n = 0
    for _topic, stamp, m in messages(run, ["/scan"]):
        n += 1
        if n % every:
            continue
        i = int(np.searchsorted(t, stamp))
        if not 0 < i < len(t) or abs(t[i] - stamp) > 0.03:
            continue
        if track[i, COL["h10"]] < MIN_MATCH or track[i, COL["wheel_v"]] < 0.3:
            continue
        angles = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment
        yield (track[i, COL["x"]], track[i, COL["y"]], track[i, COL["yaw"]]), angles, np.array(m.ranges, float)


def fragments_per_scan(scans, angles, params, laser_x: float = LASER_X, laser_y: float = 0.0) -> float:
    """Mean car-sized clusters per scan, by gap_follow's own segmenter: the tracker's raw input."""
    from gap_follow import gap_logic, opponent_tracking as ot
    total = 0
    for ranges in scans:
        r = np.where(np.isfinite(ranges), ranges, 30.0)
        clean, valid = gap_logic.sanitize_ranges(r, 10.0, 0.02)
        total += len(ot.segment_objects(clean, valid, angles, laser_x, laser_y, params))
    return total / max(len(scans), 1)


def score_shares(hits, passes, shares, grid, poses, angles, real_scans, params, min_hits: int = 3) -> list:
    """For each beam-layer threshold: fragments per scan cast from real poses, against the real scans'."""
    from run_analysis.car_measure import cast_scan
    real = fragments_per_scan(real_scans, angles, params)
    rows = []
    for share in shares:
        layer = open_walls(hits, passes, min_hits, share)
        cast = [cast_scan(layer, grid.res, grid.ox, grid.oy, x + LASER_X * math.cos(yaw), y + LASER_X * math.sin(yaw),
                          yaw, angles) for x, y, yaw in poses]
        rows.append(dict(share=share, fragments=fragments_per_scan(cast, angles, params), real=real))
    return rows


def write_map(path, occupied, resolution: float, ox: float, oy: float) -> None:
    """A map_server image and yaml at ``path``.png and ``path``_map.yaml, as the simulator reads them."""
    import yaml
    from PIL import Image
    path = Path(path)
    image = np.where(occupied[::-1], 0, 254).astype(np.uint8)
    Image.fromarray(image).save(path.with_name(path.name + ".png"))
    meta = dict(image=path.name + ".png", resolution=resolution, origin=[ox, oy, 0.0], negate=0,
                occupied_thresh=0.65, free_thresh=0.196)
    path.with_name(path.name + "_map.yaml").write_text(yaml.safe_dump(meta))


def main(argv=None) -> int:
    from run_analysis.loc import DEFAULT_OUT, Grid
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="located runs to build from")
    ap.add_argument("--map", required=True, help="the saved map they were located on")
    ap.add_argument("--loc", default=str(DEFAULT_OUT), help="where loc.py wrote (default %(default)s)")
    ap.add_argument("--check", nargs="*", default=[], help="located runs to score both maps on, held out of the build")
    ap.add_argument("--min-hits", type=int, default=3)
    ap.add_argument("--min-share", type=float, default=0.35, help="for the contact walls")
    ap.add_argument("--beam-share", type=float, default=0.255,
                    help="for the beam walls; fitted to the fragments per scan of a held run")
    ap.add_argument("--out", required=True, help="path stem: <out>.png, <out>_map.yaml, <out>_contact.png")
    ap.add_argument("--score-shares", help="comma list of --beam-share values to score against the --check runs' "
                                           "fragments per scan, instead of writing maps")
    ap.add_argument("--config", default=str(Path(__file__).resolve().parents[2] / "src" / "gap_follow" / "config"
                                            / "gap_follow.yaml"), help="gap_follow YAML, for --score-shares")
    args = ap.parse_args(argv)
    grid = Grid.from_map(args.map)
    hits = np.zeros(grid.occ.shape, int)
    passes = np.zeros(grid.occ.shape, int)
    for run in args.runs:
        for (x, y, yaw), angles, ranges in located_scans(run, args.loc):
            lx, ly = x + LASER_X * math.cos(yaw), y + LASER_X * math.sin(yaw)
            accumulate(hits, passes, grid.res, grid.ox, grid.oy, lx, ly, yaw, angles[::2], ranges[::2])
    if args.score_shares:
        import yaml
        from gap_follow import opponent_tracking as ot
        if not args.check:
            ap.error("--score-shares needs --check runs")
        values = yaml.safe_load(Path(args.config).read_text())["gap_follow_node"]["ros__parameters"]
        params = ot.OpponentParams.from_mapping(values)
        poses, scans = [], []
        for run in args.check:
            for pose, angles, ranges in located_scans(run, args.loc, every=25):
                poses.append(pose)
                scans.append(ranges)
        for row in score_shares(hits, passes, [float(v) for v in args.score_shares.split(",")], grid, poses,
                                angles, scans, params, args.min_hits):
            print(f"beam share {row['share']:.3f}: {row['fragments']:.3f} car-sized fragments per scan, real {row['real']:.3f}")
        return 0
    walls = rebuild(grid.occ, hits, passes, args.min_hits, args.min_share)
    beams = open_walls(hits, passes, args.min_hits, args.beam_share)
    write_map(args.out, beams, grid.res, grid.ox, grid.oy)
    write_map(args.out + "_contact", walls, grid.res, grid.ox, grid.oy)
    Path(args.out + "_contact_map.yaml").unlink()
    print(f"wrote {args.out}: {grid.occ.sum()} wall cells before, {walls.sum()} contact, {beams.sum()} beam")
    if args.check:
        poses, scans = [], []
        for run in args.check:
            for pose, angles, ranges in located_scans(run, args.loc, every=25):
                poses.append(pose)
                scans.append(ranges[::4])
        angles = angles[::4]
        for name, occupied in (("saved map", grid.occ), ("contact", walls), ("beams", beams)):
            b = wall_bias(occupied, grid.res, grid.ox, grid.oy, poses, angles, scans)
            print(f"{name:10s} real minus map: median {b['median']:+.3f} m, p10 {b['p10']:+.3f}, p90 {b['p90']:+.3f}; "
                  f"within 0.4 m {b['close']:.3f}, real nearer {b['shorter']:.3f}, real further {b['longer']:.3f}")
    return 0


if __name__ == "__main__":
    root = Path(__file__).resolve().parents[2]
    sys.path[:0] = [str(root / "tools"), str(root / "src" / "gap_follow")]
    raise SystemExit(main())
