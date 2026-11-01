#!/usr/bin/env python3
"""One spot of a course, close up: the map, where each run drove, a real scan, and why.

    M=data/rb2/racerbot_auto/20261030-185036-fixed; R=data/rb2/racerbot_runs/20261030
    python3 tools/run_analysis/spot.py --map $M --at 7.0,-9.8 --radius 1.5 $R-185036 $R-184826 \\
        --scan --intent --out /tmp/bend.png
    python3 tools/run_analysis/spot.py --map $M --at 7.0,-9.8 $R-185036 \\
        --layer tools/f1tenth_sim/maps/course_2026_10_30_scans_map.yaml --trace /tmp/trace.csv --out /tmp/b.png

Draws the saved map, any extra map layers (a simulator track, say), the located path of
each run and of any trace_gap.py CSV, and with --scan the real scan from the first pass
of the first run, put on the map by its located pose. With --intent it also prints what
the driving node decided while each run was inside the circle. Needs loc.py's tracks.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np

LASER_X = 0.26          # m ahead of base_link, docs/hardware-reference.md
COLOURS = ("tab:green", "tab:cyan", "tab:orange", "tab:purple", "tab:brown", "tab:pink")


def passes(x, y, cx: float, cy: float, radius: float) -> list:
    """(first, last + 1) index pairs of each visit inside the circle."""
    inside = np.hypot(np.asarray(x) - cx, np.asarray(y) - cy) < radius
    edges = np.flatnonzero(np.diff(np.r_[0, inside.astype(int), 0]))
    return list(zip(edges[::2], edges[1::2]))


def scan_world(pose, ranges, angles, laser_x: float = LASER_X) -> np.ndarray:
    """(N, 2) map points of a scan's finite returns from a base_link pose (x, y, yaw)."""
    x, y, yaw = pose
    r, a = np.asarray(ranges, float), np.asarray(angles, float)
    ok = np.isfinite(r) & (r > 0.02)
    lx, ly = x + laser_x * math.cos(yaw), y + laser_x * math.sin(yaw)
    return np.column_stack([lx + r[ok] * np.cos(yaw + a[ok]), ly + r[ok] * np.sin(yaw + a[ok])])


def load_layer(yaml_path):
    """(occupied array with row 0 at the lowest y, extent) of any map_server yaml."""
    import yaml
    from PIL import Image
    yaml_path = Path(yaml_path)
    meta = yaml.safe_load(yaml_path.read_text())
    image = np.array(Image.open(yaml_path.parent / meta["image"]).convert("L"))[::-1]
    h, w = image.shape
    ox, oy = meta["origin"][:2]
    res = float(meta["resolution"])
    return image < 128, (ox, ox + w * res, oy, oy + h * res)


def main(argv=None) -> int:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from run_analysis.bagread import messages
    from run_analysis.intent_report import parse
    from run_analysis.loc import COL, DEFAULT_OUT, Grid, load_track
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="*", help="located runs")
    ap.add_argument("--map", required=True, help="the map the runs were located on")
    ap.add_argument("--at", required=True, help="x,y of the spot, map frame, m")
    ap.add_argument("--radius", type=float, default=1.5, help="m around the spot (default %(default)s)")
    ap.add_argument("--layer", action="append", default=[], help="another map_server yaml to draw over it")
    ap.add_argument("--trace", action="append", default=[], help="a trace_gap.py CSV to draw")
    ap.add_argument("--scan", action="store_true", help="draw the first run's scan nearest the spot")
    ap.add_argument("--intent", action="store_true", help="print /drive_intent decisions inside the circle")
    ap.add_argument("--loc", default=str(DEFAULT_OUT))
    ap.add_argument("--out", required=True, help="PNG to write")
    args = ap.parse_args(argv)
    cx, cy = (float(v) for v in args.at.split(","))
    grid = Grid.from_map(args.map)
    extent = (grid.ox, grid.ox + grid.occ.shape[1] * grid.res, grid.oy, grid.oy + grid.occ.shape[0] * grid.res)
    fig, ax = plt.subplots(figsize=(10, 8))
    image = np.ones(grid.occ.shape + (3,))
    image[grid.occ] = (0.72, 0.72, 1.0)
    ax.imshow(image, origin="lower", extent=extent)
    for i, layer in enumerate(args.layer):
        occupied, layer_extent = load_layer(layer)
        shade = np.zeros(occupied.shape + (4,))
        shade[occupied] = (0.1, 0.1, 0.1, 0.8) if i == 0 else (0.8, 0.1, 0.1, 0.6)
        ax.imshow(shade, origin="lower", extent=layer_extent)
    for k, run in enumerate(args.runs):
        track = load_track(run, args.loc)
        held = track[:, COL["lb"]] > 0
        t, x, y = track[:, COL["t"]], track[:, COL["x"]], track[:, COL["y"]]
        ax.plot(x[held], y[held], color=COLOURS[k % len(COLOURS)], lw=0.6, label=Path(run).name)
        visits = passes(x[held], y[held], cx, cy, args.radius)
        print(f"{Path(run).name}: {len(visits)} passes inside {args.radius} m")
        if args.scan and k == 0 and visits:
            a, b = visits[0]
            i = a + int(np.argmin(np.hypot(x[held][a:b] - cx, y[held][a:b] - cy)))
            stamp, pose = t[held][i], (x[held][i], y[held][i], track[held][i, COL["yaw"]])
            for _topic, s, m in messages(run, ["/scan"]):
                if s >= stamp:
                    angles = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment
                    points = scan_world(pose, m.ranges, angles)
                    ax.scatter(points[:, 0], points[:, 1], s=2, c="red", label=f"scan at {stamp - t[0]:.1f} s")
                    break
        if args.intent:
            spans = [(t[held][a], t[held][b - 1]) for a, b in visits]
            for _topic, s, m in messages(run, ["/drive_intent"]):
                if any(lo <= s <= hi for lo, hi in spans):
                    r = parse(m.data)
                    if r is not None:
                        px, py = np.interp(s, t, x), np.interp(s, t, y)
                        print(f"   {s - t[0]:7.2f} ({px:5.2f},{py:6.2f}) {r['state']:16s} cmd {r['speed']:4.2f} "
                              f"steer {r['steering']:+.3f} bound {','.join(r['binding']) or '-'}")
    for k, path in enumerate(args.trace):
        with open(path) as f:
            rows = list(csv.DictReader(f))
        ax.plot([float(r["x"]) for r in rows], [float(r["y"]) for r in rows], "r", lw=1.2, label=Path(path).stem)
        if rows and int(float(rows[-1].get("contact", 0) or 0)):
            ax.plot(float(rows[-1]["x"]), float(rows[-1]["y"]), "rx", ms=12, mew=3)
    ax.add_patch(plt.Circle((cx, cy), args.radius, fill=False, ls="--", color="grey"))
    ax.set_xlim(cx - 2 * args.radius, cx + 2 * args.radius)
    ax.set_ylim(cy - 1.6 * args.radius, cy + 1.6 * args.radius)
    ax.set_aspect("equal")
    ax.legend(fontsize=8)
    fig.savefig(args.out, dpi=110, bbox_inches="tight")
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    root = Path(__file__).resolve().parents[2]
    sys.path[:0] = [str(root / "tools"), str(root / "src" / "gap_follow")]
    raise SystemExit(main())
