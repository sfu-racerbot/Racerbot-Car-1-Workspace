#!/usr/bin/env python3
"""Draw several runs' final SLAM maps side by side, each with SLAM's driven path.

    python3 tools/run_analysis/render_maps.py <out.png> <run_dir> [<run_dir> ...]

Path colour is time since the run's first SLAM pose. Written 2026-10-27 to compare
the 2026-10-26 mapping runs (gyro odometry on vs off); kept here 2026-10-31. Uses
slam_bag.load (first call reads the bag with ROS sourced, then analysis/slam.npz).
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # tools/
from run_analysis import slam_bag  # noqa: E402


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) < 2:
        print(__doc__.split('\n\n')[1], file=sys.stderr)
        return 2
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    out, runs = Path(argv[0]), [Path(r) for r in argv[1:]]
    fig, axes = plt.subplots(1, len(runs), figsize=(6 * len(runs), 6), squeeze=False)
    for ax, run in zip(axes[0], runs):
        d = slam_bag.load(run)
        ax.set_title(run.name)
        ax.set_aspect('equal')
        if d['map'] is None:
            ax.text(0.5, 0.5, 'no /map', ha='center', transform=ax.transAxes)
            continue
        res, ox, oy, g = d['map']
        img = np.where(g < 0, 128, np.where(g > 50, 0, 255)).astype(np.uint8)
        ax.imshow(img, cmap='gray', origin='lower',
                  extent=[ox, ox + g.shape[1] * res, oy, oy + g.shape[0] * res])
        sp = d['slam_pose']
        if len(sp):
            ax.scatter(sp[:, 1], sp[:, 2], c=sp[:, 0] - sp[0, 0], s=0.5, cmap='turbo')
            ax.set_xlim(sp[:, 1].min() - 2.0, sp[:, 1].max() + 2.0)
            ax.set_ylim(sp[:, 2].min() - 2.0, sp[:, 2].max() + 2.0)
    fig.tight_layout()
    fig.savefig(out, dpi=60)
    print(f'wrote {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
