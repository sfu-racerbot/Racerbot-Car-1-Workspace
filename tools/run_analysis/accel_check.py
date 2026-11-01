#!/usr/bin/env python3
"""Does the car reach the acceleration it is commanded? Per run, inside each LB hold.

    python3 tools/run_analysis/accel_check.py <run_dir> [<run_dir> ...]

Commanded acceleration = slope of /drive speed; achieved = slope of /odom wheel
speed (0.2 s moving average). Compared on the ticks where the command is speeding
up (commanded slope > 0.5 m/s^2). Written 2026-10-27 to check car 1 follows
gap_follow's acceleration ramp; kept here 2026-10-31. Uses runlog.load_run,
so the first call on a run reads its bag (ROS sourced) and caches analysis/run.npz.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # tools/
from run_analysis.runlog import load_run  # noqa: E402

DT = 0.02
SPEEDING_UP = 0.5  # m/s^2


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if not argv:
        print(__doc__.split('\n\n')[1], file=sys.stderr)
        return 2
    for run_dir in argv:
        run = load_run(Path(run_dir))
        cmd_a, ach_a = [], []
        for a, b in run.lb_windows():
            grid = np.arange(a + 0.5, b - 0.5, DT)
            if grid.size < 50:
                continue
            cmd_v = np.interp(grid, run.drive[:, 0], run.drive[:, 3])
            ach_v = np.interp(grid, run.odom[:, 0], run.odom[:, 1])
            k = int(0.2 / DT)
            smooth = np.convolve(ach_v, np.ones(k) / k, mode='same')
            cmd_a += list(np.gradient(cmd_v, DT))
            ach_a += list(np.gradient(smooth, DT))
        cmd_a, ach_a = np.array(cmd_a), np.array(ach_a)
        up = cmd_a > SPEEDING_UP
        if not up.any():
            print(f'{Path(run_dir).name}: no LB hold long enough with the command speeding up')
            continue
        p = lambda x, q: np.percentile(x, q)  # noqa: E731
        print(f'{Path(run_dir).name}: {int(up.sum())} ticks speeding up; commanded p50/p90 '
              f'{p(cmd_a[up], 50):.2f}/{p(cmd_a[up], 90):.2f} m/s^2; achieved p50/p90/p99 '
              f'{p(ach_a[up], 50):.2f}/{p(ach_a[up], 90):.2f}/{p(ach_a[up], 99):.2f} m/s^2; '
              f'achieved max {ach_a.max():.2f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
