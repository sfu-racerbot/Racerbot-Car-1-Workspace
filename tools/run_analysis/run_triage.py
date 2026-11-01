#!/usr/bin/env python3
"""What a recording really is, before anyone draws a conclusion from it.

    python3 tools/run_analysis/run_triage.py data/racerbot_runs/<time> [more runs]

For each run: which driving code ran, how long LB was held and in how many holds, how far the wheels went and
how fast, and whether the car was moved by hand between holds. A run carried between holds cannot map: nothing the
car senses saw it move. The answer then names where a clean replay can start from
(replay_map_course.py --from-seconds).

Why: on 2026-10-31, most of the 30 Oct recordings that "failed to map" turned out to be other teams' algorithms
or a few seconds of driving (docs/superpowers/specs/2026-10-30-slam-reliability.md). Carrying is judged from the
car's own scans and wheels: placing single scans on a reference map once called a car carried 12 m that wasn't.
"""
import argparse
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

# Driving and mapping code that can appear in processes.txt, ours and team A's.
DRIVERS = re.compile(r'\b(gap_follow_node|pure_pursuit_node|auto_map_race_node|stanley\w*_node|lqr\w*_node|'
                     r'mpc\w*_node|neural_racer_node|ftg_node|disparity_extender_node|racerbot_[ab]\w*|'
                     r'async_slam_toolbox_node|joy_teleop)\b')
STILL_MPS = 0.02  # wheel speed of a parked car
MOVED_M = 0.15  # median change of the scan, parked, that means the car is somewhere else
SHORT_S = 5.0  # less LB than this is not a mapping run


def algorithms(processes_text):
    """The driving and mapping code in a run's metadata/processes.txt, sorted, each once."""
    return sorted(set(DRIVERS.findall(processes_text)))


def hold_stats(odom, windows):
    """[(start, seconds, metres, top m/s)] per LB hold, from odom rows (t, v)."""
    odom = np.asarray(odom, dtype=float).reshape(-1, 2)
    out = []
    for start, end in windows:
        rows = odom[(odom[:, 0] >= start) & (odom[:, 0] <= end)]
        if len(rows) > 1:
            metres = float(np.sum(np.abs(rows[1:, 1]) * np.diff(rows[:, 0])))
            top = float(np.max(rows[:, 1]))
        else:
            metres, top = 0.0, 0.0
        out.append((float(start), float(end - start), metres, top))
    return out


def scan_change(a, b):
    """Median change of the ranges both scans saw, in metres: about zero for the same place."""
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    both = np.isfinite(a) & np.isfinite(b) & (a > 0) & (b > 0)
    return float(np.median(np.abs(a[both] - b[both]))) if both.any() else float('nan')


def moved_between(scan_t, scans, odom, windows, still_mps=STILL_MPS, moved_m=MOVED_M):
    """For each gap between holds: whether the scan changed while the wheels never turned (carried)."""
    odom = np.asarray(odom, dtype=float).reshape(-1, 2)
    flags = []
    for (_, end), (start, _) in zip(windows[:-1], windows[1:]):
        before = int(np.searchsorted(scan_t, end, side='right')) - 1
        after = int(np.searchsorted(scan_t, start))
        if before < 0 or after >= len(scan_t):
            flags.append(False)
            continue
        between = odom[(odom[:, 0] > end + 0.5) & (odom[:, 0] < start)]
        wheels_still = len(between) == 0 or bool(np.all(np.abs(between[:, 1]) < still_mps))
        flags.append(wheels_still and scan_change(scans[before], scans[after]) > moved_m)
    return flags


def verdict(holds, carried):
    """One line: whether this run can be judged as a mapping run, and from where."""
    if not holds or sum(h[1] for h in holds) < SHORT_S:
        return 'not a mapping run: LB held under 5 s'
    if any(carried):
        last = max(i for i, c in enumerate(carried) if c) + 1
        return (f'carried between holds {last} and {last + 1}: nothing before can map. A clean replay starts '
                f'at the hold after it (--from-seconds {holds[last][0] - holds[0][0] + 0.0:.0f} or later)')
    return 'one continuous car: usable'


def triage(run_dir):
    from run_analysis import runlog
    run = runlog.load_run(run_dir)
    processes = Path(run_dir) / 'metadata' / 'processes.txt'
    windows = run.lb_windows()
    odom = run.odom[:, :2] if len(run.odom) else np.zeros((0, 2))
    holds = hold_stats(odom, windows)
    carried = moved_between(run.scan_t, run.scan, odom, windows)
    t0 = run.scan_t[0] if len(run.scan_t) else 0.0
    return {
        'algorithms': (algorithms(processes.read_text(errors='replace')) if processes.exists()
                       else ['unknown: no metadata/processes.txt']),
        'seconds': float(run.scan_t[-1] - t0) if len(run.scan_t) else 0.0,
        'holds': [(start - t0, seconds, metres, top) for start, seconds, metres, top in holds],
        'carried': carried, 'verdict': verdict(holds, carried)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('runs', type=Path, nargs='+')
    args = parser.parse_args(argv)
    for run in args.runs:
        try:
            result = triage(run)
        except (OSError, ValueError) as exc:
            print(f'{run.name}: unreadable: {exc}')
            continue
        metres = sum(h[2] for h in result['holds'])
        print(f"{run.name}: {result['seconds']:.0f} s, {', '.join(result['algorithms']) or 'no driving code'}; "
              f"{len(result['holds'])} LB hold(s), {metres:.0f} m by the wheels")
        for index, (start, seconds, hold_m, top) in enumerate(result['holds']):
            moved = '  then carried' if index < len(result['carried']) and result['carried'][index] else ''
            print(f'    hold {index + 1}: at {start:5.1f} s for {seconds:5.1f} s, {hold_m:5.1f} m, top {top:.2f} m/s{moved}')
        print(f"  -> {result['verdict']}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
