#!/usr/bin/env python3
"""How much of each commanded speed the car really delivered, measured against a map.

    python3 tools/run_analysis/loc.py RUN --map MAP          # first: locate the run
    python3 tools/run_analysis/speed_gain.py RUN [RUN ...]

For every LB hold (releases shorter than 0.5 s merged, since the car rolls through them)
it compares distances over the hold:

- map:       the path loc.py located on the map, independent of the wheels;
- commanded: the integral of the /drive speed;
- odom:      the integral of /odom's speed;
- wheel:     the integral of the VESC's measured eRPM (/sensors/core).

and prints

- realisation    = map / commanded: true speed per unit of commanded speed;
- effective gain = wheel eRPM integral / map: eRPM per true m/s, wheel slip included;
- odom / map:      how far /odom over-reads;
- suggested gain = command gain / realisation: the motor command gain
  (vesc.yaml /** speed_to_erpm_gain) that would have delivered the command.

The command gain is read from the run's own metadata/params/ackermann_to_vesc_node.yaml,
so it is the one the run was driven with. "pooled" weights holds by distance.
Car 1's 2026-11-02 recalibration (docs/hardware-reference.md) rests on this measurement;
see docs/rb1-lap-gap-2026-10-30.md. Reads the bag (ROS sourced) and loc.py's result;
writes nothing.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

MERGE_GAP = 0.5     # s: an LB release shorter than this leaves the car rolling
MIN_HOLD = 8.0      # s: shorter holds are mostly a standing start
MIN_FIT = 0.85      # fraction of beams within 10 cm for a pose step to count as map distance


def merge_short_releases(t, held, max_gap: float = MERGE_GAP) -> np.ndarray:
    """``held`` with every release shorter than ``max_gap`` seconds filled in."""
    t, held = np.asarray(t, float), np.asarray(held, bool).copy()
    on = np.flatnonzero(held)
    for a, b in zip(on[:-1], on[1:]):
        if b - a > 1 and t[b] - t[a] < max_gap:
            held[a:b] = True
    return held


def forward_integral(t, value, t0: float, t1: float) -> float:
    """Trapezoid integral of max(value, 0) over [t0, t1]; nan with fewer than two samples."""
    t, value = np.asarray(t, float), np.asarray(value, float)
    m = (t >= t0) & (t <= t1)
    if m.sum() < 2:
        return math.nan
    v = np.clip(value[m], 0.0, None)
    return float(np.sum(0.5 * (v[1:] + v[:-1]) * np.diff(t[m])))


def map_distance(x, y, fit, min_fit: float = MIN_FIT) -> tuple:
    """(distance, fraction of steps used): path length over steps whose two ends both fit
    the map, scaled up by the fraction used so a few poor matches do not shorten the hold."""
    x, y, fit = (np.asarray(a, float) for a in (x, y, fit))
    if len(x) < 2:
        return math.nan, 0.0
    step = np.hypot(np.diff(x), np.diff(y))
    good = (fit[1:] >= min_fit) & (fit[:-1] >= min_fit)
    used = float(good.mean())
    if used == 0.0:
        return math.nan, 0.0
    return float(step[good].sum() / used), used


def hold_summary(map_m: float, commanded_m: float, odom_m: float, wheel_erpm_s: float,
                 command_gain: float) -> dict:
    """The ratios for one hold, from its four distances (wheel in eRPM * s)."""
    realisation = map_m / commanded_m
    return {
        'map_m': map_m, 'commanded_m': commanded_m,
        'realisation': realisation,
        'effective_gain': wheel_erpm_s / map_m,
        'odom_over_map': odom_m / map_m,
        'suggested_gain': command_gain / realisation,
    }


def pooled(rows: list, command_gain: float) -> dict:
    """Distance-weighted over holds: the suggested gain from summed distances."""
    m = sum(r['map_m'] for r in rows)
    c = sum(r['commanded_m'] for r in rows)
    return {'map_m': m, 'commanded_m': c, 'realisation': m / c, 'suggested_gain': command_gain * c / m}


def command_gain_of(run) -> float:
    import yaml
    path = Path(run) / 'metadata' / 'params' / 'ackermann_to_vesc_node.yaml'
    params = yaml.safe_load(path.read_text())
    return float(next(iter(params.values()))['ros__parameters']['speed_to_erpm_gain'])


def analyse(run, loc_dir, min_hold: float = MIN_HOLD) -> tuple:
    """(command gain, one dict per hold) for a located run."""
    from run_analysis.bagread import messages
    from run_analysis.laps import holds
    from run_analysis.loc import COL, load_track
    a = load_track(run, loc_dir)
    series = {'/drive': [], '/odom': [], '/sensors/core': []}
    for topic, stamp, msg in messages(run, list(series)):
        if topic == '/drive':
            series[topic].append((stamp, msg.drive.speed))
        elif topic == '/odom':
            series[topic].append((stamp, msg.twist.twist.linear.x))
        else:
            series[topic].append((stamp, msg.state.speed))
    series = {k: np.array(v).reshape(-1, 2) for k, v in series.items()}
    gain = command_gain_of(run)
    t = a[:, COL['t']]
    held = merge_short_releases(t, a[:, COL['lb']] > 0)
    rows = []
    for i, j in holds(held):
        t0, t1 = t[i], t[j - 1]
        if t1 - t0 < min_hold:
            continue
        dist, used = map_distance(a[i:j, COL['x']], a[i:j, COL['y']], a[i:j, COL['h10']])
        parts = [forward_integral(*series[k].T, t0, t1) for k in ('/drive', '/odom', '/sensors/core')]
        if not (dist > 0 and all(p > 0 for p in parts)):
            continue
        row = hold_summary(dist, parts[0], parts[1], parts[2], gain)
        row.update(start=t0 - t[0], end=t1 - t[0], fit=used, mean_speed=dist / (t1 - t0))
        rows.append(row)
    return gain, rows


def main(argv=None) -> int:
    from run_analysis.loc import DEFAULT_OUT
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', help='run directories, located with loc.py')
    ap.add_argument('--loc', default=str(DEFAULT_OUT), help='where loc.py wrote (default %(default)s)')
    ap.add_argument('--min-hold', type=float, default=MIN_HOLD, help='seconds (default %(default)s)')
    args = ap.parse_args(argv)
    every, gains = [], set()
    for run in args.runs:
        gain, rows = analyse(run, args.loc, args.min_hold)
        gains.add(gain)
        print(f'{Path(run).name}: command gain {gain:.1f}')
        for r in rows:
            print(f"   hold {r['start']:6.1f}-{r['end']:6.1f} s  map {r['map_m']:6.1f} m (fit {r['fit']:.2f}) "
                  f"mean {r['mean_speed']:.2f} m/s  realisation {r['realisation']:.3f}  "
                  f"effective gain {r['effective_gain']:5.0f}  odom/map {r['odom_over_map']:.3f}  "
                  f"suggested gain {r['suggested_gain']:5.0f}")
        every += [(gain, r) for r in rows]
    if len(gains) == 1 and every:
        p = pooled([r for _, r in every], gains.pop())
        print(f"pooled over {len(every)} holds, {p['map_m']:.0f} m: realisation {p['realisation']:.3f}, "
              f"suggested gain {p['suggested_gain']:.0f}")
    elif every:
        print('runs used different command gains: no pooled figure')
    return 0


if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    raise SystemExit(main())
