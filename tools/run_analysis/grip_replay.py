"""What gap_follow's adaptive grip would have chosen on a recorded cons.

Replays a recorded run through gap_follow's own adaptive-grip logic --
gap_logic.AdaptiveGrip, LowPass, SteeringLag and adaptive_grip_step, called
in exactly the order gap_follow_node._lateral_limit calls them -- on the
recorded /drive ticks, /imu/data yaw rates and /odom wheel speeds, and
reports the lateral-acceleration limit it would have set while the car was
really on the floor.

The gym's tyres never slide, so offline replay against a recorded run is the
only place this feature can be judged. This tool changes nothing and writes
nothing but its cache (<run>/analysis/grip_replay.npz) and report.

    RB_WS="$(git rev-parse --show-toplevel)" rb env ROS_DOMAIN_ID=74 \
        python3 tools/run_analysis/grip_replay.py <run> [--set name=value ...]
             [--compare-fixed V]

Needs ROS sourced (rosbag2_py) to read the bag; arrays are cached in
<run>/analysis/grip_replay.npz and the summaries run unsourced afterwards.

Gyro truth: "/imu/status" is not in these bags, so the gyro is trusted the
same way gap_follow's staleness fallback behaves without a status: when the
latest sample is younger than imu_timeout_sec at the tick (and its yaw rate
is finite). Odometry is fresh when the latest /odom is younger than
odom_timeout_sec. Ticks are /drive receive times; dt is the difference of
consecutive tick times; a tick with dt <= 0 or > 0.5 s is skipped and the
filters restart, and every skip is counted in the report.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path

import numpy as np
import yaml

_TOOLS = os.path.abspath(os.path.dirname(__file__))
_WS = os.path.dirname(_TOOLS)
for _p in (_TOOLS, os.path.join(_WS, 'src')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from gap_follow import gap_logic  # noqa: E402

GRIP_RATIO_TAU = gap_logic.GRIP_RATIO_TAU_SEC
NODE_CONFIG = Path(_WS) / 'src' / 'gap_follow' / 'config' / 'gap_follow.yaml'
RUNS_DIR = Path.home() / '.ros' / 'racerbot_runs'
CACHE = 'grip_replay.npz'
REPORT = 'grip_replay.txt'

# The node's slew limiter is fed dt = min(command_slew_max_dt, elapsed), but
# this replay skips a tick whose raw interval exceeds 0.5 s and restarts the
# filters (see module docstring).
MAX_DT_SEC = 0.5

# Demand bands, in m/s^2, matching the run report's grip table
# (analysis/report.md: "achieved / commanded curvature, by commanded lateral
# accel"). The last band collects everything above 3.0 so no judged tick is
# dropped from the table.
BAND_NAMES = ('0.3-1.0', '1.0-1.5', '1.5-2.0', '2.0-2.5', '2.5-3.0', '3.0-5.0')
BAND_EDGES = ((0.3, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 2.5), (2.5, 3.0),
              (3.0, math.inf))


# --------------------------------------------------------------- parameters

PARAM_KEYS = (
    'max_lateral_accel', 'adaptive_grip_ceiling', 'adaptive_grip_rise_per_s',
    'adaptive_grip_cut_factor', 'adaptive_grip_hold_ratio',
    'adaptive_grip_slip_ratio', 'adaptive_grip_judge_fraction',
    'adaptive_grip_steering_lag_sec', 'adaptive_grip_max_steering_change',
    'imu_timeout_sec', 'odom_timeout_sec', 'wheelbase',
)


def load_params(overrides=()) -> dict:
    """The adaptive-grip and timeout parameters, from gap_follow.yaml.

    overrides: NAME=VALUE strings (--set), applied on top as floats.
    """
    node = yaml.safe_load(NODE_CONFIG.read_text())['gap_follow_node']
    params = {k: float(node['ros__parameters'][k]) for k in PARAM_KEYS}
    for item in overrides:
        name, _, value = item.partition('=')
        if name not in PARAM_KEYS or not value:
            raise ValueError(f'--set {item}: expected NAME=VALUE with NAME one of '
                             f'{", ".join(PARAM_KEYS)}')
        params[name] = float(value)
    return params


# ------------------------------------------------------------------- reading

MESSAGE_CLASSES = {
    '/drive': 'ackermann_msgs/msg/AckermannDriveStamped',
    '/imu/data': 'sensor_msgs/msg/Imu',
    '/odom': 'nav_msgs/msg/Odometry',
}


def read_replay_bag(bag_dir: Path) -> dict:
    """(receive time, value) rows from the run bag: /drive steering,
    /imu/data yaw rate, /odom wheel speed."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag_dir = Path(bag_dir)
    storage = 'mcap' if any(bag_dir.glob('*.mcap')) else 'sqlite3'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    topics = list(MESSAGE_CLASSES)
    present = {t.name for t in reader.get_all_topics_and_types()}
    missing = [t for t in topics if t not in present]
    if missing:
        raise ValueError(f'{bag_dir}: bag is missing {missing}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    names = dict(zip(topics, ('drive', 'gyro', 'odom')))
    classes = {topic: get_message(text) for topic, text in MESSAGE_CLASSES.items()}
    rows = {n: [] for n in names.values()}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        t = t_ns / 1e9
        msg = deserialize_message(raw, classes[topic])
        if topic == '/drive':
            rows['drive'].append((t, msg.drive.steering_angle))
        elif topic == '/imu/data':
            rows['gyro'].append((t, msg.angular_velocity.z))
        else:
            rows['odom'].append((t, msg.twist.twist.linear.x))
    out = {n: np.array(rows[n], dtype=np.float64).reshape(-1, 2)
           for n in ('drive', 'gyro', 'odom')}
    for n, topic in (('drive', '/drive'), ('gyro', '/imu/data')):
        if len(out[n]) < 2:
            raise ValueError(f'{bag_dir}: {topic}: only {len(out[n])} messages')
    return out


# ------------------------------------------------------------------- replay

def _gyro_gate(gyro_t, gyro_w, t, timeout_sec):
    """(trusted, yaw rate) at tick t: latest sample at/before t, fresh enough,
    finite gap_logic-style verdict. No sample or non-finite -> untrusted
    (the node's 'no imu' / 'imu yaw not finite' reasons)."""
    i = int(np.searchsorted(gyro_t, t, side='right')) - 1
    if i < 0:
        return False, math.nan
    yaw = float(gyro_w[i])
    if not math.isfinite(yaw) or not (t - gyro_t[i]) < timeout_sec:
        return False, math.nan
    return True, yaw


def _speed_at(odom_t, odom_v, t, timeout_sec):
    """Latest /odom twist speed at/before t, or NaN when none, stale
    (>= odom_timeout_sec old) or non-finite -- no judgement either way,
    the limit holds (the node's _odom_fresh rule)."""
    i = int(np.searchsorted(odom_t, t, side='right')) - 1
    if i < 0:
        return math.nan
    speed = float(odom_v[i])
    if not math.isfinite(speed) or not (t - odom_t[i]) < timeout_sec:
        return math.nan
    return speed


def replay(ticks, gyro, odom, params) -> dict:
    """Ticks: rows (t, steering) from /drive; gyro: rows (t, yaw_rate);
    odom: rows (t, speed); params: the adaptive-grip and timeout parameters
    (dict). Runs the node's per-tick sequence with gap_logic's own classes.
    Returns arrays per tick: 't', 'limit', 'ratio' (the filtered ratio after
    the tick, NaN when not judged), 'judged' (bool), 'trusted' (bool),
    'speed', 'demand' (speed**2 * |tan(judged steering)| / wheelbase, NaN
    when not judged); plus 'restarts' (count of dt gaps)."""
    ticks = np.asarray(ticks, dtype=np.float64).reshape(-1, 2)
    gyro = np.asarray(gyro, dtype=np.float64).reshape(-1, 2)
    odom = np.asarray(odom, dtype=np.float64).reshape(-1, 2)
    base = float(params['max_lateral_accel'])
    ceiling = max(float(params['adaptive_grip_ceiling']), base)
    lag_sec = float(params['adaptive_grip_steering_lag_sec'])
    max_change = float(params['adaptive_grip_max_steering_change'])
    imu_timeout = float(params['imu_timeout_sec'])
    odom_timeout = float(params['odom_timeout_sec'])
    wheelbase = float(params['wheelbase'])

    def fresh_grip():
        # Adaptive grip built as gap_follow_node builds it (line ~690).
        return gap_logic.AdaptiveGrip(
            base=base,
            ceiling=ceiling,
            rise_per_s=float(params['adaptive_grip_rise_per_s']),
            cut_factor=float(params['adaptive_grip_cut_factor']),
            hold_ratio=float(params['adaptive_grip_hold_ratio']),
            slip_ratio=float(params['adaptive_grip_slip_ratio']),
            judge_fraction=float(params['adaptive_grip_judge_fraction']))

    grip = fresh_grip()
    ratio_filter = gap_logic.LowPass(GRIP_RATIO_TAU)
    lag = gap_logic.SteeringLag(lag_sec)
    gyro_t, gyro_w = gyro[:, 0], gyro[:, 1]
    odom_t, odom_v = odom[:, 0], odom[:, 1]

    out = {k: [] for k in ('t', 'limit', 'ratio', 'judged', 'trusted',
                           'speed', 'demand')}
    restarts = 0
    prev_t = None
    for t, steering in ticks:
        t = float(t)
        if prev_t is None or not 0.0 < t - prev_t <= MAX_DT_SEC:
            if prev_t is not None:
                restarts += 1
            # Skipped tick: nothing recorded, the filters restart from scratch.
            grip = fresh_grip()
            ratio_filter = gap_logic.LowPass(GRIP_RATIO_TAU)
            lag = gap_logic.SteeringLag(lag_sec)
        else:
            dt = t - prev_t
            steering = float(steering)
            # gap_follow_node._lateral_limit, in its own order:
            grip.follow_base(base, ceiling)
            judged_steering = lag.step(dt, steering, max_change)
            trusted, yaw = _gyro_gate(gyro_t, gyro_w, t, imu_timeout)
            if trusted:
                speed = _speed_at(odom_t, odom_v, t, odom_timeout)
                limit_before = grip.limit
                limit = gap_logic.adaptive_grip_step(
                    grip, ratio_filter, yaw, speed, judged_steering,
                    wheelbase, dt)
                if math.isfinite(speed) and math.isfinite(judged_steering):
                    demand = speed * speed * abs(math.tan(judged_steering)) \
                        / wheelbase
                else:
                    demand = math.nan
                # gap_logic.LowPass keeps its state as None until a finite
                # sample starts the chain; NaN, to the judge, means the same.
                ratio = ratio_filter.value
                ratio = math.nan if ratio is None else float(ratio)
                # A tick is judged when grip.update actually weighed the
                # ratio: finite ratio, finite demand, and a demand the node
                # would judge (adaptive_grip_judge_fraction of the limit).
                judged = (math.isfinite(ratio) and math.isfinite(demand)
                          and demand >= grip.judge_fraction * limit_before)
            else:
                # The node's untrusted-gyro rule: the limit restarts at the
                # base and the ratio smoothing starts again.
                grip.limit = grip.base
                ratio_filter.reset()
                limit, speed, demand, ratio, judged = grip.limit, math.nan, \
                    math.nan, math.nan, False
            out['t'].append(t)
            out['limit'].append(float(limit))
            out['ratio'].append(ratio if judged else math.nan)
            out['judged'].append(bool(judged))
            out['trusted'].append(bool(trusted))
            out['speed'].append(speed)
            out['demand'].append(demand if judged else math.nan)
        prev_t = t
    result = {
        't': np.array(out['t'], dtype=np.float64),
        'limit': np.array(out['limit'], dtype=np.float64),
        'ratio': np.array(out['ratio'], dtype=np.float64),
        'judged': np.array(out['judged'], dtype=bool),
        'trusted': np.array(out['trusted'], dtype=bool),
        'speed': np.array(out['speed'], dtype=np.float64),
        'demand': np.array(out['demand'], dtype=np.float64),
        'restarts': restarts,
    }
    for key in ('t', 'limit', 'ratio', 'speed', 'demand'):
        result[key] = result[key].reshape(-1)
    return result


# ---------------------------------------------------------------- summarize

def summarize(result, base) -> dict:
    """{'ticks', 'judged_fraction', 'trusted_fraction', 'limit_max',
    'limit_mean_when_cornering' (demand >= 0.5 * base), 'time_above_base_s',
    'cuts' (ticks where the limit dropped), 'ratio_by_demand_band':
    {band: (n, median ratio)} for bands 0.3-1.0, 1.0-1.5, 1.5-2.0, 2.0-2.5,
    2.5-3.0, 3.0-5.0 m/s^2}."""
    limit = result['limit']
    t = result['t']
    judged = result['judged']
    trusted = result['trusted']
    demand = result['demand']
    n = len(t)
    cornering = judged & np.isfinite(demand) & (demand >= 0.5 * base)
    cuts = 0
    time_above = math.nan
    limit_mean = math.nan
    if n:
        cuts = int(np.sum(np.diff(limit) < 0.0))
        if n >= 2:
            time_above = float(np.sum(np.diff(t)[limit[:-1] > base]))
        if np.any(cornering):
            limit_mean = float(np.mean(limit[cornering]))

    ratio = result['ratio']
    bands = {}
    for name, (lo, hi) in zip(BAND_NAMES, BAND_EDGES):
        in_band = judged & np.isfinite(demand) & (demand >= lo) & (demand < hi)
        count = int(np.sum(in_band))
        median = float(np.median(ratio[in_band])) if count else math.nan
        bands[name] = (count, median)
    return {
        'ticks': n,
        'judged_fraction': float(np.mean(judged)) if n else math.nan,
        'trusted_fraction': float(np.mean(trusted)) if n else math.nan,
        'limit_max': float(np.max(limit)) if n else math.nan,
        'limit_mean_when_cornering': limit_mean,
        'time_above_base_s': time_above,
        'cuts': cuts,
        'ratio_by_demand_band': bands,
    }


# ------------------------------------------------------------------- report

def _share_above(result, base, threshold) -> float:
    """Share of cornering ticks whose limit exceeded threshold."""
    cornering = result['judged'] & np.isfinite(result['demand']) \
        & (result['demand'] >= 0.5 * base)
    if not np.any(cornering):
        return math.nan
    return float(np.mean(result['limit'][cornering] > threshold))


def report_lines(run_name, result, summary, base, params, compare_fixed) -> list:
    p = params
    bands = summary['ratio_by_demand_band']
    lines = [
        f'Adaptive grip replay: {run_name}',
        '  (gap_follow\'s own adaptive-grip logic on the recorded /drive ticks;',
        '   gyro trusted on staleness only -- /imu/status is not in these bags)',
        f"  base {p['max_lateral_accel']:.2f} ceiling {p['adaptive_grip_ceiling']:.2f} "
        f"rise {p['adaptive_grip_rise_per_s']:.2f} cut {p['adaptive_grip_cut_factor']:.2f} "
        f"hold {p['adaptive_grip_hold_ratio']:.2f} slip {p['adaptive_grip_slip_ratio']:.2f} "
        f"judge {p['adaptive_grip_judge_fraction']:.2f} lag {p['adaptive_grip_steering_lag_sec']:.2f}s",
        f"  ticks {summary['ticks']} (restarts {result['restarts']}: dt <= 0 or > "
        f"{MAX_DT_SEC} s, skipped and filters restarted)",
        f"  trusted {summary['trusted_fraction'] * 100:.1f}% "
        f"judged {summary['judged_fraction'] * 100:.1f}%",
        f"  lateral limit: max {summary['limit_max']:.2f} m/s^2, mean when cornering "
        f"(demand >= {0.5 * base:.2f} m/s^2) {summary['limit_mean_when_cornering']:.2f}"
        if math.isfinite(summary['limit_mean_when_cornering']) else
        f"  lateral limit: max {summary['limit_max']:.2f} m/s^2, never cornering",
        f"  above the base {p['max_lateral_accel']:.2f} for "
        f"{summary['time_above_base_s']:.2f} s (ticks with limit > base, "
        f"scaled by their tick intervals)",
        f"  cuts {summary['cuts']} (ticks where the limit dropped)",
    ]
    lines.append('  grip ratio by demand band (judged ticks; median of the filtered opinion:')
    lines.append('  last band collects everything above 3.0 m/s^2):')
    for name, edges in zip(BAND_NAMES, BAND_EDGES):
        n, median = bands[name]
        if n:
            lines.append(f'    {name} m/s^2: n={n:5d} ratio {median:.3f}')
        else:
            lines.append(f'    {name} m/s^2:   none judged')
    for threshold in compare_fixed:
        share = _share_above(result, base, threshold)
        if math.isfinite(share):
            lines.append(f'  above {threshold:.1f} while cornering: {share * 100:.1f}% '
                         'of cornering ticks')
        else:
            lines.append(f'  above {threshold:.1f} while cornering: no cornering ticks')
    return lines


def resolve_run(name) -> Path:
    run_dir = Path(name).expanduser()
    if run_dir.exists():
        return run_dir
    guess = RUNS_DIR / name
    if guess.exists():
        return guess
    raise FileNotFoundError(f'run {name}: no such directory (tried {run_dir}, {guess})')


def load_replay(run_dir, refresh: bool = False) -> dict:
    """The run's arrays (drive, gyro, odom), from the bag if no cache yet.

    The cache lives in <run>/analysis/grip_replay.npz -- mirror this to a
    copy of the run before running on recorded data you must not touch.
    """
    run_dir = Path(run_dir).expanduser()
    analysis = run_dir / 'analysis'
    analysis.mkdir(exist_ok=True)
    cache = analysis / CACHE
    if refresh or not cache.exists():
        data = read_replay_bag(run_dir / 'bag')
        np.savez_compressed(cache, **data)
    z = np.load(cache)
    return {k: z[k] for k in z.files}


# --------------------------------------------------------------------- main

def main(argv=None) -> int:
    """grip_replay.py RUN [RUN ...] [--set name=value ...] [--compare-fixed V]
    Prints one summary block per run (and writes <run>/analysis/grip_replay.txt).
    --compare-fixed V also reports, per run, the share of cornering ticks
    where the adaptive limit exceeded V."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('runs', nargs='+', metavar='RUN',
                    help='run directory or name under ~/.ros/racerbot_runs/')
    ap.add_argument('--set', dest='overrides', action='append', default=[],
                    metavar='NAME=VALUE', help='override a parameter from gap_follow.yaml')
    ap.add_argument('--compare-fixed', dest='compare_fixed', type=float,
                    action='append', default=[], metavar='V',
                    help='report the share of cornering ticks with limit > V')
    ap.add_argument('--refresh', action='store_true', help='re-read the bag')
    args = ap.parse_args(argv)

    params = load_params(args.overrides)
    base = params['max_lateral_accel']
    for run in args.runs:
        run_dir = resolve_run(run)
        data = load_replay(run_dir, refresh=args.refresh)
        result = replay(data['drive'], data['gyro'], data['odom'], params)
        summary = summarize(result, base)
        text = '\n'.join(report_lines(
            run_dir.name, result, summary, base, params, args.compare_fixed)) + '\n'
        (Path(run_dir) / 'analysis' / REPORT).write_text(text)
        print(text, end='')
    return 0


if __name__ == '__main__':
    sys.exit(main())
