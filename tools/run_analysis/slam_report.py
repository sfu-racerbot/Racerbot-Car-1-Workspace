"""Post-run SLAM health metrics; numerical functions require only NumPy."""
from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src' / 'race_diagnostics'))
from race_diagnostics.run_events import parse_lap_closed, parse_lap_quality, parse_lap_unjudged
from run_analysis import slam_bag, slam_metrics, runlog


def heading_drift(a, b, window_s=1.0):
    """Compare unwrapped headings at common times, relative to their first difference."""
    a = np.asarray(a, dtype=float).reshape(-1, 2)
    b = np.asarray(b, dtype=float).reshape(-1, 2)
    missing = dict(final_deg=math.nan, max_abs_deg=math.nan, rms_rate_deg_s=math.nan)
    if len(a) < 2 or len(b) < 2 or window_s <= 0:
        return missing
    a = a[np.argsort(a[:, 0])]
    b = b[np.argsort(b[:, 0])]
    mask = (a[:, 0] >= b[0, 0]) & (a[:, 0] <= b[-1, 0])
    t = a[mask, 0]
    if len(t) < 2:
        return missing
    drift = np.unwrap(a[:, 1])[mask] - np.interp(t, b[:, 0], np.unwrap(b[:, 1]))
    drift -= drift[0]
    count = int((t[-1] - t[0]) / window_s)
    ends = t[0] + np.arange(count + 1) * window_s
    changes = np.diff(np.interp(ends, t, drift)) / window_s
    return dict(final_deg=float(np.degrees(drift[-1])),
                max_abs_deg=float(np.degrees(np.max(np.abs(drift)))),
                rms_rate_deg_s=float(np.degrees(np.sqrt(np.mean(changes ** 2))))
                if len(changes) else math.nan)


def slam_pose(slam):
    """Return map-frame base poses, composing TF when /pose is missing."""
    if not slam:
        return np.empty((0, 4))
    pose = np.asarray(slam.get('pose', []), dtype=float).reshape(-1, 4)
    if len(pose):
        return pose
    mo = np.asarray(slam.get('map_odom', []), dtype=float).reshape(-1, 4)
    base = np.asarray(slam.get('odom_base', []), dtype=float).reshape(-1, 4)
    if not len(mo) or not len(base):
        return np.empty((0, 4))
    mo = mo[np.argsort(mo[:, 0])]
    base = base[(base[:, 0] >= mo[0, 0]) & (base[:, 0] <= mo[-1, 0])]
    t = base[:, 0]
    x = np.interp(t, mo[:, 0], mo[:, 1])
    y = np.interp(t, mo[:, 0], mo[:, 2])
    yaw = np.interp(t, mo[:, 0], np.unwrap(mo[:, 3]))
    c, s = np.cos(yaw), np.sin(yaw)
    return np.column_stack((t, x+c*base[:, 1]-s*base[:, 2],
                            y+s*base[:, 1]+c*base[:, 2], yaw+base[:, 3]))


def slam_yaw(slam):
    return slam_pose(slam)[:, [0, 3]]


def map_consistency(scan_t, ranges, angle_min, angle_increment, range_max,
                    poses, grid_map, laser_x=0.26, cells=1):
    """Fraction of finite LiDAR endpoints close to an occupied map cell."""
    poses = np.asarray(poses, dtype=float).reshape(-1, 4)
    if grid_map is None or len(poses) < 2:
        return math.nan
    resolution, ox, oy, grid = grid_map
    ranges = np.asarray(ranges, dtype=float)
    hits = total = 0
    for t, scan in zip(scan_t, ranges):
        if t < poses[0, 0] or t > poses[-1, 0]:
            continue
        valid = np.isfinite(scan) & (scan < range_max)
        angle = angle_min + np.arange(len(scan))[valid] * angle_increment
        r = scan[valid]
        x = np.interp(t, poses[:, 0], poses[:, 1])
        y = np.interp(t, poses[:, 0], poses[:, 2])
        yaw = np.interp(t, poses[:, 0], np.unwrap(poses[:, 3]))
        ix = np.floor((x + laser_x*np.cos(yaw) + r*np.cos(yaw+angle) - ox) / resolution).astype(int)
        iy = np.floor((y + laser_x*np.sin(yaw) + r*np.sin(yaw+angle) - oy) / resolution).astype(int)
        matched = np.zeros(len(ix), dtype=bool)
        for dy in range(-cells, cells + 1):
            for dx in range(-cells, cells + 1):
                xx, yy = ix + dx, iy + dy
                inside = (xx >= 0) & (xx < grid.shape[1]) & (yy >= 0) & (yy < grid.shape[0])
                matched[inside] |= grid[yy[inside], xx[inside]] > 50
        hits += int(matched.sum())
        total += len(ix)
    return hits / total if total else math.nan


def lap_windows(laps, run_start, run_end):
    start = run_start
    windows = []
    for lap in sorted(laps, key=lambda item: item['t']):
        end = lap['t']
        if start <= end <= run_end:
            windows.append((lap['lap'], start, end))
            start = end
    return windows


def scan_drop_timeline(drop_times, proc_records, node='/slam_toolbox', bin_s=10.0):
    drops = list(drop_times)
    times = drops + [r['t'] for r in proc_records]
    if not times:
        return []
    start = min(times)
    result = []
    for index in range(int((max(times)-start) // bin_s) + 1):
        t0 = start + index*bin_s
        cpu = [r['cpu_pct'] for r in proc_records if r['node'] == node
               and r['cpu_pct'] is not None and t0 <= r['t'] < t0 + bin_s]
        result.append(dict(t0=t0, drops=sum(t0 <= t < t0+bin_s for t in drops),
                           cpu_mean=float(np.mean(cpu)) if cpu else math.nan,
                           cpu_max=max(cpu) if cpu else math.nan))
    return result


def path_length(rows):
    rows = np.asarray(rows, dtype=float)
    return float(np.hypot(np.diff(rows[:, 1]), np.diff(rows[:, 2])).sum()) if len(rows) else 0.0


def _read_rosout(run_dir, refresh=False):
    """Cache receive times and text separately from slam_bag's cache."""
    cache = Path(run_dir) / 'analysis' / 'rosout.jsonl'
    if cache.exists() and not refresh:
        return [json.loads(line) for line in cache.read_text().splitlines() if line.strip()]
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag = Path(run_dir) / 'bag'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap' if list(bag.glob('*.mcap')) else 'sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    records = []
    if '/rosout' in types:
        reader.set_filter(rosbag2_py.StorageFilter(topics=['/rosout']))
        message_type = get_message(types['/rosout'])
        while reader.has_next():
            _, raw, stamp = reader.read_next()
            msg = deserialize_message(raw, message_type)
            records.append({'t': stamp * 1e-9, 'name': msg.name, 'msg': msg.msg})
    cache.parent.mkdir(parents=True, exist_ok=True)
    cache.write_text(''.join(json.dumps(record) + '\n' for record in records))
    return records


def _read_odom_imu(run_dir):
    """Recover gyro odometry even when slam_bag cannot return its arrays."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag = Path(run_dir) / 'bag'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap' if list(bag.glob('*.mcap')) else 'sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    rows = []
    if '/odom/imu' in types:
        reader.set_filter(rosbag2_py.StorageFilter(topics=['/odom/imu']))
        message_type = get_message(types['/odom/imu'])
        while reader.has_next():
            _, raw, stamp = reader.read_next()
            pose = deserialize_message(raw, message_type).pose.pose
            rows.append((stamp * 1e-9, pose.position.x, pose.position.y, slam_bag._yaw(pose.orientation)))
    return np.asarray(rows, dtype=float).reshape(-1, 4)


def load_inputs(run_dir, log_path=None, load_slam=None, load_scans=None, load_rosout=None):
    """Load recorded inputs, preserving partial runs when topics are absent."""
    import yaml

    path = Path(run_dir).expanduser()
    run = {'path': path}
    try:
        run['slam'] = (load_slam or slam_bag.load)(path)
    except ValueError as exc:
        if 'SLAM did not map this run' not in str(exc):
            raise
        run.update(slam=None, no_slam_reason=str(exc))
        if (path / 'bag').is_dir():
            run['odom_imu'] = _read_odom_imu(path)
    try:
        scans = (load_scans or runlog.load_run)(path)
    except ValueError as exc:
        if 'bag needs /scan and /drive, has ' not in str(exc) and not str(exc).endswith(': no /scan messages'):
            raise
        scans = None
        run['no_scans_reason'] = str(exc)
    if scans is not None:
        for key in ('scan_t', 'scan', 'angle_min', 'angle_increment', 'range_min', 'range_max'):
            run[key] = scans[key] if isinstance(scans, dict) else getattr(scans, key)
        odom = scans.get('odom', []) if isinstance(scans, dict) else scans.odom
        odom = np.asarray(odom)
        if odom.size and odom.shape[1] == 7:
            run['wheel_speed'] = odom[:, [0, 1]]
            run['odom'] = np.column_stack((odom[:, 0], odom[:, 3:5], 2*np.arctan2(odom[:, 5], odom[:, 6])))
    files = list((path / 'metadata' / 'params').glob('*slam_toolbox*.yaml'))
    run['params'] = {}
    if files:
        latest = max(files, key=lambda f: (f.stat().st_mtime_ns, f.name))
        data = yaml.safe_load(latest.read_text()) or {}
        for name, node in data.items():
            if name.rsplit('/', 1)[-1] == 'slam_toolbox':
                run['params'] = node.get('ros__parameters', {})
                break
    proc = path / 'system' / 'proc.jsonl'
    run['proc_records'] = [json.loads(line) for line in proc.read_text().splitlines() if line.strip()] if proc.exists() else []
    log = Path(log_path).expanduser() if log_path is not None else path / 'launch.log'
    run['log_text'] = log.read_text(errors='replace') if log.exists() else None
    rosout = (load_rosout or _read_rosout)(path)
    laps, quality, drops, unjudged, profile_laps = {}, {}, [], set(), set()
    # _write_profile logs this only after writing the CSV: auto_map_race_node.py:1818-1822.
    profile_line = re.compile(r'Generated \d+-point racing profile at .+ \([\d.]+-[\d.]+m/s\)\.')
    latest_lap = None
    for record in rosout:
        msg = record.get('msg', '')
        lap = parse_lap_closed(msg)
        if lap:
            lap['t'] = record['t']
            laps[lap['lap']] = lap
            latest_lap = lap['lap']
        note = parse_lap_unjudged(msg)
        if note:
            unjudged.add(note['lap'])
        if latest_lap is not None and profile_line.search(msg):
            profile_laps.add(latest_lap)
        q = parse_lap_quality(msg)
        if q:
            quality[q['lap']] = q
        if record.get('name') == 'slam_toolbox' and 'Message Filter dropping message' in msg:
            drops.append(record['t'])
    # Launch output supplements rosout; its lap times are only needed if rosout lacks one.
    latest_lap = None
    for line in (run['log_text'] or '').splitlines():
        lap = parse_lap_closed(line)
        if lap:
            latest_lap = lap['lap']
        if lap and lap['lap'] not in laps:
            stamp = re.search(r'\[(\d{10}\.\d+)\]', line)
            if stamp:
                lap['t'] = float(stamp.group(1))
            laps[lap['lap']] = lap
        note = parse_lap_unjudged(line)
        if note:
            unjudged.add(note['lap'])
        if latest_lap is not None and profile_line.search(line):
            profile_laps.add(latest_lap)
        q = parse_lap_quality(line)
        if q:
            quality[q['lap']] = q
    run.update(laps=list(laps.values()), quality=list(quality.values()), drops=drops,
               unjudged_laps=sorted(unjudged), profile_laps=sorted(profile_laps))
    return run


def _rows4(value):
    return np.asarray(value if value is not None else [], dtype=float).reshape(-1, 4)


def _span_rows(rows, start, end):
    """Clip a path to a common interval, interpolating its two endpoints."""
    if len(rows) < 2 or end <= start:
        return np.empty((0, 4))
    times = np.r_[start, rows[(rows[:, 0] > start) & (rows[:, 0] < end), 0], end]
    return np.column_stack([times] + [np.interp(times, rows[:, 0], rows[:, i]) for i in range(1, 4)])


def build_report(run):
    """Build seven report sections from loaded, optional inputs."""
    slam = run.get('slam') or {}
    pose = slam_pose(slam)
    no_slam = not len(pose) and not len(_rows4(slam.get('map_odom')))
    lines = []
    if no_slam:
        lines.append('No SLAM in this run: ' + run.get('no_slam_reason', 'no /pose or map-to-odometry transform'))
    summary = {}
    odom = _rows4(slam.get('odom', run.get('odom')))
    oi = _rows4(slam.get('odom_imu', run.get('odom_imu')))
    frame = run.get('params', {}).get('odom_frame')
    tf_frame = slam.get('odom_frame')
    speeds = np.asarray(run.get('wheel_speed', []), dtype=float).reshape(-1, 2)
    if len(speeds):
        moving = np.abs(speeds[:, 1]); moving = moving[moving > .3]
    elif len(odom) > 1:
        dt = np.diff(odom[:, 0]); valid = dt > 0
        moving = np.hypot(np.diff(odom[:, 1]), np.diff(odom[:, 2]))[valid] / dt[valid]
        moving = moving[moving > .3]
    else:
        moving = []
    setup = dict(parameter_frame=frame, tf_frame=tf_frame, frame_mismatch=bool(frame and tf_frame and frame != tf_frame),
                 distance_m=path_length(odom) if len(odom) else None,
                 mean_speed_m_s=float(np.mean(moving)) if len(moving) else None,
                 max_speed_m_s=float(np.max(moving)) if len(moving) else None)
    summary['Setup'] = setup
    lines += ['Setup:', f"  odom frame: parameter {frame or 'missing'}, TF {tf_frame or 'missing'}" + (' (MISMATCH)' if setup['frame_mismatch'] else ''),
              f"  /odom distance: {_fmt(setup['distance_m'])} m; moving speed mean/max: {_fmt(setup['mean_speed_m_s'])} / {_fmt(setup['max_speed_m_s'])} m/s"]
    heading = {}
    lines.append('Heading drift:')
    for label, a, b in [('/odom vs SLAM', odom, pose), ('/odom/imu vs SLAM', oi, pose), ('/odom vs /odom/imu', odom, oi)]:
        if len(a) >= 2 and len(b) >= 2 and not (no_slam and 'SLAM' in label):
            result = heading_drift(a[:, [0, 3]], b[:, [0, 3]])
            heading[label] = result
            lines.append(f"  {label}: final {_fmt(result['final_deg'])} deg, max {_fmt(result['max_abs_deg'])} deg, RMS rate {_fmt(result['rms_rate_deg_s'])} deg/s")
        else:
            lines.append(f'  {label}: missing heading samples')
    summary['Heading drift'] = heading
    lines.append('Corrections and loop closures:')
    mo, base = _rows4(slam.get('map_odom')), _rows4(slam.get('odom_base'))
    if no_slam:
        corr = {}; lines.append('  no SLAM')
    elif not len(mo) or not len(base):
        corr = {}; lines.append('  missing map-to-odometry or base transform')
    else:
        log = run.get('log_text')
        corr = slam_metrics.summarize(slam_metrics.correction_events(mo, base), slam_metrics.loop_closure_times(log or ''))
        if log is None:
            corr['loop_closures'] = None
            corr['jumps_at_loop_closures'] = None
        lines.append(f"  updates {corr['updates']}, loop closures {corr['loop_closures'] if corr['loop_closures'] is not None else 'unknown (no launch log)'}, max move {corr['max_move_m']:.2f} m, max turn {corr['max_rotate_deg']:.2f} deg, jumps >0.3 m {corr['jumps_over_0_3m']}, jumps >1.5 m {corr['jumps_over_false_jump_m']}, jumps near closures {corr['jumps_at_loop_closures'] if corr['jumps_at_loop_closures'] is not None else 'unknown'}")
    summary['Corrections and loop closures'] = corr
    lines.append('Laps:')
    quality = {q['lap']: q for q in run.get('quality', [])}
    laps = run.get('laps', [])
    summary['Laps'] = []
    for lap in sorted(laps, key=lambda item: item['lap']):
        q = quality.get(lap['lap'])
        if q and not q.get('passed'):
            verdict = 'failed (accepted at the ceiling)' if q.get('accepted_at_ceiling') else 'failed'
            status = f"failed {q['metric']}={q['value']:.1f} deg > {q['limit']:.1f} deg"
            if q.get('accepted_at_ceiling'):
                status += ' (accepted at the ceiling)'
        elif q and q.get('passed'):
            verdict = status = 'passed'
        elif lap['lap'] in run.get('unjudged_laps', []) or lap['lap'] < lap.get('of', lap['lap']):
            verdict, status = 'not judged', 'not judged (only the last lap is)'
        elif lap['lap'] in run.get('profile_laps', []):
            verdict = status = 'passed'
        else:
            verdict, status = 'unknown', 'unknown (no quality line or profile evidence)'
        entry = dict(lap, quality=q, verdict=verdict)
        summary['Laps'].append(entry)
        lines.append(f"  lap {lap['lap']}/{lap.get('of', '?')}: {lap.get('distance_m', math.nan):.1f} m, closest {lap.get('closest_m', math.nan):.2f} m, corrections {lap.get('corrections', '?')}, quality {status}")
    if not laps:
        lines.append('  no closed lap messages')
    lines.append('Map consistency:')
    consistency = {'overall': None, 'laps': {}}
    scan_t = np.asarray(run.get('scan_t', []), dtype=float)
    scans = np.asarray(run.get('scan', []))
    grid = slam.get('map')
    if run.get('no_scans_reason'):
        lines.append('  no scans: ' + run['no_scans_reason'])
    elif no_slam:
        lines.append('  no SLAM')
    elif (grid is None or len(pose) < 2 or not len(scan_t) or not len(scans)
          or any(run.get(key) is None for key in ('angle_min', 'angle_increment', 'range_max'))):
        lines.append('  missing final map, SLAM pose, or scans')
    else:
        def score(mask):
            return map_consistency(scan_t[mask], scans[mask], run['angle_min'], run['angle_increment'], run['range_max'], pose, grid)
        consistency['overall'] = score(np.ones(len(scan_t), dtype=bool))
        lines.append(f"  overall: {_fmt_percent(consistency['overall'])}")
        timed = [lap for lap in laps if 't' in lap]
        for lap, start, end in lap_windows(timed, float(scan_t[0]), float(scan_t[-1])):
            value = score((scan_t >= start) & (scan_t < end))
            consistency['laps'][str(lap)] = value
            lines.append(f'  lap {lap}: {_fmt_percent(value)}')
    summary['Map consistency'] = consistency
    drops, proc = run.get('drops', []), run.get('proc_records', [])
    timeline = scan_drop_timeline(drops, proc)
    summary['Scan drops'] = dict(count=len(drops), timeline=timeline, system_log=bool(proc))
    lines += ['Scan drops:', f'  /rosout drops: {len(drops)}; ' + ('CPU bins (10 s):' if proc else 'no system log')]
    for row in timeline:
        lines.append(f"  {row['t0']:.1f}: {row['drops']} drops, CPU mean/max {_fmt(row['cpu_mean'])} / {_fmt(row['cpu_max'])} %")
    lines.append('Wheel vs scan-matched distance:')
    distance = {}
    if no_slam:
        lines.append('  no SLAM')
    elif len(odom) < 2 or len(pose) < 2:
        lines.append('  missing /odom or SLAM pose')
    else:
        start, end = max(odom[0, 0], pose[0, 0]), min(odom[-1, 0], pose[-1, 0])
        wheel, matched = _span_rows(odom, start, end), _span_rows(pose, start, end)
        if len(wheel):
            w, s = path_length(wheel), path_length(matched)
            distance = dict(wheel_m=w, slam_m=s, ratio=w/s if s else None, start=float(start), end=float(end))
            lines.append(f"  wheel {w:.2f} m, SLAM {s:.2f} m, wheel/SLAM ratio {_fmt(distance['ratio'])}")
        else:
            lines.append('  no overlapping time span')
    summary['Wheel vs scan-matched distance'] = distance
    return lines, summary


def _fmt(value):
    return f'{value:.2f}' if value is not None and math.isfinite(value) else 'unknown'


def _fmt_percent(value):
    return f'{100*value:.1f}%' if value is not None and math.isfinite(value) else 'unknown (no endpoints)'


def _plots(runs, out):
    """Write time-series overlays, without requiring matplotlib for metrics."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    out.mkdir(parents=True, exist_ok=True)
    for kind in ('heading', 'path', 'drops_cpu'):
        fig, ax = plt.subplots()
        other = ax.twinx() if kind == 'drops_cpu' else None
        plotted = False
        for label, run in runs:
            slam = run.get('slam') or {}
            odom = _rows4(slam.get('odom', run.get('odom')))
            pose = slam_pose(slam)
            if kind == 'path':
                for name, rows in (('wheel', odom), ('SLAM', pose)):
                    if len(rows):
                        ax.plot(rows[:, 1], rows[:, 2], label=f'{label} {name}')
                        plotted = True
                ax.set(xlabel='x (m)', ylabel='y (m)', title='Recorded paths (each in its own frame)')
                ax.axis('equal')
            elif kind == 'heading':
                for name, rows in (('/odom', odom), ('/odom/imu', _rows4(slam.get('odom_imu', run.get('odom_imu')))), ('SLAM', pose)):
                    if len(rows):
                        yaw = np.unwrap(rows[:, 3])
                        ax.plot(rows[:, 0] - rows[0, 0], np.degrees(yaw-yaw[0]), label=f'{label} {name}')
                        plotted = True
                ax.set(xlabel='seconds since first sample', ylabel='heading change (deg)', title='Heading')
            else:
                bins = scan_drop_timeline(run.get('drops', []), run.get('proc_records', []))
                if bins:
                    t = np.array([b['t0'] for b in bins]); t -= t[0]
                    ax.plot(t, [b['drops'] for b in bins], label=f'{label} drops')
                    other.plot(t, [b['cpu_mean'] for b in bins], '--', label=f'{label} CPU')
                    plotted = True
                ax.set(xlabel='seconds since first event', ylabel='drops per 10 s', title='Scan drops and SLAM CPU')
                other.set_ylabel('CPU % (one core = 100%)')
        if plotted:
            handles, labels = ax.get_legend_handles_labels()
            if other:
                h, l = other.get_legend_handles_labels(); handles += h; labels += l
            ax.legend(handles, labels, fontsize='small')
            fig.tight_layout()
            fig.savefig(out / f'{kind}.png')
        plt.close(fig)


def main(argv=None, load_slam=None, load_scans=None, load_rosout=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', nargs='?')
    parser.add_argument('--log')
    parser.add_argument('--refresh', action='store_true')
    parser.add_argument('--no-plots', action='store_true')
    parser.add_argument('--compare', nargs=2, metavar=('RUN_A', 'RUN_B'))
    parser.add_argument('--out')
    args = parser.parse_args(argv)
    if bool(args.run) == bool(args.compare):
        parser.error('provide one run or --compare RUN_A RUN_B')

    def load(path):
        sl = load_slam or (lambda p: slam_bag.load(p, refresh=args.refresh))
        sc = load_scans or (lambda p: runlog.load_run(p, refresh=args.refresh))
        ro = load_rosout or (lambda p: _read_rosout(p, refresh=args.refresh))
        return load_inputs(path, args.log, sl, sc, ro)

    if args.compare:
        paths = [Path(p).expanduser().resolve() for p in args.compare]
        out = Path(args.out).expanduser().resolve() if args.out else Path.cwd() / f'slam_compare_{paths[0].name}_{paths[1].name}'
        if any(out == p or p in out.parents for p in paths):
            parser.error('--out must be outside both runs')
        # All cache writes go to temporary copies, never the input runs.
        with tempfile.TemporaryDirectory(prefix='slam_compare_') as temp:
            runs = []
            for index, path in enumerate(paths):
                copy = Path(temp) / str(index)
                shutil.copytree(path, copy)
                runs.append((path.name, load(copy)))
            reports = [build_report(run)[1] for _, run in runs]
            print(f'{paths[0].name} | {paths[1].name}')
            for section in reports[0]:
                print(f'{section}: {json.dumps(_json_clean(reports[0][section]))} | {json.dumps(_json_clean(reports[1][section]))}')
            if not args.no_plots:
                _plots(runs, out)
        return 0
    run = load(Path(args.run).expanduser())
    lines, summary = build_report(run)
    out = Path(args.run).expanduser() / 'analysis' / 'slam_report'
    out.mkdir(parents=True, exist_ok=True)
    text = '\n'.join(lines) + '\n'
    (out / 'slam_report.txt').write_text(text)
    (out / 'summary.json').write_text(json.dumps(_json_clean(summary), indent=2, allow_nan=False) + '\n')
    if not args.no_plots:
        _plots([(Path(args.run).name, run)], out)
    print(text, end='')
    return 0


def _json_clean(value):
    if isinstance(value, dict):
        return {key: _json_clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_clean(item) for item in value]
    if isinstance(value, np.generic):
        value = value.item()
    return None if isinstance(value, float) and not math.isfinite(value) else value


if __name__ == '__main__':
    sys.exit(main())
