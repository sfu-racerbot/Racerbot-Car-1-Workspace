#!/usr/bin/env python3
"""Pure pursuit lookahead tuner: sweep min_lookahead on a saved map + racing line.

SIMULATION ONLY. Drives the existing headless harness (run_validation.py's
run_pure_solo / PathFollower / PathPlan / FidelityPlant) on a *saved* SLAM map
and racing-line CSV, rejects every unsafe candidate, and recommends the
fastest safe one. Nothing here touches a config the car loads: the
recommendation is written to a separate YAML in the output directory.

Why it judges the *driven* trajectory: on indoor_oval (2026-11-06) the racing
line cleared the walls with the full oriented footprint, but pure pursuit ran
up to ~0.20 m wide of it on a corner exit at ~3.6 m/s and touched the wall.
So a candidate passes only if every seed completes its flying lap, the
simulator's own chassis contact check never fires, and the oriented chassis
footprint keeps at least --clearance-threshold from every occupied cell.
Ranking is lexicographic over passing candidates only (no weighted score), so a
crashing candidate can never be selected.

    python3 tools/f1tenth_sim/tune_lookahead.py \\
        --map data/rb1/racerbot_sim/sessions/<s>/mapping_82/<ts>/map.yaml \\
        --line data/rb1/racerbot_sim/sessions/<s>/mapping_82/<ts>/raceline_profiled.csv \\
        --out .sim/tune/indoor_tight

Outputs (in --out): lookahead_results.csv, lookahead_results.json and, only
when something passes, recommended_lookahead.yaml.
"""
from __future__ import annotations

import argparse
import csv
import datetime as _dt
import hashlib
import json
import math
import shutil
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]

NO_WINNER = 'NO SAFE CONFIGURATION FOUND'
TIE_FRACTION = 0.01            # lap times within 1% are tied
CANDIDATES = (0.4, 0.5, 0.6, 0.7, 0.8)
MAX_LOOKAHEAD = 1.5
LOOKAHEAD_GAIN = 0.15
START_POS_JITTER_M = 0.05      # per seed: uniform +/-, each axis
START_YAW_JITTER_DEG = 3.0


# ------------------------------------------------------------- pure helpers
def sha256_file(path) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 16), b''):
            h.update(block)
    return h.hexdigest()


class LineError(ValueError):
    """The racing-line CSV is missing or unusable. Never fall back to a built-in line."""


def load_line(path):
    """(xy, speed or None, sha256) from a racing-line CSV: x, y[, speed], header optional."""
    path = Path(path)
    if not path.is_file():
        raise LineError(f'racing-line CSV not found: {path}')
    try:
        rows = []
        with open(path, newline='') as f:
            for row in csv.reader(f):
                if not row or row[0].strip().startswith('#'):
                    continue
                try:
                    rows.append([float(v) for v in row])
                except ValueError:
                    if rows:      # a non-numeric row after data is corruption, not a header
                        raise
        table = np.asarray(rows, dtype=float)
    except (OSError, UnicodeDecodeError, ValueError) as exc:
        raise LineError(f'racing-line CSV unreadable: {path} ({exc})') from exc
    if table.ndim != 2 or table.shape[0] < 3 or table.shape[1] < 2 or not np.all(np.isfinite(table)):
        raise LineError(f'racing-line CSV has no usable x,y rows: {path} (shape {table.shape})')
    speed = table[:, 2].copy() if table.shape[1] >= 3 else None
    return table[:, :2].copy(), speed, sha256_file(path)


def effective_lookahead(speed, min_lookahead, gain=LOOKAHEAD_GAIN, max_lookahead=MAX_LOOKAHEAD):
    """racing_math.adaptive_lookahead: L_d = clip(gain*|v| + min, min, max)."""
    from pure_pursuit import racing_math
    return racing_math.adaptive_lookahead(speed, gain, min_lookahead, max_lookahead)


class MapDistance:
    """Continuous distance (m) from a world point to the nearest occupied cell's edge.

    Same occupancy rule as gym's track loader (pixel <= 128 occupied, > 128
    free, so a SLAM map's unknown grey counts as free). Sampling gym's own
    per-cell distance field snaps every clearance to multiples of the map
    resolution (0.05, 0.071, 0.10 m at 5 cm): far too coarse for a 0.10 m
    threshold. Here the nearest occupied cell comes from the EDT's index map
    and the distance is measured from the query point to that cell's square,
    so it is continuous; <= 0 inside an occupied cell.
    """

    def __init__(self, image, resolution, origin_xy):
        from scipy import ndimage
        img = np.asarray(image)
        self.occ = img <= 128
        self.res = float(resolution)
        self.ox, self.oy = float(origin_xy[0]), float(origin_xy[1])
        self.h, self.w = self.occ.shape
        _, self.idx = ndimage.distance_transform_edt(~self.occ, return_indices=True)

    @classmethod
    def from_yaml(cls, map_yaml):
        import yaml
        from PIL import Image
        meta = yaml.safe_load(Path(map_yaml).read_text())
        img = np.array(Image.open(Path(map_yaml).parent / meta['image']).convert('L'))
        if meta.get('negate'):
            img = 255 - img
        return cls(img, meta['resolution'], meta['origin'][:2])

    def body_clearance(self, corners, search_m=0.5):
        """Exact clearance (m) between an oriented rectangle and every occupied cell.

        ``corners`` is the (4, 2) chassis outline, consecutive, as
        FidelityPlant._chassis_outline gives it. Each occupied cell is a closed
        square; the result is the minimum vertex-to-shape distance both ways,
        which is the exact distance between two disjoint convex polygons and
        0.0 when they overlap (a vertex of either then lies inside the other --
        exact here, since a cell is narrower than the body and cannot cross it
        without one). Cells beyond ``search_m`` of the body's
        bounding box are not examined, so results saturate at ``search_m``.
        """
        corners = np.asarray(corners, dtype=float)
        lo = corners.min(axis=0) - search_m
        hi = corners.max(axis=0) + search_m
        c0 = max(0, int(math.floor((lo[0] - self.ox) / self.res)))
        c1 = min(self.w - 1, int(math.floor((hi[0] - self.ox) / self.res)))
        r0 = max(0, self.h - 1 - int(math.floor((hi[1] - self.oy) / self.res)))
        r1 = min(self.h - 1, self.h - 1 - int(math.floor((lo[1] - self.oy) / self.res)))
        if c0 > c1 or r0 > r1:
            return 0.0                                   # body entirely off the map
        rr, cc = np.nonzero(self.occ[r0:r1 + 1, c0:c1 + 1])
        if rr.size == 0:
            return float(search_m)
        x0 = self.ox + (cc + c0) * self.res
        y0 = self.oy + (self.h - 1 - (rr + r0)) * self.res
        x1, y1 = x0 + self.res, y0 + self.res
        # body frame: origin at corners[0], axes along its two edges
        u = corners[1] - corners[0]; length = float(np.hypot(*u)); u = u / length
        v = corners[3] - corners[0]; width = float(np.hypot(*v)); v = v / width
        cv = np.stack([np.c_[x0, y0], np.c_[x1, y0], np.c_[x1, y1], np.c_[x0, y1]], axis=1) - corners[0]
        bx, by = cv @ u, cv @ v                          # (n, 4) cell vertices in the body frame
        d_cell = np.hypot(np.maximum.reduce([-bx, np.zeros_like(bx), bx - length]),
                          np.maximum.reduce([-by, np.zeros_like(by), by - width]))
        px, py = corners[:, 0][None, :], corners[:, 1][None, :]      # body vertices vs squares
        dx = np.maximum.reduce([x0[:, None] - px, np.zeros((x0.size, 4)), px - x1[:, None]])
        dy = np.maximum.reduce([y0[:, None] - py, np.zeros((y0.size, 4)), py - y1[:, None]])
        return float(min(d_cell.min(), np.hypot(dx, dy).min()))

    def __call__(self, x, y):
        c = int(math.floor((x - self.ox) / self.res))
        r = self.h - 1 - int(math.floor((y - self.oy) / self.res))
        if not (0 <= r < self.h and 0 <= c < self.w):
            return 0.0                                   # off the map: treat as blocked
        if self.occ[r, c]:
            return 0.0
        rr, cc = self.idx[0][r, c], self.idx[1][r, c]
        cx0 = self.ox + cc * self.res                    # that occupied cell's square
        cy0 = self.oy + (self.h - 1 - rr) * self.res
        dx = max(cx0 - x, 0.0, x - (cx0 + self.res))
        dy = max(cy0 - y, 0.0, y - (cy0 + self.res))
        return math.hypot(dx, dy)


def aggregate(runs, threshold):
    """Per (min_lookahead, speed_mode): pass/fail with reason and worst-case metrics."""
    groups = {}
    for r in runs:
        groups.setdefault((r['min_lookahead'], r['speed_mode']), []).append(r)
    out = []
    for (la, mode), rs in sorted(groups.items()):
        reasons = []
        for r in sorted(rs, key=lambda r: r['seed']):
            why = []
            if not r['lap_completed']:
                why.append('flying lap not completed')
            if r['collision']:
                why.append('collision')
            if not (r['min_clearance_m'] >= threshold):
                why.append(f"clearance {r['min_clearance_m']:.3f} m < {threshold:.3f} m")
            if why:
                reasons.append(f"seed {r['seed']}: " + ', '.join(why))
        completed = [r for r in rs if r['lap_completed']]
        out.append(dict(
            min_lookahead=la, speed_mode=mode, seeds=sorted(r['seed'] for r in rs),
            passed=not reasons, fail_reason='; '.join(reasons) or None,
            worst_lap_time_s=max(r['lap_time_s'] for r in completed) if len(completed) == len(rs) else None,
            worst_min_clearance_m=min(r['min_clearance_m'] for r in rs),
            worst_max_abs_cte_m=max(r['max_abs_cte_m'] for r in rs),
            mean_rms_cte_m=float(np.mean([r['rms_cte_m'] for r in rs])),
            mean_abs_steer_rate_rad_s=float(np.mean([r['mean_abs_steer_rate_rad_s'] for r in rs])),
        ))
    return out


def rank(aggregates, stress_mode='stress', base_mode='cap_1.5'):
    """(ranking_mode, ordered passing aggregates). Lexicographic, passing only:
    stress mode if anything passes there, else the 1.5 m/s cap; then lowest
    worst lap time with times within 1% tied, then lowest worst max CTE, then
    lowest mean steering rate."""
    for mode in (stress_mode, base_mode):
        pool = [a for a in aggregates if a['speed_mode'] == mode and a['passed']]
        if pool:
            break
    else:
        return None, []
    ordered = []
    pool = list(pool)
    while pool:
        fastest = min(a['worst_lap_time_s'] for a in pool)
        tied = [a for a in pool if a['worst_lap_time_s'] <= fastest * (1.0 + TIE_FRACTION)]
        best = min(tied, key=lambda a: (a['worst_max_abs_cte_m'], a['mean_abs_steer_rate_rad_s'],
                                        a['worst_lap_time_s'], a['min_lookahead']))
        ordered.append(best)
        pool.remove(best)
    return mode, ordered


def recommended_yaml(winner, provenance) -> str:
    lines = ['# Recommended pure pursuit lookahead -- written by tools/f1tenth_sim/tune_lookahead.py.',
             '# SIMULATION RESULT ONLY: not loaded by anything. Same structure as',
             '# src/pure_pursuit/config/pure_pursuit.yaml; copy by hand after on-car testing.',
             f"# winner: min_lookahead {winner['min_lookahead']} in speed mode {winner['speed_mode']}: "
             f"worst lap {winner['worst_lap_time_s']:.3f} s, min clearance {winner['worst_min_clearance_m']:.3f} m, "
             f"max CTE {winner['worst_max_abs_cte_m']:.3f} m"]
    for k, v in provenance.items():
        lines.append(f'# {k}: {json.dumps(v, sort_keys=True)}')
    lines += ['pure_pursuit_node:', '  ros__parameters:',
              f"    min_lookahead: {winner['min_lookahead']}",
              f'    max_lookahead: {MAX_LOOKAHEAD}',
              f'    lookahead_speed_gain: {LOOKAHEAD_GAIN}', '']
    return '\n'.join(lines)


def write_outputs(out_dir, runs, aggregates, ranking_mode, ranking, provenance, extra=None):
    """Write CSV + JSON always; YAML only with a winner (and remove a stale one)."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    fields = ['min_lookahead', 'max_lookahead', 'lookahead_speed_gain', 'speed_mode', 'speed_cap_mps',
              'seed', 'lap_completed', 'collision', 'lap_time_s', 'min_clearance_m', 'max_abs_cte_m',
              'rms_cte_m', 'mean_abs_steer_rate_rad_s', 'map_id', 'line_sha256']
    with open(out_dir / 'lookahead_results.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader()
        for r in runs:
            w.writerow(r)
    winner = ranking[0] if ranking else None
    doc = dict(provenance=provenance, ranking_mode=ranking_mode, winner=winner,
               verdict=NO_WINNER if winner is None else 'WINNER',
               ranking=[dict(min_lookahead=a['min_lookahead'], speed_mode=a['speed_mode']) for a in ranking],
               aggregates=aggregates, runs=runs, **(extra or {}))
    (out_dir / 'lookahead_results.json').write_text(json.dumps(doc, indent=2, sort_keys=True))
    yaml_path = out_dir / 'recommended_lookahead.yaml'
    if winner is None:
        if yaml_path.exists():
            yaml_path.unlink()
    else:
        yaml_path.write_text(recommended_yaml(winner, provenance))
    return doc


# --------------------------------------------------------------- simulation
def stage_track(map_yaml, line_xy, work_dir) -> tuple[str, str, str]:
    """Copy a saved SLAM map into a gym track directory. Returns (gym path, map id, image sha256).

    Gym loads <stem>_map.yaml beside the path; its own lap bookkeeping needs a
    centerline, so the supplied racing line is written as <stem>_centerline.csv.
    No <stem>_raceline.csv is written: there is no built-in line to fall back to.
    """
    import yaml
    map_yaml = Path(map_yaml)
    if not map_yaml.is_file():
        raise FileNotFoundError(f'map yaml not found: {map_yaml}')
    meta = yaml.safe_load(map_yaml.read_text())
    image = (map_yaml.parent / meta['image']) if not Path(meta['image']).is_absolute() else Path(meta['image'])
    if not image.is_file():
        raise FileNotFoundError(f'map image not found: {image}')
    image_sha = sha256_file(image)
    map_id = f'{map_yaml.parent.name}_{image_sha[:12]}'
    work = Path(work_dir) / 'track'
    work.mkdir(parents=True, exist_ok=True)
    stem = 'track'
    shutil.copyfile(image, work / f'{stem}{image.suffix}')
    # gym's TrackSpec takes only these keys (a SLAM map's 'mode: trinary' is refused)
    keep = ('resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh')
    staged = {k: meta[k] for k in keep if k in meta}
    staged['image'] = f'{stem}{image.suffix}'
    (work / f'{stem}_map.yaml').write_text(yaml.safe_dump(staged, sort_keys=False))
    with open(work / f'{stem}_centerline.csv', 'w') as f:
        f.write('x_m,y_m\n')
        for x, y in line_xy:
            f.write(f'{x:.6f},{y:.6f}\n')
    for stale in (work / f'{stem}_raceline.csv',):
        if stale.exists():
            stale.unlink()
    return str(work / stem), map_id, image_sha


def _plan(line_xy, line_speed, cap):
    import run_validation as rv
    from pure_pursuit import racing_math
    xy = np.asarray(line_xy, float)
    seg_len = racing_math.compute_segment_lengths(xy, closed=True)
    if line_speed is None:
        curvature = racing_math.estimate_path_curvature(xy, closed=True)
        speed = racing_math.compute_velocity_profile(
            seg_len, curvature, v_max=cap, v_min=0.5, a_lat_max=2.5, a_accel_max=3.0,
            a_brake_max=8.0, closed=True, smoothing_passes=8, friction_ellipse=True)
    else:
        speed = np.minimum(np.asarray(line_speed, float), cap)
    return rv.PathPlan(xy=xy, seg_len=seg_len, speed=np.asarray(speed, float))


def start_pose(line_xy, seed):
    """The line's first point, heading along it, jittered per seed (+/-5 cm, +/-3 deg)."""
    xy = np.asarray(line_xy, float)
    yaw = math.atan2(xy[1, 1] - xy[0, 1], xy[1, 0] - xy[0, 0])
    rng = np.random.default_rng(10_000 + int(seed))
    dx, dy = rng.uniform(-START_POS_JITTER_M, START_POS_JITTER_M, 2)
    dyaw = math.radians(rng.uniform(-START_YAW_JITTER_DEG, START_YAW_JITTER_DEG))
    return np.array([[xy[0, 0] + dx, xy[0, 1] + dy, yaw + dyaw]])


def run_one(job) -> dict:
    """One candidate x speed mode x seed: a warm-up lap from rest, then a timed flying lap."""
    sys.path.insert(0, str(HERE))
    import run_validation as rv
    track, line_xy, line_speed, la, mode, cap, seed, prov, car_id = job
    plan = _plan(line_xy, line_speed, cap)
    length = float(np.sum(plan.seg_len))
    flying_timeout = 3.0 * length / cap
    steer, clear, where = [], [], []
    dist = MapDistance.from_yaml(f'{track}_map.yaml')

    def hook(plant, steering, _speed):
        steer.append(float(steering))
        clear.append(dist.body_clearance(plant._chassis_outline(0)))
        t = plant.truth(0)
        where.append((round(float(t[0]), 3), round(float(t[1]), 3), round(float(t[4]), 3),
                      round(float(t[3]), 2)))

    r = rv.run_pure_solo(track, seed, timeout_s=2.0 * flying_timeout + 10.0,
                         law_params=dict(min_lookahead=la, max_lookahead=MAX_LOOKAHEAD,
                                         lookahead_speed_gain=LOOKAHEAD_GAIN),
                         car_id=car_id, plan=plan, max_speed=cap, max_laps=2,
                         start_pose=start_pose(line_xy, seed), step_hook=hook)
    laps = r.get('lap_times_s') or []
    flying = laps[1] - laps[0] if len(laps) >= 2 else None
    contact = int(r.get('contact_steps', 0) or 0)
    dt = rv.CONTROL_DT
    return dict(
        min_lookahead=la, max_lookahead=MAX_LOOKAHEAD, lookahead_speed_gain=LOOKAHEAD_GAIN,
        speed_mode=mode, speed_cap_mps=cap, seed=seed,
        lap_completed=flying is not None and flying <= flying_timeout,
        collision=contact > 0, contact_steps=contact,
        lap_time_s=round(flying, 4) if flying is not None else None,
        warmup_lap_s=round(laps[0], 4) if laps else None,
        min_clearance_m=round(min(clear), 4) if clear else 0.0,
        min_clearance_pose=where[int(np.argmin(clear))] if clear else None,
        min_clearance_step=int(np.argmin(clear)) if clear else None,
        max_abs_cte_m=r['max_cross_track_m'], rms_cte_m=r['rms_cross_track_m'],
        mean_abs_steer_rate_rad_s=round(float(np.mean(np.abs(np.diff(steer)))) / dt, 4) if len(steer) > 1 else 0.0,
        stop_steps=r.get('stop_steps'), avoid_steps=r.get('avoid_steps'),
        map_id=prov['map_id'], line_sha256=prov['line_sha256'])


def _git_state():
    def git(*a):
        return subprocess.run(['git', '-c', 'safe.directory=*', '-C', str(ROOT), *a],
                              capture_output=True, text=True, timeout=30)
    try:
        head = git('rev-parse', 'HEAD').stdout.strip() or 'unknown'
        dirty = bool(git('status', '--porcelain', '--untracked-files=no').stdout.strip())
    except (OSError, subprocess.SubprocessError):
        head, dirty = 'unknown', None
    return head, dirty


def _sim_config(car_id):
    sys.path.insert(0, str(HERE))
    import run_validation as rv
    rv.select_profile(rv.PROFILE.name, car_id)
    c = rv.CALIBRATION
    return dict(fidelity_profile=rv.PROFILE.name, car_id=rv._CAR_ID, control_dt_s=rv.CONTROL_DT,
                footprint=dict(body_length_m=c.body_length, body_width_m=c.body_width,
                               reference='rear axle (base_link)',
                               body_center_ahead_of_reference_m=c.wheelbase / 2.0, wheelbase_m=c.wheelbase),
                noise=dict(pose_noise_xy_m=c.pose_noise_xy, pose_noise_yaw_rad=c.pose_noise_yaw,
                           odom_speed_noise=c.odom_speed_noise, lidar_noise_std_m=c.lidar_noise_std,
                           lidar_dropout_rate=c.lidar_dropout_rate, steering_delay_ticks=c.steering_delay_ticks,
                           pose_delay_ticks=c.pose_delay_ticks,
                           start_pose_jitter=f'+/-{START_POS_JITTER_M} m per axis, +/-{START_YAW_JITTER_DEG} deg (seeded)'),
                steering_limit_rad=rv.STEERING_LIMIT, max_steering_rate=float(rv.PURE_PURSUIT_PARAMS['max_steering_rate']),
                unknown_cells='gym: pixel <= 128 occupied, > 128 free (SLAM unknown 205 counts as free)',
                clearance_method='exact distance between the oriented chassis rectangle and every occupied '
                                 'cell square of the staged map within 0.5 m (MapDistance.body_clearance); '
                                 '0 on overlap; values saturate at 0.5 m',
                collision='FidelityPlant contact_steps (its own chassis-vs-map check)')


def _stress_speed():
    import yaml
    cfg = yaml.safe_load((ROOT / 'src/pure_pursuit/config/auto_map_race.yaml').read_text())
    v = cfg.get('auto_map_race_node', {}).get('ros__parameters', {}).get('profile_max_speed')
    return float(v) if v else 3.0


def sweep(track, line_xy, line_speed, candidates, modes, seeds, prov, car_id, workers):
    jobs = [(track, line_xy, line_speed, la, mode, cap, s, prov, car_id)
            for la in candidates for (mode, cap) in modes for s in seeds]
    with ProcessPoolExecutor(max(1, workers)) as pool:
        return list(pool.map(run_one, jobs))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', required=True, help='saved occupancy map YAML (its image beside it)')
    ap.add_argument('--line', required=True, help='racing-line CSV: x, y[, speed]')
    ap.add_argument('--out', required=True)
    ap.add_argument('--cap-speed', type=float, default=1.5, help='baseline speed cap, m/s')
    ap.add_argument('--stress-speed', type=float, default=None,
                    help="default: auto_map_race.yaml profile_max_speed (3.0 if unset)")
    ap.add_argument('--min-lookahead', type=float, nargs='+', default=list(CANDIDATES))
    ap.add_argument('--seeds', type=int, nargs='+', default=[0, 1, 2])
    ap.add_argument('--clearance-threshold', type=float, default=0.10)
    ap.add_argument('--car', default='rb1')
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--no-recheck', action='store_true', help='skip the winner re-run')
    a = ap.parse_args(argv)

    line_xy, line_speed, line_sha = load_line(a.line)       # fails loudly, never falls back
    out = Path(a.out)
    track, map_id, image_sha = stage_track(a.map, line_xy, out)
    stress = a.stress_speed if a.stress_speed is not None else _stress_speed()
    modes = [(f'cap_{a.cap_speed:g}', a.cap_speed), ('stress', stress)]
    head, dirty = _git_state()
    prov = dict(map_path=str(Path(a.map).resolve()), map_image_sha256=image_sha, map_id=map_id,
                line_path=str(Path(a.line).resolve()), line_sha256=line_sha,
                git_commit=head, git_dirty=dirty,
                controller=dict(law='pure_pursuit', min_lookahead_candidates=a.min_lookahead,
                                max_lookahead=MAX_LOOKAHEAD, lookahead_speed_gain=LOOKAHEAD_GAIN,
                                lookahead_basis='measured speed (use_measured_lookahead)'),
                simulator=_sim_config(a.car), seeds=a.seeds,
                speed_modes={m: c for m, c in modes}, clearance_threshold_m=a.clearance_threshold,
                timestamp=_dt.datetime.now(_dt.timezone.utc).isoformat(timespec='seconds'),
                command=' '.join(['python3', 'tools/f1tenth_sim/tune_lookahead.py'] + (argv or sys.argv[1:])))
    sys.path.insert(0, str(ROOT / 'src' / 'pure_pursuit'))
    eff = {f'{la:g}': {m: round(effective_lookahead(c, la), 4) for m, c in modes} for la in a.min_lookahead}
    flags = {}
    for m, c in modes:
        vals = [effective_lookahead(c, la) for la in a.min_lookahead]
        flags[m] = 'non-discriminating (identical effective lookahead)' if len(set(round(v, 9) for v in vals)) < len(vals) else 'discriminating'

    runs = sweep(track, line_xy, line_speed, a.min_lookahead, modes, a.seeds, prov, a.car, a.workers)
    # verify every run used the supplied line
    assert all(r['line_sha256'] == line_sha for r in runs)
    aggregates = aggregate(runs, a.clearance_threshold)
    ranking_mode, ranking = rank(aggregates, 'stress', modes[0][0])
    extra = dict(effective_lookahead_m=eff, speed_mode_flags=flags)
    winner = ranking[0] if ranking else None
    if winner is not None and not a.no_recheck:
        cap = dict(modes)[winner['speed_mode']]
        again = sweep(track, line_xy, line_speed, [winner['min_lookahead']], [(winner['speed_mode'], cap)],
                      a.seeds, prov, a.car, a.workers)
        first = sorted((r for r in runs if r['min_lookahead'] == winner['min_lookahead']
                        and r['speed_mode'] == winner['speed_mode']), key=lambda r: r['seed'])
        keys = ('lap_completed', 'collision', 'lap_time_s', 'min_clearance_m', 'max_abs_cte_m', 'rms_cte_m',
                'mean_abs_steer_rate_rad_s')
        diffs = [(r1['seed'], k, r1[k], r2[k]) for r1, r2 in zip(first, sorted(again, key=lambda r: r['seed']))
                 for k in keys if r1[k] != r2[k]]
        extra['winner_check'] = dict(identical=not diffs, differences=diffs[:20],
                                     all_laps_completed=all(r['lap_completed'] for r in again))
    write_outputs(out, runs, aggregates, ranking_mode, ranking, prov, extra)

    print(f'line {a.line}\n  sha256 {line_sha}\nmap {a.map} (id {map_id})\n')
    print('effective lookahead (m) per candidate:')
    for la in a.min_lookahead:
        print(f"  min {la:<4g}: " + ', '.join(f'{m} ({c:g} m/s) {eff[f"{la:g}"][m]:.3f}' for m, c in modes))
    for m, f in flags.items():
        print(f'  {m}: {f}')
    print('\npass / fail:')
    for g in aggregates:
        print(f"  min {g['min_lookahead']:<4g} {g['speed_mode']:8s} {'PASS' if g['passed'] else 'FAIL'}"
              + (f"  worst lap {g['worst_lap_time_s']:.2f} s" if g['worst_lap_time_s'] else '')
              + f"  min clr {g['worst_min_clearance_m']:.3f}  max CTE {g['worst_max_abs_cte_m']:.3f}"
              + ('' if g['passed'] else f"  <- {g['fail_reason']}"))
    if winner is None:
        print(f'\n{NO_WINNER}')
    else:
        print(f"\nWINNER: min_lookahead {winner['min_lookahead']} ({winner['speed_mode']}"
              + (', ranked on the 1.5 m/s cap: nothing passed at stress speed' if ranking_mode != 'stress' else '') + ')')
        print(f"  worst lap {winner['worst_lap_time_s']:.3f} s, min clearance {winner['worst_min_clearance_m']:.3f} m, "
              f"max CTE {winner['worst_max_abs_cte_m']:.3f} m, mean RMS CTE {winner['mean_rms_cte_m']:.3f} m")
        if 'winner_check' in extra:
            print(f"  re-run: {'identical' if extra['winner_check']['identical'] else 'DIFFERENT'}"
                  f", laps completed: {extra['winner_check']['all_laps_completed']}")
    print(f'\nreproduce: {prov["command"]}')
    return 0 if winner is not None else 2


if __name__ == '__main__':
    sys.exit(main())
