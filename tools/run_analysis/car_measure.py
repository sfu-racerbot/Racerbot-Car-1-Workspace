#!/usr/bin/env python3
"""Measure the real car from recorded runs, for calibrating the simulator.

    python3 tools/run_analysis/car_measure.py data/racerbot_runs/20261030-185036 ... \\
        --json measured.json

Needs each run located first with loc.py. True speed and heading come from that, so
none of these numbers rest on the wheels or on /odom's yaw, except where those are
what is being measured. Reads the bags and writes only where --json says.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np

WHEELBASE = 0.36        # m, docs/hardware-reference.md
DT = 0.025              # s, the plant's tick
LINKAGE_DEG = (-34, -30, -26, -22, -18, -14, -10, -7, -4, -2, -0.5, 0.5, 2, 4, 7, 10, 14, 18, 22, 26, 30, 34)
GRIP_BINS = (0.25, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 5, 6, 8, 12)
STEER_LAG = 0.14        # s, the measured lag of command to rack
FULL_LOCK = 0.46        # rad, what the rack reaches; Team A commands past it


# --- reading ---------------------------------------------------------------------------

def read_series(run) -> dict:
    """Time-stamped arrays of what the car was told and what its own sensors said."""
    from run_analysis.bagread import messages
    drive, odom, gyro = [], [], []
    for topic, stamp, m in messages(run, ["/drive", "/odom", "/sensors/imu/raw"]):
        if topic == "/drive":
            drive.append((stamp, m.drive.steering_angle, m.drive.speed))
        elif topic == "/odom":
            odom.append((stamp, m.twist.twist.linear.x, m.twist.twist.angular.z))
        else:
            gyro.append((stamp, math.radians(m.angular_velocity.z)))
    return {name: np.array(rows) for name, rows in (("drive", drive), ("odom", odom), ("gyro", gyro))}


def measure_run(run, loc_dir) -> dict:
    """A located run as uniform 40 Hz arrays of command, sensors and true motion."""
    from run_analysis.loc import COL, load_track
    from run_analysis.sectors import MAX_TRUE_SPEED, true_speed
    track = load_track(run, loc_dir)
    series = read_series(run)
    if not len(series["drive"]):
        raise ValueError(f"{run}: no /drive in this run")
    tt = track[:, COL["t"]]
    tv = true_speed(tt, track[:, COL["x"]], track[:, COL["y"]])
    trusted = (track[:, COL["h20"]] > 0.8) & (tv < MAX_TRUE_SPEED) & (track[:, COL["lb"]] > 0)
    t = np.arange(tt[0], tt[-1], DT)
    drive, odom, gyro = series["drive"], series["odom"], series["gyro"]
    held = drive[np.clip(np.searchsorted(drive[:, 0], t, side="right") - 1, 0, len(drive) - 1)]
    yaw = np.interp(t, tt, np.unwrap(track[:, COL["yaw"]]))
    return dict(name=Path(run).name, t=t, v=np.interp(t, tt, tv),
                good=np.interp(t, tt, trusted.astype(float)) > 0.9,
                scan_yaw_rate=np.gradient(np.convolve(yaw, np.ones(5) / 5, "same"), t),
                steer=held[:, 1], speed_cmd=held[:, 2], wheel=np.interp(t, odom[:, 0], odom[:, 1]),
                odom_yaw=np.interp(t, odom[:, 0], odom[:, 2]), gyro=np.interp(t, gyro[:, 0], gyro[:, 1]),
                raw_gyro=gyro, raw_odom=odom)


# --- lag -------------------------------------------------------------------------------

def best_lag(t, measured, cmd_t, cmd, lags) -> tuple:
    """The lag making cmd(t - lag) closest to ``measured``, and its RMS."""
    best = (math.nan, math.inf)
    for lag in lags:
        error = np.interp(np.asarray(t) - lag, cmd_t, cmd) - np.asarray(measured)
        rms = float(np.sqrt(np.mean(error ** 2)))
        if rms < best[1]:
            best = (float(lag), rms)
    return best


# --- steering --------------------------------------------------------------------------

def steering_from_motion(v, yaw_rate, wheelbase: float = WHEELBASE) -> np.ndarray:
    """The rack angle a turn implies, from the bicycle model."""
    return np.arctan(wheelbase * np.asarray(yaw_rate) / np.maximum(np.asarray(v), 1e-3))


def linkage_table(cmd, actual, edges, min_samples: int = 15) -> list:
    """Rows of (median command, median actual, samples), per ``edges`` bin of command."""
    cmd, actual = np.asarray(cmd), np.asarray(actual)
    rows = []
    for low, high in zip(edges[:-1], edges[1:]):
        chosen = (cmd >= low) & (cmd < high)
        if chosen.sum() >= min_samples:
            rows.append((float(np.median(cmd[chosen])), float(np.median(actual[chosen])), int(chosen.sum())))
    return rows


def replay_servo(cmd, dt: float, delay: int, gain: float, rate: float) -> np.ndarray:
    """A command through a transport delay and a slew-limited proportional servo."""
    cmd = np.asarray(cmd, float)
    delayed = np.r_[np.full(delay, cmd[0]), cmd][:len(cmd)]
    out = np.empty(len(cmd))
    angle = delayed[0]
    for i, target in enumerate(delayed):
        angle += min(max(gain * (target - angle), -rate), rate) * dt
        out[i] = angle
    return out


# --- speed loop ------------------------------------------------------------------------

def simulate_speed_loop(cmd, v0: float, dt: float, delay: int, accel_gain: float, accel_max: float,
                        brake_gain: float, brake_max: float) -> np.ndarray:
    """The plant's speed loop, on its own: proportional per direction, limited each way."""
    out = np.empty(len(cmd))
    v = v0
    for i in range(len(cmd)):
        diff = cmd[max(0, i - delay)] - v
        a = min(accel_gain * diff, accel_max) if diff >= 0.0 else max(brake_gain * diff, -brake_max)
        v = max(0.0, v + a * dt)
        out[i] = v
    return out


def fit_speed_loop(segments, dt: float, grid: dict) -> tuple:
    """The best of ``grid`` over ``segments`` of (command, measured speed). Returns (setting, rms)."""
    names = sorted(grid)
    best, best_rms = None, math.inf
    for values in itertools.product(*(grid[n] for n in names)):
        setting = dict(zip(names, values))
        sq, n = 0.0, 0
        for cmd, measured in segments:
            sim = simulate_speed_loop(cmd, measured[0], dt, **setting)
            sq += float(np.sum((sim - measured) ** 2))
            n += len(cmd)
        rms = math.sqrt(sq / max(n, 1))
        if rms < best_rms:
            best, best_rms = setting, rms
    return best, best_rms


def hard_stops(t, wheel, cmd, min_speed: float = 2.0, stopped: float = 0.3, longest: float = 2.5) -> list:
    """(start index, end index) where the command went to zero and the wheels followed.

    A command that dips while the car keeps rolling is not a stop; it never reaches
    ``stopped`` within ``longest`` seconds, or the command comes back, and it is left out.
    """
    t, wheel, cmd = np.asarray(t), np.asarray(wheel), np.asarray(cmd)
    out = []
    i = 1
    while i < len(t):
        if wheel[i] > min_speed and cmd[i] < stopped <= cmd[i - 1]:
            j = i
            while j < len(t) - 1 and wheel[j] > stopped and t[j] - t[i] < longest:
                j += 1
            if wheel[j] <= stopped and cmd[i:j + 1].max() < stopped:
                out.append((i, j))
            i = j
        i += 1
    return out


def stop_distance(v0: float, delay: float, gain: float, brake_max: float, end_speed: float = 0.28,
                  dt: float = 0.001) -> float:
    """Metres from the command dropping until ``end_speed``, for the plant's speed loop."""
    distance = v0 * delay
    v = v0
    while v > end_speed:
        v = max(0.0, v - min(gain * v, brake_max) * dt)
        distance += v * dt
    return distance


def fit_braking(stops, delays, gains, maxima, end_speed: float = 0.28) -> list:
    """Every (delay, gain, ceiling), best first, by squared relative distance error over
    ``stops`` of (true start speed m/s, measured distance m)."""
    rows = []
    for delay, gain, top in itertools.product(delays, gains, maxima):
        error = sum(((stop_distance(v0, delay, gain, top, end_speed) - d) / d) ** 2 for v0, d in stops)
        rows.append((error, delay, gain, top))
    return sorted(rows)


# --- odometry --------------------------------------------------------------------------

def speed_ratio(wheel, true_v, min_speed: float = 1.0) -> float:
    """Wheel speed over true speed, by least squares through zero."""
    wheel, true_v = np.asarray(wheel), np.asarray(true_v)
    keep = true_v > min_speed
    return float(np.sum(wheel[keep] * true_v[keep]) / np.sum(true_v[keep] ** 2))


# --- LiDAR -----------------------------------------------------------------------------

def scan_statistics(scans) -> dict:
    """From a (scans, beams) range array taken while parked: noise and dropouts per beam."""
    r = np.asarray(scans, dtype=float)
    good = np.isfinite(r) & (r > 0.0)
    filled = np.where(good, r, np.nan)
    with np.errstate(all="ignore"):
        std = np.nanstd(filled, axis=0)
    return dict(noise=float(np.nanmedian(std)), dropout=float(1.0 - good.mean()),
                dropout_by_beam=1.0 - good.mean(axis=0),
                median=np.nanmedian(filled, axis=0), std_by_beam=std)


def self_hits(median_ranges, angles, max_range: float = 0.35) -> list:
    """(angle, range) of beams that return from within ``max_range`` whatever the car does."""
    return [(float(a), float(r)) for a, r in zip(angles, median_ranges)
            if np.isfinite(r) and r < max_range]


def edge_statistics(scans, jump: float = 0.3, isolated: float = 0.1) -> dict:
    """Per scan, how broken up the returns are: range jumps and lone returns."""
    r = np.asarray(scans, dtype=float)
    r = np.where(np.isfinite(r) & (r > 0.0), r, np.nan)
    with np.errstate(invalid="ignore"):
        steps = np.abs(np.diff(r, axis=1))
        lone = (steps[:, :-1] > isolated) & (steps[:, 1:] > isolated)
        return dict(jumps=float(np.mean(np.nansum(steps > jump, axis=1))),
                    isolated=float(np.mean(lone.sum(axis=1))))


# --- the map ---------------------------------------------------------------------------

def cast_scan(occupied, resolution: float, ox: float, oy: float, x: float, y: float, yaw: float,
              angles, max_range: float = 12.0, step: float = 0.01) -> np.ndarray:
    """Ranges a laser at (x, y, yaw) would read on an occupancy grid; inf past max_range."""
    steps = np.arange(step, max_range, step)
    dirs = yaw + np.asarray(angles)
    rows = np.floor((y + np.outer(steps, np.sin(dirs)) - oy) / resolution).astype(int)
    cols = np.floor((x + np.outer(steps, np.cos(dirs)) - ox) / resolution).astype(int)
    inside = (rows >= 0) & (rows < occupied.shape[0]) & (cols >= 0) & (cols < occupied.shape[1])
    hit = np.zeros_like(inside)
    hit[inside] = occupied[rows[inside], cols[inside]]
    first = hit.argmax(axis=0)
    return np.where(hit.any(axis=0), steps[first], np.inf)


# --- report ----------------------------------------------------------------------------

def _gyro_bias(runs) -> float:
    rows = []
    for r in runs:
        parked = np.abs(np.interp(r["raw_gyro"][:, 0], r["raw_odom"][:, 0], r["raw_odom"][:, 1])) < 0.005
        if parked.sum() > 50:
            rows.append(float(np.median(r["raw_gyro"][parked, 1])))
    return float(np.median(rows)) if rows else 0.0


def report(runs) -> dict:
    """Every car item the runs can measure, as plain numbers."""
    out = {"runs": [r["name"] for r in runs]}
    bias = _gyro_bias(runs)
    out["gyro_bias_rad_s"] = bias
    keep = [r["good"] & (r["v"] > 1.0) for r in runs]
    # Odometry.
    out["wheel_over_true"] = speed_ratio(np.concatenate([r["wheel"][r["good"]] for r in runs]),
                                         np.concatenate([r["v"][r["good"]] for r in runs]))
    out["wheel_noise_share"] = float(np.median(
        [np.std(r["wheel"][k] - out["wheel_over_true"] * r["v"][k]) / np.mean(r["v"][k]) for r, k in zip(runs, keep)]))
    lead, scale = [], []
    for r, k in zip(runs, keep):
        gz = r["gyro"] - bias
        lead.append(best_lag(r["t"][k], gz[k], r["t"], r["odom_yaw"], np.arange(-0.1, 0.4, 0.0125))[0])
        late = np.interp(r["t"][k] - lead[-1], r["t"], r["odom_yaw"])
        scale.append(float(np.sum(late * gz[k]) / np.sum(gz[k] ** 2)))
    out["odom_yaw_leads_gyro_s"] = float(np.median(lead))
    out["odom_yaw_over_gyro"] = float(np.median(scale))
    out["scan_behind_gyro_s"] = float(np.median(
        [best_lag(r["t"][k], r["scan_yaw_rate"][k], r["t"], r["gyro"] - bias, np.arange(-0.1, 0.2, 0.0025))[0]
         for r, k in zip(runs, keep)]))
    # Steering: only where the tyres follow the rack, so yaw tells where the rack is.
    cmd, rack = [], []
    for r in runs:
        gz = r["gyro"] - bias
        k = r["good"] & (r["v"] > 0.7) & (r["v"] < 3.5) & (np.abs(r["v"] * gz) < 1.5)
        cmd.append(np.interp(r["t"][k] - STEER_LAG, r["t"], r["steer"]))
        rack.append(steering_from_motion(r["v"][k], gz[k]))
    cmd, rack = np.concatenate(cmd), np.concatenate(rack)
    out["steering_linkage"] = linkage_table(cmd, rack, np.radians(LINKAGE_DEG))
    out["steering_trim_rad"] = float(np.median(rack[np.abs(cmd) < 0.005]))
    fits = []
    for delay in range(0, 8):
        for gain in (6.0, 8.0, 10.0, 12.0, 15.0, 20.0, 30.0):
            for rate in (0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.2, 4.0, 6.0):
                sq, n = 0.0, 0
                for r in runs:
                    gz = r["gyro"] - bias
                    k = r["good"] & (r["v"] > 1.0) & (np.abs(r["v"] * gz) < 2.0)
                    yaw = r["v"] * np.tan(replay_servo(0.85 * r["steer"], DT, delay, gain, rate)) / WHEELBASE
                    sq += float(np.sum((yaw[k] - gz[k]) ** 2))
                    n += int(k.sum())
                fits.append((math.sqrt(sq / n), delay, gain, rate))
    rms, delay, gain, rate = min(fits)
    out["servo"] = dict(delay_ticks=delay, gain=gain, rate=rate, yaw_rms=rms)
    # Speed loop, on true speed, and top speed.
    segments = []
    for r in runs:
        edges = np.flatnonzero(np.diff(np.r_[0, r["good"].astype(int), 0]))
        for i, j in zip(edges[::2], edges[1::2]):
            if j - i > 60:
                segments.append((r["speed_cmd"][i:j][:2400], np.convolve(r["v"][i:j], np.ones(3) / 3, "same")[:2400]))
    grid = dict(delay=[1, 2, 3, 4, 5, 6], accel_gain=[3.0, 5.0, 7.0, 10.0], accel_max=[3.0, 4.0, 5.0],
                brake_gain=[3.0, 5.0, 7.0, 10.0], brake_max=[3.0, 4.0, 5.0])
    best, rms = fit_speed_loop(segments[:30], DT, grid)
    out["speed_loop"] = dict(best, rms=rms)
    out["peak_wheel"] = float(max(np.percentile(r["wheel"], 99.9) for r in runs))
    out["peak_true"] = float(max(np.percentile(r["v"][r["good"]], 99.5) for r in runs))
    # Stops.
    stops = []
    for r in runs:
        for i, j in hard_stops(r["t"], r["wheel"], r["speed_cmd"]):
            stops.append(dict(run=r["name"], wheel_start=float(r["wheel"][i]), seconds=float(r["t"][j] - r["t"][i]),
                              wheel_m=float(np.sum(r["wheel"][i:j]) * DT)))
    out["stops"] = stops
    # Grip, by side.
    asked, achieved, ax = [], [], []
    for r, k in zip(runs, keep):
        late = np.interp(r["t"] - STEER_LAG, r["t"], np.clip(r["steer"], -FULL_LOCK, FULL_LOCK))
        asked.append((r["v"] ** 2 * np.tan(late) / WHEELBASE)[k])
        achieved.append((r["v"] * (r["gyro"] - bias))[k])
        ax.append(np.gradient(np.convolve(r["v"], np.ones(7) / 7, "same"), r["t"])[k])
    asked, achieved, ax = map(np.concatenate, (asked, achieved, ax))
    rows = []
    for low, high in zip(GRIP_BINS[:-1], GRIP_BINS[1:]):
        both = (np.abs(asked) >= low) & (np.abs(asked) < high)
        left, right = both & (asked > 0), both & (asked < 0)
        if both.sum() >= 10:
            rows.append(dict(low=low, high=high, n=int(both.sum()), both=float(np.median(np.abs(achieved[both]))),
                             left=float(np.median(achieved[left])) if left.sum() >= 5 else None,
                             right=float(np.median(-achieved[right])) if right.sum() >= 5 else None))
    out["grip"] = rows
    band = (np.abs(asked) >= 3.0) & (np.abs(asked) < 5.0)
    out["grip_by_long_accel"] = [
        dict(low=a, high=b, n=int((band & (ax >= a) & (ax < b)).sum()),
             achieved=float(np.median(np.abs(achieved[band & (ax >= a) & (ax < b)]))))
        for a, b in ((-9, -3), (-3, -1.5), (-1.5, -0.5), (-0.5, 0.5), (0.5, 1.5), (1.5, 9))
        if (band & (ax >= a) & (ax < b)).sum() >= 8]
    return out


def lidar_report(run, self_range: float = 0.45) -> dict:
    """The LiDAR of one run: noise and self hits while parked, dropouts and texture moving.

    Needs no localisation. Parked means /odom under 1 cm/s; a run with no parked
    scans reports no noise.
    """
    from run_analysis.bagread import messages
    parked, moving, ages, stamps = [], [], [], []
    speed, angles = 0.0, None
    for topic, stamp, m in messages(run, ["/odom", "/scan"]):
        if topic == "/odom":
            speed = m.twist.twist.linear.x
            continue
        ranges = np.array(m.ranges, float)
        if angles is None:
            angles = m.angle_min + np.arange(ranges.size) * m.angle_increment
        ages.append(stamp - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
        stamps.append(stamp)
        (parked if abs(speed) < 0.01 else moving).append(ranges)
    out = dict(run=Path(run).name, scans=len(stamps), rate_hz=float(1.0 / np.median(np.diff(stamps))),
               stamp_age_ms=float(1e3 * np.median(ages)), parked_scans=len(parked))
    if moving:
        m = np.array(moving[::4])
        out["moving_no_echo"] = float(1.0 - (np.isfinite(m) & (m > 0.0)).mean())
        out.update({f"moving_{k}": v for k, v in edge_statistics(m).items()})
    if len(parked) >= 50:
        stats = scan_statistics(np.array(parked))
        near, far = stats["median"] < 1.0, stats["median"] > 4.0
        out.update(parked_noise=stats["noise"], parked_noise_near=float(np.nanmedian(stats["std_by_beam"][near])),
                   parked_noise_far=float(np.nanmedian(stats["std_by_beam"][far])),
                   self_hits=[(int(i), round(r, 3)) for i, (a, r) in enumerate(zip(angles, stats["median"]))
                              if np.isfinite(r) and r < self_range and a < -2.2])
    return out


def main(argv=None) -> int:
    from run_analysis.loc import DEFAULT_OUT
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="*", help="run directories located with loc.py")
    ap.add_argument("--loc", default=str(DEFAULT_OUT), help="where loc.py wrote (default %(default)s)")
    ap.add_argument("--json", help="also write the numbers here")
    ap.add_argument("--lidar", action="store_true", help="report each run's LiDAR instead; needs no loc.py")
    ap.add_argument("--fit-stop", action="append", default=[], metavar="SPEED,METRES",
                    help="fit braking to measured stops (true start speed, wall distance) instead; no runs read")
    ap.add_argument("--check-stop", action="append", default=[], metavar="SPEED,METRES",
                    help="with --fit-stop: held-out stops to predict with the best fits")
    args = ap.parse_args(argv)
    if args.fit_stop:
        fit = [tuple(float(v) for v in s.split(",")) for s in args.fit_stop]
        check = [tuple(float(v) for v in s.split(",")) for s in args.check_stop]
        rows = fit_braking(fit, np.arange(0.0, 0.2001, 0.025), np.arange(2.0, 14.01, 1.0), np.arange(2.5, 9.01, 0.25))
        for error, delay, gain, top in rows[:8]:
            held = "; ".join(f"{v0:.2f} m/s: {stop_distance(v0, delay, gain, top):.2f} m (real {d:.2f})" for v0, d in check)
            print(f"delay {delay:.3f} s, gain {gain:.1f}/s, ceiling {top:.2f} m/s^2: error {error:.4f}" + (f" | {held}" if held else ""))
        return 0
    if args.lidar:
        out = [lidar_report(run) for run in args.runs]
        text = json.dumps(out, indent=1)
        print(text)
        if args.json:
            Path(args.json).write_text(text)
        return 0
    out = report([measure_run(run, args.loc) for run in args.runs])
    text = json.dumps(out, indent=1)
    print(text)
    if args.json:
        Path(args.json).write_text(text)
    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    raise SystemExit(main())
