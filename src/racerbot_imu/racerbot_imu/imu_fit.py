"""Offline IMU analysis for Phase 0's recordings (P0.5). No rclpy import.

Each function takes plain arrays already resampled onto one time base, so it
works on any recording source: race_diagnostics run directories, a bag, or a
simulator. The reference yaw rate is the LiDAR scan-matched one from
tools/run_analysis/scan_motion.py -- a measurement independent of both IMUs.
"""
import numpy as np


def delay_ticks(reference, delayed, max_lag: int = 20) -> int:
    """Lag (in samples) by which `delayed` trails `reference`: the shift that
    maximises their normalised cross-correlation. Positive = delayed lags."""
    a = np.asarray(reference, dtype=np.float64)
    b = np.asarray(delayed, dtype=np.float64)
    if a.shape != b.shape or a.size <= 2 * max_lag:
        raise ValueError('need two equal-length traces longer than 2 * max_lag')
    a = a - a.mean()
    b = b - b.mean()
    best, best_lag = -np.inf, 0
    for lag in range(-max_lag, max_lag + 1):
        if lag >= 0:
            x, y = a[:a.size - lag], b[lag:]
        else:
            x, y = a[-lag:], b[:b.size + lag]
        denom = np.sqrt((x * x).sum() * (y * y).sum())
        c = (x * y).sum() / denom if denom > 0 else -np.inf
        if c > best:
            best, best_lag = c, lag
    return best_lag


def scale_fit(reference, measured) -> float:
    """Least-squares slope of measured against reference, through the origin."""
    x = np.asarray(reference, dtype=np.float64)
    y = np.asarray(measured, dtype=np.float64)
    denom = (x * x).sum()
    if denom <= 0:
        raise ValueError('reference has no signal')
    return float((x * y).sum() / denom)


def lever_arm_slope(yaw_rate, accel_x, offset_x: float, base_accel_x=None) -> float:
    """In steady turns, an IMU offset_x ahead of the rear axle reads
    a_x = base_accel_x - w^2 * offset_x. Returns the slope of the measured
    a_x against that prediction: ~1 when the offset and axes are right."""
    w = np.asarray(yaw_rate, dtype=np.float64)
    base = 0.0 if base_accel_x is None else np.asarray(base_accel_x, dtype=np.float64)
    predicted = -w * w * offset_x
    return scale_fit(predicted, np.asarray(accel_x, dtype=np.float64) - base)


def rigid_body_slope(yaw_rate, accel_x_a, accel_x_b, x_a: float, x_b: float) -> float:
    """Two IMUs on one rigid car, at x_a and x_b along it: in steady turns
    a_x,a - a_x,b = -w^2 (x_a - x_b). Slope ~1 when both mounts and both
    axis calibrations are right; -1 means the offsets are swapped."""
    w = np.asarray(yaw_rate, dtype=np.float64)
    diff = np.asarray(accel_x_a, dtype=np.float64) - np.asarray(accel_x_b, dtype=np.float64)
    return scale_fit(-w * w * (x_a - x_b), diff)


def choose_primary(stats: dict, delay_margin_s: float = 0.010) -> str:
    """Pick the IMU that feeds the network (plan, P0.5):

      1. lower delay wins, if the two differ by more than delay_margin_s;
      2. otherwise lower noise *while driving* (gyro, then accel) wins;
      3. on a tie, the one marked more rigidly mounted.

    stats: {name: {'delay_s', 'gyro_noise_driving', 'accel_noise_driving', 'rigid'}}
    """
    if len(stats) == 1:
        return next(iter(stats))
    (na, a), (nb, b) = sorted(stats.items())
    if abs(a['delay_s'] - b['delay_s']) > delay_margin_s:
        return na if a['delay_s'] < b['delay_s'] else nb
    ka = (a['gyro_noise_driving'], a['accel_noise_driving'])
    kb = (b['gyro_noise_driving'], b['accel_noise_driving'])
    if ka != kb:
        return na if ka < kb else nb
    return na if a.get('rigid', False) and not b.get('rigid', False) else (
        nb if b.get('rigid', False) else na)
