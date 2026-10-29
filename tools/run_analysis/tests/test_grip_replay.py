"""grip_replay: what gap_follow's adaptive grip would have chosen on the floor.

The analyses in grip_replay.py run on recorded bags and need ROS sourced
(rosbag2_py), but replay/summarize are pure: these tests run unsourced.

    python3 -m pytest tools/run_analysis/tests -q

Oracles: synthetic runs whose answer is built in from the parameters. The
parameter values are the shipped ones from src/gap_follow/config/
gap_follow.yaml (max_lateral_accel 2.5, adaptive_grip_ceiling 3.0,
adaptive_grip_rise_per_s 0.2, adaptive_grip_cut_factor 0.85,
adaptive_grip_hold_ratio 0.95, adaptive_grip_slip_ratio 0.90,
adaptive_grip_judge_fraction 0.8, adaptive_grip_steering_lag_sec 0.15,
adaptive_grip_max_steering_change 0.05, imu_timeout_sec 0.1,
odom_timeout_sec 0.5, wheelbase 0.36) and the rules are gap_logic's own
(grip_ratio/AdaptiveGrip.update/SteeringLag.step docstrings), so every
expected value below has a closed form, stated per test.
"""
import math
import os
import sys

import numpy as np
import pytest

_TOOLS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_WS = os.path.dirname(_TOOLS)
for _p in (_TOOLS, os.path.join(_WS, 'src')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from run_analysis import grip_replay  # noqa: E402
from gap_follow import gap_logic  # noqa: E402

WHEELBASE = 0.36          # m, gap_follow.yaml
STEER = 0.2               # rad: judged (|0.2| > 0.05), demand ~5.1 > 0.8 * 2.5
SPEED = 3.0               # m/s: judged (>= 1.0 m/s)
BASE = 2.5                # m/s^2, max_lateral_accel
CEILING = 3.0             # m/s^2, adaptive_grip_ceiling
RISE = 0.2                # m/s^2/s, adaptive_grip_rise_per_s
LAG = 0.15                # s, adaptive_grip_steering_lag_sec
DT = 0.025                # tick period: 40 Hz control ticks
GYRO_DT = 0.005           # 200 Hz gyro

# Closed form: at constant steering and a yaw rate that exactly makes the
# turn, the grip ratio is 1.0 (gap_logic.grip_ratio), so tick t (once judged)
# sets limit = base + rise * (t - LAG): each judged tick contributes its dt of
# judged time. Judging starts at tick LAG + DT (the first tick whose lag
# window is full), so the ceiling is reached at
# t = LAG + (CEILING - BASE) / RISE = 0.15 + 2.5 = 2.65 s, checked within one
# tick below.
LAG_FULL_T = LAG                     # tick time when the lag window is full
CEILING_T = LAG_FULL_T + (CEILING - BASE) / RISE

PARAMS = {
    'max_lateral_accel': BASE,
    'adaptive_grip_ceiling': CEILING,
    'adaptive_grip_rise_per_s': RISE,
    'adaptive_grip_cut_factor': 0.85,
    'adaptive_grip_hold_ratio': 0.95,
    'adaptive_grip_slip_ratio': 0.90,
    'adaptive_grip_judge_fraction': 0.8,
    'adaptive_grip_steering_lag_sec': LAG,
    'adaptive_grip_max_steering_change': 0.05,
    'imu_timeout_sec': 0.1,
    'odom_timeout_sec': 0.5,
    'wheelbase': WHEELBASE,
}


def _yaw_exact(speed, steering):
    """The yaw rate of a perfectly gripping car: speed * tan(steer) / L."""
    return speed * math.tan(steering) / WHEELBASE


def _synthetic(duration, steer_fn, speed_fn, yaw_fn):
    """A full synthetic run: 40 Hz ticks, 200 Hz gyro, odom at the gyro rate."""
    ticks = np.arange(0.0, duration, DT)
    gyro_t = np.arange(0.0, duration, GYRO_DT)
    steer = np.asarray([steer_fn(t) for t in ticks], float)
    speed = np.asarray([speed_fn(t) for t in gyro_t], float)
    gyro = np.column_stack([gyro_t, [yaw_fn(t) for t in gyro_t]])
    odom = np.column_stack([gyro_t, speed])
    return np.column_stack([ticks, steer]), gyro, odom


def _scene(duration=20.0):
    """Perfect grip at STEER/SPEED: ratio exactly 1.0 (closed form above)."""
    return _synthetic(
        duration, lambda t: STEER, lambda t: SPEED,
        lambda t: _yaw_exact(SPEED, STEER))


def test_perfect_grip_rises_to_ceiling_formulated():
    # oracle: ratio = yaw*speed / (speed^2 tan(steer)/L) = 1.0 exactly for the
    # constructed yaw; limit = base + rise * (t - LAG) on judged ticks, so the
    # ceiling is reached at CEILING_T and the run never cuts.
    ticks, gyro, odom = _scene()
    r = grip_replay.replay(ticks, gyro, odom, PARAMS)
    assert r['restarts'] == 0
    n_expected = int(round(20.0 / DT)) - 1          # first tick is skipped: no dt
    assert len(r['t']) == n_expected
    assert np.all(r['trusted'])
    judged_t = r['t'][r['judged']]
    # judging starts exactly when the lag window is full, one tick after LAG
    assert judged_t[0] == pytest.approx(LAG_FULL_T + DT, abs=1e-6)
    # the filtered ratio is exactly 1.0 wherever judged (constant input)
    assert np.allclose(r['ratio'][r['judged']], 1.0, rtol=1e-9)
    # still at the base at/just before the first judging tick
    at = np.searchsorted(r['t'], LAG_FULL_T)
    assert r['limit'][at] == pytest.approx(BASE, abs=1e-9)
    # ceiling reached exactly at CEILING_T and never before
    idx = int(np.searchsorted(r['t'], CEILING_T, side='left'))
    assert r['t'][idx] == pytest.approx(CEILING_T, abs=1e-6)
    assert r['limit'][idx] == pytest.approx(CEILING, abs=1e-6)
    assert r['limit'][idx - 1] < CEILING
    assert np.all(np.diff(r['limit']) >= -1e-12)    # never cuts
    assert r['limit'][-1] == pytest.approx(CEILING, abs=1e-6)
    # only the lag window before the first judged tick is unjudged
    assert r['judged'].sum() == len(r['t']) - round(LAG / DT)


def test_sliding_cut_never_below_base():
    # oracle: perfect grip until t = 4 s, then yaw rate x 0.85. The filtered
    # ratio decays as 0.85 + 0.15 exp(-a/tau) (gap_logic.LowPass) and crosses
    # the shipped slip_ratio 0.90 at a = tau ln 3 = 0.3296 s, so the first cut
    # is to cut_factor * ceiling = 2.55 (> base), and the next tick's cut
    # clamps at the base 2.5: the run has exactly two drops and the limit ends
    # exactly at the base, never below it.
    yaw_fn = lambda t: _yaw_exact(SPEED, STEER) * (1.0 if t < 4.0 else 0.85)
    ticks, gyro, odom = _synthetic(12.0, lambda t: STEER, lambda t: SPEED, yaw_fn)
    r = grip_replay.replay(ticks, gyro, odom, PARAMS)
    # ceiling first, then one cut to cut_factor * ceiling...
    assert r['limit'].max() == pytest.approx(CEILING, abs=1e-6)
    cut_idx = np.nonzero(np.diff(r['limit']) < -1e-9)[0]
    assert len(cut_idx) == 2
    assert r['limit'][cut_idx[0] + 1] == pytest.approx(0.85 * CEILING, abs=1e-6)
    # ...then a cut clamped at the base, where it stays
    assert r['limit'][cut_idx[1] + 1] == pytest.approx(BASE, abs=1e-9)
    assert np.all(r['limit'][cut_idx[1] + 1:] == pytest.approx(BASE, abs=1e-9))
    assert r['limit'].min() >= BASE - 1e-12
    s = grip_replay.summarize(r, BASE)
    assert s['cuts'] == 2


def test_gyro_gap_restarts_limit_at_base_at_first_untrusted_tick():
    # oracle: perfect grip until 4 s, no gyro samples for 0.5 s, then perfect
    # again. A tick is untrusted when the latest sample is older than
    # imu_timeout_sec = 0.1 (the bag has no /imu/status, so staleness is the
    # only gate): the first untrusted tick is the first tick with
    # t - 3.995 >= 0.1, i.e. t = 4.1, and the node's rule restarts the limit
    # at the base there. Judging resumes at the first trusted tick and the
    # limit is back at the ceiling after another (CEILING - BASE) / RISE of
    # judged ticks. Exactly one drop (ceiling -> base) in the whole run.
    duration = 12.0
    gap = (4.0, 4.5)
    gyro_t = np.arange(0.0, duration, GYRO_DT)
    keep = ~((gyro_t >= gap[0]) & (gyro_t < gap[1]))
    gyro_t = gyro_t[keep]
    yaw = _yaw_exact(SPEED, STEER)
    gyro = np.column_stack([gyro_t, np.full(len(gyro_t), yaw)])
    ticks = np.arange(0.0, duration, DT)
    drive = np.column_stack([ticks, np.full(len(ticks), STEER)])
    odom = np.column_stack([np.arange(0.0, duration, GYRO_DT),
                            np.full(int(duration / GYRO_DT), SPEED)])
    r = grip_replay.replay(drive, gyro, odom, PARAMS)
    t = r['t']
    trust = r['trusted']
    # the first untrusted tick is the first whose age (t - 3.995) reaches 0.1 s:
    # 4.075 - 3.995 = 0.080 (trusted), 4.1 - 3.995 = 0.105 (untrusted). The tick
    # grid is not exact in binary, so the index is taken from the trust flags,
    # not from a searchsorted on the float 4.1.
    first_untrusted = int(np.argmax(~trust))
    assert t[first_untrusted] == pytest.approx(4.1, abs=1e-6)
    assert t[first_untrusted - 1] == pytest.approx(4.075, abs=1e-6)
    assert np.all(trust[:first_untrusted])
    # every untrusted tick lies inside the gap, none after it
    assert np.all(~trust[first_untrusted:int(np.searchsorted(t, gap[1], 'left'))])
    assert np.all(trust[int(np.searchsorted(t, gap[1], 'left')):])
    # the limit restarts exactly at the base at the first untrusted tick
    assert r['limit'][first_untrusted] == pytest.approx(BASE, abs=1e-9)
    assert r['limit'][first_untrusted - 1] == pytest.approx(CEILING, abs=1e-6)
    # no judging while untrusted
    assert not np.any(r['judged'][~trust])
    assert np.all(np.isnan(r['ratio'][~trust]))
    assert np.all(np.isnan(r['demand'][~trust]))
    # back at the ceiling exactly RECOVER_T after judging resumes
    recover_t = gap[1] - DT + (CEILING - BASE) / RISE
    idx = int(np.searchsorted(t, recover_t, side='left'))
    assert r['t'][idx] == pytest.approx(recover_t, abs=1e-6)
    assert r['limit'][idx] == pytest.approx(CEILING, abs=1e-6)
    assert r['limit'][idx - 1] < CEILING
    s = grip_replay.summarize(r, BASE)
    assert s['cuts'] == 1


def test_steering_step_not_judged_for_one_lag_window():
    # oracle: a step to 0.30 rad (a change of 0.1 > max_steering_change 0.05)
    # at t = 3.0 makes every tick with t <= 3.0 + lag (0.15) unjudged: that
    # tick's lag window still holds the old steering, so SteeringLag.step
    # returns NaN (|0.30 - 0.20| > 0.05). At the next tick the window is
    # wholly in the new steering (|0.30 - 0.30| <= 0.05) and judging resumes.
    # The limit stays where it is through the window and the car holds the new
    # turn (ratio 1.0 again), so the run never cuts.
    ticks = np.arange(0.0, 8.0, DT)
    steer_fn = lambda t: STEER if t <= 3.0 else 2.0 * STEER
    gyro_t = np.arange(0.0, 8.0, GYRO_DT)
    gyro = np.column_stack(
        [gyro_t, [_yaw_exact(SPEED, steer_fn(t)) for t in gyro_t]])
    odom = np.column_stack([gyro_t, np.full(len(gyro_t), SPEED)])
    drive = np.column_stack([ticks, [steer_fn(t) for t in ticks]])
    r = grip_replay.replay(drive, gyro, odom, PARAMS)
    t = r['t']
    # perfect grip up to the step: limit at the ceiling from CEILING_T on
    assert r['limit'][int(np.searchsorted(t, CEILING_T, side='left'))] == \
        pytest.approx(CEILING, abs=1e-6)
    # unjudged for the whole lag window after the step
    window = (t > 3.0 + 1e-12) & (t <= 3.0 + LAG + 1e-12)
    assert np.any(window)
    assert not np.any(r['judged'][window])
    assert np.all(np.isnan(r['ratio'][window]))
    assert np.all(np.isnan(r['demand'][window]))
    assert np.all(r['limit'][window] == pytest.approx(CEILING, abs=1e-6))
    # and judged again immediately after, no cut (the car holds the new turn)
    after = t > 3.0 + LAG + DT
    assert np.all(r['judged'][after])
    assert np.all(np.diff(r['limit'][after]) >= -1e-12)
    assert grip_replay.summarize(r, BASE)['cuts'] == 0


def test_not_judged_below_one_mps_or_near_centre_steering():
    # ground rule, grip_ratio (gap_logic.grip_ratio): NaN, so not judged, for
    # speed < 1.0 m/s or |steering| < 0.05 rad.
    # Case A: |steering| = 0.049 < 0.05, but at 4.0 m/s the demand
    # (16 tan(0.049)/0.36 = 2.18) is above judge_fraction * limit = 2.0, so
    # the demand gate passes and grip_ratio's steering rule is what blocks
    # judging.
    speed_hi = 4.0
    ticks, gyro, odom = _synthetic(
        5.0, lambda t: 0.049, lambda t: speed_hi,
        lambda t: _yaw_exact(speed_hi, 0.049))
    r = grip_replay.replay(ticks, gyro, odom, PARAMS)
    assert not np.any(r['judged'])
    assert np.all(np.isnan(r['ratio']))
    assert np.all(r['limit'] == pytest.approx(BASE, abs=1e-9))
    # Case B: speed 0.9 < 1.0 m/s at STEER: the ratio divides speed^2, so
    # grip_ratio is NaN there too and nothing is judged.
    speed_lo = 0.9
    ticks, gyro, odom = _synthetic(
        5.0, lambda t: STEER, lambda t: speed_lo,
        lambda t: _yaw_exact(speed_lo, STEER))
    r = grip_replay.replay(ticks, gyro, odom, PARAMS)
    assert not np.any(r['judged'])
    assert np.all(r['limit'] == pytest.approx(BASE, abs=1e-9))


def test_replay_calls_gap_logic_not_a_copy(monkeypatch):
    # parity: the test swaps in a counting wrapper around the real
    # gap_logic.adaptive_grip_step; the count must equal the number of trusted
    # ticks. A copy in grip_replay would never be patched and so would record
    # zero calls where every tick is trusted.
    calls = []
    real = gap_logic.adaptive_grip_step

    def counting(*args):
        calls.append(1)
        return real(*args)

    monkeypatch.setattr(gap_logic, 'adaptive_grip_step', counting)
    ticks, gyro, odom = _scene(5.0)
    r = grip_replay.replay(ticks, gyro, odom, PARAMS)
    assert len(calls) == int(r['trusted'].sum())
    assert len(calls) > 0


def test_summarize_bands_and_fractions_closed_form():
    # oracle: a hand-built result, described exactly:
    # t = 0..4 s, 1 s ticks; ticks 2 and 3 judged (ratio 1.0 / 0.9, demands
    # 1.2 and 2.6, of the bands 1.0-1.5 and 2.5-3.0); limits 2.5, 2.5, 2.6,
    # 2.6, 2.5 -> one cut (tick 4). Cornering = demand >= 0.5 * base = 1.25,
    # so only tick 3 is cornering (limit 2.6) and the cornering mean is 2.6.
    # time above base = the intervals whose starting tick is above base
    # (t=2 and t=3, 1 s each) = 2.0 s. Judged/trusted fractions 3/5 each.
    r = {
        't': np.array([0.0, 1.0, 2.0, 3.0, 4.0]),
        'limit': np.array([2.5, 2.5, 2.6, 2.6, 2.5]),
        'ratio': np.array([np.nan, np.nan, 1.0, 0.9, np.nan]),
        'judged': np.array([False, False, True, True, True]),
        'trusted': np.array([False, False, True, True, True]),
        'speed': np.array([0.0, 0.0, 3.0, 3.5, 4.0]),
        'demand': np.array([np.nan, np.nan, 1.2, 2.6, np.nan]),
        'restarts': 0,
    }
    s = grip_replay.summarize(r, BASE)
    assert s['ticks'] == 5
    assert s['judged_fraction'] == pytest.approx(3.0 / 5.0)
    assert s['trusted_fraction'] == pytest.approx(3.0 / 5.0)
    assert s['limit_max'] == pytest.approx(2.6, abs=1e-9)
    assert s['limit_mean_when_cornering'] == pytest.approx(2.6, abs=1e-9)
    assert s['time_above_base_s'] == pytest.approx(2.0, abs=1e-9)
    assert s['cuts'] == 1
    bands = s['ratio_by_demand_band']
    assert set(bands) == {'0.3-1.0', '1.0-1.5', '1.5-2.0', '2.0-2.5',
                          '2.5-3.0', '3.0-5.0'}
    n, med = bands['1.0-1.5']
    assert n == 1 and med == pytest.approx(1.0, abs=1e-9)
    n, med = bands['2.5-3.0']
    assert n == 1 and med == pytest.approx(0.9, abs=1e-9)
    for band in ('0.3-1.0', '1.5-2.0', '2.0-2.5', '3.0-5.0'):
        assert bands[band][0] == 0
