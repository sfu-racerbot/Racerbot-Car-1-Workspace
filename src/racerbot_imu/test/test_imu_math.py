"""IMU conditioning math, the cross-check, the P0.5 fits and the axis
calibration helpers. ROS-free.

Runner: python3 -m pytest src/racerbot_imu/test/test_imu_math.py -v   (unsourced)
Oracles: closed forms (unit factors, rotation matrices, rigid-body kinematics
a_x = -w^2 x in a steady turn), car 2's 2026-08-19 all-zero measurement, and
the rules in docs/superpowers/specs/2026-10-23-neural-racer-plan.md, Phase 0.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np  # noqa: E402
import pytest  # noqa: E402

from racerbot_imu import calibrate_axes, imu_crosscheck, imu_fit, imu_math  # noqa: E402

G0 = imu_math.G0


def _rot(axis, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    if axis == 'z':
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    if axis == 'x':
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


# --- units, axes, validity -------------------------------------------------------

def test_to_si_vesc_units():
    g, a = imu_math.to_si((90.0, 0, 0), (0, 0, 1.0), imu_math.VESC_GYRO_SCALE, imu_math.VESC_ACCEL_SCALE)
    assert g[0] == pytest.approx(math.pi / 2, abs=1e-12)
    assert a[2] == pytest.approx(9.80665, abs=1e-12)


def test_rotate_known_mount():
    """IMU yawed +90 deg on the car: the IMU's +x points along base +y."""
    r = _rot('z', 90)
    assert imu_math.rotate_to_base((1.0, 0.0, 0.0), r) == pytest.approx([0.0, 1.0, 0.0], abs=1e-12)


@pytest.mark.parametrize('m', [np.diag([1.0, 1.0, -1.0]), np.eye(3) * 2, np.zeros((3, 3))])
def test_rotation_must_be_proper(m):
    with pytest.raises(imu_math.ImuSampleError, match='proper'):
        imu_math.check_rotation(m)


def test_all_zero_rejected():
    with pytest.raises(imu_math.ImuSampleError, match='all zero'):
        imu_math.validate((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    imu_math.validate((0.0, 0.0, 0.0), (0.0, 0.0, 1e-6))       # one non-zero field passes


@pytest.mark.parametrize('bad', [math.nan, math.inf])
def test_non_finite_rejected(bad):
    with pytest.raises(imu_math.ImuSampleError, match='non-finite'):
        imu_math.validate((bad, 0.0, 0.0), (0.0, 0.0, 1.0))


def test_wrong_axis_count_rejected():
    with pytest.raises(imu_math.ImuSampleError, match='3 axes'):
        imu_math.validate((0.0, 0.0), (0.0, 0.0, 1.0))


@pytest.mark.parametrize('norm, ok', [(G0, True), (G0 + 0.5, True), (G0 - 0.5, True),
                                      (G0 + 0.51, False), (G0 - 0.51, False), (1.0, False)])
def test_gravity_check_bounds(norm, ok):
    samples = np.tile([0.0, 0.0, norm], (50, 1))
    if ok:
        imu_math.gravity_check(samples, tol=0.5)
    else:
        with pytest.raises(imu_math.ImuSampleError, match='gravity'):
            imu_math.gravity_check(samples, tol=0.5)


def test_gravity_check_catches_upside_down():
    with pytest.raises(imu_math.ImuSampleError, match='from \\+z'):
        imu_math.gravity_check(np.tile([0.0, 0.0, -G0], (10, 1)))


def test_gravity_check_rejects_nan():
    with pytest.raises(imu_math.ImuSampleError, match='finite'):
        imu_math.gravity_check(np.array([[0.0, math.nan, G0]]))


def test_saturation_boundary():
    assert not imu_math.saturated((34.9, 0, 0), (0, 0, 9.8), 34.9, 156.9)       # at the limit
    assert imu_math.saturated((34.91, 0, 0), (0, 0, 9.8), 34.9, 156.9)          # one step past
    assert imu_math.saturated((0, 0, 0), (0, -157.0, 9.8), 34.9, 156.9)


# --- gyro bias ------------------------------------------------------------------------

def _feed(est, seconds, speed=0.0, bias=(0.01, -0.02, 0.03), noise=0.001, t0=0.0, rate=50):
    rng = np.random.default_rng(0)
    done = False
    for k in range(int(seconds * rate) + 1):
        g = np.array(bias) + rng.normal(0, noise, 3)
        done = est.update(t0 + k / rate, g, (0.0, 0.0, G0), speed) or done
    return done


def test_bias_needs_full_window():
    est = imu_math.GyroBiasEstimator(window_sec=2.0)
    assert not _feed(est, 1.9) and est.bias is None
    est = imu_math.GyroBiasEstimator(window_sec=2.0)
    assert _feed(est, 2.0)
    assert est.bias == pytest.approx([0.01, -0.02, 0.03], abs=1e-3)   # mean of the window


def test_one_moving_tick_resets_the_window():
    est = imu_math.GyroBiasEstimator(window_sec=2.0)
    _feed(est, 1.5)
    est.update(1.52, (0, 0, 0), (0, 0, G0), 0.5)          # moving
    assert not _feed(est, 1.5, t0=1.54) and est.bias is None


def test_bias_never_updates_while_moving():
    est = imu_math.GyroBiasEstimator(window_sec=2.0)
    _feed(est, 2.0)
    before = est.bias.copy()
    _feed(est, 5.0, speed=1.0, bias=(0.0, 0.0, 1.0), t0=3.0)
    assert np.array_equal(est.bias, before)


def test_vibrating_window_rejected():
    est = imu_math.GyroBiasEstimator(window_sec=2.0, gyro_std_tol=0.02)
    assert not _feed(est, 2.0, noise=0.05) and est.bias is None


# --- axis calibration ----------------------------------------------------------------------

def test_axes_from_synthetic_mounts():
    """20 random proper mount rotations: gravity and a forward push expressed
    in IMU axes recover each rotation within TOL, with noise added.

    TOL derivation: the push mean of 50 samples at 0.05 m/s^2 noise has
    0.05/sqrt(50) = 0.0071 m/s^2 error per axis on a 1.5 m/s^2 push, i.e.
    0.27 deg per axis; 3 sigma over two axes is 1.15 deg. A real bug (axes
    swapped, sign flipped) is tens of degrees."""
    TOL = 1.15
    rng = np.random.default_rng(1)
    for _ in range(20):
        q = rng.standard_normal(4)
        q /= np.linalg.norm(q)
        w, x, y, z = q
        r_imu_base = np.array([[1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                               [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
                               [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)]])
        still = (r_imu_base @ [0, 0, G0]) + rng.normal(0, 0.02, (200, 3))
        push = (r_imu_base @ [1.5, 0, G0]) + rng.normal(0, 0.05, (50, 3))
        r = imu_math.axes_from_gravity_and_push(still, push)
        err = np.degrees(np.arccos(np.clip((np.trace(r @ r_imu_base) - 1) / 2, -1, 1)))
        assert err < TOL


def test_axes_rejects_parallel_push():
    with pytest.raises(imu_math.ImuSampleError, match='parallel'):
        imu_math.axes_from_gravity_and_push(np.tile([0, 0, G0], (10, 1)), np.tile([0, 0, G0 + 2], (10, 1)))


def test_push_samples_keeps_the_push():
    up = np.array([0.0, 0.0, G0])
    a = np.vstack([np.tile(up, (20, 1)), np.tile(up + [1.0, 0, 0], (10, 1))])
    assert len(calibrate_axes.push_samples(a, up)) == 10


def test_spin_check_sign():
    assert calibrate_axes.spin_check(np.tile([0, 0, 1.0], (10, 1))) == pytest.approx(1.0)
    with pytest.raises(imu_math.ImuSampleError, match='axes are wrong'):
        calibrate_axes.spin_check(np.tile([0, 0, -1.0], (10, 1)))


# --- the cross-check -----------------------------------------------------------------------

def test_agreement_within_tolerance():
    ag = imu_crosscheck.GyroAgreement(0.05, 0.05, 0.5)
    assert ag.update(0.0, 1.00, 1.09)[0]          # tol = 0.05 + 0.05 * 1.0 = 0.10
    ag2 = imu_crosscheck.GyroAgreement(0.05, 0.0, 0.0)
    assert not ag2.update(0.0, 1.00, 1.09)[0]     # without the relative term it faults


def test_disagreement_needs_hold_time():
    ag = imu_crosscheck.GyroAgreement(0.05, 0.0, 0.5)
    assert ag.update(0.0, 0.0, 1.0)[0]
    assert ag.update(0.4, 0.0, 1.0)[0]
    assert not ag.update(0.5, 0.0, 1.0)[0]
    assert ag.update(0.6, 0.0, 0.0)[0]            # agreement again clears it


def test_missing_value_counts_as_disagreeing():
    ag = imu_crosscheck.GyroAgreement(0.05, 0.0, 0.0)
    assert not ag.update(0.0, 0.5, None)[0] and ag.disagreement == math.inf


def test_delay_alignment():
    """Two identical yaw traces, one 30 ms late, agree once the delays are
    given and disagree in a fast turn when they are not."""
    t = np.arange(0, 2, 0.02)
    w = 2.0 * np.sin(2 * math.pi * 1.0 * t)
    a = list(zip(t, w))
    b = list(zip(t + 0.03, w))
    wa = imu_crosscheck.aligned_window_mean(a, 1.0, 0.2, 0.0)
    assert imu_crosscheck.aligned_window_mean(b, 1.0, 0.2, 0.03) == pytest.approx(wa, abs=1e-9)
    assert abs(imu_crosscheck.aligned_window_mean(b, 1.0, 0.2, 0.0) - wa) > 0.1


def test_window_mean_needs_samples():
    assert imu_crosscheck.aligned_window_mean([(0.0, 1.0)], 0.1, 0.2, 0.0) is None


# --- P0.5 fits ------------------------------------------------------------------------------

def test_delay_recovered_from_synthetic():
    rng = np.random.default_rng(0)
    ref = np.convolve(rng.standard_normal(600), np.ones(8) / 8, mode='same')
    delayed = np.roll(ref, 3) + rng.normal(0, 0.01, ref.size)
    assert imu_fit.delay_ticks(ref, delayed) == 3
    assert imu_fit.delay_ticks(delayed, ref) == -3


def test_scale_recovered():
    x = np.linspace(-2, 2, 200)
    assert imu_fit.scale_fit(x, 1.07 * x) == pytest.approx(1.07, abs=0.01)


def test_lever_arm_slope():
    w = np.linspace(0.5, 2.5, 100)
    ax = -w ** 2 * 0.45
    assert imu_fit.lever_arm_slope(w, ax, 0.45) == pytest.approx(1.0, abs=0.02)
    assert imu_fit.lever_arm_slope(w, ax, 0.30) == pytest.approx(1.5, abs=0.02)


def test_two_imu_rigid_body():
    w = np.linspace(0.5, 2.5, 100)
    a_front, a_vesc = -w ** 2 * 0.45, -w ** 2 * 0.15
    assert imu_fit.rigid_body_slope(w, a_front, a_vesc, 0.45, 0.15) == pytest.approx(1.0, abs=0.02)
    assert imu_fit.rigid_body_slope(w, a_front, a_vesc, 0.15, 0.45) == pytest.approx(-1.0, abs=0.02)


def test_primary_choice_rule():
    stats = {'front': {'delay_s': 0.025, 'gyro_noise_driving': 0.001, 'accel_noise_driving': 0.1},
             'vesc': {'delay_s': 0.010, 'gyro_noise_driving': 0.01, 'accel_noise_driving': 0.5}}
    assert imu_fit.choose_primary(stats) == 'vesc'              # delay wins whatever the noise
    stats['vesc']['delay_s'] = 0.020                            # within 10 ms: noise decides
    assert imu_fit.choose_primary(stats) == 'front'
    stats['front'].update(gyro_noise_driving=0.01, accel_noise_driving=0.5)
    stats['vesc']['rigid'] = True                               # tie: the rigid mount
    assert imu_fit.choose_primary(stats) == 'vesc'
    assert imu_fit.choose_primary({'vesc': stats['vesc']}) == 'vesc'
