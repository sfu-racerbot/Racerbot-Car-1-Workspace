"""IMU conditioning math: units, axes, bias, validity. No rclpy import.

Why this exists (docs/superpowers/specs/2026-10-23-neural-racer-plan.md, A3
and Phase 0): vesc_driver copies the VESC's raw IMU values straight into a
sensor_msgs/Imu, and its own packet parser gives them in deg/s and g, where
REP-145 says rad/s and m/s^2. On car 2 the IMU also read all zeros (switched
off in the VESC firmware). Everything here turns a raw stream into one a
controller can trust, or refuses it.
"""
import math
from dataclasses import dataclass

import numpy as np

G0 = 9.80665                       # m/s^2, standard gravity
VESC_GYRO_SCALE = math.pi / 180.0  # deg/s -> rad/s
VESC_ACCEL_SCALE = G0              # g -> m/s^2


class ImuSampleError(ValueError):
    """This sample, or this IMU, cannot be trusted."""


@dataclass(frozen=True)
class ImuSample:
    t: float                # s, receive stamp
    gyro: np.ndarray        # (3,) rad/s, base_link axes
    accel: np.ndarray       # (3,) m/s^2 specific force, base_link axes


def validate(gyro_raw, accel_raw) -> None:
    """Raise for a sample that carries no information.

    All six fields exactly 0.0 is what a VESC with its IMU disabled sends
    (measured on car 2, 2026-08-19). A working IMU at rest still reads
    gravity on some axis, so this never fires on real data.
    """
    g = np.asarray(gyro_raw, dtype=np.float64)
    a = np.asarray(accel_raw, dtype=np.float64)
    if g.shape != (3,) or a.shape != (3,):
        raise ImuSampleError('gyro and accel must each have 3 axes')
    if not (np.all(np.isfinite(g)) and np.all(np.isfinite(a))):
        raise ImuSampleError('non-finite IMU field')
    if not (np.any(g != 0.0) or np.any(a != 0.0)):
        raise ImuSampleError('all zero: IMU disabled in firmware, or not fitted')


def to_si(gyro_raw, accel_raw, gyro_scale: float, accel_scale: float):
    """Raw axes times the input scales (VESC: pi/180 and G0)."""
    return (np.asarray(gyro_raw, dtype=np.float64) * gyro_scale,
            np.asarray(accel_raw, dtype=np.float64) * accel_scale)


def check_rotation(r) -> np.ndarray:
    """A proper rotation (orthonormal, determinant +1) as a 3x3 array."""
    m = np.asarray(r, dtype=np.float64).reshape(3, 3)
    if not np.allclose(m @ m.T, np.eye(3), atol=1e-6) or abs(np.linalg.det(m) - 1.0) > 1e-6:
        raise ImuSampleError('r_base_imu must be a proper rotation (orthonormal, det +1)')
    return m


def rotate_to_base(vec, r_base_imu) -> np.ndarray:
    return check_rotation(r_base_imu) @ np.asarray(vec, dtype=np.float64)


def saturated(gyro_si, accel_si, gyro_limit: float, accel_limit: float) -> bool:
    """Any axis beyond its range (in the sensor's own axes). At the front
    bumper, a wall strike is exactly when this happens."""
    return bool(np.any(np.abs(gyro_si) > gyro_limit) or np.any(np.abs(accel_si) > accel_limit))


def gravity_check(accel_samples, tol: float = 0.5, max_tilt_deg: float = 10.0) -> np.ndarray:
    """Mean specific force over a stationary window, which must be gravity
    pointing up (+z in base_link): |a| within G0 +/- tol, and within
    max_tilt_deg of +z. Returns the mean vector."""
    a = np.asarray(accel_samples, dtype=np.float64).reshape(-1, 3)
    if a.shape[0] == 0 or not np.all(np.isfinite(a)):
        raise ImuSampleError('gravity check needs finite samples')
    mean = a.mean(axis=0)
    norm = float(np.linalg.norm(mean))
    if not abs(norm - G0) <= tol + 1e-9:          # epsilon: exactly at tol passes
        raise ImuSampleError(f'gravity check: |a| = {norm:.3f} m/s^2, expected {G0:.3f} +/- {tol}')
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, mean[2] / norm))))
    if tilt > max_tilt_deg:
        raise ImuSampleError(f'gravity check: up axis is {tilt:.1f} deg from +z (axes or mount wrong)')
    return mean


class GyroBiasEstimator:
    """Mean gyro over a stationary window.

    Stationary means |odom speed| < speed_tol for the whole window and every
    gyro axis's standard deviation < gyro_std_tol. Re-estimates each time the
    car is still for a full window; never updates while it moves.
    """

    def __init__(self, window_sec: float = 2.0, speed_tol: float = 0.01,
                 gyro_std_tol: float = 0.02):
        self.window_sec = float(window_sec)
        self.speed_tol = float(speed_tol)
        self.gyro_std_tol = float(gyro_std_tol)
        self._t, self._g, self._a = [], [], []
        self.bias = None
        self.last_window_accel = None

    def update(self, t: float, gyro_si, accel_si, odom_speed: float) -> bool:
        """Feed one sample; True when this sample completed a new estimate."""
        if not (math.isfinite(odom_speed) and abs(odom_speed) < self.speed_tol):
            self._t, self._g, self._a = [], [], []
            return False
        self._t.append(t)
        self._g.append(np.asarray(gyro_si, dtype=np.float64))
        self._a.append(np.asarray(accel_si, dtype=np.float64))
        if self._t[-1] - self._t[0] < self.window_sec - 1e-9:
            return False
        g = np.array(self._g)
        a = np.array(self._a)
        self._t, self._g, self._a = [], [], []
        if np.any(g.std(axis=0) >= self.gyro_std_tol):
            return False            # vibrating or being handled: not a clean window
        self.bias = g.mean(axis=0)
        self.last_window_accel = a
        return True


def axes_from_gravity_and_push(still_accel, push_accel) -> np.ndarray:
    """r_base_imu from two recordings, in the IMU's own axes:

      still_accel: samples with the car still and level (gravity = up)
      push_accel:  samples during a gentle straight push forward

    Up is the mean still vector. Forward is the mean push vector with
    gravity removed and its up component projected out (Gram-Schmidt).
    Left = up x forward. Rows of the result are base_link's x, y, z
    expressed in IMU axes, so r @ v_imu = v_base.
    """
    up = np.asarray(still_accel, dtype=np.float64).reshape(-1, 3).mean(axis=0)
    push = np.asarray(push_accel, dtype=np.float64).reshape(-1, 3).mean(axis=0) - up
    if np.linalg.norm(up) < 1.0:
        raise ImuSampleError('still recording shows no gravity')
    z = up / np.linalg.norm(up)
    fwd = push - (push @ z) * z
    if np.linalg.norm(fwd) < 0.2 * max(np.linalg.norm(push), 1e-9) or np.linalg.norm(fwd) < 0.05:
        raise ImuSampleError('push is parallel to gravity or too gentle to give a forward axis')
    x = fwd / np.linalg.norm(fwd)
    y = np.cross(z, x)
    return np.stack([x, y, z])
