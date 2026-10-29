"""Car-axis simulated IMU measurements from the gym's true state (ROS-free)."""

from dataclasses import dataclass

import numpy as np

from imu_bridge.imu_health import ImuHealth


GRAVITY = 9.80665  # imu_bridge.convert.G
BNO_X_OFFSET = 0.36  # imu_bridge/config/vehicle_state.yaml
VESC_X_OFFSET = 0.18
FAULTS = ('none', 'stale', 'status_fault', 'bias', 'scale', 'zero')


@dataclass(frozen=True)
class ImuNoise:
    gyro_std: float  # rad/s, per axis
    accel_std: float  # m/s^2, per axis


# Measured run 20261026-194514, bag-relative 16.0..18.0 s, within the
# initial wheel-stationary stretch (/odom: 100 samples, max |v| = 0 m/s).
# Population std after axis-mean removal, averaged across the three axes.
# /imu/bno/data: 505 samples; /imu/vesc/data: 100 samples. This later
# stationary window avoids the initial constant BNO gyro readings.
BNO_NOISE = ImuNoise(0.00190187904182856, 0.0217090362070021)
VESC_NOISE = ImuNoise(0.0012783737292913, 0.0138812011098596)


def ideal_reading(speed, prev_speed, yaw_rate, prev_yaw_rate, dt, x_offset):
    """Specific force and gyro at an IMU ahead of the rear-axle base_link.

    All values are SI in car axes (x forward, y left, z up). Derivatives
    use the simulated tick duration, never the ROS scheduling interval.
    """
    if not np.all(np.isfinite([speed, prev_speed, yaw_rate, prev_yaw_rate, dt, x_offset])):
        raise ValueError('IMU inputs must be finite')
    if dt <= 0:
        raise ValueError('IMU dt must be positive')
    return ((float((speed - prev_speed) / dt - x_offset * yaw_rate**2),
             float(speed * yaw_rate + x_offset * (yaw_rate - prev_yaw_rate) / dt),
             GRAVITY), (0.0, 0.0, float(yaw_rate)))


@dataclass(frozen=True)
class ImuSample:
    bno_acc: tuple
    bno_gyro: tuple
    vesc_acc: tuple
    vesc_gyro: tuple


class SimImu:
    """One controlled car's primary BNO and VESC IMUs, with injectable faults."""

    def __init__(self, rng, bno_noise=BNO_NOISE, vesc_noise=VESC_NOISE,
                 fault='none', bias=0.1, scale=1.1):
        self.rng = rng
        self.bno_noise, self.vesc_noise = bno_noise, vesc_noise
        self.bias, self.scale = bias, scale
        self.fault = fault
        self._previous = None
        self._health = ImuHealth('bno')
        state, reason = self._health.status(0.0)
        self._status = dict(state=state, reason=reason, imu='bno')

    @property
    def fault(self):
        return self._fault

    @fault.setter
    def fault(self, value):
        if value not in FAULTS:
            raise ValueError(f'unknown IMU fault {value!r}; expected one of {FAULTS}')
        self._fault = value

    def step(self, speed, yaw_rate, dt):
        previous_speed, previous_rate = self._previous or (speed, yaw_rate)
        readings = [ideal_reading(speed, previous_speed, yaw_rate, previous_rate, dt, offset)
                    for offset in (BNO_X_OFFSET, VESC_X_OFFSET)]
        self._previous = (speed, yaw_rate)
        if self.fault == 'stale':
            return None

        if self.fault == 'zero':
            zero = (0.0, 0.0, 0.0)
            self._health.sample(0.0, zero, zero, zero, zero, speed, True)
            state, reason = self._health.status(0.0)
            self._status = dict(state=state, reason=reason, imu='bno')
            return ImuSample(zero, zero, zero, zero)

        output = []
        for (acc, gyro), noise in zip(readings, (self.bno_noise, self.vesc_noise)):
            acc = np.array(acc) + self.rng.normal(0.0, noise.accel_std, 3)
            gyro = np.array(gyro) + self.rng.normal(0.0, noise.gyro_std, 3)
            if self.fault == 'bias':
                gyro[2] += self.bias
            elif self.fault == 'scale':
                gyro[2] *= self.scale
            output.extend((tuple(float(v) for v in acc), tuple(float(v) for v in gyro)))
        # Startup calibration/gravity are known by construction. As on the
        # real hardware, health cannot detect a biased or mis-scaled gyro.
        self._status = dict(state='ok', reason=None, imu='bno')
        if self.fault == 'status_fault':
            self._status.update(state='fault', reason='simulated fault (imu_fault=status_fault)')
        return ImuSample(*output)

    def status(self):
        """Last sample's verdict; a stale IMU leaves its last status latched."""
        return self._status.copy()
