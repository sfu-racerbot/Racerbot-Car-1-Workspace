"""Car-axis simulated IMU measurements from the gym's true state (ROS-free)."""

from dataclasses import dataclass


GRAVITY = 9.80665  # imu_bridge.convert.G
BNO_X_OFFSET = 0.36  # imu_bridge/config/vehicle_state.yaml
VESC_X_OFFSET = 0.18
FAULTS = ('none', 'stale', 'status_fault', 'bias', 'scale', 'zero')


@dataclass(frozen=True)
class ImuNoise:
    gyro_std: float  # rad/s, per axis
    accel_std: float  # m/s^2, per axis


# Measured run 20261026-194514, bag-relative 2.296746016..4.313826323 s.
# /odom: 102 samples, max |v| = 0 m/s. Population std after axis-mean removal,
# averaged across the three axes. /imu/bno/data: 530 samples (gyro constant
# throughout this window); /imu/vesc/data: 75 samples.
BNO_NOISE = ImuNoise(0.0, 0.0212891782443367)
VESC_NOISE = ImuNoise(0.00130242474454961, 0.0131795356723403)
