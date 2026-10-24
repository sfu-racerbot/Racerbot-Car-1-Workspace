# racerbot_imu

> **Who this is for:** anyone bringing up, checking or calibrating car 1's IMUs.
> **Read first:** [the neural racer plan, Phase 0](../../docs/superpowers/specs/2026-10-23-neural-racer-plan.md#phase-0-imu-ready) — why the IMU needs this before anything uses it.
> **You'll be able to:** get a trustworthy `/imu/data` stream, cross-checked by a second IMU, and know when it is not trustworthy.

Car 1 has two IMUs: the VESC's onboard one and a separate unit at the front bumper. This package turns their raw streams into one stream a controller can trust, or refuses.

It publishes nothing that can move the car. It is support code, not driving code.

## Highlights

- **Fixes the units.** `vesc_driver` publishes the VESC's raw IMU values in deg/s and g, where ROS expects rad/s and m/s² (REP-145). That's off by 57× and 9.8×. The conditioner converts once, with the scales in config.
- **Refuses bad data instead of passing it on.** These all stop the data stream rather than publishing zeros, so a consumer sees a stale stream instead of a "car not turning":
  - all six fields exactly zero (the IMU switched off in VESC firmware, as on car 2)
  - a failed gravity check (wrong units or mount)
  - a stale input
  - saturated samples (a wall strike at the bumper)
- **Two IMUs check each other.** Yaw rate is the same everywhere on a rigid car, so the two gyros must agree. A disagreement lasting 0.5 s is a fault. With two sensors there is no majority, so the car stops rather than guessing which one is right.
- **Calibration helpers.** `calibrate_axes` finds each IMU's mount rotation from a still recording and a hand push. `imu_fit.py` measures delay, scale and the front-bumper lever arm against LiDAR scan matching.

## Running

**Terminal 1** — bringup (the VESC driver publishes `/sensors/imu/raw`):

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

**Terminal 2** — both conditioners and the selector:

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch racerbot_imu imu_launch.py
```

Use `front:=false` to run the VESC's IMU alone. That's the right mode if the front unit is wired *through* the VESC, which reports one IMU at a time.

**Working when:** with the car still for 2 s, `ros2 topic echo /imu/status --once` shows `"state": "ok"`, and `/imu/data` runs at about 50 Hz.

**If it doesn't:** `/imu/vesc/status` and `/imu/front/status` each name their problem:

- `all zero` means the IMU is disabled in VESC Tool → App Settings → IMU, or not fitted
- `gravity check` means the units or axes are wrong
- `input stale` means nothing is publishing the raw topic

## Before car 1 can use it

`config/imu_rb1.yaml` holds placeholders, each marked `TODO(P0.x)`, until Phase 0 measures them on the car:

- each IMU's mount rotation (`calibrate_axes`)
- the front unit's topic and units
- which IMU is primary
- both delays and the cross-check tolerance

## Tests

```bash
python3 -m pytest src/racerbot_imu/test/test_imu_math.py -v          # unsourced
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 -m pytest src/racerbot_imu/test/ -v                          # all 49, sourced
```

**Working when:** 36 unsourced, then 49 sourced, pass.
