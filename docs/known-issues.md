# Known issues: workspace discrepancies and doc problems

> **Who this is for:** anyone with an afternoon to fix things. Each entry is something found wrong, stale or inconsistent while writing the workspace docs for sfuracerbot.ca (October 2026), and left for later on purpose.
> **Read first:** nothing.
> **What's in it:** one entry per problem: what is wrong, the evidence, and the suggested fix.

Each entry was checked against car 1 at `881c409` unless it says otherwise. Car 2 is [Racerbot-Car-2-Workspace](https://github.com/sfu-racerbot/Racerbot-Car-2-Workspace) at `779c060`. When you fix one, delete its entry in the same commit.

## Code and config

### `bringup_launch.py` remaps a topic the mux never publishes

`src/f1tenth_system/f1tenth_stack/launch/bringup_launch.py` starts `ackermann_mux` with `remappings=[('ackermann_cmd_out', 'ackermann_drive')]`. The mux publishes on `ackermann_cmd` (`src/f1tenth_system/ackermann_mux/src/ackermann_mux.cpp`, `create_publisher(... "ackermann_cmd" ...)`), and `ackermann_to_vesc` subscribes to `ackermann_cmd`. The remap matches nothing, so it does nothing, and the drive path works only because both ends use the default name.

**Fix:** delete the remap, or remap the real name on purpose. `f1tenth_system` is vendored with local fixes, so read `docs/git-setup.md` first. Car 2 has the same line.

### The car's install is a plain build, while the docs say "always `--symlink-install`"

`CLAUDE.md` and `docs/concepts.md` say every build uses `--symlink-install`, so YAML and launch edits in `src/` take effect without a rebuild. The packages currently in `install/` were built without it: `install/<pkg>/lib/python3.12/site-packages/` holds `<pkg>-0.1.0-py3.12.egg-info` copies (gap_follow, pure_pursuit, race_diagnostics, racerbot_doctor, imu_bridge), and `log/latest_build/logger_all.log` shows `symlink_install=False`. So editing `src/gap_follow/config/gap_follow.yaml` and relaunching changes nothing until that package is rebuilt, which contradicts what the docs promise.

An earlier session recorded that `--symlink-install` breaks `ros2 run` with `importlib.metadata.PackageNotFoundError` on Jazzy and Python 3.12. That did not reproduce on 2026-10-24. A fresh `colcon build --symlink-install --packages-select drive_intent gap_follow` into a separate install base resolved `distribution('gap-follow')` and the `gap_follow_node` entry point.

**Fix:** rebuild once with `rm -rf build install && colcon build --symlink-install --parallel-workers 1`, then confirm `ros2 launch gap_follow gap_follow_launch.py` still starts. If the `PackageNotFoundError` comes back, record the exact steps here and correct `CLAUDE.md` and `docs/concepts.md`.

### Car 1's geometry is car 2's, unmeasured

Wheelbase 0.36 m, width 0.30 m and the LiDAR offset 0.26 m were tape-measured on car 2 (2026-08-24) and copied to car 1. `car_length: 0.58` was never measured on either car. Already marked `TODO(verify on car 1)` in `docs/hardware-reference.md` and `docs/car1-vs-car2.md`. The race planner, both safety layers and the dashboard overlay all depend on these numbers.

**Fix:** measure wheelbase, tire-to-tire width, LiDAR-to-front-axle and bumper-to-bumper on car 1; change every file in its row of `docs/hardware-reference.md#where-each-number-lives` together.

### Both simulators model car 2's steering, not car 1's

`racerbot_sim/sim_bridge.py` and `tools/f1tenth_sim/sim_fidelity/calibration.py` use car 2's servo map and full-lock limits (0.436 rad). Car 1's full stick is 0.38 rad. Simulator results for car 1 are therefore run on a car that steers further than car 1 can. Recorded in `docs/car1-vs-car2.md` as a known gap.

**Fix:** after car 1's full-lock circles are trusted, give both simulators car 1's servo map and limits behind a car selection, with a test that fails if the two disagree with `vesc.yaml` and `joy_teleop.yaml`.

### The VESC IMU on car 1 reports all zeros

`/sensors/imu/raw` publishes at 50 Hz with every field `0.0` (measured 2026-08-19, `docs/hardware-reference.md`). The BNO085 now covers yaw rate, but the VESC IMU is still either not fitted or disabled in firmware.

**Fix:** check **VESC Tool → App Settings → IMU** (sample rate 0 means off). Either enable it or mark the VESC IMU as absent in `imu_bridge`'s car 1 config so nothing waits on it.

## Documentation

### `README.md` says the doctor still checks car 2's hardware

The **Notes** section of `README.md` ends with: "`racerbot_doctor`'s LiDAR and camera checks still describe **car 2's** hardware ... and will report those layers as failing on car 1". Commit `9fe90a4` ("racerbot_doctor: car 1's hardware") changed the doctor to the USB Hokuyo and the UVC webcam, and `CLAUDE.md` describes it as car 1's version.

**Fix:** delete that bullet from `README.md`.

### `docs/concepts.md` and `CLAUDE.md` describe a build the car doesn't have

See [the install entry above](#the-cars-install-is-a-plain-build-while-the-docs-say-always---symlink-install). Until it's resolved, `docs/concepts.md#the-flag-that-saves-you-the-most-time` is wrong for the car as it stands.

## Cross-car drift

These are differences between the two repos that are not hardware differences, so one of the two is behind. They are not in `docs/car1-vs-car2.md` because nothing about car 1's hardware requires them.

| What | Car 1 | Car 2 |
|---|---|---|
| `gap_follow_launch.py max_speed:=` | Present (`gap_follow/launch_speed.py`) | Missing |
| `map_course` IMU options (`imu`, `gyro_odom`, `fused_speed`) | Present | Missing |
| `map_course record_bag` default | `true` | `false` |

**Fix:** decide per row whether car 2 should take car 1's version, and port it there.
