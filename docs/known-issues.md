# Known issues: workspace discrepancies and doc problems

> **Who this is for:** anyone with an afternoon to fix things. Each entry is something found wrong, stale or inconsistent while writing the workspace docs for sfuracerbot.ca (October 2026), and left for later on purpose.
> **Read first:** nothing.
> **What's in it:** one entry per problem: what is wrong, the evidence, and the suggested fix.

Each entry was checked against car 1 at `881c409` unless it says otherwise. Car 2 is [Racerbot-Car-2-Workspace](https://github.com/sfu-racerbot/Racerbot-Car-2-Workspace) at `779c060`. When you fix one, delete its entry in the same commit.

## Contents

- [Code and config](#code-and-config)
- [Setup on the car](#setup-on-the-car)
- [Documentation](#documentation)
- [Cross-car drift](#cross-car-drift)

## Code and config

### Two IMU stacks publish `/imu/data`

`imu_bridge`'s `imu_bridge_node`, started by `vehicle_state_launch.py` and by `map_course`, publishes `/imu/data`. `racerbot_imu`'s `imu_select_node`, started by `racerbot_imu imu_launch.py`, which `neural_racer` requires, also publishes `/imu/data` and `/imu/status` (`src/racerbot_imu/racerbot_imu/imu_select_node.py`, `create_publisher(Imu, '/imu/data', ...)`). Running both chains gives two publishers with different calibrations on one topic, and every consumer gets a blend.

**Fix:** give `/imu/data` one owner, or namespace one of the two packages. Add a static test that fails if both declare the same output topic.

### Two nodes still declare the pre-October 0.26 rad steering default

`pure_pursuit_node.py` declares `max_steering_angle` with a default of 0.26 and `auto_map_race_node.py` declares `profile_max_steering_angle` with 0.26. The shipped YAMLs set 0.33 and 0.30, so the race path is unaffected, but any launch or test that builds either node without its YAML gets the old limit. `docs/racing-autonomy.md` ("`max_steering_angle` uses the smaller magnitude (`0.26`)") quotes the same old value.

**Fix:** match the declared defaults to the YAMLs, or read them from `docs/hardware-reference.md`'s numbers, and correct the racing-autonomy sentence. Check which node tests rely on the declared default first.

### `racerbot_a`'s `reactive` README shows `enable_deadman:=false`

`src/racerbot_a/src/reactive/README.md` (team submodule, checked in the main checkout on 2026-10-24) gives `ros2 launch reactive reactive_launch.py enable_deadman:=false` as a launch command, twice. The workspace policy (`docs/architecture.md`) says never to set `enable_deadman` to `false`. `racerbot_a` also builds its own `gap_follow_node`, `ftg_node`, `disparity_extender_node` and `pure_pursuit_node`, which publish `AckermannDriveStamped`. None of them are covered by this repo's deadman tests.

**Fix:** in the `racerbot_a` repo, remove those commands, check each of its driving nodes has the LB deadman gate and the four deny-path tests, then bump the submodule.

### The particle filter broadcasts a transform tf2 rejects (upstream, verify)

`src/particle_filter/particle_filter/particle_filter.py` broadcasts a transform with `header.frame_id = '/map'` and `child_frame_id = '/laser'`, with leading slashes. tf2 in ROS 2 does not accept frame IDs that start with `/`, and `laser` already has `base_link` as its parent from the static transform. Nothing in the workspace reads this transform (`pure_pursuit` uses `/pf/viz/inferred_pose`), so the race path is unaffected, but it may be spamming errors or giving `laser` two parents.

**Fix:** run the filter and check `ros2 run tf2_ros tf2_monitor`. If the broadcast is rejected or harmful, disable it in the filter's config or patch it upstream; `particle_filter` is a submodule.

### Wiimote steering is not the F710's full stick

`src/racerbot_wiimote/racerbot_wiimote_driver/config/wiimote_teleop.yaml` sets the steering scale to `0.34` with the comment "the same as the F710's full stick", and `docs/wiimote.md` repeats it. The F710's full stick is 0.38 rad on car 1 and 0.436 rad on car 2 (`joy_teleop.yaml`). Same on both cars.

**Fix:** decide whether the Wiimote should match full stick, then change either the value or the comment and the doc.

### Car 1's geometry is car 2's, unmeasured

Wheelbase 0.36 m, width 0.30 m and the LiDAR offset 0.26 m were tape-measured on car 2 (2026-08-24) and copied to car 1. `car_length: 0.58` was never measured on either car. Already marked `TODO(verify on car 1)` in `docs/hardware-reference.md` and `docs/car1-vs-car2.md`. The race planner, both safety layers and the dashboard overlay depend on these numbers.

**Fix:** measure wheelbase, tire-to-tire width, LiDAR-to-front-axle and bumper-to-bumper on car 1, then change every file in its row of `docs/hardware-reference.md#where-each-number-lives` together.

### Both simulators model car 2's steering, not car 1's

`racerbot_sim/sim_bridge.py` and `tools/f1tenth_sim/sim_fidelity/calibration.py` use car 2's servo map and full-lock limits (0.436 rad). Car 1's full stick is 0.38 rad, so simulator results for car 1 come from a car that steers further than car 1 can. Already recorded in `docs/car1-vs-car2.md` as a known gap.

**Fix:** once car 1's full-lock circles are trusted, give both simulators car 1's servo map and limits behind a car selection, with a test that fails if they disagree with `vesc.yaml` and `joy_teleop.yaml`.

## Setup on the car

### The install is a plain build, while the docs say "always `--symlink-install`"

`CLAUDE.md` and `docs/concepts.md` say every build uses `--symlink-install`, so YAML and launch edits in `src/` take effect without a rebuild. The packages in `install/` were built without it: `install/<pkg>/lib/python3.12/site-packages/` holds `<pkg>-0.1.0-py3.12.egg-info` copies (gap_follow, pure_pursuit, race_diagnostics, racerbot_doctor, imu_bridge), and `log/latest_build/logger_all.log` shows `symlink_install=False`. Editing `src/gap_follow/config/gap_follow.yaml` and relaunching therefore changes nothing until that package is rebuilt. `docs/steering-laws.md` makes the same promise ("No rebuild needed").

An earlier session recorded that `--symlink-install` breaks `ros2 run` with `importlib.metadata.PackageNotFoundError` on Jazzy and Python 3.12. That did not reproduce on 2026-10-24: a fresh `colcon build --symlink-install --packages-select drive_intent gap_follow` into a separate install base resolved `distribution('gap-follow')` and the `gap_follow_node` entry point.

**Fix:** rebuild once with `rm -rf build install && colcon build --symlink-install --parallel-workers 1`, then confirm `ros2 launch gap_follow gap_follow_launch.py` still starts. If `PackageNotFoundError` comes back, record the exact steps here and correct `CLAUDE.md` and `docs/concepts.md`.

### `osqp` is not installed, so MPC cannot start

`python3 -c "import osqp"` fails on car 1 (2026-10-24). `docs/steering-laws.md`, "Installing osqp for MPC", says "On the car it was installed with", a line ported from car 2. `race_launch.py steering_law:=mpc` refuses to start until it is installed. The README's quick start doesn't mention it.

**Fix:** `pip install --user --break-system-packages osqp==1.1.3`, check `numpy` stays at 1.26.4, and add the step to the README's setup.

### The VESC command timeout is not recorded

`ackermann_mux` publishes nothing when `/drive` times out, so when a driving node goes quiet the motor stops only through the VESC firmware's own command timeout, set in VESC Tool and not visible from ROS. Its value isn't written down for either car.

**Fix:** on each car, read **VESC Tool → App Settings → General → Timeout** and record it in `docs/hardware-reference.md`.

### The VESC IMU: two docs disagree about which car reads zeros

`docs/hardware-reference.md` (car 1) says `/sensors/imu/raw` on "this car" publishes at 50 Hz with every field `0.0` (measured 2026-08-19, probably text ported from car 2). `src/racerbot_imu/README.md` says all-zero fields happen "as on car 2" and treats car 1's VESC IMU as working.

**Fix:** on car 1, with bringup running, `ros2 topic echo /sensors/imu/raw --field angular_velocity` and rotate the car by hand. Correct whichever doc is wrong. If it is zero, check **VESC Tool → App Settings → IMU** (a sample rate of 0 means off).

## Documentation

### `docs/ros-simulator.md` and `tools/f1tenth_sim/README.md` predate the 0.26 m LiDAR offset

- `docs/ros-simulator.md`, "What it does and does not model", says the simulator models "the 0.33m LiDAR offset", explains why gym's collision flag never fires, and says the 1081 beams match "the Hokuyo UST-10LX". `sim_bringup_launch.py` uses 0.26 m; `CLAUDE.md` records gym's flag firing since 2026-10-20; car 1's LiDAR is a UST-30LX.
- `tools/f1tenth_sim/README.md`, "The headline", says "Nobody has re-run the probe to find out whether gym's flag fires now". `CLAUDE.md` records it measured firing on 2026-10-20, along with the yaw-zeroing response `FidelityPlant` now disables.

**Fix:** update both notes to the measured state.

## Cross-car drift

These differences between the two repos are not hardware differences, so one of the two is behind. They are not in `docs/car1-vs-car2.md` because nothing about car 1's hardware requires them.

| What | Car 1 | Car 2 |
|---|---|---|
| `gap_follow_launch.py max_speed:=` | Present (`gap_follow/launch_speed.py`) | Missing |
| `map_course` IMU options (`imu`, `gyro_odom`, `fused_speed`) | Present | Missing |
| `map_course record_bag` default | `true` | `false` |
| `imu_bridge`'s `vehicle_state_node` (fused speed, measured steering, gyro odometry) | Present | Missing |

**Fix:** decide per row whether car 2 should take car 1's version, and port it there.
