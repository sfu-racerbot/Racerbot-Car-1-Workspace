# racerbot-ws

> **Sister repositories:** This Car 1 workspace shares its software lineage with [Racerbot Car 2 Workspace](https://github.com/sfu-racerbot/Racerbot-Car-2-Workspace). Shared software updates should be reviewed across both repositories, while car-specific hardware configuration remains separate.
ROS2 Jazzy workspace for the team's roboracer/F1TENTH car (Jetson Orin Nano Super, JetPack 7.2, Ubuntu 24.04).

Recent changes to the team's own packages are logged in [CHANGELOG.md](CHANGELOG.md) — check it after pulling to see what changed and whether anything still needs on-car validation.

## Documentation

Start here if you're new to the car or the codebase:

| Doc | What's in it |
|---|---|
| [docs/concepts.md](docs/concepts.md) | New to ROS2/colcon/this workspace? What `ros2 launch`/`colcon build`/`source` actually do, what every top-level folder is for, and how a package is laid out — start here if any of that is unfamiliar |
| [docs/adding-your-own-code.md](docs/adding-your-own-code.md) | **Adding your own code? Start here.** Where new packages go, what they're required to have (depends on whether it can move the car), and how to build/run them |
| [docs/architecture.md](docs/architecture.md) | The full node/topic graph, what talks to what, and the safety/priority model — **read this before writing any driving code** |
| [docs/racing-autonomy.md](docs/racing-autonomy.md) | The map-based race stack (SLAM → localization → recorded racing line → curvature-paced velocity profile → pure pursuit control) — the algorithm, the math, and how to tune it |
| [docs/writing-your-own-node.md](docs/writing-your-own-node.md) | The full contract for driving code specifically, using `gap_follow` as a worked template |
| [docs/web-dashboard.md](docs/web-dashboard.md) | Live browser dashboard of the car's map/scan/pose — read-only, safe to run alongside anything |
| [docs/hardware-reference.md](docs/hardware-reference.md) | VESC, LiDAR, joystick — exact addresses, ports, config values, and gotchas for this specific car |
| [docs/realsense-camera.md](docs/realsense-camera.md) | Intel RealSense D435i color/depth over ROS2 — install notes and verified performance |
| [docs/usb-camera-livestream.md](docs/usb-camera-livestream.md) | Live MJPEG video stream from a USB webcam, viewable in any browser — camera picks, how it works, security note |
| [docs/operations.md](docs/operations.md) | Step-by-step procedures: driving, mapping, localizing, running autonomy, shutting down |
| [docs/troubleshooting.md](docs/troubleshooting.md) | Real issues hit during bring-up and how they were diagnosed |
| [docs/git-setup.md](docs/git-setup.md) | How this repo is versioned: `src/` git submodules, cloning/updating them, and checking upstream f1tenth repos for updates |

This file stays a short quick-start; the docs above are the full reference.

## Hardware summary (see [docs/hardware-reference.md](docs/hardware-reference.md) for exact values)
- **VESC** motor controller — USB serial, exposed as a stable `/dev/sensors/vesc` symlink (udev rule at `/etc/udev/rules.d/99-vesc.rules`).
- **Hokuyo UST-30LX** LiDAR — **USB serial** (not Ethernet), exposed as a stable `/dev/sensors/hokuyo` symlink (udev rule at `/etc/udev/rules.d/99-hokuyo.rules`).
- **Logitech F710** joystick — LB is the teleop deadman switch (hold to drive manually).

## Layout (`src/`)
| Package | Source | Purpose |
|---|---|---|
| `f1tenth_system` (+ `ackermann_mux`, `teleop_tools`, `vesc`) | vendored (plain tracked files, **not** a git submodule — see [docs/git-setup.md](docs/git-setup.md)) | VESC driver, `urg_node` (Hokuyo), joystick teleop, command muxing |
| `transport_drivers` | git submodule, `humble` | serial transport dependency for `vesc` |
| `particle_filter` (+ `range_libc`) | git submodules, `foxy-devel`/`humble-devel` | Monte Carlo localization against a saved map |
| `realsense-ros` | git submodule, `ros2-master` | Intel RealSense D435i driver — color/depth over ROS2 |
| `gap_follow` | local package | baseline reactive autonomy — follow-the-gap on `/scan` → `/drive`, no map needed. Code/algorithm detail: [src/gap_follow/README.md](src/gap_follow/README.md) |
| `pure_pursuit` | local package | map-based race controller — pure pursuit over a curvature-paced recorded racing line, plus the tools to record and pace one. Pipeline/workflow: [docs/racing-autonomy.md](docs/racing-autonomy.md) |
| `web_dashboard` | local package | live browser dashboard of the map/LIDAR/pose, streamed over a WebSocket — read-only, not an autonomy node, safe to run alongside anything else. Workflow: [docs/web-dashboard.md](docs/web-dashboard.md) |
| `racerbot_launch` | local package | launch glue not owned by any single driver repo (currently: SLAM, autonomous mapping, automated map-to-race, and race-day localization+pure_pursuit) |
| `usb_cam_stream` | local package | live MJPEG video stream from a USB webcam, served over plain HTTP for viewing in any browser. Detail: [docs/usb-camera-livestream.md](docs/usb-camera-livestream.md) |

`slam_toolbox` is installed system-wide via apt (`ros-jazzy-slam-toolbox`), not vendored in `src/`.

## One-time setup (already done on this machine)
- ROS2 Jazzy + dev tools installed, `rosdep` initialized.
- `racerbotcar-1` and `racermember1` added to the `dialout` group (VESC + Hokuyo USB serial access) and `input` group (joystick device access). **Group membership only applies to sessions started after it was added** — open a fresh terminal if you hit a permission error on `/dev/sensors/vesc`, `/dev/sensors/hokuyo`, or `/dev/input/js0`.
- `racermember1` has ACL access scoped to this workspace only — see `getfacl racerbot-ws`.

## Build
```bash
source /opt/ros/jazzy/setup.bash
cd ~/racerbot-ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```
On the Orin Nano's 8GB RAM, prefer `colcon build --symlink-install --parallel-workers 1` if you hit OOM during a full rebuild.

Every new shell needs both `source /opt/ros/jazzy/setup.bash` and `source install/setup.bash` (in that order) before running any `ros2`/`colcon` command.

## Running
Use separate terminals so each layer can be killed/restarted independently.

1. **Drivers** (start here, every time — this alone never moves the car: it's the shared foundation layer with no control layer of its own, see [docs/architecture.md](docs/architecture.md#the-node-graph)):
   ```bash
   ros2 launch f1tenth_stack bringup_launch.py
   ```

2. **Manual teleop**, in a second terminal, on top of the drivers:
   ```bash
   ros2 launch f1tenth_stack teleop_launch.py
   ```
   Hold **LB** on the F710 (must be in **XInput mode**) to drive manually.

3. **Mapping** (drive the car around the track manually while this runs):
   ```bash
   ros2 launch racerbot_launch slam_launch.py
   ```
   Save the result: `ros2 run nav2_map_server map_saver_cli -f <map_name>`

4. **Localization** against a saved map:
   - Copy the map files you saved from step 3 (`<map_name>.yaml` + `.pgm`) into `src/particle_filter/maps/`.
   - Set `map_server.ros__parameters.map` in `src/particle_filter/config/localize.yaml` to `<map_name>` (no extension).
   - Rebuild (`colcon build --symlink-install --packages-select particle_filter`) so the map gets installed into the share directory.
   - ```bash
     ros2 launch particle_filter localize_launch.py
     ```
   - In RViz, use "2D Pose Estimate" to seed the particle filter's initial guess.

5. **Baseline autonomous driving** (reactive, no map/localization required — run this instead of `teleop_launch.py`, not alongside it, or its always-on `/teleop` will mask `/drive` at the mux):
   ```bash
   ros2 launch gap_follow gap_follow_launch.py
   ```
   Tunables live in `src/gap_follow/config/gap_follow.yaml` (speed limits, steering limits, safety bubble, emergency stop distance). Defaults are deliberately conservative — **the first run on a new car should be with the wheels off the ground.**

For race-day localization+pure_pursuit, autonomous mapping, and every other workflow: see [docs/operations.md](docs/operations.md).

## Notes
- **Current safety policy:** every autonomy node in this workspace (`gap_follow`, `pure_pursuit`, and any new one) requires the driver to hold **LB** on the physical controller for the car to move, on top of the usual `ackermann_mux` arbitration — see [docs/architecture.md](docs/architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car). This stays in force until the team explicitly confirms the car's behavior is trustworthy enough to relax it — don't set any node's `enable_deadman` parameter to `false` unilaterally.
- The official `f1tenth`/roboracer driver repos don't have a `jazzy` branch yet; everything here is the `humble-devel`/`humble`/`foxy-devel` source built against ROS2 Jazzy. If a future `rosdep update`/dependency bump breaks the build, check each package's upstream (submodule or vendored — see [docs/git-setup.md](docs/git-setup.md)) for a newer ROS2-distro branch before patching locally.
- `src/f1tenth_system/f1tenth_stack/config/joy_teleop.yaml`'s `human_control` profile was patched locally: upstream ships `drive-steering_angle` mapped to `axis: 2` (this F710's left trigger, not the right stick). Changed to `axis: 3`. `f1tenth_system` is vendored (not a submodule) specifically so this fix — and the LiDAR serial-port fix below — could be committed; see [docs/git-setup.md](docs/git-setup.md) before pulling in any upstream changes to it.
- This car's LiDAR is a **Hokuyo UST-30LX over USB serial**, not the Ethernet UST-10LX some other team workspaces use — `src/f1tenth_system/f1tenth_stack/config/sensors.yaml` was patched locally accordingly (`serial_port`, not `ip_address`/`ip_port`). See [docs/hardware-reference.md](docs/hardware-reference.md) before assuming otherwise.
- Simulator (`f1tenth_gym_ros`) is intentionally not installed here — sim testing happens on a separate machine.

