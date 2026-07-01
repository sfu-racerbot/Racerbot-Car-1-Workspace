# Architecture

How the car's software is put together: every node, every topic, and how they connect. Everything communicates over ROS2 Jazzy topics — there is no shared memory or direct function calls between packages, so this topic map *is* the system.

## The node graph

This is what `ros2 launch f1tenth_stack bringup_launch.py` actually starts, plus the optional layers you add on top of it (mapping, localization, autonomy).

```
                                    ┌─────────────┐
                                    │  F710 pad   │
                                    └──────┬──────┘
                                           │ USB (XInput mode required)
                                           ▼
                                     ┌───────────┐
                                     │  joy_node │
                                     └─────┬─────┘
                                           │ /joy  (sensor_msgs/Joy, ~15-20Hz)
                                           ▼
                                    ┌─────────────┐
                                    │  joy_teleop │
                                    └──────┬──────┘
                                           │ /teleop  (AckermannDriveStamped)
                                           │ ALWAYS publishing — see "Safety model" below
                                           ▼
  /drive  ◄── your autonomy node    ┌─────────────┐
  (AckermannDriveStamped)  ────────►│ ackermann_mux│
                                    └──────┬──────┘
                                           │ /ackermann_cmd  (AckermannDriveStamped)
                                           ▼
                                 ┌─────────────────────┐
                                 │ ackermann_to_vesc_node│
                                 └──────────┬───────────┘
                                            │ /commands/motor/speed (Float64)
                                            │ /commands/servo/position (Float64)
                                            ▼
                                    ┌─────────────────┐
                                    │ vesc_driver_node │◄──── serial (/dev/sensors/vesc)
                                    └────────┬─────────┘             │
                                             │                       ▼
                          /sensors/core, /sensors/imu,           ┌───────┐
                          /sensors/imu/raw,                      │ VESC  │──► drive motor
                          /sensors/servo_position_command        └───┬───┘
                                             │                       │
                                             ▼                   PPM/PWM
                                    ┌──────────────────┐             │
                                    │ vesc_to_odom_node │             ▼
                                    └────────┬──────────┘      steering servo
                                             │
                                    /odom (nav_msgs/Odometry)
                                    tf: odom → base_link


  ┌──────────────┐
  │ Hokuyo UST-30LX│──USB (/dev/sensors/hokuyo)──►┌──────────┐
  └──────────────┘                                    │ urg_node │──► /scan (sensor_msgs/LaserScan)
                                                        └──────────┘

  static_transform_publisher ──► tf: base_link → laser (fixed offset: 0.27m fwd, 0.11m up)
```

Everything below `/scan` and `/odom` is optional and layered on top depending on what you're doing:

```
/scan ──┬──► gap_follow_node ──────────────► /drive   (reactive, no map needed)
        │
        ├──► slam_toolbox (mapping mode) ──► /map, saved to a .yaml+.pgm file
        │
        └──► particle_filter (localization,  ──► /pf/viz/inferred_pose, /pf/pose/odom
              needs a saved map + /odom)          (your planner would consume these)
```

Race day layers one more node on top of `particle_filter`'s output — see [racing-autonomy.md](racing-autonomy.md) for the full pipeline (map once, localize, record a lap, pace it into a racing line, then race it):

```
/pf/viz/inferred_pose ──┬──► pure_pursuit_node ──► /drive   (needs a saved map, a working
/scan ───────────────────┘    localization launch, and a recorded + paced racing line .csv —
                               see docs/racing-autonomy.md)
```

`web_dashboard` is passive and layers on top of whatever's already running — it never publishes, so it isn't part of the driving path at all, just a viewer for it (see [web-dashboard.md](web-dashboard.md)):

```
/map ───────────────────┐
/scan ───────────────────┼──► web_dashboard_node ──► WebSocket ──► any browser on the network
/pf/viz/inferred_pose ───┘    (read-only; no /drive, no /joy, exempt from the deadman policy below)
```

## Topic reference

All topics as they actually appear on the bus during a full `bringup_launch.py` run (verified via `ros2 topic list` / `ros2 node info`, not just read from source):

| Topic | Type | Published by | Subscribed by |
|---|---|---|---|
| `/joy` | `sensor_msgs/Joy` | `joy_node` | `joy_teleop` |
| `/teleop` | `ackermann_msgs/AckermannDriveStamped` | `joy_teleop` | `ackermann_mux` |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | your autonomy node (e.g. `gap_follow`, `pure_pursuit`) | `ackermann_mux` |
| `/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | `ackermann_mux` | `ackermann_to_vesc_node` |
| `/commands/motor/speed` | `std_msgs/Float64` | `ackermann_to_vesc_node` | `vesc_driver_node` |
| `/commands/servo/position` | `std_msgs/Float64` | `ackermann_to_vesc_node` | `vesc_driver_node` |
| `/commands/motor/{duty_cycle,current,brake,position}` | `std_msgs/Float64` | (unused by this stack — direct low-level VESC control, available if you need it) | `vesc_driver_node` |
| `/sensors/core` | `vesc_msgs/VescStateStamped` | `vesc_driver_node` | `vesc_to_odom_node` |
| `/sensors/imu`, `/sensors/imu/raw` | `sensor_msgs/Imu` | `vesc_driver_node` | (nothing by default — the VESC's onboard IMU, available if you want it) |
| `/sensors/servo_position_command` | `std_msgs/Float64` | `vesc_driver_node` | `vesc_to_odom_node` |
| `/odom` | `nav_msgs/Odometry` | `vesc_to_odom_node` | `particle_filter` (if running) |
| `/scan` | `sensor_msgs/LaserScan` | `urg_node` | `gap_follow`, `slam_toolbox`, `particle_filter` (whichever is running) |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `static_transform_publisher`, `vesc_to_odom_node` | RViz, `slam_toolbox`, `particle_filter` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `urg_node`, `ackermann_mux` | RViz / `ros2 topic echo` for debugging |

## Package reference

| Package | Where it comes from | Role |
|---|---|---|
| `f1tenth_stack` (in `f1tenth_system`) | vendored (plain tracked files, not a submodule — see [git-setup.md](git-setup.md)) | Owns the launch files and all the YAML configs (`vesc.yaml`, `sensors.yaml`, `mux.yaml`, `joy_teleop.yaml`) that wire everything below together |
| `vesc_driver`, `vesc_ackermann`, `vesc_msgs` (in `vesc`) | vendored (nested inside `f1tenth_system`) | Talks to the VESC over serial; converts between Ackermann drive commands and raw VESC motor/servo commands |
| `serial_driver`, `io_context` (in `transport_drivers`) | git submodule, `humble` | Low-level serial port library the VESC driver is built on |
| `urg_node` | apt (`ros-jazzy-urg-node`) | Hokuyo LiDAR driver — this car's Hokuyo is USB serial, not Ethernet (see [hardware-reference.md](hardware-reference.md#lidar--hokuyo-ust-30lx)) |
| `joy`, `joy_teleop`, `teleop_tools` | apt / vendored | Gamepad input and teleop mapping |
| `ackermann_mux` (in `f1tenth_system`) | vendored (nested inside `f1tenth_system`) | Arbitrates between teleop and autonomy commands — see safety model below |
| `particle_filter`, `range_libc` | git submodules, `foxy-devel`/`humble-devel` | Monte Carlo localization against a saved map |
| `slam_toolbox` | apt (`ros-jazzy-slam-toolbox`) | Builds a map by driving the car around manually |
| `gap_follow` | local, `src/gap_follow` | Baseline reactive autonomy (follow-the-gap) — see [writing-your-own-node.md](writing-your-own-node.md), this package *is* the worked example |
| `pure_pursuit` | local, `src/pure_pursuit` | Map-based race controller (pure pursuit over a curvature-paced recorded racing line) plus the tools to record and pace one — see [racing-autonomy.md](racing-autonomy.md) |
| `web_dashboard` | local, `src/web_dashboard` | Read-only live browser dashboard of the map/scan/pose over a WebSocket — never publishes anything, not subject to the deadman policy below — see [web-dashboard.md](web-dashboard.md) |
| `racerbot_launch` | local, `src/racerbot_launch` | Launch files that don't belong to any single driver package (currently: SLAM, autonomous mapping, and race-day localization+pure_pursuit) |

## The safety model (read this before writing autonomy code)

`ackermann_mux` picks between two input channels and publishes the winner to `/ackermann_cmd`:

```yaml
joystick:   topic: teleop, priority: 100, timeout: 0.2s
navigation: topic: drive,  priority: 10,  timeout: 0.2s
```

Higher priority wins, *as long as that channel hasn't gone silent for more than its timeout*. The intent is straightforward: a human on the joystick should always be able to override autonomy.

**Important, verified behavior:** `joy_teleop`'s `default` profile (in `joy_teleop.yaml`) has no deadman-button restriction — it runs unconditionally and continuously publishes a neutral `(steering=0, speed=0)` command to `/teleop` at all times, whether or not LB is held. This means `/teleop` **never times out** as long as `joy_node` + `joy_teleop` are running, so it **always** wins arbitration over `/drive` — even when nobody is touching the controller.

This was directly confirmed by test: with the full bringup running and LB *not* held, publishing a distinct, continuous command to `/drive` had zero effect on `/ackermann_cmd`, which stayed locked at `0.0 / 0.0` the whole time.

**Practical consequence: your autonomy node's `/drive` commands will never reach the VESC while `joy_node` and `joy_teleop` are both running.** See [operations.md](operations.md#running-autonomy-gap_follow-pure_pursuit-or-your-own-node) for the correct procedure. There's an unused hook for this in `joy_teleop.yaml` — an `autonomous_control` profile bound to the RB button — but it currently just publishes an `Int8` to `/dev/null` (a no-op placeholder), it isn't wired to anything that would let `joy_teleop` stop overriding `/drive`. If your team wants a clean "flip RB to hand off to autonomy" workflow, that's the place to build it — it doesn't exist yet.

### Workspace policy: the LB deadman button is mandatory for every node that can move the car

**Current, standing policy — read this before running or writing any driving code.** Regardless of the `ackermann_mux` arbitration above, no code in this workspace — autonomous or not — is allowed to move the car unless the driver is actively holding **LB** on the physical controller. Manual teleop already works this way (`joy_teleop`'s `human_control` profile is deadman-gated). **This policy stays in force, unrelaxed, until the team has explicitly confirmed the car's behavior is trustworthy enough to change it.**

Both autonomy nodes currently in this workspace enforce this in code, not just by convention: `gap_follow_node` and `pure_pursuit_node` each subscribe to `/joy` directly and refuse to publish a non-zero drive command unless button index `deadman_button` (default `4`, i.e. LB) is currently held on a live `/joy` stream (`joy_timeout_sec`, default `0.5s`) — checked *first*, ahead of every other watchdog either node has. This is a **second, independent safety layer on top of** the mux arbitration above: even once `/teleop` has been silenced (see [operations.md](operations.md#running-autonomy-gap_follow-pure_pursuit-or-your-own-node)), the autonomy node itself still won't drive without LB held. Concretely, this means **`joy_node` must be left running** — with LB held — while any autonomy node drives; only `joy_teleop` should be stopped.

Each node exposes this as an `enable_deadman` parameter (default `true` in both `gap_follow.yaml` and `pure_pursuit.yaml`). **Do not set it to `false` on either node** — that would be a unilateral decision to bypass the current policy, not just a tuning change. **Any new autonomy node added to this workspace must implement the same check before it's allowed to drive the car** — see [writing-your-own-node.md](writing-your-own-node.md#the-interface-contract) for the required pattern.

## Frame conventions

- `base_link`: origin of the car, at the rear axle (matches `wheelbase: 0.25m` in `vesc.yaml`, used by `vesc_to_odom_node` for odometry)
- `laser`: the Hokuyo's frame, offset `+0.27m` forward / `+0.11m` up from `base_link` (fixed, via `static_transform_publisher`) — see [hardware-reference.md](hardware-reference.md#lidar--hokuyo-ust-30lx) for this car's actual LiDAR connection (USB, not Ethernet)
- `odom`: continuous but drifting frame, published by `vesc_to_odom_node` from wheel-speed + servo-angle integration (no encoders/IMU fusion — this is dead-reckoning only)
- `map`: only exists once `slam_toolbox` or `particle_filter`'s `map_server` is running

Note `slam_toolbox`'s configured `base_frame` is `laser`, not `base_link` (see `f1tenth_stack/config/f1tenth_online_async.yaml`) — a deliberate upstream choice, not a typo, but worth knowing if you're debugging a `tf` tree.
