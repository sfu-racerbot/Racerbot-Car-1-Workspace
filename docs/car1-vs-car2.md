# Car 1 vs car 2: what must stay different

> **Who this is for:** anyone (person or Claude) about to port a change from the [car 2 workspace](https://github.com/sfu-racerbot/Racerbot-Car-2-Workspace) into this one, or wondering why a car 2 instruction doesn't work here.
> **Read first:** [git-setup.md](git-setup.md#remote): how the two repos relate (sister repos, not upstream and fork).
> **You'll be able to:** take car 2's improvements without breaking car 1's LiDAR, camera, steering or dashboard, and prove you didn't.
> **Time:** 5 minutes to read; the check at the end takes seconds.

The two cars run almost the same code, but they are **not the same hardware**. Changes from car 2's [workspace](glossary.md) (its whole repo of ROS packages) are regularly ported into this one.

A port that copies car 2's files over car 1's breaks car 1 **silently**. It builds fine. Then the [LiDAR](glossary.md) (the spinning laser scanner) never publishes, or the doctor reports a camera that was never fitted, or the steering hits a limit it shouldn't.

This page is the list of things that must survive every port. It is also the instruction to Claude: **when asked to port anything from car 2, read this page first, keep every car 1 value below, and run the check at the end before calling the port done.**

## The differences

### Hardware

| Thing | Car 1 (this repo) | Car 2 | Where car 1's version lives |
|---|---|---|---|
| LiDAR | Hokuyo **UST-30LX over USB**. Enumerates as a serial port (`15d1:0000`), reached through the `/dev/sensors/hokuyo` link | Hokuyo UST-10LX over **Ethernet**, `192.168.0.10` on `enP8p1s0`, with an `nmcli` profile called `hokuyo` | `f1tenth_stack/config/sensors.yaml` (`serial_port: "/dev/sensors/hokuyo"`, `ip_address`/`ip_port` commented out); udev rule `/etc/udev/rules.d/99-hokuyo.rules` |
| Camera | a plain **UVC USB webcam** at `/dev/video0`. Streamed to the dashboard by `usb_cam_stream`, started with `webcam_launch.py`. Publishes no ROS image topic | Intel **RealSense D435i**, with the `realsense-ros` driver and `realsense_camera_launch.py` | `racerbot_launch/launch/webcam_launch.py`; `device:` under `usb_cam_stream_node` in `racerbot_launch/config/web_dashboard_rb1.yaml`. Car 1 has no `realsense-ros` submodule at all |
| Steering and speed calibration | servo `= -0.83 × steering angle + 0.366`, clamped to `0.05 … 0.75`; `speed_to_erpm_gain` `3650.0` (and `-4040.0` for `vesc_to_odom_node`, tape-calibrated 2026-10-21) | car 2's own measured values | `f1tenth_stack/config/vesc.yaml` (commit `d6d0749`). `pure_pursuit.yaml`'s `max_steering_angle` is derived from it; re-derive if it changes |
| Steering limits | full stick **0.38 rad**; autonomy `max_steering_angle` **0.33**; race planner `profile_max_steering_angle` **0.30**. From car 1's 2026-10-21 circles | full stick 0.436, autonomy 0.349, planner 0.30, from car 2's 2026-10-16 circles | `joy_teleop.yaml` scale; `gap_follow.yaml`, `pure_pursuit.yaml`, `auto_map_race.yaml`; `odom_calibration.yaml` `planner_max_steering_rad`. Quoted in [hardware-reference.md](hardware-reference.md) |
| IMU (BNO085) | wired to the **Jetson's own I2C bus** (bus 7, address `0x4A`, header pins 1/3/5/6), no microcontroller. imu-calibration's `tools/bno_i2c_bridge.py --pty /tmp/bno085 --no-dyncal` turns it into the serial stream `imu_bridge` reads. `imu_bridge_node` restarts its BNO driver when no samples arrive for 2 s, because the bridge's port appears after it starts | a BNO085 behind a **microcontroller on USB** (`/dev/ttyACM1`) | `imu_bridge/launch/vehicle_state_launch.py` (starts the bridge, points `bno_port` at `/tmp/bno085`); the `_watch_bno` restart in `imu_bridge/imu_bridge_node.py`; `vehicle_state.yaml` geometry (`wheelbase`, `bno_x_offset` 0.36: the BNO sits on the front axle). The BNO calibration and mount (`rpy_deg: [90, 0, 90]`, nominal) live in `~/imu-calibration`, not in this repo |
| Geometry | **same numbers as car 2** for now (wheelbase `0.36`, LiDAR `0.26 0.0 0.11`), assumed rather than measured on car 1 | measured on car 2, 2026-08-24 | see [hardware-reference.md](hardware-reference.md#physical-dimensions-used-in-config). **TODO(verify on car 1):** measure; if car 1 differs, this row becomes a real difference |

### Software and setup

| Thing | Car 1 | Car 2 | Where car 1's version lives |
|---|---|---|---|
| Web dashboard id | **`rb1`** | car 2's own id | `racerbot_launch/config/web_dashboard_rb1.yaml`, loaded by `dashboard_launch.py` |
| Dashboard server at boot | **yes**, `web-dashboard.service` runs `dashboard_launch.py` (camera still by hand) | started by hand | `tools/systemd/web-dashboard.service`, installed by `tools/systemd/install.sh`. See [web-dashboard.md](web-dashboard.md#car-1s-boot-services) |
| Remote desktop (x11vnc) | **follows the live display**: GDM's login screen `:0` when nobody is logged in, the GNOME session after. Runs as root from a root-owned copy in `/usr/local/sbin` | hard-codes `-display :1` (a logged-in session), which crash-loops on car 1 | `tools/systemd/x11vnc-follow.sh`, `x11vnc.service`, `novnc.service`. Hostnames `rd-rb1` / `ssh-rb1` |
| Self-check | `racerbot_doctor` checks the USB Hokuyo (USB → `cdc_acm` → `/dev/sensors/hokuyo` → port free) and any UVC webcam at `/dev/video0` | checks the Ethernet link, IP, ping and TCP port, and the RealSense | `racerbot_doctor/car.py` (LiDAR and camera sections) and the LiDAR/camera ladders in `checks.py`, plus their tests. See [the doctor's README](../src/racerbot_doctor/README.md) |
| [Odometry](glossary.md) (wheel-based position) calibration label | `vehicle_model: "... (car 1)"` | car 2's | `odom_calibration/config/odom_calibration.yaml` |
| Simulator vehicle model | **not car 1's.** Both simulators (`tools/f1tenth_sim/sim_fidelity/calibration.py`, `racerbot_sim/sim_bridge.py`) model car 2's servo map and full-lock limits. Known gap, not a difference to protect: replace with car 1's numbers when someone measures car 1's full lock | car 2's | — |

### Set up on the car itself, not in git

These live on car 1's Jetson, so a port can't break them. A **reinstall** can, though, and nothing in the repo would notice.

| Thing | What car 1 needs | Check |
|---|---|---|
| udev rules | `/etc/udev/rules.d/99-vesc.rules` and `99-hokuyo.rules`: each makes a stable `/dev/sensors/…` link with `MODE=0666` | `ls -l /dev/sensors/` shows `vesc` and `hokuyo` |
| `input` group | the user that runs bringup must be in group `input`. Without it, `joy_node` runs but publishes no `/joy`, and **the car cannot be driven**: LB does nothing | `id` lists `input`; fix with `sudo usermod -aG input $USER`, then log out and in |
| `video` group | needed to open the webcam | `id` lists `video` |

`ros2 run racerbot_doctor doctor` checks all three.

## Porting from car 2, step by step

1. **Port only what was asked for.** Car 2 is not upstream. Don't take whole files or whole directories "to stay in sync".
2. **For every file you touch, check it against the tables above.** If car 2's version of a file in the "Where car 1's version lives" column differs, merge the change *into* car 1's file by hand. Never replace the file.
3. **Never bring back** car 2's Ethernet LiDAR settings, anything RealSense, or car 2's dashboard config.
4. **`f1tenth_system` has its own extra checklist.** Five local fixes, two of which fail unsafely: see [git-setup.md](git-setup.md#updating-f1tenth_system--manual-no-submodule-machinery).
5. **Run the checks below.** If one fails, the port broke car 1. Fix the port; don't edit the test.

**Terminal 1** — no ROS needed for the first command.

```bash
cd ~/racerbot-ws
python3 -m pytest src/racerbot_launch/test/ -v
```

**Working when:** every test passes, including all of `test_car1_hardware_config.py`. That file fails if car 2's Ethernet LiDAR settings, a RealSense launch, a car 2 dashboard config, car 2's steering calibration or limits, or a doctor that disagrees with car 1's udev rule or webcam path comes back. It also fails if car 1's I2C BNO085 setup is lost: the bridge, `--no-dyncal`, the driver restart, `map_course` running the IMU side, or `vehicle_state`'s wheelbase.

**If it doesn't:** the failing test's name says which row of the tables above was clobbered.

Then the hardware check.

**Terminal 2** — with the car powered and `bringup_launch.py` running in another terminal.

```bash
ros2 run racerbot_doctor doctor
```

**Working when:** the LiDAR section shows `/dev/sensors/hokuyo` and `/scan` passing, and nothing mentions Ethernet, `nmcli` or a RealSense.

## Updating this page

When you find a new car 1 difference, add a row here **and**, if it lives in a file, a check to `src/racerbot_launch/test/test_car1_hardware_config.py`. A difference that's only written down will eventually be overwritten by a port.

## See also

- [hardware-reference.md](hardware-reference.md): car 1's devices, ports and measured numbers in detail.
- [git-setup.md](git-setup.md): how the repos relate, and the `f1tenth_system` sync checklist.
- [troubleshooting.md](troubleshooting.md#joy-is-silent-and-lb-does-nothing-not-in-the-input-group): the `input` group failure as you'd meet it.
