# Hardware reference

Physical connections, addresses, and config values for this specific car. Everything here is what's actually configured on this machine, verified against the live hardware — not generic F1TENTH/roboracer defaults.

## Compute

- Jetson Orin Nano Super (Developer Kit), JetPack 7.2, L4T R39.2, Ubuntu 24.04.4 "noble", aarch64, 8GB RAM
- ROS2 **Jazzy**, installed natively (not in a container)
- Accounts: `racerbotcar-1` (sudo/admin) and `racermember1` (no sudo, ACL-scoped to this workspace only — see main [README.md](../README.md))

## VESC (motor + steering controller)

| | |
|---|---|
| Connection | USB, exposed as `/dev/sensors/vesc` (symlink to whatever `/dev/ttyACM*` it enumerates as) |
| udev rule | `/etc/udev/rules.d/99-vesc.rules` — matches idVendor `0483`, idProduct `5740` (STMicroelectronics Virtual COM Port), `GROUP=dialout`, `MODE=0660` |
| Firmware | 7.0 (confirmed via `vesc_driver_node`'s connection log) |
| Group access | `racerbotcar-1` and `racermember1` are both in the `dialout` group |

**Config** (`src/f1tenth_system/f1tenth_stack/config/vesc.yaml`):
```yaml
speed_to_erpm_gain: 4614.0
speed_to_erpm_offset: 0.0
steering_angle_to_servo_gain: -1.2135
steering_angle_to_servo_offset: 0.5304
servo_min: 0.15
servo_max: 0.85
wheelbase: 0.25  # meters, used for odometry
```
The servo formula: `servo_position = -1.2135 × steering_angle + 0.5304`. Steering angle `0.0` → servo position `0.5304` (center). This is what you'll see repeatedly if you inspect `/commands/servo/position` — `0.5304` is neutral, not a bug.

**Servo output must be enabled in the VESC's own firmware config** — this is separate from anything in this repo and isn't visible to `ros2`/`colcon` at all. If a freshly-flashed or reset VESC has steering that does nothing despite `fault_code: 0` and a clean serial connection, check this first. Requires **VESC Tool** (official app, vesc-project.com) connected over USB — which means stopping the ROS bringup first, since only one process can hold the serial port. Full story in [troubleshooting.md](troubleshooting.md).

Reading or writing the VESC's app configuration (including that servo-output flag) is **not possible from this ROS stack** — `vesc_driver` only implements motor/servo *control* commands (`COMM_SET_SERVO_POS`, `COMM_SET_RPM`, etc.), not the config read/write protocol (`COMM_GET_APPCONF`/`COMM_SET_APPCONF`) that VESC Tool uses. Don't go looking for a `ros2 service`/CLI way to do this — there isn't one here.

## LiDAR — Hokuyo UST-30LX

| | |
|---|---|
| Connection | USB (not Ethernet) — enumerates as `/dev/ttyACM*`, exposed as a stable `/dev/sensors/hokuyo` symlink |
| udev rule | `/etc/udev/rules.d/99-hokuyo.rules` — matches idVendor `15d1`, idProduct `0000` (Hokuyo Data Flex for USB, URG-Series USB Driver), `MODE=0666`, `SYMLINK+="sensors/hokuyo"` |
| Config | `src/f1tenth_system/f1tenth_stack/config/sensors.yaml` sets `serial_port: "/dev/sensors/hokuyo"` (`ip_address`/`ip_port` are commented out — this car doesn't use `urg_node`'s Ethernet mode at all) |
| Scan rate | ~40Hz, ±180° (`angle_min`/`angle_max`: ±3.14 rad in `sensors.yaml`) |
| Frame | `laser`, offset from `base_link`: +0.27m forward, +0.11m up, no rotation — **verify this offset if the UST-30LX's mount differs from the UST-10LX this figure was originally measured for** |

If `/dev/sensors/hokuyo` doesn't show up after plugging the LiDAR in: `lsusb` should list `ID 15d1:0000`, and `ls -la /dev/ttyACM*` should show a device owned by group `dialout`/mode `0666`. `sudo udevadm control --reload-rules && sudo udevadm trigger` re-applies the rule without a reboot.

No static IP, dedicated NIC, or `nmcli` connection is needed for this car's LiDAR — that's a Car 2–specific detail (Ethernet Hokuyo UST-10LX) that doesn't apply here.

## Camera — UVC USB webcam

Car 1 uses a standard UVC webcam at `/dev/video0`; it does **not** have an Intel RealSense camera. `src/usb_cam_stream/config/usb_cam_stream.yaml` captures it at 1280×720 and 30 FPS and serves the monitoring stream on port 9090. Verify the device path with `v4l2-ctl --list-devices` if USB enumeration changes. See [usb-camera-livestream.md](usb-camera-livestream.md).

## Joystick — Logitech F710

| | |
|---|---|
| Mode | **Must be XInput** (switch on the back of the transceiver/controller). DirectInput mode produces no `/dev/input/js*` device on this system at all — silently, no error anywhere. |
| USB vendor:product (XInput) | `046d:c21f` |
| Deadman switch | LB (button index 4) — hold to enable manual driving |
| Speed axis | 1 (left stick, vertical) |
| Steering axis | 3 (right stick, horizontal) |
| RB (button index 5) | Bound to an `autonomous_control` profile in `joy_teleop.yaml` that currently does nothing functional — publishes to `/dev/null`. See [architecture.md](architecture.md#the-safety-model-read-this-before-writing-autonomy-code). |
| Group access | Joystick device nodes are group `input`; `racerbotcar-1` and `racermember1` are both members |

**Don't trust assumed Xbox-style axis numbering** — it was wrong here too (upstream config had steering on axis 2, which is actually the left trigger on this pad in XInput mode, not the right stick). This car's `f1tenth_system` is vendored specifically so this fix (and the LiDAR serial-port fix above) can be committed — see [git-setup.md](git-setup.md). If you ever swap controllers, re-verify empirically:
```bash
ros2 topic echo /joy
```
watch which `axes[]` index moves as you work each stick/trigger, and which `buttons[]` index flips to `1` for each button, rather than assuming.

## Network

- `wlP1p1s0`: WiFi, normal DHCP, used for everything (internet, SSH, ROS traffic, etc.) — the LiDAR is USB, not Ethernet, so there's no dedicated wired NIC or static IP for it on this car
- `tailscale0`: Tailscale VPN interface, present for remote access if configured

## Physical dimensions used in config

- Wheelbase: 0.25m (`vesc.yaml`, used for odometry — verify this matches the actual chassis if it's ever swapped)
- LiDAR mount offset from `base_link`: +0.27m forward, +0.11m up (`static_transform_publisher` args in `bringup_launch.py`) — carried over from the UST-10LX's measured mount; **re-verify against the UST-30LX's actual physical mount** if it hasn't been re-measured yet
