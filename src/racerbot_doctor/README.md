# racerbot_doctor: one command that tells you what's broken

> **Who this is for:** anyone about to drive the car, or whose car isn't working and who doesn't know where to start.
> **Read first:** [operations.md — Every session, before anything else](../../docs/operations.md#every-session-before-anything-else) (sourcing ROS).
> **You'll be able to:** check the VESC, LiDAR, joystick and webcam in one go, and get told which exact thing is wrong and how to fix it. With teleop running, it also tells you why the car won't move when you hold LB.
> **Time:** 5 seconds to run.

The doctor selects hardware checks from the running car's [profile](../racerbot_car/README.md), the configuration that describes its devices. On rb1 it checks the USB Hokuyo and UVC webcam; on rb2 it checks the Ethernet Hokuyo and RealSense D435i.

`racerbot_doctor` is a self-check for the car. Run it before a session, or the moment something doesn't work. It answers "is it plugged in?", "did the driver load?" and "is data actually coming out?" for every device, in that order.

## Highlights

- **One command, every device:** VESC (motor controller), Hokuyo LiDAR, F710 gamepad, USB webcam, the bringup nodes, and your terminal's ROS setup.
- **Answers "I'm holding LB and the car doesn't move".** With `teleop_launch.py` running, a *Drive path* section follows the command hop by hop: `/joy` → `/teleop` → `/ackermann_cmd` → `/commands/motor/speed`. It names the first hop that goes quiet, whether LB was held during the sample, and whether the VESC was actually told to move.
- **Tells "not plugged in" apart from "driver didn't load".** Each device is checked layer by layer: USB → Linux kernel driver → device file and permissions → ROS driver process → message rate → message content. The report stops at the **first** broken layer, so you know where to look.
- **Read-only, so it's safe to run any time, even with bringup up.** It never publishes on a topic, and never opens the VESC's or the LiDAR's serial port, the camera, or the gamepad. Permissions are checked without opening anything. A test scans the package source and fails if any of that is ever added.
- **Every failure comes with a fix line**: the actual command or physical action, e.g. `sudo usermod -aG input $USER` or "flip the X/D switch on the back to X".
- **Reads the battery** (3S LiPo) from the VESC's own voltage measurement, and tells "battery off" (VESC alive on USB power only) apart from "battery low".
- **Works before bringup, too.** With nothing running it checks all the hardware layers and reports the ROS layers as "not running", which is not a failure.
- **Exit code** 0 = nothing failed, 1 = something failed, 2 = car selection failed or the doctor crashed, so scripts can use it.
- **Tests for both cars**, including deliberately broken checks to confirm the tests detect regressions, per [TEST_QUALITY_STANDARDS.md](../../TEST_QUALITY_STANDARDS.md).

**Why it exists:** most entries in [troubleshooting.md](../../docs/troubleshooting.md) are an hour spent finding something a ten-line check would have caught. A gamepad switched to the wrong mode, a missing device link, a VESC with its battery off. This tool runs all of those checks at once.

## Running it

First [source](../../docs/glossary.md#sourcing) ROS and this workspace (the two `source` lines every terminal needs; see [operations.md](../../docs/operations.md#every-session-before-anything-else)).

**Terminal 1** — any terminal will do; the doctor runs and exits.

```bash
ros2 run racerbot_doctor doctor
```

**Working when:** the last line reads `0 FAIL, ...`. A `WARN` is worth reading but won't stop you driving.

**If it doesn't:** read the `FAIL` lines. Each has a `fix:` line under it. The summary at the bottom names the first broken layer of each device.

On a development machine, select the car whose hardware is being inspected. **Terminal 1**, after [sourcing](../../docs/glossary.md#sourcing) ROS and the workspace:

```bash
ros2 run racerbot_doctor doctor --car rb2 --no-live
```

**Working when:** the LiDAR section starts with `Ethernet link` and the camera section checks the RealSense. Missing hardware can still produce `FAIL` or `WARN` findings.

`--car` overrides automatic identity selection. Without it, the doctor uses the Jetson serial or `RACERBOT_CAR`. If neither resolves a car, it exits 2 with instructions; an unknown `--car` also exits 2 and lists the known IDs.

To check the ROS side as well (topic rates, the battery, VESC faults), start bringup first in another terminal, then run the doctor:

**Terminal 1** — the hardware drivers. Leave it running.

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

**Working when:** log lines stop scrolling after a few seconds. A `vesc_driver_node` `[FATAL]` here means the VESC isn't reachable, and the doctor will say why.

**Terminal 2** — the doctor.

```bash
ros2 run racerbot_doctor doctor
```

**Working when:** the VESC, LiDAR and Joystick sections end in `PASS` lines for their `rate` layers, and the VESC shows a battery voltage.

**Car won't move under [teleop](../../docs/glossary.md) (manual driving with the gamepad)?** Start `teleop_launch.py` too, then run the doctor **while holding LB and pushing the left stick**. The *Drive path (teleop)* section at the bottom tells you where the command stops.

If it ends `PASS motor command`, ROS is doing its job and the problem is below ROS: the battery switch, ESC power or motor wiring.

Options:

| Flag | What it does |
|---|---|
| `--car rb1` or `--car rb2` | Override automatic car selection for this inspection |
| `--window 5` | Sample each topic for 5 s instead of 3 (more stable rate numbers) |
| `--no-live` | Hardware and OS checks only; never joins the ROS graph |
| `--no-color` | Plain text, e.g. for pasting into a chat |

## Reading the report

Each line has a status:

| Status | Meaning |
|---|---|
| `PASS` | This layer is fine. |
| `FAIL` | This layer is broken. Everything above it is `SKIP`ped, because checking `/scan` when the cable is out tells you nothing new. |
| `WARN` | Works, but not as it should (low battery, no webcam plugged in). Doesn't fail the run. |
| `INFO` | Nothing wrong, just worth knowing: usually "bringup isn't running, so I stopped here". |
| `SKIP` | Not checked, and the line says why. |

Here's an example, captured on car 1 on 2026-10-20 with bringup and teleop running. The car would not move with LB held:

```
LiDAR
  PASS  USB device                         15d1:0000 at 1-2.2
  PASS  kernel driver                      cdc_acm -> /dev/ttyACM0
  PASS  /dev/sensors/hokuyo                -> ttyACM0, read/write OK
  PASS  serial port free                   held only by urg_node_driver
  PASS  ROS driver urg_node_driver         running (pid 87877)
  PASS  /scan rate                         38.7 Hz
  PASS  /scan content                      1081 beams, 100% with a return

Joystick
  PASS  USB receiver                       F710 in XInput mode (this cannot tell whether the controller itself is switched on)
  PASS  kernel driver                      xpad -> /dev/input/js0
  FAIL  input permissions                  event5: joy_node reads /dev/input/event5 and you cannot -- it will run, find no controller, and publish no /joy, so the car cannot be driven
                                           fix: `sudo usermod -aG input $USER`, log out and in (or reboot), then restart bringup
  SKIP  ROS driver joy_node .. /joy rate  (2 layers) blocked by: input permissions

Drive path (teleop)
  FAIL  /joy rate                          no /joy, so joy_teleop has nothing to act on -- see Joystick above
  SKIP  /teleop rate .. motor command  (5 layers) blocked by: /joy rate
```

The camera never fails the run (team decision, 2026-10-01): the car drives without it, so camera problems show as `WARN`.

## What it checks, device by device

<details>
<summary>The full layer list and thresholds. Skip this unless you're changing the doctor or a threshold.</summary>

Hardware addresses and device paths come from `src/racerbot_car/cars/ID/car.yaml`. Driver defaults and thresholds live in [`racerbot_doctor/car.py`](racerbot_doctor/car.py), beside their sources. LiDAR calibration is in each car's `calibration/sensors.yaml`.

| Device | Layers, bottom → top |
|---|---|
| **VESC** | USB `0483:5740` present → `cdc_acm` kernel driver made a `/dev/ttyACM*` → `/dev/sensors/vesc` points at it and you can read/write it (`dialout` group) → no other program has the port open → `vesc_driver_node` running → `/sensors/core` ≥ 25 Hz → `fault_code` 0 → battery |
| **LiDAR** (USB) | USB `15d1:0000` present → `cdc_acm` made a `/dev/ttyACM*` → `/dev/sensors/hokuyo` points at it and you can read/write it (`99-hokuyo.rules`) → no program other than `urg_node` has the port open → `urg_node_driver` running → `/scan` ≥ 30 Hz → at least one beam has a return (`WARN` under 50 %) |
| **LiDAR** (rb2 Ethernet) | Interface `enP8p1s0` present with carrier → host IP `192.168.0.15/24` → ping `192.168.0.10` → TCP port `10940` probe, only while `urg_node_driver` is down → driver running → `/scan` ≥ 30 Hz → scan content as above |
| **Joystick** | F710 receiver present **in XInput mode** (`c21f`; DirectInput `c219` fails with "flip the switch") → `xpad` made `/dev/input/js*` → you can read **both** `js*` and `event*` (`input` group; `joy_node` uses `event*`, see below) → `joy_node` running → `/joy` ≥ 5 Hz |
| **Camera** | some USB device bound to `uvcvideo` (any make of webcam) → `/dev/video0` exists and you can read/write it (`video` group) → `camera_stream_node` running (`INFO` if not: it's optional, started by `webcam_launch.py`). No rate layer: the stream goes straight to the browser on port 9090, not over a ROS topic |
| **Camera** (rb2 RealSense) | USB `8086:0b3a` → USB speed ≥ 5000 Mb/s (`WARN` below) → `uvcvideo` created video nodes → `realsense2_camera_node` running → `/camera/camera/color/image_raw` ≥ 8 Hz. Camera failures become `WARN`; a profile without a camera reports `INFO` |
| **Bringup** | `ackermann_mux`, `ackermann_to_vesc_node`, `vesc_to_odom_node` running → `/odom` ≥ 25 Hz (`INFO`, not `FAIL`, while no servo command has ever been sent: `vesc_to_odom_node` waits for the first one) → `base_link→laser` transform is `0.26 0 0.11` (±1 mm) → `INFO` if something publishes `/teleop` (that blocks autonomy's `/drive`) |
| **Drive path** | only with `teleop_launch.py` running: `/joy` → `/teleop` → `/ackermann_cmd` → `/commands/motor/speed`, each ≥ 5 Hz (each hop republishes once per `/joy` message) → LB held in at least one `/joy` message (`INFO` if not) → at least one non-zero motor speed command (`WARN` if LB was held but the stick wasn't moved) |
| **Environment** | `ROS_DISTRO=jazzy` → workspace sourced → `ROS_DOMAIN_ID` isn't the test domain 79 → `racerbot_sim` not running (`FAIL` if it's running beside the real drivers) |

**"Bringup not running" vs "a driver died".** If *no* bringup driver is running, the ROS-driver layer says `INFO`: nothing has been started, so nothing is wrong. If *some* are running and one is missing, that one is a `FAIL` ("it crashed or failed to start"). That's how a driver that didn't load is told apart from bringup simply not being up.

**Battery thresholds (3S LiPo).** The VESC measures its own input voltage, which is the pack voltage. Its driver publishes that as `voltage_input` on `/sensors/core`.

| Reading | Verdict |
|---|---|
| under 5.0 V | `FAIL`: no battery. The VESC is running on USB power only |
| under 10.2 V (3.4 V/cell) | `FAIL`: critically low |
| under 10.8 V (3.6 V/cell) | `WARN`: low |
| over 12.75 V (4.25 V/cell) | `WARN`: more than a full 3S pack. Is a 4S pack fitted? |

Limits: the per-cell figure is the total divided by 3. The VESC can't see individual cells, so one bad cell can hide behind a healthy total. The reading hasn't yet been checked against a multimeter on this car.

**Why `js0` being readable isn't enough.** Jazzy's `joy_node` uses SDL2, which opens the gamepad's `/dev/input/event*` device file, not `js0`. On this Jetson `js0` is world-readable but `event*` is `root:input`, mode `0660`.

So a user outside the `input` group passes a `js0`-only check, yet `joy_node` runs, finds no controller, and publishes nothing. That happened on 2026-10-20, and it is why the doctor checks both files.

**Limitations.**
- It can't tell whether the F710 *controller* is switched on: the USB receiver looks the same either way. Press a button and watch `ros2 topic echo /joy` if in doubt.
- It can't see other users' processes holding the VESC port without root.

</details>

## How it's built

<details>
<summary>For anyone changing the code. Skip otherwise.</summary>

| File | Job |
|---|---|
| `racerbot_doctor/system.py` | The only code that touches the OS: walks `/sys/bus/usb/devices`, `/proc`, `/dev`. It never opens a device node (a file representing hardware); access is checked with `os.access`. Every root path is a parameter so tests use a fake tree. |
| `racerbot_doctor/live.py` | The only code that imports `rclpy`. Subscribes for a few seconds and counts messages. Creates no publishers. |
| `racerbot_doctor/checks.py` | All the verdicts. Pure functions, no I/O. |
| `racerbot_doctor/snapshot.py` | The plain data passed between them, and the layer ladder. |
| `racerbot_doctor/report.py`, `cli.py` | Printing and the command line. |
| `racerbot_doctor/car.py` | Every threshold and hardware ID, each with its source. |

`spec_for(profile)` builds frozen hardware specifications by device kind. The CLI passes the same specification to OS gathering, live sampling and report checks.

rb1 never runs network probes; rb2 never checks the Hokuyo serial link or video device permissions. Only rb2 samples a camera ROS topic. The legacy `check_lidar`, `check_camera` and default Python arguments retain rb1 behaviour for existing callers.

Running the tests: everything except `test_live.py` runs with no ROS at all. `test_live.py` needs ROS sourced and the isolated test domain:

**Terminal 1**, at the workspace root:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 -m pytest src/racerbot_doctor/test/ -v
```

**Working when:** `190 passed` on domain 79. On domain 72, 189 pass and `test_runs_on_the_isolated_test_domain` fails because it explicitly requires 79. Unsourced, `test_live.py` fails to collect because `rclpy` is unavailable; do not add `--continue-on-collection-errors`.

### Dual-car regression verification (2026-11-01)

The Task 12b checks ran with `RB_WS=$PWD rb env ROS_DOMAIN_ID=72 python3 -m pytest`, using `-q -p no:cacheprovider`. Each mutation below was applied alone, tested, reverted and tested again. The separate domain assertion passed on domain 79.

| Mutation | Selected tests | Failed under mutation | Passed after revert |
|---|---|---|---|
| A: return rb1 specifications for every profile | rb2 specification and rb2 checks | 15 (2 still passed) | 17 |
| B: swap the cars' specifications | Specification tests | 5 (LB test still passed) | 6 |
| C: rb1 also requests ping/TCP | rb1 isolation | 1 | 1 |
| D: rb2 stats the Hokuyo link | rb2 isolation | 1 | 1 |
| E: replace the Ethernet ladder with an INFO-only report | rb2 LiDAR checks | 9 | 9 |
| F: replace the RealSense ladder with an INFO-only report | rb2 camera checks | 6 | 6 |
| G: IP parser returns a fabricated address | IP parsing | 2 | 2 |
| H: carrier always reports down | Carrier filesystem fixture | 1 | 1 |
| I: invert the automatic TCP probe gate | Driver up/down TCP gating | 2 | 2 |
| J: corrupt gathered VESC target and probe results | Remaining rb2 gather tests | 8 | 8 |
| K: change the LB index from 4 to 5 | Teleop [deadman](../../docs/glossary.md#deadman) config oracle | 1 | 1 |
| L: CLI always returns 2 | CLI behaviour and rb2 probe request | 14 | 14 |
| M: omit the camera subscription | rb2 live topic/rate check | 1 | 1 |
| N: absent camera reports WARN | No-camera profile | 1 | 1 |

Comparison with `refs/car2/main` found 28 car-2-only test names, all retained: 14 in `test_rb2_checks.py`, 13 in `test_rb2_network.py` and the LB test in `test_car_specs.py`. Shared car 1 tests remain in their original files.

The rb2 fixture keeps the recorded IP output and carrier files; camera mutations select the RealSense by USB ID.

The unsourced full suite stopped at the same single `test_live.py` import error before and after this change. Explicitly excluding that ROS-dependent file ran all 183 non-live cases successfully. The car 1 hardware guard retained its two baseline failures at `test_car1_hardware_config.py:108` and `:119`, with 15 passes.

The plan this was built from: [docs/superpowers/plans/2026-10-01-racerbot-doctor.md](../../docs/superpowers/plans/2026-10-01-racerbot-doctor.md).

</details>

## See also

- [troubleshooting.md](../../docs/troubleshooting.md): the long-form fixes the doctor's `fix:` lines point towards.
- [hardware-reference.md](../../docs/hardware-reference.md): where the LiDAR offset and the other measured numbers come from.
