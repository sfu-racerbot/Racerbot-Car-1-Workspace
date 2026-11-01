# Plan (DRAFT): one workspace for car 1 and car 2

> **Status:** draft, 2026-10-31. **Do not start implementing.** Work is in flight on both
> car repos; once it is merged and both are pushed to GitHub, re-run the inventory in
> step 0 (every number below is from 2026-10-31 and will move) and resolve the open
> questions at the bottom.
> **Decision owner:** the tech team lead. Decided 2026-10-31: one repo, a car 1 / car 2
> mode, both cars must behave exactly the same, data files stay in git in separate
> per-car folders.

## Goal

One repo runs both cars. The only things that differ between them are the hardware and
calibration values in that car's profile, and the car's own data folder. The same code
and the same tuning run on both. A test enforces that split, so the two cars can't drift
apart again. With no more ports between repos, `docs/car1-vs-car2.md`'s porting checklist
gets retired.

## Inventory, 2026-10-31

Car 1 = `racerbot-ws` @ `ffbfac0`. Car 2 = `Racerbot-Car-2-Workspace` @ `7876ea3`.

- **Packages:** identical set, except car 2 has a `realsense-ros` submodule. Same
  submodules otherwise (`particle_filter`, `range_libc`, `transport_drivers`,
  `racerbot_a`, `racerbot_b`, `web_dashboards`).
- **Code drift:** about 128 files outside `config/` differ, mostly in `race_diagnostics`
  (27), `racerbot_launch` (25), `gap_follow` (17), `racerbot_doctor` (14), `pure_pursuit`
  (9), `racerbot_sim` (8) and `imu_bridge` (7). Car 2 is ahead in places (e.g. `4ec8e7b`,
  auto_record catches more of a run's start). Car 1 is ahead in others.
- **Config drift:** `joy_teleop.yaml`, `sensors.yaml`, `vesc.yaml`, `gap_follow.yaml`,
  `pure_pursuit.yaml`, `auto_map_race.yaml`, `waypoint_recorder.yaml`,
  `vehicle_state.yaml`, plus car 2 only has `web_dashboard_rb2.yaml`. Much of the
  "tuning" difference is really calibration. Example: `max_steering_angle` is 0.349 on
  car 2 and 0.33 on car 1 because the servos differ. Car 2 also keeps the IMU-threshold
  features off until it records its own thresholds.
- **Data:** car 2 already commits its data to `data/`, written straight there by the
  tools and committed by `tools/save_runs.sh`. Contents: `racerbot_runs/` 968 MB of
  rosbags, `racerbot_auto/` 32 MB, `racerbot_sim/` 133 MB, all since 2026-10-25. Car 1
  has no `data/` and writes to `~/.ros/` (outside git).
- **Stale doc row:** the BNO085 is on Jetson I2C on **both** cars. `docs/car1-vs-car2.md`
  says car 2 uses a USB microcontroller, which is wrong.

## Design

### Layout

```
cars/
  machines.yaml          # Jetson serial number -> car id; the only identity lookup
  rb1/
    profile.yaml         # hardware + calibration + derived limits; nothing else
  rb2/
    profile.yaml
data/
  rb1/{racerbot_runs,racerbot_auto,racerbot_sim}/
  rb2/{racerbot_runs,racerbot_auto,racerbot_sim}/
  shared/                # optional: maps/lines both cars race on
```

Shared tuning stays where it is now (`gap_follow.yaml`, `pure_pursuit.yaml`, ...), one
copy, used by both cars.

### What goes in a profile (allow-list, test-enforced)

- **Hardware:**
  - LiDAR model and transport: `serial` + port, or `ethernet` + IP/port
  - camera: `uvc` + device, or `realsense`
  - dashboard site id
- **Calibration:** `vesc.yaml` values (`speed_to_erpm_gain`, servo gain/offset/clip,
  `vesc_to_odom` gain), `joy_teleop` steering scale, IMU mount and thresholds.
- **Geometry:** wheelbase, `base_link -> laser` transform, BNO offset.
- **Values derived from calibration:** full-stick angle, autonomy `max_steering_angle`,
  planner `profile_max_steering_angle`, `odom_calibration`'s planner limit. These are
  computed from the servo calibration where possible, not hand-copied
  (`test_steering_limits_fit_servo.py` already does this math).
- **IMU feature enables, per car**, until each car has recorded its own thresholds. A
  temporary exception; see open question 2.

Anything else in a profile fails `test_profile_allow_list`. That test is what makes
"exact same behaviour" true.

### How a car knows which car it is

1. Bringup reads `/proc/device-tree/serial-number` and looks it up in
   `cars/machines.yaml`. Everything that resolves the car id goes through one pure-Python
   module (no rclpy), e.g. `racerbot_launch/car_profile.py`.
2. `RACERBOT_CAR=rb1|rb2` overrides the lookup, for dev boxes and the simulator. On a
   machine that's in `machines.yaml`, an override that disagrees with the table is
   refused.
3. **Fail closed.** If the machine isn't in the table and no override is set, every
   bringup refuses to start and prints the serial plus the line to add. The simulator
   launches need an explicit id.
4. **Hardware cross-check** at bringup and in `racerbot_doctor`: `rb1` needs
   `/dev/sensors/hokuyo`, `rb2` needs the Ethernet Hokuyo reachable, and the camera has
   to match the profile. A mismatch stops bringup. This catches a wrong
   `machines.yaml` entry or a swapped Jetson.
5. The car id goes into every run's `metadata/` and into `/drive_intent`, so any
   recording can be traced to a car.

### How profiles reach the nodes

- Launch files load shared YAML first, then the car's profile on top. Node code is
  unchanged.
- Profile keys map onto the existing ROS parameters, so the dashboard's live tuning and
  `snapshot_params` keep working.
- `odom_calibration`'s writers (`vesc_yaml.py`, `teleop_yaml.py`) write to
  `cars/<id>/profile.yaml` instead of the shared `vesc.yaml` / `joy_teleop.yaml`.
- The web dashboard's live-tuning save writes shared tuning to the shared file, and
  refuses a key that belongs in a profile.
- `racerbot_doctor`: both ladders (USB Hokuyo + UVC, Ethernet Hokuyo + RealSense) ship,
  and the profile picks one. `car.py`'s thresholds come from the profile.
- The simulators use the selected car's servo map and steering limits. This fixes the
  known gap where both sims model car 2.

### Data

- Every tool defaults to `data/<car id>/...`, using car 2's existing `data/` scheme with
  an extra car level.
- `save_runs.sh` commits only `data/<this car>/`, so the two cars never touch each
  other's files and their pushes don't conflict.
- Car 1's existing `~/.ros/racerbot_auto` and `~/.ros/racerbot_runs` get copied into
  `data/rb1/`, if wanted (open question 1).
- Dashboard delete panel: allowed for `data/<this car>/` only. It must still refuse the
  git-tracked demo maps and the other car's folder.

## Steps (each lands green on both cars before the next)

0. **Re-inventory** once both repos are final: re-diff code and config, and list every
   feature each car has that the other lacks.
1. **Pick the surviving repo** (proposal: car 1's) and bring car 2's history in. Either
   use `git merge --allow-unrelated-histories` of car 2's repo, or keep car 2's repo
   archived and read-only as the history.
2. **Merge code to one version per file**, feature by feature, never "theirs wins". For
   each drifted file, take the union of the features and keep the tests from both
   sides. Car-specific branches in code (doctor ladders, camera launch) become
   profile-driven.
3. **Add `cars/`, `car_profile.py`, the fail-closed lookup and the hardware
   cross-check.** Tests: unknown serial refuses; an override that disagrees refuses;
   each profile against its car's expected hardware (this replaces
   `test_car1_hardware_config.py` with a version for both cars); the allow-list test.
4. **Move calibration out of the shared YAMLs into the profiles.** Prove nothing
   changed: for each car, dump the effective parameters of every node before and after
   (`snapshot_params` output or launch-time param dump) and diff. They must be
   identical.
5. **Unify tuning.** Decide value by value where `gap_follow.yaml` /
   `pure_pursuit.yaml` / `auto_map_race.yaml` differ for reasons other than
   calibration. This is the step that actually makes behaviour identical, and each
   choice is the team lead's call.
6. **Data folders.** Move car 2's `data/*` to `data/rb2/`, point every tool at
   `data/<id>/`, and port `save_runs.sh`.
7. **Retire the porting docs.** Rewrite `docs/car1-vs-car2.md` as "how car profiles
   work" and update `CLAUDE.md`, `docs/git-setup.md` and `docs/hardware-reference.md`
   (with one table per car). Archive car 2's repo.
8. **Roll out per car**, following the usual test order: static topic check, then wheels
   off the ground with LB held, then floor at low speed. Car 2 switches its checkout's
   remote to the merged repo only after car 1 passes.

## Risks

- **A wrong profile means the wrong servo map and wrong steering limits.** Mitigated by
  the fail-closed lookup, the hardware cross-check, and the before/after
  parameter-equality check in step 4.
- **Repo size.** Rosbags add about 1 GB per car per week at current rates. GitHub warns
  at large repo sizes and rejects files over 100 MB, and every clone pays for it. Decide
  before the merge between plain git, Git LFS for `bag/`, or committing reports only
  (open question 1).
- **The merged code has never run on either car.** Step 8 exists for that reason.
- **A shared change now reaches both cars at once.** That's the point of the merge, but
  "tested on car 1" no longer covers car 2.

## Open questions (team lead)

1. **Run recordings in git:** do rosbags stay plain git as on car 2, move to Git LFS, or
   do we commit only each run's reports and metadata? Do we copy car 1's old `~/.ros`
   runs in?
2. **Per-car tuning overrides:** forbidden outright (proposed), or allowed for a
   temporary, named list such as the IMU feature enables until car 2 has its own
   thresholds?
3. **Surviving repo and name:** keep car 1's (proposed) and rename it, e.g. to
   `Racerbot-Workspace`?
4. **Car 2's hardware today:** still the Ethernet UST-10LX and the RealSense D435i, and
   dashboard id `rb2`?
5. **Car 1's geometry:** measure it before the merge, so `rb1/profile.yaml` doesn't start
   with car 2's numbers?
