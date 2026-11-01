# Plan (DRAFT): one workspace for car 1 and car 2

> **Status:** draft, 2026-10-31, all open questions answered. **Do not start implementing.**
> Development happens on one car at a time and the other car ports the improvements over;
> work on car 2 is in flight now. Start only when the team lead says both repos are done
> and won't move. Then re-clone both from GitHub and redo step 0 (every number below is
> from 2026-10-31 and will move).
> **Decision owner:** the tech team lead. Decisions are listed at the bottom.

## Goal

One repo, renamed **Racerbot Car Workspace** (`sfu-racerbot/Racerbot-Car-Workspace`, from
today's `Racerbot-Car-1-Workspace`), runs both cars with the same code. The merge keeps
the **latest and best version of every feature and tool** from either repo. What differs
between the cars is, all of it on GitHub so it's readable from the dev box:

- hardware and calibration, in the car's profile
- explicit per-car tuning overrides, in the car's folder
- the car's own data folder, with logs in Git LFS

A test enforces that split, so nothing car-specific hides in shared files. With no more
ports between repos, `docs/car1-vs-car2.md`'s porting checklist gets retired.

## Inventory, 2026-10-31 (redo in step 0)

Car 1 = `racerbot-ws` @ `ffbfac0`. Car 2 = `Racerbot-Car-2-Workspace` @ `7876ea3`.

- **Packages:** identical set, except car 2 has a `realsense-ros` submodule. Same
  submodules otherwise (`particle_filter`, `range_libc`, `transport_drivers`,
  `racerbot_a`, `racerbot_b`, `web_dashboards`).
- **Code drift:** about 128 files outside `config/` differ, mostly in `race_diagnostics`
  (27), `racerbot_launch` (25), `gap_follow` (17), `racerbot_doctor` (14), `pure_pursuit`
  (9), `racerbot_sim` (8) and `imu_bridge` (7). Each repo is ahead in places (e.g. car 2's
  `4ec8e7b`, auto_record catches more of a run's start).
- **Config drift:** `joy_teleop.yaml`, `sensors.yaml`, `vesc.yaml`, `gap_follow.yaml`,
  `pure_pursuit.yaml`, `auto_map_race.yaml`, `waypoint_recorder.yaml`,
  `vehicle_state.yaml`, plus car 2 only has `web_dashboard_rb2.yaml`. Much of the
  "tuning" difference is really calibration: `max_steering_angle` is 0.349 on car 2 and
  0.33 on car 1 because the servos differ. Car 2 also keeps the IMU-threshold features
  off until it records its own thresholds.
- **Data:** car 2 commits its data to `data/`, written straight there by the tools and
  committed by `tools/save_runs.sh`. Contents: `racerbot_runs/` 968 MB of rosbags,
  `racerbot_auto/` 32 MB, `racerbot_sim/` 133 MB, all since 2026-10-25, in plain git.
  Car 1 has no `data/` and writes to `~/.ros/` (outside git).

## Design

### Layout

```
cars/
  machines.yaml          # Jetson serial number -> car id; the only identity lookup
  rb1/
    profile.yaml         # hardware + calibration + derived limits; no tuning
    tuning.yaml          # this car's overrides of shared tuning (may be empty)
  rb2/
    profile.yaml
    tuning.yaml
data/
  rb1/{racerbot_runs,racerbot_auto,racerbot_sim}/
  rb2/{racerbot_runs,racerbot_auto,racerbot_sim}/
  shared/                # optional: maps/lines both cars race on
```

Shared tuning stays where it is now (`gap_follow.yaml`, `pure_pursuit.yaml`, ...), one
copy, used by both cars. `cars/<id>/tuning.yaml` overrides it per car.

### What goes in a profile (allow-list, test-enforced)

- **Hardware:**
  - LiDAR model and transport: `serial` + port (rb1), or `ethernet` + IP/port (rb2)
  - camera: `uvc` + device (rb1), or `realsense` (rb2)
  - dashboard site id (`rb1` / `rb2`)
- **Calibration:** `vesc.yaml` values (`speed_to_erpm_gain`, servo gain/offset/clip,
  `vesc_to_odom` gain), `joy_teleop` steering scale, the IMU mount and the IMU
  thresholds measured on that car.
- **Limits derived from calibration:** full-stick angle, autonomy `max_steering_angle`,
  planner `profile_max_steering_angle`, `odom_calibration`'s planner limit. These are
  computed from the servo calibration where possible, not hand-copied
  (`test_steering_limits_fit_servo.py` already does this math).

**Not per-car:** geometry. Both cars use the same chassis, so wheelbase, the
`base_link -> laser` transform, the BNO offset, `car_width` and `car_length` stay shared.
The BNO085 is on Jetson I2C on both cars, so the IMU bridge setup is shared too.

Anything else in `profile.yaml` fails `test_profile_allow_list`. Tuning belongs in
`tuning.yaml`.

### Per-car tuning overrides

- **Order:** shared YAML, then the car's `profile.yaml`, then the car's `tuning.yaml`.
  Last one wins.
- **Each override needs a reason.** Every key in `tuning.yaml` must name a parameter the
  node really declares (test, so a typo can't silently do nothing) and carry a comment
  saying why this car differs.
- **Overrides are visible:**
  - `racerbot_doctor` prints the car's overrides next to the shared values
  - each run's `metadata/` records the car id and its overrides
  - a docs table generated per car lists them

  "How do the cars differ?" is answered by reading one short file per car.
- **Safety limits never become looser per car.** A test refuses an override of
  `enable_deadman`, a TTC or emergency threshold made more permissive, or a `max_speed`
  above the shared cap. A car may only make those stricter.
- **The dashboard's live-tuning save** writes to the running car's `tuning.yaml` when the
  key is already overridden there. Otherwise it saves to the shared file, and offers
  "this car only" as the alternative. This needs a matching change in the
  `web_dashboards` submodule's `tuning.py` contract.

### How a car knows which car it is

1. Bringup reads `/proc/device-tree/serial-number` and looks it up in
   `cars/machines.yaml`. Everything that resolves the car id goes through one pure-Python
   module (no rclpy), e.g. `racerbot_launch/car_profile.py`.
2. `RACERBOT_CAR=rb1|rb2` overrides the lookup, for the dev box and the simulator. On a
   machine that's in `machines.yaml`, an override that disagrees with the table is
   refused.
3. **Fail closed.** If the machine isn't in the table and no override is set, every
   bringup refuses to start and prints the serial plus the line to add. The simulator
   launches need an explicit id.
4. **Hardware cross-check** at bringup and in `racerbot_doctor`: `rb1` needs
   `/dev/sensors/hokuyo`, `rb2` needs the Ethernet Hokuyo reachable, and the camera has
   to match the profile. A mismatch stops bringup. This catches a wrong
   `machines.yaml` entry or a swapped Jetson.
5. The car id goes into every run's `metadata/` and into `/drive_intent`.

### How profiles reach the nodes

- Launch files load shared YAML, then the profile, then `tuning.yaml`. Node code is
  unchanged.
- Profile keys map onto the existing ROS parameters, so live tuning and
  `snapshot_params` keep working.
- `odom_calibration`'s writers (`vesc_yaml.py`, `teleop_yaml.py`) write to
  `cars/<id>/profile.yaml` instead of the shared `vesc.yaml` / `joy_teleop.yaml`.
- `racerbot_doctor`: both ladders (USB Hokuyo + UVC, Ethernet Hokuyo + RealSense) ship,
  and the profile picks one. `car.py`'s thresholds come from the profile.
- The simulators use the selected car's servo map and steering limits. This fixes the
  known gap where both sims model car 2.

### Data, in Git LFS

- Every tool defaults to `data/<car id>/...`, using car 2's existing `data/` scheme with
  an extra car level.
- **`.gitattributes` puts the heavy binaries in LFS:** rosbags
  (`data/**/bag/**`, `*.mcap`, `*.db3`) and image files under `data/` (`*.pgm`, `*.png`).
  Small text files (`metadata/`, logs, CSV racing lines, `map.yaml`, `commands.txt`) stay
  plain git so diffs and `grep` keep working.
- **Migrate car 2's existing data into LFS** when it moves. Use `git lfs migrate import`
  on the merged history before the first push of the merged repo, so ~1 GB of plain-git
  blobs don't stay in the history forever.
- **Check the GitHub LFS storage and bandwidth quota** for the `sfu-racerbot` org before
  migrating. At ~1 GB per car per week, it will need a data pack, a pruning policy (e.g.
  bags of idle runs not committed), or both. Step 0 measures the real rate.
- **Clones and fetches:**
  - A plain clone skips the bags: set `lfs.fetchexclude` for `data/**/bag/**`, with a
    setup script for the cars and the dev box.
  - Fetch a run on demand with `git lfs pull --include 'data/rb1/racerbot_runs/<run>/**'`.
    `tools/run_analysis` gets a `--fetch` helper for this.
- **`save_runs.sh`** (from car 2):
  - commits only `data/<this car>/`, so the two cars never touch each other's files and
    their pushes don't conflict
  - refuses to commit if `git lfs` isn't installed, instead of committing bags as plain
    git
- Car 1's existing `~/.ros/racerbot_auto` and `~/.ros/racerbot_runs` are copied into
  `data/rb1/`.
- Dashboard delete panel: allowed for `data/<this car>/` only. It must still refuse the
  git-tracked demo maps and the other car's folder.

## Steps (each lands green, tests from both repos, before the next)

0. **Re-clone and re-inventory** once the team lead says both repos are finished:
   - fresh clones of both from GitHub
   - re-diff code and config
   - list every feature and tool each car has that the other lacks, or has in an older
     form
   - measure the data growth rate
1. **Rename and merge histories.**
   - Rename this repo to `Racerbot-Car-Workspace` on GitHub. Old URLs redirect, but
     update `origin` on both cars and the dev box anyway.
   - Bring car 2's history in with `git merge --allow-unrelated-histories` on a branch,
     and later archive car 2's repo read-only.
2. **Merge code to one version per file, keeping the latest and best of each feature and
   tool.** Never "theirs wins".
   - Read both versions and their commit history, and take the newer or better version
     of each feature. Usually that's the union.
   - Keep the tests from both sides. Where the two versions disagree on behaviour, the
     one with stronger tests and newer real-car evidence wins. Write down each
     non-obvious choice in the merge PR.
   - Code that branches per car (doctor ladders, camera launch, `realsense-ros`
     submodule needed only by rb2) becomes profile-driven.
3. **Add `cars/`, `car_profile.py`, the fail-closed lookup and the hardware
   cross-check.** Tests:
   - an unknown serial refuses
   - an override that disagrees with the table refuses
   - each profile matches its car's expected hardware (replaces
     `test_car1_hardware_config.py` with a version for both cars)
   - the profile allow-list and the safety-never-looser checks
4. **Move calibration out of the shared YAMLs into the profiles.** Prove nothing changed:
   for each car, dump every node's effective parameters before and after (launch-time
   param dump against each repo as it was) and diff. They must be identical, except for
   the step 5 decisions.
5. **Shared tuning plus per-car overrides.** Where `gap_follow.yaml` /
   `pure_pursuit.yaml` / `auto_map_race.yaml` / `vehicle_state.yaml` differ for reasons
   other than calibration, the better-evidenced value becomes shared. A value that is
   really car-specific goes in that car's `tuning.yaml` with its reason. The proposed
   list goes to the team lead for approval before it's committed.
6. **Data folders and LFS:**
   - `.gitattributes` and the LFS migration of car 2's data into `data/rb2/`
   - car 1's `~/.ros` data into `data/rb1/`
   - every tool pointed at `data/<id>/`
   - `save_runs.sh` ported
7. **Retire the porting docs:**
   - rewrite `docs/car1-vs-car2.md` as "how car profiles work"
   - update `CLAUDE.md`, `README.md`, `docs/git-setup.md` and
     `docs/hardware-reference.md` (one table per car, shared geometry)
   - fix the stale "car 2 BNO on USB" comment at `test_car1_hardware_config.py:174` when
     that test is replaced
8. **Roll out per car**, following the usual test order: static topic check, then wheels
   off the ground with LB held, then floor at low speed. Car 2 switches its checkout to
   the merged repo only after car 1 passes, and its first bringup on it must report
   `rb2` and pass the doctor.

## Risks

- **A wrong profile means the wrong servo map and steering limits.** Mitigated by the
  fail-closed lookup, the hardware cross-check, and the before/after parameter-equality
  check in step 4.
- **LFS quota and cost.** Mitigated by measuring in step 0, checking the org's quota
  before migrating, `fetchexclude`, and a pruning policy.
- **The merged code has never run on either car.** Step 8 exists for that reason.
- **A shared change now reaches both cars at once.** That's the point of the merge, but
  "tested on car 1" no longer covers car 2.

## Decisions (team lead, 2026-10-31)

1. **Run recordings:** stay in git, in **Git LFS**.
2. **Per-car tuning:** allowed, in `cars/<id>/tuning.yaml`, on GitHub so the dev box can
   read it.
3. **Repo:** this one survives, renamed **Racerbot Car Workspace**.
4. **Car 2's hardware:** Ethernet Hokuyo UST-10LX, RealSense D435i, BNO085 on Jetson I2C,
   dashboard id `rb2`.
5. **Geometry:** the same on both cars (same chassis), so it's shared.
6. **Merge rule:** keep the latest and best features and tools from either repo.
7. **When:** after work on car 2 is finished and neither repo is moving. Re-clone both
   and merge then, not before.

Assumed, not asked: car 1's old `~/.ros` runs are copied into `data/rb1/`.
