# Unified Car Workspace Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** One repo, "Racerbot Car Workspace", that runs car 1 (`rb1`) and car 2 (`rb2`). It keeps the latest and best version of every feature, test and tool from both car repos. The only things that differ per car are its hardware/calibration profile, its explicit tuning overrides and its data folder.

**Architecture:**
- A new pure-Python package `racerbot_car` works out which car it's running on. It reads the Jetson's serial number, refuses to start if the machine is unknown, and refuses if the plugged-in hardware contradicts the profile.
- Each car's calibration and tuning live as ROS-parameter overlay files. Every launch file loads them on top of the shared configs.
- The car 2 history comes in through one `--allow-unrelated-histories` merge. Each file is merged three-way, with car 1 at `ffbfac0` (the commit car 2 last ported from) as the base.
- Behaviour is proven unchanged by an "effective launch parameters" oracle. It captures what every launch file would start, and with which parameters, in each old repo. The merged repo is diffed against both captures, and only a list of behaviour changes that the team lead approved is allowed to differ.

**Tech Stack:** ROS 2 Jazzy, ament_python, launch / launch_ros, PyYAML, pytest, git + git-lfs, GitHub (`sfu-racerbot` org, Free plan).

**Spec:** this document's "Design" section and the decisions below. The earlier one-page draft of this plan is superseded.

---

## Decisions (team lead, 2026-10-31)

1. One repo for both cars, renamed **Racerbot Car Workspace**. The car 1 repo is the one that stays (but see Q1).
2. The merge keeps the **latest and best features and tools** from both repos. Car 2 has had a lot of gap_follow and SLAM work. Both cars' Claude sessions have added tests and tools, and all of them are kept.
3. **Per-car tuning is allowed.** It is stored in git so the dev box can read it.
4. **Run and log data move to Git LFS**, in per-car folders.
5. **Car 2's hardware:** Ethernet Hokuyo UST-10LX at `192.168.0.10`, RealSense D435i, dashboard id `rb2`.
6. **Both cars:** the BNO085 is on Jetson I2C, and the geometry is the same (same chassis).
7. **When:** both repos were declared final on 2026-10-31.

## Open questions

Answered in chat; each answer gets recorded here before the task it blocks starts.

- **Q1. History and LFS (blocks Tasks 18 and 20).** Both repos already hold their bags in plain git history: car 1 has 1.67 GB, car 2 about 1.1 GB. LFS only covers files committed after it's switched on. The options:
  - **(A)** Rename and keep the history. LFS for new files only. Every future clone still downloads about 2.8 GB of old bags.
  - **(B)** Rename, rewrite the history into LFS, and force-push `main`. Every clone (cars, dev box, worktrees, other sessions' branches) must re-clone.
  - **(C)** Create a new repo with the rewritten history, push it once, and archive both old repos read-only. Same re-clone cost as (B), but nothing is overwritten, and the old repos stay as a complete fallback.

  **Recommended: (C).**
- **Q2. LFS budget (blocks Task 20).** GitHub Free gives 10 GiB of LFS storage and 10 GiB of download bandwidth a month, then $0.07/GiB-month and $0.0875/GiB downloaded, but only if an org admin sets a budget. With the default $0 budget, LFS is blocked for the rest of the month. GitHub can't delete individual LFS objects; storage only shrinks by deleting the repo. At about 2.8 GB now plus about 2 GB a week, 10 GiB runs out in roughly a month. **Recommended:** an org admin sets a monthly LFS budget (e.g. $10), and `save_runs.sh` skips runs where the car never moved, by default.
- **Q3. Car 2 SSH (blocks Task 9).** The dev box key `~/.ssh/id_ed25519.pub` must be authorized for `racerbotcar-2@racerbotcar-2.tetra-rainbow.ts.net`.
- **Q4. Car 2's newer driving behaviour on car 1 (blocks Task 13's approval).**
  - The changes: car 2's gap_follow (`aim_depth_bias`, 5 m swept look-ahead, `opponent_follow_floor`, `time_trial`), its four-state speed filter, the `auto_record` timings, the SLAM gyro tuning, and map_course on the gyro.
  - **Recommended:** take them as shared defaults on both cars, validated in the simulator and then wheels-up and on the floor on car 1 before racing. The alternative is keeping car 1's old values as `rb1` tuning overrides until car 1 is floor-tested.
- **Q5. Dashboard live-tuning save (blocks Task 15).** A small change to `sfu-racerbot/web-dashboards` would let a node's save target be a list of files, with a value saved to the last file that already holds the key. The fallback, with no submodule change: saves always go to the shared file.
- **Q6. Boot services (blocks Task 16).** Car 1's `tools/systemd` set (dashboard, x11vnc/noVNC remote desktop, SSH) would go on both cars and replace car 2's `racerbot-dashboard.service`. Does car 2 have its own Cloudflare tunnel (`rd-rb2` / `ssh-rb2`)?
- **Q7. Data commits during the work.** The cars may keep running `save_runs.sh`. At cutover, any new commits on either `main` must be data-only (checked by script) and get merged in. **No code commits to either `main` until cutover.**
- **Q8. Execution method.** See "Execution" at the end.

---

## Inventory (2026-10-31, refresh in Task 20)

- **Car 1** = `sfu-racerbot/Racerbot-Car-1-Workspace` `main` @ `ae7ceeb`, 381 commits. **Car 2** = `Racerbot-Car-2-Workspace` `main` @ `9c2c2ed`, 234 commits.
- **The histories are unrelated** (no shared commit).
  - Car 2 is fetched into this repo as `refs/car2/main`.
  - A full clone is at `~/racerbot-merge/car2`.
  - Analysis reports are in the job's tmp: `tree_diff.txt`, `port_report.tsv`.
- **Outside `data/`:**
  - 85 files exist only on car 2, 69 only on car 1, and 149 on both with different content.
  - Taking car 1 at `ffbfac0` as the base, 137 of the 149 merge cleanly, 9 conflict, and 3 are submodule links or missing.
  - The 9 conflicts: `CLAUDE.md`, `docs/{car1-vs-car2,hardware-reference,launchstart,run-diagnostics,web-dashboard}.md`, `racerbot_launch/{launch/dashboard_launch.py,scripts/map_course,test/test_map_course_imu.py}`.
  - Every driving-code file merges cleanly, but the result carries car 2's hardware values (see Task 10).
- **Car 2 deleted four car 1 files** that existed at `ffbfac0`: `racerbot_launch/launch/webcam_launch.py`, `racerbot_launch/test/test_car1_hardware_config.py`, `odom_calibration/test/data/vesc_car1_f03f784.yaml` and `race_diagnostics/test/test_record_run_shutdown.py`. Car 1 deleted nothing. **A car 2 deletion is never applied automatically.**
- **Size of each side's changes since `ffbfac0`:**
  - Car 2: 219 files, +20.8k / −1.4k lines (mostly new tools)
  - Car 1: 62 files, +2.3k lines (plus `tools/imu_calibration/`)
- **Only car 2 has:**
  - gap_follow `0a0f39d` (`aim_depth_bias`, swept clearance 3 → 5 m, `opponent_follow_floor`, `time_trial`)
  - the four-state speed filter `588d049`
  - map_course on the gyro `268948d`, rebuild from the bag `4120aed`, SLAM gyro prior `ca79651`
  - `map_localize` `fe0c9ca`, `check_start` `117dde8`, the auto_map_race SLAM correction `949cb6e`
  - auto_record `4ec8e7b` (start delay 1.5 s, poll 0.25 s, `live_topics`)
  - about 25 `tools/run_analysis` tools, `tools/dev/` (mutation checks, node start check, `rostest.sh`), `tools/f1tenth_sim` additions and measured floor profile, `tools/mutations/`
  - `src/realsense-ros` and `racerbot_launch/systemd/racerbot-dashboard.service`
- **Only car 1 has:**
  - `tools/imu_calibration/` (absorbed, with history, holding car 1's calibration)
  - `tools/systemd/` (dashboard, x11vnc, noVNC) and `tools/udev/` (vesc, hokuyo, wiimote LEDs)
  - `tools/run_analysis/{accel_check,clean_map,extract_map,render_maps}.py`
  - `webcam_launch.py` and `test_record_run_shutdown.py`
- **Calibration that differs** (car 1 → car 2):
  - `joy_teleop` scale 0.38 → 0.436
  - `sensors.yaml` `serial_port: /dev/sensors/hokuyo` → `ip_address: 192.168.0.10`, `ip_port: 10940`
  - `vesc.yaml`: `speed_to_erpm_gain` 3650 → 4614, servo gain/offset −0.83/0.366 → −0.78/0.5904, `servo_min`/`max` 0.05/0.75 → 0.15/0.95, `vesc_to_odom` `speed_to_erpm_gain` −4040 → −4447.983786
  - `max_steering_angle` 0.33 → 0.349 in `gap_follow`, `pure_pursuit` and `neural_racer`, and `planner_max_steering_rad` in `odom_calibration`
  - `arc_guard_max_correction` 0.165 → 0.1745
  - `odom_calibration.yaml` `vehicle_model` and its own copies of the VESC values
  - the dashboard camera (`device: /dev/video0` → RealSense `image_topic`)
- **Submodules** for the merged repo:
  - `racerbot_a` `8ef1d8c` (car 1's pin, team A's latest)
  - `web_dashboards` `f5b711d` (car 2's pin, that repo's latest)
  - `realsense-ros` `775b167` (car 2 only)
  - the others are identical on both sides
- **Tests:** car 1 has 2,832 `def test` in 208 files, car 2 has 3,304 in 222. 194 files are on both sides, 14 only on car 1 and 28 only on car 2.
- **Data in git (plain):** car 1 `data/` 1.67 GB (bags split at 50 MB; one `analysis/run.npz` is 46 MB), car 2 about 1.1 GB.
- **Car 1's Jetson** (read-only check): hostname `racerbot-car1`, serial `1421723021900`, clean at `ae7ceeb`, no git-lfs, 382 GB free. `~/.ros/racerbot_runs` holds only 15 `replay-odom_imu` experiments, whose source runs are already in `data/`, so they are not imported.
- **Car 2's Jetson:** not yet surveyed (Q3).
- **Hard-coded paths:**
  - `~/racerbot-ws/data/racerbot_*` appears in about 20 launch defaults, node defaults and YAMLs across `race_diagnostics`, `racerbot_launch`, `pure_pursuit`, `racerbot_sim`, `tools/run_analysis`
  - `/home/racerbotcar-1` appears in `waypoint_recorder.yaml`, `waypoint_recorder_node.py`, `waypoint_recorder_launch.py`, `tools/systemd/install.sh`, `.vscode/settings.json`

## Global Constraints

- **The safety policy doesn't change.** Every node that can publish `/drive`, `/ackermann_cmd` or `/commands/motor|servo/*` keeps its own LB deadman gate. `enable_deadman: false` never appears in shipped config or in any `cars/` overlay.
- **CLAUDE.md test-quality Hard rules 1 and 2 apply to every task.** Every new test is seen to fail against a stub before it counts. A failing test is never weakened or deleted to make a suite pass. A test that looks wrong gets flagged to the team lead.
- **ROS tests always run sourced on an isolated domain:** `RB_WS=<worktree> rb python3 -m pytest ...`. `rb` sets `ROS_DOMAIN_ID=79` and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, and `RB_WS` points it at the worktree's own build. Without `RB_WS`, `rb` tests `~/racerbot-ws`, not the branch. Never use `--continue-on-collection-errors`.
- **Build on the dev box** with `RB_WS=<ws> rb colcon build --symlink-install --packages-skip range_lib`. Also skip `realsense2_camera realsense2_camera_msgs realsense2_description` if librealsense is missing. On a Jetson, add `--parallel-workers 1`.
- **`racerbot_car` imports no `rclpy`,** so it works unsourced, like `drive_intent`.
- **Car ids** match `^rb[0-9]+$`. Today they are `rb1` and `rb2`.
- **Git:** never push or force-push `main` of any existing repo, and never merge. Work goes on branch `unify/main` in this repo, through draft PRs. Cutover (Task 20) follows the team lead's Q1 answer.
- **Never edit `src/racerbot_a`** (team A's repo). Pin it only.
- **Car 1 hardware facts stay true for `rb1`:**
  - USB-serial Hokuyo UST-30LX at `/dev/sensors/hokuyo` (USB `15d1:0000`, `cdc_acm`)
  - UVC webcam at `/dev/video0`
  - the servo map, `speed_to_erpm_gain` values and steering limits listed in the inventory
  - dashboard id `rb1`
  - BNO085 on I2C bus 7 at `0x4A` through `tools/imu_calibration/tools/bno_i2c_bridge.py --pty /tmp/bno085 --no-dyncal`
- **Car 2 hardware facts stay true for `rb2`:**
  - Ethernet UST-10LX at `192.168.0.10:10940` on `enP8p1s0` (host `192.168.0.15/24`)
  - RealSense D435i (USB `8086:0b3a`, topic `/camera/camera/color/image_raw`)
  - the servo map, `speed_to_erpm_gain` values and steering limits listed in the inventory
  - dashboard id `rb2`
  - BNO085 on Jetson I2C
- **Shared on both cars:** wheelbase 0.36, `base_link -> laser` `0.26 0.0 0.11`, `car_width` 0.31, `car_length` 0.58, `bno_x_offset` 0.36.
- **Agents:** at most 3 paid Claude agents at once and at most 1 Haiku. Driving-code diffs and merge decisions are reviewed inline by the controller. Load on the shared dev box is checked before full suites (bananasaurs runs there too).
- **Docs** follow the `novice-docs` and `racerbot-docs-voice` skills.

## Review Focus

1. **The wrong car id on a Jetson would put the other car's servo map and steering limits on it.** Tests: an unknown serial refuses, and an env override that disagrees with the table refuses (Task 3). A hardware contradiction stops bringup: an `rb2` profile with a Hokuyo USB id present, or an `rb1` profile with a RealSense present (Task 5).
2. **A launch path that reads a shared config without the car overlay would silently fall back to the node's declared defaults,** since the calibration keys leave the shared files. Tests:
   - `layered()` raises if any required calibration key is missing from the car's overlay (Task 4).
   - A static test forbids direct readers of split configs (Task 5).
   - Zero-diff `launch_params` gates (Tasks 5 and 13).
3. **A car 2 deletion or omission could silently drop a car 1 feature,** such as `test_record_run_shutdown.py` or the `record_run.py` shutdown fix from car 1's PR #9. Tests: the ledger has to cover 100% of hunks, each with a decision (Task 10). The merged test-function count is at least the union of both sides (Task 11).
4. **A tuning overlay could loosen a safety limit, or carry a mistyped key that does nothing.** Tests: the guard rules with deny cases for every rule (Task 14).
5. **A car without git-lfs would commit bags as plain git.** Tests:
   - `save_runs.sh` refuses without LFS.
   - `git check-attr filter` reports `lfs` for the bag, npz, pgm and png paths of both cars.
   - A pre-commit guard rejects any staged blob over 10 MB under `data/` that isn't an LFS pointer (Task 17).
6. **A test that reads one YAML value could pass on `rb1` values only.** Tests: every config-reading test that a calibration key affects runs once per car (Task 5 step 4, Task 12).

---

## Design

### Identity

- `src/racerbot_car/cars/machines.yaml` maps each Jetson's `/proc/device-tree/serial-number` (NUL-stripped) to a car id: `{"1421723021900": rb1, "<car 2 serial>": rb2}`.
- `RACERBOT_CAR` overrides the lookup only on a machine that isn't in the table (the dev box, the simulator). On a machine that is in the table, a disagreeing override refuses.
- An unknown machine with no override refuses, with a message that prints the serial and the line to add to `machines.yaml`.
- Every bringup and every control-layer launch resolves the id while the launch description is being generated. Before any node starts, it runs the hardware contradiction check:
  - Each profile lists `distinctive_usb_ids`: `rb1` → `15d1:0000`, `rb2` → `8086:0b3a`.
  - If another car's distinctive id is present on this machine, the launch refuses.
  - An unplugged sensor never blocks a launch; only positive evidence of the other car's hardware does.
- Simulator launches need `car:=<id>`, which defaults to `$RACERBOT_CAR`.

### Car folder (installed as package data, resolved to `src/` via realpath like `odom_calibration` does)

```
src/racerbot_car/cars/
  machines.yaml
  rb1/
    car.yaml                 # identity + hardware descriptors (schema in Task 3)
    calibration/             # ROS-param overlays, keys limited by the calibration allow-list
      vesc.yaml joy_teleop.yaml sensors.yaml
      gap_follow.yaml pure_pursuit.yaml neural_racer.yaml odom_calibration.yaml
    tuning/                  # ROS-param overlays of shared tuning; every key has a "# why:" comment
    imu/                     # racer_imu config/car.yaml + calib/*.json for this car
  rb2/ (same)
```

- **Load order for a config file:** shared file, then `calibration/<file>`, then `tuning/<file>`. The last one wins.
- **Dashboard site configs** stay at `racerbot_launch/config/web_dashboard_<id>.yaml`. `dashboard_launch.py` picks the one for the current car.

### Data

- **Folders:** `data/<id>/{racerbot_runs,racerbot_auto,racerbot_sim,imu_calibration}/`, resolved by `racerbot_car.paths.data_dir(kind)`. Nothing else may spell out a data path.
- **`save_runs.sh`:** commits only `data/<this car>/`, and refuses without git-lfs.
- **LFS** holds `*.mcap`, `*.db3`, `*.npz`, `*.pgm`, `*.png`, `*.jpg` under `data/`.
- **What each machine downloads by default:** each car skips the other car's data (`lfs.fetchexclude=data/<other>/**`). The dev box skips all of `data/**` and fetches runs on demand.

---

## File Structure

| Path | Responsibility |
|---|---|
| `src/racerbot_car/` (new ament_python) | `identity.py` (serial, machines, resolve), `paths.py` (workspace and data dirs), `profile.py` (`car.yaml` schema), `layers.py` (overlay files, effective values), `guards.py` (calibration allow-list, safety rules), `hardware.py` (USB-id contradiction check), `cars/` (data), `test/` |
| `tools/dev/launch_params.py` (new) + `tools/dev/launch_matrix.yaml` + `tools/dev/tests/test_launch_params.py` | the effective-parameter oracle |
| `tools/dev/unify/three_way.py`, `hunk_ledger.py` (new, deleted at cutover) | merge tooling |
| `docs/superpowers/plans/unify-ledger.md`, `unify-behaviour-changes.md`, `unify-car2-survey.md` (new) | review artifacts |
| `src/f1tenth_system/f1tenth_stack/launch/*bringup_launch.py`, `teleop_launch.py` | identity + hardware check + layered configs (vendored Addition 7, `docs/git-setup.md`) |
| every other launch that loads a split config | layered configs |
| `src/odom_calibration/odom_calibration/calibration_node.py` | write targets are the car's overlays |
| `src/racerbot_doctor/racerbot_doctor/{car,checks,cli,live}.py` | car-independent specs; both ladders; car id + overrides in the report |
| `src/race_diagnostics/race_diagnostics/run_metadata.py` | records car id, overlay files and hashes |
| `src/racerbot_launch/test/test_car_profiles.py` (replaces `test_car1_hardware_config.py`) | both cars' hardware facts |
| `tools/save_runs.sh`, `.gitattributes`, `tools/dev/setup_clone.sh`, `tools/dev/hooks/pre-commit` | LFS |
| `docs/car-profiles.md` (new; replaces `docs/car1-vs-car2.md`), `CLAUDE.md`, `README.md`, `docs/{git-setup,hardware-reference,operations,launchstart,run-diagnostics,web-dashboard}.md` | docs |

---

## Phase 1: References and the oracle

### Task 1: Reference builds of both old repos

**Files:** none in the repo. Creates `~/racerbot-merge/ref-car1` (clone of this repo at `ae7ceeb`) and uses `~/racerbot-merge/car2` (at `9c2c2ed`).

- [ ] **Step 1:** `git clone /home/bryan/racerbot-ws ~/racerbot-merge/ref-car1 && git -C ~/racerbot-merge/ref-car1 checkout ae7ceeb && git -C ~/racerbot-merge/ref-car1 submodule update --init --recursive`. Do the same for `car2` at `9c2c2ed`.
- [ ] **Step 2:** Build each: `RB_WS=~/racerbot-merge/ref-car1 rb colcon build --symlink-install --packages-skip range_lib`. For car 2, also skip the realsense packages if they fail.
  **Expected:** `Summary: N packages finished`, with no failures beyond the skipped packages.
- [ ] **Step 3:** Run both full suites as they are, to record the baseline: `RB_WS=<ref> rb python3 -m pytest src tools/f1tenth_sim/tests tools/racerbot_sim/tests tools/run_analysis/tests -q -p no:cacheprovider`. Save the pass/fail lists to `~/racerbot-merge/ref/<car>-tests.txt`. Known x86 flakes (corner-setup, trust-region 1.6e-6, the two imu_calibration `test_online` failures that depend on car 1's mount) are recorded, not fixed.

### Task 2: The effective-launch-parameters oracle

**Files:**
- Create: `tools/dev/launch_params.py`, `tools/dev/launch_matrix.yaml`, `tools/dev/tests/test_launch_params.py`, `tools/dev/tests/fixtures/launch_params/{demo_launch.py,demo.yaml,overlay.yaml}`

**Interfaces:**
- Produces:
  - `capture(launch_package: str, launch_file: str, launch_args: list[str]) -> dict`, returning `{"nodes": [{"package", "executable", "name", "namespace", "params": {dotted_key: value}, "remaps": [[from, to]]}], "processes": [[cmd...]]}`, sorted by (namespace, name, executable)
  - `flatten_params(entries: list[str | dict], node_fqn: str) -> dict`
  - `diff_captures(a: dict, b: dict) -> list[str]`, with lines like `"+ /gap_follow_node aim_depth_bias=0.0"`, `"- ..."`, `"~ /vesc_driver_node servo_min 0.05 -> 0.15"`
  - CLI: `launch_params.py capture --matrix tools/dev/launch_matrix.yaml --car <id> --out <dir>` and `launch_params.py diff <dirA> <dirB> [--allow <changes.md>]`
- **Approach:** patch `launch_ros.actions.Node.execute` so it runs `_perform_substitutions(context)`, records `self._Node__expanded_parameter_arguments`, node name, namespace and remappings, and returns `None`. Patch `launch.actions.ExecuteProcess.execute` to record the command and return `None`. Then run `LaunchService.include_launch_description(...)` + `run()`. Nothing is ever spawned.
- **Flattening:** files are loaded with PyYAML. Blocks keyed by `/**`, by the node's name or by its fully qualified name apply, in list order, and later ones win. Nested dicts flatten to dotted keys.

- [ ] **Step 1: Write the failing tests:**
  - `test_capture_never_spawns`: the fixture launch has a Node whose executable doesn't exist and an `ExecuteProcess(['false'])`. Capture returns both, and `psutil` shows no child process.
  - `test_flatten_later_entry_wins`: `[demo.yaml, overlay.yaml]` → `params['servo_min'] == 0.15`, where `demo.yaml` sets 0.05 and the overlay sets 0.15.
  - `test_flatten_wildcard_and_fqn_blocks`: a `/**` block plus a `/ns/node` block both apply.
  - `test_flatten_nested_dict_dotted`: `human_control: {axis_mappings: {drive-steering_angle: {scale: 0.38}}}` → key `human_control.axis_mappings.drive-steering_angle.scale`.
  - `test_condition_false_node_absent`: `IfCondition('false')` → the node isn't in the output.
  - `test_include_followed`: a node from an included launch file is present.
  - `test_diff_reports_added_removed_changed`: exact lines for one of each.
  - `test_matrix_covers_every_launch_file`: every `*_launch.py` and `record_run.py` under `src/` (excluding `racerbot_a`, `racerbot_b`, `web_dashboards`, `realsense-ros`, `particle_filter`) is listed in `launch_matrix.yaml` or under its `excluded:` with a reason.
- [ ] **Step 2:** `RB_WS=<worktree> rb python3 -m pytest tools/dev/tests/test_launch_params.py -v`. **Expected:** FAIL, because the module doesn't exist.
- [ ] **Step 3:** Implement `launch_params.py`. Fill `launch_matrix.yaml`: one entry per launch file × the variants people actually run (from `docs/operations.md` and `docs/launchstart.md`). For example, `gap_follow_launch.py` with defaults, `opponent_awareness:=false`, `launch_boost:=false`, `fused_speed:=true`; `race_launch.py run_dir:=<fixture run dir in tools/dev/tests/fixtures>`. Entries that only apply to one car are marked `cars: [rb1]` (e.g. `webcam_launch.py`, `realsense_camera_launch.py`).
- [ ] **Step 4:** Tests pass. Then the Hard rule 1 proof: make `flatten_params` return `{}`, and make `diff_captures` return `[]`. Each must turn its tests red; then revert.
- [ ] **Step 5: Capture the references:**
  - `RB_WS=~/racerbot-merge/ref-car1 rb python3 <worktree>/tools/dev/launch_params.py capture --car rb1 --out ~/racerbot-merge/ref/rb1`
  - the same with car 2 → `ref/rb2`

  The old repos have no `racerbot_car`, so `--car` only selects matrix entries there. **Expected:** one JSON file per entry, no errors. An entry that fails in an old repo is recorded in the matrix as `ref_fails: <reason>`.
- [ ] **Step 6: Commit** `tools/dev/launch_params.py tools/dev/launch_matrix.yaml tools/dev/tests/`: "tools/dev: launch_params, what every launch would start and with which parameters".

---

## Phase 2: Profiles on car 1 alone (zero behaviour change, proven)

Work on branch `unify/main`, created from `origin/main` (`ae7ceeb`).

### Task 3: `racerbot_car` package: identity, paths, profile

**Files:**
- Create: `src/racerbot_car/{package.xml,setup.py,setup.cfg,resource/racerbot_car,racerbot_car/__init__.py,identity.py,paths.py,profile.py}`, `src/racerbot_car/cars/machines.yaml`, `src/racerbot_car/cars/rb1/car.yaml`, `src/racerbot_car/test/{test_identity.py,test_paths.py,test_profile.py}`
- `setup.py` installs `cars/**` under `share/racerbot_car/cars`.

**Interfaces (produces):**
- `identity.SERIAL_PATH = '/proc/device-tree/serial-number'`; `class UnknownCarError(RuntimeError)`; `class CarMismatchError(RuntimeError)`
- `identity.read_serial(path: str = SERIAL_PATH) -> str | None`: strips NULs and whitespace; returns `None` if the file is missing, unreadable or empty
- `identity.load_machines(path: Path, known_ids: set[str]) -> dict[str, str]`: `ValueError` on a duplicate serial, an id not in `known_ids`, or an id not matching `^rb[0-9]+$`
- `identity.resolve_car_id(env: Mapping[str, str], serial: str | None, machines: dict[str, str], known_ids: set[str]) -> str`
- `identity.current_car_id() -> str`: real env, serial and `paths.cars_dir()`
- `paths.cars_dir() -> Path`: `$RACERBOT_CARS_DIR`, else `realpath(share/racerbot_car/cars)`
- `paths.workspace_root() -> Path`: `$RACERBOT_WS`, else `~/racerbot-ws`
- `paths.DATA_KINDS = ('racerbot_runs', 'racerbot_auto', 'racerbot_sim', 'imu_calibration')`; `paths.data_dir(kind: str, car_id: str | None = None) -> Path`: `ValueError` on an unknown kind
- `profile.CarProfile` (frozen dataclass): `id`, `name`, `lidar: LidarSerial(port, usb_id) | LidarEthernet(ip, port, iface, host_ip)`, `camera: CameraUVC(device) | CameraRealSense(usb_id, topic) | None`, `dashboard_site: str`, `hostnames: dict[str, str]`, `distinctive_usb_ids: tuple[str, ...]`
- `profile.load_profile(car_id: str, cars_dir: Path | None = None) -> CarProfile`: `ProfileError` on an unknown key, a missing key or a wrong type
- `cars/rb1/car.yaml`:
  - `name: car 1`
  - `lidar: {kind: serial, port: /dev/sensors/hokuyo, usb_id: "15d1:0000"}`
  - `camera: {kind: uvc, device: /dev/video0}`
  - `dashboard_site: rb1`
  - `hostnames: {remote_desktop: rd-rb1, ssh: ssh-rb1}`
  - `distinctive_usb_ids: ["15d1:0000"]`

- [ ] **Step 1: Write the failing tests:**
  - `test_serial_strips_nul` (`b"1421723021900\x00"` → `"1421723021900"`)
  - `test_serial_missing_file_is_none`, `test_serial_empty_is_none`
  - `test_known_serial_resolves`
  - `test_known_serial_with_matching_env_resolves`
  - `test_known_serial_with_other_env_refuses` (`pytest.raises(CarMismatchError, match="rb2")`)
  - `test_unknown_serial_without_env_refuses_and_prints_line` (`match=r'"999": rb1'`-style hint containing the serial)
  - `test_unknown_serial_with_env_resolves`
  - `test_no_serial_with_env_resolves` (dev box)
  - `test_env_not_a_known_car_refuses`
  - `test_machines_duplicate_serial_rejected`, `test_machines_unknown_id_rejected`
  - `test_data_dir_layout` (`RACERBOT_WS=/x` → `/x/data/rb1/racerbot_runs`), `test_data_dir_unknown_kind_rejected`
  - `test_profile_rb1_matches_car1_facts` (oracle: the Global Constraints list), `test_profile_unknown_key_rejected`, `test_profile_missing_lidar_rejected`, `test_profile_wrong_lidar_kind_rejected`
- [ ] **Step 2:** Run them (they're pure Python, so unsourced works): `python3 -m pytest src/racerbot_car/test -v`. **Expected:** FAIL on import.
- [ ] **Step 3:** Implement. Rules for `resolve_car_id`: a serial in the table wins, and an env value that disagrees raises `CarMismatchError`. With no table match, an env value in `known_ids` is used. Anything else raises `UnknownCarError`.
- [ ] **Step 4:** Tests pass. Hard rule 1: make `resolve_car_id` return `'rb1'`; at least the mismatch, unknown and env tests must fail. Make `read_serial` skip the NUL strip; `test_serial_strips_nul` must fail. Revert.
- [ ] **Step 5: Commit** "racerbot_car: which car this is, and where its files live".

### Task 4: Overlays, effective values, and the calibration allow-list

**Files:** Create `src/racerbot_car/racerbot_car/{layers.py,guards.py}` and `src/racerbot_car/test/{test_layers.py,test_guards_calibration.py}`.

**Interfaces:**
- Consumes: `paths.cars_dir()`, `identity.current_car_id()`
- Produces:
  - `layers.overlay_files(config_name: str, car_id: str | None = None) -> list[str]`: the existing `[cars/<id>/calibration/<name>, cars/<id>/tuning/<name>]`, in that order
  - `layers.layered(shared_path: str, config_name: str, car_id: str | None = None) -> list[str]`: `[shared_path, *overlay_files]`. Raises `MissingCalibrationError` if any key that `guards.CALIBRATION_KEYS[config_name]` requires is missing from the car's calibration overlay.
  - `layers.effective_values(paths: list[str], node_name: str) -> dict`: the same flattening rules as Task 2's `flatten_params`, implemented separately. The oracle must run in the old repos, which have no `racerbot_car`. `test_effective_values_agrees_with_launch_params` runs both on Task 2's fixtures and requires identical dicts.
  - `guards.CALIBRATION_KEYS: dict[str, dict[str, frozenset[str]]]`, mapping config name → node block → dotted keys. Initial content: every key in the inventory's calibration list. For example, `'vesc.yaml': {'vesc_driver': {...'servo_min','servo_max','speed_to_erpm_gain'...}, 'ackermann_to_vesc_node': {...}, 'vesc_to_odom_node': {'speed_to_erpm_gain', ...}}`. Node block names are copied exactly from the current YAMLs.
  - `guards.check_calibration_overlay(path: Path, config_name: str) -> list[str]`: problems, e.g. `"rb1/calibration/vesc.yaml: vesc_driver.max_speed is not a calibration key"`

- [ ] **Step 1: Write the failing tests:**
  - `test_layered_order_shared_calibration_tuning`
  - `test_layered_raises_when_required_key_missing` (`match="servo_min"`)
  - `test_layered_no_overlay_for_unsplit_config` (`mux.yaml` → `[shared]` only)
  - `test_effective_values_overlay_wins`
  - `test_check_overlay_rejects_non_calibration_key`
  - `test_check_overlay_rejects_unknown_node_block`
  - `test_every_car_has_every_required_calibration_key`, parametrized over the `cars/*` dirs
- [ ] **Step 2:** Run. **Expected:** FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Tests pass. Hard rule 1: make `layered` skip the missing-key check, and make `overlay_files` return the files reversed. Both must turn tests red. Revert.
- [ ] **Step 5: Commit.**

### Task 5: Move car 1's calibration into `cars/rb1`, and wire every launch

**Files:**
- Create: `src/racerbot_car/cars/rb1/calibration/{vesc,joy_teleop,sensors,gap_follow,pure_pursuit,neural_racer,odom_calibration}.yaml`, `src/racerbot_car/racerbot_car/hardware.py`, `src/racerbot_car/test/test_hardware.py`, `src/racerbot_launch/test/test_launches_layer_car_overlays.py`
- Modify:
  - the shared YAMLs: remove exactly the keys in `CALIBRATION_KEYS`, and replace each one with the comment `# per car: src/racerbot_car/cars/<id>/calibration/<file>`
  - every launch file that passes one of those configs: the three `*bringup_launch.py`, `teleop_launch.py`, `gap_follow_launch.py`, `pure_pursuit_launch.py`, the `racerbot_launch` launches that include them, `neural_racer`'s launch, `odom_calibration_launch.py`, the sim launches
  - every Python reader of those YAMLs, found with `grep -rln -E "(vesc|joy_teleop|sensors|gap_follow|pure_pursuit|neural_racer|odom_calibration)\.yaml" src tools --include=*.py`
  - `racerbot_launch/test/test_steering_limits_fit_servo.py` and every other test reading a moved key: parametrize over `cars/*`
  - `src/f1tenth_system/f1tenth_stack/package.xml` and the other packages' `package.xml`: add `<exec_depend>racerbot_car</exec_depend>`
  - `docs/git-setup.md`: vendored Addition 7

**Interfaces:**
- Consumes: Task 4's `layered`, `effective_values`; Task 3's `current_car_id`, `load_profile`
- Produces:
  - `hardware.usb_ids_present(root: Path = Path('/sys/bus/usb/devices')) -> set[str]`, `"vvvv:pppp"` lowercase, read-only
  - `hardware.contradiction(profile: CarProfile, all_profiles: list[CarProfile], present: set[str]) -> str | None`: names the other car and the id found
  - `hardware.CarHardwareError(RuntimeError)`
  - `layers.launch_car_id() -> str` (added to Task 4's module): `current_car_id()`, then `hardware.contradiction(load_profile(id), all profiles, usb_ids_present())`; raises `CarHardwareError` with the contradiction message

- [ ] **Step 1: Write the failing tests:**
  - `test_contradiction_rb1_with_realsense_present`: `rb1` + `{"8086:0b3a"}` → mentions `rb2`
  - `test_contradiction_rb2_with_hokuyo_present`
  - `test_no_contradiction_when_sensor_unplugged`: `rb1` + `set()` → `None`
  - `test_usb_ids_read_from_sysfs_fixture`
  - In `test_launches_layer_car_overlays.py`:
    - `test_no_launch_passes_split_config_unlayered` (static: any launch source that names a split config must call `layered(`)
    - `test_no_python_reads_split_config_directly` (static, with an allow-list that gives a reason per entry)
    - `test_bringup_refuses_unknown_machine`: build `generate_launch_description()` with `RACERBOT_CARS_DIR` pointing at a fixture where the serial is unknown and no env is set; `pytest.raises(UnknownCarError)`
    - `test_bringup_refuses_contradicting_hardware`: a fixture `sysfs` holding `8086:0b3a` under `rb1` → `CarHardwareError`
- [ ] **Step 2:** Run. **Expected:** FAIL.
- [ ] **Step 3:** Write the `rb1` overlays: car 1's current values, byte-for-byte the same numbers, with the existing "why" comments moved along with them. Remove the keys from the shared YAMLs. Wire every launch and reader through `layered` / `effective_values`. In the bringups, call `launch_car_id()` first.
- [ ] **Step 4:** Parametrize the calibration-reading tests over `rb1` (and `rb2` once it exists). Each test's oracle stays its existing one: the servo clip math, and the docs quoted in `docs/hardware-reference.md`.
- [ ] **Step 5: The zero-diff gate:**
  - build the worktree: `RB_WS=<wt> rb colcon build --symlink-install --packages-skip range_lib`
  - capture: `RB_WS=<wt> RACERBOT_CAR=rb1 rb python3 tools/dev/launch_params.py capture --car rb1 --out ~/racerbot-merge/cap/p2-rb1`
  - diff: `python3 tools/dev/launch_params.py diff ~/racerbot-merge/ref/rb1 ~/racerbot-merge/cap/p2-rb1`

  **Expected: no output.** Any line is a bug in this task, never an approved change.
- [ ] **Step 6:** Run the full suites as in Task 1, step 3. **Expected:** the same pass/fail lists as `ref/car1-tests.txt` plus the new tests. Hard rule 1: remove one key from `cars/rb1/calibration/vesc.yaml`; both `test_every_car_has_every_required_calibration_key` and the zero-diff gate must catch it. Revert.
- [ ] **Step 7: Commit:** "Car 1's calibration moves into src/racerbot_car/cars/rb1; every launch loads it on top of the shared configs".

### Task 6: `odom_calibration` writes to the car's overlays

**Files:**
- Modify: `src/odom_calibration/odom_calibration/calibration_node.py` (`_resolve_config` → `racerbot_car` overlay paths), `src/odom_calibration/config/odom_calibration.yaml` (the duplicate VESC values move to `cars/<id>/calibration/odom_calibration.yaml`)
- Test: `src/odom_calibration/test/test_vesc_yaml.py`, plus a new `test_writes_target_car_overlay.py`

**Interfaces:**
- Consumes: `layers.overlay_files`
- Produces: `calibration_node.CalibrationNode.vesc_yaml_path == realpath(cars/<id>/calibration/vesc.yaml)` when `vesc_yaml_path` is empty, and the same for `joy_teleop.yaml`.

- [ ] **Step 1: Failing tests:**
  - `test_default_write_target_is_car_overlay` (`RACERBOT_CAR=rb1`, fixture cars dir)
  - `test_apply_edits_overlay_block_keys_only`: `vesc_yaml.apply` on an overlay that holds only the calibration blocks changes `servo_min` and nothing else
  - `test_odom_calibration_copy_matches_vesc_overlay`, per car: the VESC values in `cars/<id>/calibration/odom_calibration.yaml` equal those in `cars/<id>/calibration/vesc.yaml`. Oracle: they describe the same hardware.
- [ ] **Step 2:** Run → FAIL. **Step 3:** Implement. **Step 4:** Pass. Mutation proof: point the default back at the shared file, and the first test fails. **Step 5:** Commit.

### Task 7: Data folders per car (car 1 side)

**Files:**
- Move: `data/racerbot_runs`, `data/racerbot_auto`, `data/imu_calibration` → `data/rb1/…` (`git mv`; `data/README.md` and `COLCON_IGNORE` stay at `data/`)
- Modify: every file listed in the inventory's hard-coded data-path note (find with `grep -rn "racerbot-ws/data/racerbot_" src tools --include=*.py --include=*.yaml --include=*.sh`); `tools/save_runs.sh`; the dashboard's run list and delete roots in `web_dashboard_rb1.yaml`
- Test: `src/racerbot_car/test/test_no_bare_data_paths.py`, `src/racerbot_launch/test/test_run_output_paths.py` (existing; extend)

**Interfaces:**
- Consumes: `paths.data_dir`
- Produces:
  - Node parameters `output_directory` default to `''`, meaning `data_dir(<kind>)` at node start.
  - Launch defaults are computed from `data_dir`.

- [ ] **Step 1: Failing tests:**
  - `test_no_bare_data_paths`: no `.py`, `.yaml`, `.sh` or `.launch.py` under `src/` and `tools/` contains `racerbot-ws/data/racerbot_` or `data/racerbot_runs` without a car segment. Docs and `data/README.md` are exempt.
  - `test_auto_record_default_is_car_folder` (`RACERBOT_CAR=rb1`, `RACERBOT_WS=tmp` → `tmp/data/rb1/racerbot_runs`)
  - `test_auto_map_race_default_is_car_folder`
  - `test_save_runs_stages_only_this_car`: a shell test in a temp git repo; files in `data/rb2/` stay unstaged under `RACERBOT_CAR=rb1`
- [ ] **Steps 2–4:** fail → implement → pass. Mutation proof: hard-code `rb1` in `data_dir`, and the `rb2` cases must fail.
- [ ] **Step 5:** Re-run the zero-diff gate. Expected diffs are only `output_directory`-style values changing from `~/racerbot-ws/data/X` to `''`, or to the `data/rb1/X` path. List them in `unify-behaviour-changes.md` under "path only". **Commit.**

### Task 8: IMU calibration per car (car 1 side)

**Files:**
- Move: `tools/imu_calibration/config/car.yaml` → `src/racerbot_car/cars/rb1/imu/car.yaml`, and `tools/imu_calibration/calib/*.json` → `src/racerbot_car/cars/rb1/imu/calib/`
- Modify: `src/imu_bridge/config/imu_bridge.yaml` and `launch/{imu_bridge_launch,vehicle_state_launch}.py` (pass the car's `imu/car.yaml` as `racer_imu`'s `--config`, and the calib dir); `race_diagnostics/run_metadata.py` (copy the car's IMU calibration into `metadata/`); `tools/imu_calibration/README.md`
- Test: `src/imu_bridge/test/test_imu_calibration_per_car.py`

- [ ] **Step 1: Failing tests:**
  - `test_imu_bridge_launch_uses_car_imu_config` (the launch description's arguments name `cars/rb1/imu/car.yaml`)
  - `test_run_metadata_copies_car_imu_calibration`
  - `test_racer_imu_loads_calib_relative_to_car_dir`: the `bno_calib.json` values that load equal the file's values
- [ ] **Steps 2–5:** fail → implement → pass → mutation proof (point at `rb2`, and the oracle values differ) → commit. Zero-diff gate: only the config-path parameters change ("path only").

**Phase 2 gate:**
- full suites green relative to the reference
- the zero-diff gate holds, with only path-only changes
- `ros2 run racerbot_doctor doctor` unit tests pass

Open a draft PR `unify/main` → `main` titled "Unified workspace, phase 2: car profiles (car 1 only)". It is for review only and is not merged until cutover.

---

## Phase 3: Bring in car 2

### Task 9: Car 2 survey over SSH (read-only)

**Blocked by Q3.** Output: `docs/superpowers/plans/unify-car2-survey.md`.

- [ ] **Step 1:** Over SSH, read-only:
  - hostname and serial (`tr -d '\0' < /proc/device-tree/serial-number`)
  - the checkout: `git -C ~/racerbot-ws status --short`, `git log -1`, and the ahead/behind counts against `origin/main`
  - `ls ~/imu-calibration`, `git -C ~/imu-calibration status --short`, and copies of `config/car.yaml` and `calib/*.json` (`scp` into `~/racerbot-merge/car2-imu/`)
  - `systemctl list-unit-files | grep -E 'racerbot|dashboard|vnc|cloudflared|foxglove'`
  - `ls /etc/udev/rules.d`
  - `id`
  - `nmcli -t con show`
  - `lsusb`
  - `git lfs version`, `df -h ~`
  - `du -sh ~/.ros/racerbot_* ~/racerbot-ws/data/*`
- [ ] **Step 2:** Record anything uncommitted or unpushed on car 2. Raise it with the team lead before continuing, because it would be lost at cutover.
- [ ] **Step 3:** Add `"<serial>": rb2` to `machines.yaml` (committed in Task 12).

### Task 10: The hunk ledger, and the behaviour-change list

**Files:** Create `tools/dev/unify/hunk_ledger.py`, `docs/superpowers/plans/unify-ledger.md`, `docs/superpowers/plans/unify-behaviour-changes.md`.

**Interfaces:** `hunk_ledger.py <base> <side-ref> --paths-from tree_diff.txt` → one markdown row per hunk: `id | file | hunk header | -lines/+lines | class | decision | evidence`. Rows are pre-filled with `git log -L` / blame commit hashes from the side's history, so each hunk names the commit that made it.

- [ ] **Step 1:** Generate the ledger for `ffbfac0 → refs/car2/main` and for `ffbfac0 → ae7ceeb`, over every file present on both sides and every file one side deleted. Leave out new files, which are added whole, and `data/`.
- [ ] **Step 2:** Classify every row. **The controller does this inline; it is never delegated.** The classes:
  - `hw-rb2` / `hw-rb1`: car hardware or calibration → that car's overlay or profile
  - `feature`: keep it, shared
  - `skipped`: the other car never ported it → keep the car 1 or car 2 version that has it
  - `removal`: a deliberate removal → keep the removal, with evidence
  - `per-car-tuning`: a deliberate per-car behaviour difference (e.g. car 2 keeping IMU features off until it records its own thresholds) → that car's `tuning/` overlay with a `# why:`

  Known hotspots to investigate first:
  - `race_diagnostics/launch/record_run.py` together with `test_record_run_shutdown.py`: car 1's PR #9 shutdown fix versus car 2's `auto_record` rework
  - `racerbot_doctor/*` (both ladders survive)
  - `imu_bridge/vehicle_state.py` (car 2's four-state filter)
  - `gap_follow/config/gap_follow.yaml` (car 2's comments that keep features off)
  - `auto_map_race_node.py`
- [ ] **Step 3:** A check in `hunk_ledger.py --verify` exits non-zero if any row has an empty `decision`, or if the ledger's hunk set differs from a freshly generated one.
- [ ] **Step 4:** Collect every row that changes what a car does (any `feature`, `skipped` or `removal` row that touches a driving, SLAM or recording node) into `unify-behaviour-changes.md`, grouped by car: "what changes on rb1" and "what changes on rb2". **The team lead approves this file (Q4) before Task 13's gate.**
- [ ] **Step 5:** Commit the ledger and the tool.

### Task 11: The merge commit

**Files:** the whole tree.

**Interfaces:** `tools/dev/unify/three_way.py <path> --base ffbfac0 --ours HEAD --theirs refs/car2/main` writes the `git merge-file` result into the worktree and exits 1 on conflict.

- [ ] **Step 1:** On `unify/main`: `git merge --no-ff --no-commit --allow-unrelated-histories refs/car2/main`.
- [ ] **Step 2:** For each both-sides file, run `three_way.py`. Then apply the ledger:
  - `hw-rb2` hunks are removed from the shared file and their values written to `cars/rb2/…`
  - `skipped` and `removal` hunks are applied as decided
  - files car 2 deleted are restored from `HEAD` unless the ledger says `removal`
- [ ] **Step 3:** Move car 2's `data/racerbot_runs`, `racerbot_auto` and `racerbot_sim` into `data/rb2/…`. Take every file that exists on only one side as it is. Set the submodule pins: `racerbot_a` `8ef1d8c`, `web_dashboards` `f5b711d`, `realsense-ros` `775b167`. `.gitmodules` is the union, with one entry per path (car 2 had a duplicated `.gitmodules` once, `c7b019f`).
- [ ] **Step 4:** Resolve the 9 conflicted files by hand. Docs get the newer text from both sides, rewritten for both cars in Task 19.
- [ ] **Step 5:** Test-count check: for every test file present on either side, the merged count of `def test` must be ≥ the max of the two sides. Any loss must be explained by a ledger row. Script: `hunk_ledger.py --test-counts`.
- [ ] **Step 6:** Build; full suites with `RACERBOT_CAR=rb1` and then `rb2`. Commit the merge: "Merge car 2's workspace (Racerbot-Car-2-Workspace 9c2c2ed), keeping the best of both".

### Task 12: The `rb2` profile, both doctor ladders, and the camera launch

**Files:**
- Create: `src/racerbot_car/cars/rb2/{car.yaml,calibration/*.yaml,imu/…}` (IMU files from Task 9), `src/racerbot_launch/launch/camera_launch.py`
- Modify: `racerbot_doctor/{car,checks,cli,live}.py`, `racerbot_launch/launch/dashboard_launch.py`
- Replace: `racerbot_launch/test/test_car1_hardware_config.py` and car 2's car-2 guard tests (`b22b970`) with `test_car_profiles.py`, carrying every check from both

**Interfaces:**
- `racerbot_doctor.car.spec_for(profile: CarProfile) -> DoctorSpec`. `DoctorSpec` has `lidar: SerialLidarSpec | EthernetLidarSpec`, `camera: UvcCameraSpec | RealSenseSpec | None`, and the shared thresholds (battery, rates). The constants are car 1's and car 2's current `car.py` values verbatim.
- `checks.lidar_ladder(spec)` dispatches on the spec type to the existing USB ladder or the Ethernet ladder.
- `camera_launch.py` starts `webcam_launch.py` for `uvc` or `realsense_camera_launch.py` for `realsense`, chosen from the profile.

- [ ] **Step 1: Failing tests:**
  - every assertion of the old `test_car1_hardware_config.py`, now parametrized as `rb1`, plus the same set of car 2 facts as `rb2` (oracle: the Global Constraints lists)
  - `test_doctor_rb1_never_mentions_ethernet_or_realsense`, `test_doctor_rb2_never_opens_tty_or_video`
  - `test_camera_launch_picks_by_profile`
  - `test_rb2_calibration_matches_car2_repo` (oracle: the values recorded in the ledger from `9c2c2ed`)
  - the existing doctor static test (never opens a tty, camera or input node) still passes for both
- [ ] **Steps 2–5:** fail → implement → pass → mutation proof (swap the two cars' specs; both profile tests fail) → commit.

### Task 13: The two-car behaviour gate

- [ ] **Step 1:** Build the merged worktree. Capture with `--car rb1` → `cap/p3-rb1`, and with `--car rb2` → `cap/p3-rb2`.
- [ ] **Step 2:** Diff `ref/rb1` against `cap/p3-rb1`, and `ref/rb2` against `cap/p3-rb2`, each with `--allow docs/superpowers/plans/unify-behaviour-changes.md`. The tool reads the backticked `node key old -> new` lines from that file. **Expected:** no unlisted difference. Each listed one appears exactly as written.
- [ ] **Step 3:** Run the simulator validation for both cars:
  - `RACERBOT_CAR=<id> rb python3 tools/f1tenth_sim/run_validation.py --workers 2` (or the current CLI of that tool)
  - the `racerbot_sim` launch tests under each car

  Compare against the old repos' results recorded the same way.
- [ ] **Step 4:** The team lead approves `unify-behaviour-changes.md` (Q4). Record the approval in the file with the date.

---

## Phase 4: Per-car tuning, the dashboard, boot services

### Task 14: Tuning overlays and the safety rules

**Files:** Modify `src/racerbot_car/racerbot_car/guards.py`. Test: `src/racerbot_car/test/test_guards_tuning.py`. Modify `racerbot_doctor/cli.py` (prints "Car rb1 (serial …), tuning overrides: …" with each `# why:`) and `race_diagnostics/run_metadata.py` (records `car_id`, the overlay paths and their sha256).

**Interfaces:**
- `guards.SAFETY_TRUE = {'enable_deadman', 'enable_ttc', 'enable_swept_clearance'}`: an overlay may only set these to `true`
- `guards.SAFETY_FIXED = {'deadman_button'}`: may not appear in any overlay
- `guards.SAFETY_CEILING`: overlay ≤ shared. `max_speed`, `profile_max_speed`, `profile_max_brake`, `ttc_min_closing_speed`, and every key matching `*_timeout_sec`.
- `guards.SAFETY_FLOOR`: overlay ≥ shared. `ttc_threshold_sec`, `ttc_path_margin`, `car_width`, `car_length`, `optimize_car_width`, `emergency_stop_clearance`, `forward_stop_clearance`, `swept_clearance_margin`, `swept_clearance_max_length`, `emergency_stop_distance`, `profile_wall_clearance`, `narrow_corridor_clearance`, `open_corridor_clearance`, `overtake_min_side_clearance`, `body_clearance_fov_deg`.
- `guards.check_tuning_overlay(path: Path, shared_path: Path, config_name: str) -> list[str]`:
  - every key must exist in the shared file's same node block
  - no key may be in `CALIBRATION_KEYS`
  - every key needs a `# why:` comment directly above it
  - the safety rules above apply

- [ ] **Step 1: Failing tests,** one deny case per rule plus one allow case:
  - `test_tuning_rejects_enable_deadman_false`
  - `test_tuning_rejects_deadman_button`
  - `test_tuning_rejects_max_speed_above_shared`, `test_tuning_allows_max_speed_below_shared`
  - `test_tuning_rejects_timeout_longer_than_shared`
  - `test_tuning_rejects_ttc_threshold_below_shared`
  - `test_tuning_rejects_unknown_key` (typo)
  - `test_tuning_rejects_calibration_key`
  - `test_tuning_rejects_key_without_why`
  - `test_every_car_tuning_overlay_passes_guards` (parametrized over the real `cars/*/tuning/*.yaml`)
  - `test_doctor_lists_overrides_with_reasons`
  - `test_run_metadata_records_car_and_overlay_hashes`
- [ ] **Steps 2–5:** fail → implement → pass → mutation proof (flip `<=` to `<` in the ceiling check: the equal-value allow case must change outcome; delete the `SAFETY_TRUE` check: the deadman test must fail) → commit.
- [ ] **Step 6:** Write the per-car tuning rows from the ledger into `cars/<id>/tuning/` (e.g. car 2's IMU feature enables), each with its `# why:`. Re-run Task 13's gate.

### Task 15: Dashboard live tuning saves to the right file (`web-dashboards` repo)

**Blocked by Q5.** It's a separate PR in `sfu-racerbot/web-dashboards`.

**Interfaces:** in `dashboard_node.py`, a `tuning_config_files` entry may be one `<package>/<path>` (today's form) or a list of them, separated by `|` (a ROS string-array element). The save writes each value into the **last** file in the list that already holds that key under the node's block, and otherwise into the first. Its tests live in `car/ros/web_dashboard/test/`. They need a single-file case that is unchanged, a key-in-overlay case, a key-in-shared-only case, and a key-in-neither case. The existing `tuning.py` contract tests in this repo (`test_*_live_tuning.py`) still pass.

- [ ] **Steps:** failing tests in web-dashboards → implement → pass → mutation proof → PR there → once merged, bump the pin here → `web_dashboard_rb1.yaml` / `_rb2.yaml` list `<pkg>/config/<file>|racerbot_car/cars/<id>/tuning/<file>` → `test_web_dashboard_rb*_config.py` checks that every listed file exists.
- **Fallback if Q5 is "no":** keep single-file entries pointing at the shared configs, and add a doc note: "a value this car overrides in `cars/<id>/tuning/` must be changed there".

### Task 16: Boot services and udev rules for both cars

**Blocked by Q6.** **Files:**
- Modify: `tools/systemd/install.sh` (reads `current_car_id()` and the profile's `hostnames`; uses `$HOME` and `$USER`, never `/home/racerbotcar-1`) and `tools/udev/install.sh` (installs `99-hokuyo.rules` only on cars with a serial LiDAR)
- Remove: `src/racerbot_launch/systemd/racerbot-dashboard.service` (car 2), after its behaviour is checked against `tools/systemd/web-dashboard.service`
- Fix the `/home/racerbotcar-1` paths in `waypoint_recorder.{yaml,launch.py}`, `waypoint_recorder_node.py` and `.vscode/settings.json` (`~` or a `data_dir`-style resolver)
- Test: `tools/dev/tests/test_install_scripts.py`. Static: no file outside `docs/` and `data/` contains `/home/racerbotcar-`. `install.sh --dry-run` prints car-specific unit names for `RACERBOT_CAR=rb1` and `rb2`.

- [ ] **Steps:** failing tests → implement → pass → mutation proof → commit.

---

## Phase 5: Git LFS

### Task 17: The LFS setup that every clone uses

**Files:** Create `.gitattributes`, `tools/dev/setup_clone.sh` and `tools/dev/hooks/pre-commit`. Modify `tools/save_runs.sh` and `data/README.md`. Test: `tools/dev/tests/test_lfs_setup.py`.

**Interfaces:**
- `.gitattributes`: `data/**/*.mcap`, `data/**/*.db3`, `data/**/*.npz`, `data/**/*.pgm`, `data/**/*.png`, `data/**/*.jpg`, each with `filter=lfs diff=lfs merge=lfs -text`
- `setup_clone.sh`:
  - `git lfs install --local`
  - `git config core.hooksPath tools/dev/hooks`
  - on a car: `git config lfs.fetchexclude 'data/<other ids>/**'`
  - on an unknown machine (the dev box): `git config lfs.fetchexclude 'data/**'`
  - prints how to fetch one run: `git lfs pull --include 'data/rb1/racerbot_runs/<ts>/**'`
- `pre-commit`: rejects any staged blob under `data/` over 10 MB that isn't an LFS pointer, and names the file
- `save_runs.sh`: exits 2 with "git-lfs is not installed: sudo apt install git-lfs" when `git lfs version` fails; stages only `data/$(car)/`; keeps `--skip-idle` and the idle report. The default for idle runs follows Q2.

- [ ] **Step 1: Failing tests,** in a temp repo with real git-lfs (skipped with a clear reason if git-lfs isn't installed in the container; install it there in this step):
  - `test_check_attr_lfs_for_bag_paths` (`git check-attr filter data/rb1/racerbot_runs/x/bag/bag_0.mcap` → `lfs`; also for `rb2` and for npz/pgm/png)
  - `test_check_attr_not_lfs_for_metadata_yaml`
  - `test_pre_commit_rejects_large_plain_blob`, `test_pre_commit_allows_lfs_pointer`
  - `test_save_runs_refuses_without_lfs` (PATH without git-lfs)
  - `test_setup_clone_fetchexclude_on_car` and `test_setup_clone_fetchexclude_on_dev_box`
- [ ] **Steps 2–5:** fail → implement → pass → mutation proof (drop the `*.mcap` line; the check-attr test fails) → commit.

### Task 18: History rewrite, dry run (only if Q1 = B or C)

- [ ] **Step 1:** `git clone --mirror /home/bryan/racerbot-ws ~/racerbot-merge/mirror-dry`, then fetch `unify/main` into it.
- [ ] **Step 2:** `git lfs migrate import --everything --include="*.mcap,*.db3,*.npz,*.pgm,*.png,*.jpg" --include-ref=refs/heads/unify/main`. Use path-limited includes: `data/**`, plus the same patterns under `tools/f1tenth_sim/maps/` only if the team lead wants maps in LFS. **Default: `data/**` only.**
- [ ] **Step 3: Verify.**
  - For the tip, list every path with its blob. Non-LFS paths must be byte-identical to `unify/main`. For every LFS pointer, the pointer's `oid` must equal the sha256 of the original blob.
  - Script: `tools/dev/unify/verify_migration.py`. Exit 0 required.
- [ ] **Step 4:** Record the resulting sizes (`du -sh` of the pack and of `lfs/objects`) in `unify-car2-survey.md`, for the Q2 budget.

---

## Phase 6: Docs

### Task 19: Docs for one workspace and two cars

Use the `novice-docs` and `racerbot-docs-voice` skills.

- [ ] **`docs/car-profiles.md` (new):**
  - how a car knows which car it is
  - what lives in `cars/<id>/`
  - how to change calibration (the odom wizard writes it)
  - how to add a tuning override, and the safety rules
  - how to add a third car
  - the error messages and what to do about each
- [ ] **`docs/car1-vs-car2.md`:** reduced to a two-column hardware table that points at the profile files, plus a "these used to be ported" note linking to `car-profiles.md`.
- [ ] **`CLAUDE.md`:** rewritten for both cars. The safety model, test quality and known non-obvious facts stay word for word where still true. "Car 1 vs car 2" becomes "two cars, one workspace". The porting instructions are removed. The package table gains `racerbot_car`.
- [ ] **The other docs:**
  - `README.md`, `docs/git-setup.md` (repo name, LFS, Addition 7)
  - `docs/hardware-reference.md` (one section per car, shared geometry)
  - `docs/operations.md`, `docs/launchstart.md` (`data/<id>/` paths; `RACERBOT_CAR` for the simulator and the dev box)
  - `docs/run-diagnostics.md`, `docs/web-dashboard.md`, `data/README.md`
- [ ] **Docs tests:** existing docs tests (if any) pass, and `test_no_bare_data_paths` stays green. Commit.

---

## Phase 7: Cutover

### Task 20: Final sync, then publish

- [ ] **Step 1:** Re-fetch both `main`s. `hunk_ledger.py --new-commits ae7ceeb..origin/main 9c2c2ed..car2/main` must report data-only commits. Anything else stops the cutover and goes to the team lead (Q7). Merge the data-only commits: car 1's into `data/rb1`, car 2's into `data/rb2`.
- [ ] **Step 2:** Final gates:
  - full suites for both cars
  - Task 13's two-car diff
  - `hunk_ledger.py --verify`
  - the test-count check
  - mark the draft PR ready
- [ ] **Step 3: Publish per Q1, with explicit go-ahead from the team lead at this step:**
  - **(A):** the team lead merges the PR. A GitHub admin renames the repo to `Racerbot-Car-Workspace`.
  - **(B):** run Task 18 on the real mirror. The team lead force-pushes `main`, or explicitly authorizes the push, then the repo is renamed.
  - **(C):** an admin creates `sfu-racerbot/Racerbot-Car-Workspace` (public, like today). Run Task 18 on a fresh mirror, then `git push` the migrated `unify/main` as `main` and `git lfs push --all`. No force push is needed. Archiving the two old repos happens only after Task 21.

  Q2's budget must already be set.

### Task 21: Move each car onto the new repo

Car 1 first, then car 2. One paste-ready prompt per car for the Claude Code session on that car, per the standing hand-off procedure:

1. `sudo apt install git-lfs`.
2. Clone into `~/racerbot-ws-new`, then `tools/dev/setup_clone.sh`, `git submodule update --init --recursive`.
3. Copy over untracked local state:
   - `src/racerbot_a`'s `COLCON_IGNORE` (if still needed)
   - the hand-built `range_libc` Python module (rebuilt WITH_CUDA per the memory note)
   - `/etc/cloudflared` stays where it is
4. Swap directories: `~/racerbot-ws` → `~/racerbot-ws-old`, and `~/racerbot-ws-new` → `~/racerbot-ws`.
5. `colcon build --symlink-install --parallel-workers 1`.
6. Full suites, sourced and on an isolated domain, comparing collected counts.
7. `ros2 run racerbot_doctor doctor` must print the right car id and serial, with no contradiction.
8. `tools/systemd/install.sh` and `tools/udev/install.sh`.
9. Wheels off the ground with LB held, then the floor at low speed, then race speed. A person holds LB throughout.

Car 2 also removes `racerbot-dashboard.service` and retires `~/imu-calibration` (its files are already in `cars/rb2/imu`).

**Under (A),** steps 1–4 become `git remote set-url origin …` + `git pull`.

### Task 22: Retire the old setup

- [ ] Archive the old repos (C), done by a GitHub admin.
- [ ] Update memory notes: unified workspace, the new repo URL, `RACERBOT_CAR` on the dev box, `RB_WS`.
- [ ] Delete `tools/dev/unify/` and the `refs/car2/main` ref.
- [ ] Remove `~/racerbot-merge` after both cars have run on the floor.

---

## Execution

- **The controller (this session, inline):**
  - Task 10 classification
  - Task 11 conflicts and ledger application
  - every review of driving-code diffs
  - Task 13 and the Phase gates
  - Task 19 docs
  - every SSH and cutover step
- **Implementers:** at most one Claude Haiku at max effort at a time, or free opencode models, on mechanical tasks with disjoint files: Tasks 2, 3, 4, 7 (code part), 14 (guards), 16, 17. Each gets its task text and the Global Constraints. The controller reviews every diff inline before the next task starts.
- **Order:** Tasks 1–8 need nothing from the team lead. Task 9 needs Q3. Task 13's approval needs Q4. Task 15 needs Q5, Task 16 needs Q6, and Tasks 18 and 20 need Q1 and Q2.
