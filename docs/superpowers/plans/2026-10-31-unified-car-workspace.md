# Unified Car Workspace Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** One repo, this one renamed to `Racerbot-Car-Workspace`, that runs car 1 (`rb1`) and car 2 (`rb2`). It keeps the latest and best version of every feature, test and tool from both car repos. The only things that differ per car are its hardware/calibration profile, its explicit tuning overrides and its data folder (plain git, no LFS).

**Architecture:**
- A new pure-Python package `racerbot_car` works out which car it's running on. It reads the Jetson's serial number, refuses to start if the machine is unknown, and refuses if the plugged-in hardware contradicts the profile.
- Each car's calibration and tuning live as ROS-parameter overlay files. Every launch file loads them on top of the shared configs.
- The car 2 history comes in through one `--allow-unrelated-histories` merge. Each file is merged three-way, with car 1 at `ffbfac0` (the commit car 2 last ported from) as the base.
- Behaviour is proven unchanged by an "effective launch parameters" oracle. It captures what every launch file would start, and with which parameters, in each old repo. The merged repo is diffed against both captures, and only a list of behaviour changes that the team lead approved is allowed to differ.

**Tech Stack:** ROS 2 Jazzy, ament_python, launch / launch_ros, PyYAML, pytest, git, GitHub (`sfu-racerbot` org, Free plan), opencode (`openai/gpt-6.1-terra`) for implementers.

**Spec:** this document's "Design" section and the decisions below. The earlier one-page draft of this plan is superseded.

---

## Decisions (team lead, 2026-10-31)

1. **One repo for both cars:** this repo, renamed on GitHub to `Racerbot-Car-Workspace`. Car 2's history comes in through the merge. `Racerbot-Car-2-Workspace` is archived after cutover.
2. The merge keeps the **latest and best features and tools** from both repos. Car 2 has had a lot of gap_follow and SLAM work. Both cars' Claude sessions have added tests and tools, and all of them are kept.
3. **Per-car tuning is allowed.** It is stored in git so the dev box can read it.
4. **Data stays in plain git, with no LFS** (team lead, 2026-10-31, reversing the earlier LFS decision). It goes in per-car folders `data/<id>/`, and bags stay split at 50 MB, under GitHub's 100 MB file limit. History is not rewritten, so nothing is force-pushed.
5. **Car 2's hardware:** Ethernet Hokuyo UST-10LX at `192.168.0.10`, RealSense D435i, dashboard id `rb2`.
6. **Both cars:** the BNO085 is on Jetson I2C, and the geometry is the same (same chassis).
7. **Car 2's newer code is taken for both cars:** gap_follow (`aim_depth_bias`, 5 m swept look-ahead, `opponent_follow_floor`, `time_trial`), SLAM and map_course on the gyro, the four-state speed filter, `auto_record`, the tools. On car 1 it is tested in the simulator **with car 1's simulator settings** (Task 13 step 3) before car 1 drives it.
8. **The dashboard:** a PR to `sfu-racerbot/web-dashboards` is approved, so a live-tuning save goes to the file that holds the key (Task 15).
9. **Boot services:** car 2 already has its own Cloudflare tunnel, dashboard service, remote desktop and SSH, and they stay as they are. The repo keeps car 1's `tools/systemd` set for car 1 and car 2's `racerbot_launch/systemd/racerbot-dashboard.service` for car 2 (Task 16).
10. **Neither car runs or pushes over the weekend of 2026-10-31,** so both `main`s stay put while the work is under way. Task 20 still re-checks.
11. **Execution:** as in "Execution" at the end. Implementers may be opencode `openai/gpt-6.1-terra` (OpenAI subscription) or one Claude Haiku at max effort.
12. **When:** both repos were declared final on 2026-10-31. Car 2's SSH key access was granted the same day.

## Remaining questions

- **Q9.** Car 2's IMU config (`~/imu-calibration/config/car.yaml`, uncommitted on car 2) says `wheelbase: 0.33`, but the shared geometry is 0.36, tape-measured on car 2. **Default:** `cars/rb2/imu/car.yaml` gets 0.36, listed as a car 2 behaviour change (IMU estimator).
- **Q10.** Car 2's `cloudflared.service` carries its tunnel token inline in `ExecStart`, in a world-readable unit file. Car 1 moved its token to `/etc/cloudflared/token` (`--token-file`). **Default:** the car 2 hand-off prompt (Task 21) offers the same move, done by the car 2 session with sudo.
- **Q11.** Car 2's hostname is `localhost.localdomain`. Nothing here depends on it, because identity comes from the serial. **Default:** leave it, and mention it in the car 2 hand-off.

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
- **Car 2's Jetson** (read-only survey, 2026-10-31):
  - **Identity:** serial `1423025035537`, user `racerbotcar-2` (groups include `input`, `video`, `i2c`, `dialout`), hostname `localhost.localdomain`.
  - **Checkout:** clean at `9c2c2ed`. Every worktree branch is pushed or merged, and the only dirt is submodule state.
  - **IMU calibration exists only on the car:** in `~/imu-calibration` (`0d99cd2`), the modified `config/car.yaml`, the untracked `calib/bno_calib.json` and `tools/bno_i2c_bridge.py`, plus `calib/vesc_calib.json`. Copied read-only to `~/racerbot-merge/car2-imu/`.
  - **Two bridges:** car 2's `bno_i2c_bridge.py` is a different implementation from car 1's in-tree one (457 differing lines).
  - **Units:**
    - `racerbot-dashboard.service` runs `ros2 launch racerbot_launch dashboard_launch.py` with `RACERBOT_WS=/home/racerbotcar-2/racerbot-ws`
    - `foxglove-bridge.service` runs `src/web_dashboards/car/systemd/foxglove-bridge.sh`
    - `x11vnc` (`/usr/local/bin/x11vnc-autodisplay.sh`), `novnc` and `cloudflared` are outside git

    **The merge must keep `dashboard_launch.py` and that submodule path working on car 2.**
  - **udev:** `99-vesc.rules` and the RealSense rules, no Hokuyo rule.
  - **Other state:** no git-lfs, 165 GB free. `~/.ros/racerbot_auto` duplicates what's in git; `~/.ros/racerbot_replays` (3.2 GB) is derived experiments and isn't imported.
- **Repo size with no LFS:** about 2.8 GB of bags in history, plus up to about 2 GB per racing week. GitHub strongly recommends staying under 5 GB and rejects a single push over 2 GB. `save_runs.sh` pushes after every session, so its pushes stay small. The merge push carries car 2's history (about 0.45 GB as GitHub stores it), well under the limit.
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
- **Git:** never push or force-push `main`, never rewrite history, and never merge a PR. Work goes on branch `unify/main` in this repo, through a draft PR. At cutover the team lead merges the PR, and the repo is renamed (Task 20).
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

## Findings during execution (2026-10-31)

- **How ROS 2 Jazzy resolves several parameter files** (measured with an rclpy node, `--params-file A --params-file B`). It is **not** "later file wins":
  - **Entries:** every top-level block key, exactly as written, is one entry, in order of first appearance across the files (`n` and `/n` are different entries).
  - **Repeats:** a later file updates an existing entry in place; the entry doesn't move.
  - **Resolution:** a node applies its matching entries in entry order, so later entries win.
  - **Measured cases:** A={`/**`: x=1, `n`: x=2}, B={`/**`: x=3} gives x=2. A={`n`: 2}, B={`/**`: 3} gives 3. A={`/n`: 5, `/**`: 1}, B={`n`: 2} gives 2.

  **Consequence:** every overlay key must sit under the same block key the shared file uses. `layers.check_block_keys_match` enforces this, and the oracle and `effective_values` implement the model. Inline launch dicts become a `/<node>` entry at the end, so they still override.
- **Task 5 is split and reordered so every step stays green:**
  - **5a:** car 1's calibration overlays (same values), `hardware.py`, `launch_car_id`/`car_params`, the four `f1tenth_stack` launches.
  - **5b:** every other launch that passes a split config.
  - **5c:** every Python reader in nodes, tools and tests switched to `effective_values(layered(...))`, with tests parametrized per car.
  - **Task 6:** the `odom_calibration` writers.
  - **5d:** only then are the calibration keys removed from the shared YAMLs.

  The zero-diff gate runs after each.
- **The oracle's sandbox:** `pgrep`/`git`/`ps`/`dpkg-query` are simulated, `range_libc` is stubbed, and includes of unbuilt packages are recorded by name. Event handlers are never registered. Writes outside a temp sandbox, process creation and device opens all fail loudly.
- **Reference captures** are in `~/racerbot-merge/ref/rb1` (45 entries) and `ref/rb2` (44). Car 2's `racerbot_launch` needs `realsense2_camera`, so the dev box and car 1 never build `realsense-ros`.
- **Dev-box test baselines:** `~/racerbot-merge/ref/car1-tests/summary.txt`, and `car2-tests`. Known noise:
  - lint failures in upstream `teleop_tools`/`f1tenth_stack`
  - the `tools/f1tenth_sim` (`gymnasium`) and `tools/neural_racer` (`torch`) collection errors
  - two `imu_calibration` `test_online` failures that depend on car 1's IMU mount
  - 16 `racerbot_sim` failures under load (re-run alone before treating as real)
- **The simulator for Task 13** needs `tools/f1tenth_sim/setup.sh`, which installs into `tools/f1tenth_sim/.sim/` only, in each checkout used.
- **`ffbfac0` is not a safe single merge base.** Car 2 ported car 1 selectively. It skipped, for example, PR #9's `record_run` shutdown fix (`db3e9e9`) and several late gap_follow fixes (`a43c72a`, `3933bf6`, `7095855`, `0c154cb`). Against `ffbfac0`, every car 1 line that car 2 never took looks like "car 2 deleted it", and a "clean" three-way merge would drop it silently. **Tasks 10 and 11 therefore change:**
  - **Two ledgers:**
    - forward: `hunk_ledger.py ffbfac0 car2` gives the lines car 2 has that car 1 lacks (`~/racerbot-merge/ledger_car2.tsv`, 800 hunks)
    - reverse: `hunk_ledger.py car2 ffbfac0` gives the lines car 1 has that car 2 lacks (`ledger_car1_only.tsv`, 795 hunks), plus `ffbfac0..ae7ceeb` (`ledger_car1.tsv`, 121)
  - **Classification:** each hunk is car hardware (→ overlay or profile-driven code), a feature to keep, or superseded (with the commit that supersedes it).
  - **Presence check:** after the merge, a script confirms that every "keep" hunk's added lines from **both** ledgers exist in the merged file (whitespace-normalised). A miss fails Task 11.
  - **Grouping:** both ledgers are grouped by blamed commit (`ledger_by_commit.py`). The reverse ledger's biggest groups are car 1's port and hardware commits (`f03f784`, `0cd347f`, `9fe90a4`, `e0dd8fd`), car 1's IMU work `881c409`, `auto_record` `b21860f`, and the late feature fixes.
  - **Merge base per file (Task 11), best first** (`~/racerbot-merge/merge_plan.py` → `merge_plan.tsv`):
    - **Port base:** car 2's own version at its last commit that ported that file from car 1 (`edb465e`, `7bac597`, `b81b291`, …). This lays car 2's post-port work onto car 1's version and keeps every car 1 fix car 2 skipped. **This is the preferred base.** `gap_follow_node.py` merges with 0 conflicts this way, against 7 from the synthetic base.
    - **Totals for the 149 both-sides files:** 63 port-clean, 21 port-conflicts, 20 synthetic-clean, 14 synthetic-conflicts, 31 with no base (hand).
    - **Port-time edits:** car 2's own port-time edits that weren't hardware are recovered through the forward ledger plus the presence check.
    - **Synthetic base where no port base exists:** the newest blob both repos' histories contain (`~/racerbot-merge/synth_base.py`, report `synth_report.tsv`). That's the real sync point. 44 files merge cleanly from it, including `opponent_tracking.py` and the `map_course` script. 44 conflict, among them `gap_follow_node.py` (7), `record_run.py` (4) and `map_course_launch.py` (3), and those are hand-resolved.
    - **No synthetic base (59 files):** these are mostly `package.xml`/`setup.py`, docs and tests. They're merged by hand from both ledgers.
    - **`ffbfac0` is never used** as the base for a file that has a synthetic base.
    - **Gate:** `~/racerbot-merge/presence_check.py <merged tree>` must report 0 missing keep-hunks and 0 `review` hunks.

## Review Focus

1. **The wrong car id on a Jetson would put the other car's servo map and steering limits on it.** Tests: an unknown serial refuses, and an env override that disagrees with the table refuses (Task 3). A hardware contradiction stops bringup: an `rb2` profile with a Hokuyo USB id present, or an `rb1` profile with a RealSense present (Task 5).
2. **A launch path that reads a shared config without the car overlay would silently fall back to the node's declared defaults,** since the calibration keys leave the shared files. Tests:
   - `layered()` raises if any required calibration key is missing from the car's overlay (Task 4).
   - A static test forbids direct readers of split configs (Task 5).
   - Zero-diff `launch_params` gates (Tasks 5 and 13).
3. **A car 2 deletion or omission could silently drop a car 1 feature,** such as `test_record_run_shutdown.py` or the `record_run.py` shutdown fix from car 1's PR #9. Tests: the ledger has to cover 100% of hunks, each with a decision (Task 10). The merged test-function count is at least the union of both sides (Task 11).
4. **A tuning overlay could loosen a safety limit, or carry a mistyped key that does nothing.** Tests: the guard rules with deny cases for every rule (Task 14).
5. **A car could commit a file over GitHub's 100 MB limit, or the other car's data.** The first makes every later push from that car fail until someone rewrites the commit. Tests: the pre-commit guard rejects any staged file over 95 MB. `save_runs.sh` stages only `data/<this car>/`, and refuses when the staged total is over 1.5 GB, staying under GitHub's 2 GB push limit (Task 17).
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
- **`save_runs.sh`:** commits only `data/<this car>/`.
- **Plain git, no LFS.** Bags stay split at 50 MB, as both cars do today.
- **Every clone has both cars' data,** so the dev box can analyse any run without fetching anything.

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
| `tools/save_runs.sh`, `tools/dev/setup_clone.sh`, `tools/dev/hooks/pre-commit` | per-car data commits; size guards |
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

- [x] **Done 2026-10-31.** The results are in the Inventory ("Car 2's Jetson").
  - The only thing uncommitted on car 2 is its IMU calibration in `~/imu-calibration`. It's copied to `~/racerbot-merge/car2-imu/` and goes into `cars/rb2/imu/` in Task 12, with Q9's wheelbase fix.
  - `machines.yaml` gets `"1423025035537": rb2` in Task 12.
- [ ] **Before Task 11:** re-run `git status` on car 2, read-only, to confirm nothing new appeared since the survey.

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
  - the two `bno_i2c_bridge.py` implementations: car 1's in-tree copy against car 2's uncommitted one in `~/racerbot-merge/car2-imu/`. Pick one for both cars on the code. Check the `$B,...` line format, the timestamp handling (car 2 subtracts the report delay), `--no-dyncal`, and the tests in `tools/imu_calibration/tests`. List it as a behaviour change for the car that switches bridge; that car confirms it wheels-up.
  - car 2's racer_imu `car.yaml` (Q9: `wheelbase` 0.33 → 0.36)
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
- [ ] **Step 6 (added 2026-11-01, moved from Task 5c): full-lock steering angles per car.** `car.yaml` gains `steering.full_lock_rad: {left, right}` with a citation: rb1 from its 2026-10-21 circles (`docs/hardware-reference.md` "Steering limits"), rb2 from its 2026-10-16 circles (today's `racerbot_sim/sim_bridge.py` `STEERING_MAX_LEFT/RIGHT`, 0.509 / -0.448 rad). `racerbot_sim`'s bridge reads them for the selected car: `gym_bridge_node` parameter `car` (empty = `current_car_id()`, refuse if unresolvable), passed by `sim_bringup_launch.py` (argument `car`, default from `launch_car_id()`) and `sim_race_launch.py`. `tools/f1tenth_sim` reads the same values where it models full lock. This changes car 1's ROS-simulator full lock (it modelled car 2's) — record it in `unify-behaviour-changes.md`; not binding while autonomy is capped at 0.33 rad. Tests: per-car full-lock value with its citation; capture shows `gym_bridge_node.car == 'rb1'`; break = launch passes no car → fails.

### Task 13: The two-car behaviour gate

- [ ] **Step 1:** Build the merged worktree. Capture with `--car rb1` → `cap/p3-rb1`, and with `--car rb2` → `cap/p3-rb2`.
- [ ] **Step 2:** Diff `ref/rb1` against `cap/p3-rb1`, and `ref/rb2` against `cap/p3-rb2`, each with `--allow docs/superpowers/plans/unify-behaviour-changes.md`. The tool reads the backticked `node key old -> new` lines from that file. **Expected:** no unlisted difference. Each listed one appears exactly as written.
- [ ] **Step 3: Simulator A/B for car 1 (decision 7).** Car 2's newer code runs under **car 1's simulator settings**: car 1's servo map and steering limits (0.38 full stick, 0.33 autonomy, 0.30 planner), car 1's `speed_to_erpm` and grip (car 1's `sim_fidelity/calibration.py` values), and car 1's course map `car1_course_2026_10_26`. It is compared against car 1's old code, from `~/racerbot-merge/ref-car1`, under the same settings.
  - Run `tools/f1tenth_sim/eval_gap.py` (single car) and `eval_gap_multi.py` (opponent) at `--workers 2`, with the same seeds, laps and maps, once per code version. The independent verdicts are `sim_fidelity`'s chassis-vs-distance-transform contact check and the car-to-car overlap check, **never gym's collision flag**.
  - **Pass:** zero contacts for the merged code, and lap time no worse than the old code's by more than the run-to-run spread measured over 5 seeds. Any contact, or a slower lap, goes to the team lead with the traces before car 1 drives.
  - Also run car 2's settings against car 2's old code the same way, so car 1's features coming into car 2 (the few the ledger lists) are covered too.
  - Run the `racerbot_sim` launch tests with `RACERBOT_CAR=rb1` and with `rb2`.
  - Results go into `unify-behaviour-changes.md` with the commands used.
- [ ] **Step 4:** The team lead approves `unify-behaviour-changes.md`, and the approval is recorded in the file with the date. On car 1, the floor order still applies after cutover (Task 21).

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

Approved (decision 8). It's a separate PR in `sfu-racerbot/web-dashboards`, opened as a draft for the team lead to merge.

**Interfaces:** in `dashboard_node.py`, a `tuning_config_files` entry may be one `<package>/<path>` (today's form) or a list of them, separated by `|` (a ROS string-array element). The save writes each value into the **last** file in the list that already holds that key under the node's block, and otherwise into the first. Its tests live in `car/ros/web_dashboard/test/`. They need a single-file case that is unchanged, a key-in-overlay case, a key-in-shared-only case, and a key-in-neither case. The existing `tuning.py` contract tests in this repo (`test_*_live_tuning.py`) still pass.

- [ ] **Steps:** failing tests in web-dashboards → implement → pass → mutation proof → PR there → once merged, bump the pin here → `web_dashboard_rb1.yaml` / `_rb2.yaml` list `<pkg>/config/<file>|racerbot_car/cars/<id>/tuning/<file>` → `test_web_dashboard_rb*_config.py` checks that every listed file exists.
- **Until that PR is merged:** single-file entries keep pointing at the shared configs.

### Task 16: Boot services and udev rules for both cars

Decision 9: car 2's installed boot services stay exactly as they are. **Files:**
- Keep both sets of units in the repo, labelled per car:
  - `tools/systemd/` (car 1: `web-dashboard.service`, `x11vnc.service`, `x11vnc-follow.sh`, `novnc.service`, `install.sh`)
  - `src/racerbot_launch/systemd/` (car 2: `racerbot-dashboard.service` and its pair)
- Modify `tools/systemd/install.sh`: it refuses unless `current_car_id()` is `rb1`, and names car 2's units for `rb2`. It uses `$HOME` and `$USER`, never `/home/racerbotcar-1`.
- Modify `tools/udev/install.sh`: installs `99-hokuyo.rules` only on cars whose profile has a serial LiDAR. Car 2's existing rules (`99-vesc`, RealSense) are left alone.
- **Must keep working for car 2's installed units:** `ros2 launch racerbot_launch dashboard_launch.py` with `RACERBOT_WS` set, and `src/web_dashboards/car/systemd/foxglove-bridge.sh`. A test checks both still exist and that `dashboard_launch.py` resolves `rb2` from car 2's serial.
- Fix the `/home/racerbotcar-1` paths in `waypoint_recorder.{yaml,launch.py}`, `waypoint_recorder_node.py` and `.vscode/settings.json` (`~` or a `data_dir`-style resolver).
- Test: `tools/dev/tests/test_install_scripts.py`. Static: no file outside `docs/` and `data/` contains `/home/racerbotcar-`. `install.sh --dry-run` refuses under `RACERBOT_CAR=rb2`. `test_car2_installed_units_targets_exist`.

- [ ] **Steps:** failing tests → implement → pass → mutation proof → commit.

---

## Phase 5: Data commits in plain git

### Task 17: Per-car `save_runs.sh` and size guards

**Files:** Create `tools/dev/setup_clone.sh` and `tools/dev/hooks/pre-commit`. Modify `tools/save_runs.sh` (merged from both cars' versions) and `data/README.md`. Test: `tools/dev/tests/test_data_commits.py`.

**Interfaces:**
- `setup_clone.sh`: `git config core.hooksPath tools/dev/hooks`, and prints the car id this machine resolves to (or "dev box: set RACERBOT_CAR for the simulator").
- `pre-commit`: rejects any staged file over 95 MB (GitHub's hard limit is 100 MB) and names the file and its size.
- `save_runs.sh`:
  - stages only `data/$(car id)/`
  - refuses when the staged total is over 1.5 GB (GitHub refuses a push over 2 GB), and says to run it once per session
  - keeps both cars' existing behaviour: `--skip-idle`, the idle and stub report, `--no-push`, refusing while a recorder runs
  - **no LFS**

- [ ] **Step 1: Failing tests,** in a temp git repo:
  - `test_pre_commit_rejects_96mb_file`, `test_pre_commit_allows_50mb_file`
  - `test_save_runs_stages_only_this_car` (`data/rb2/x` stays unstaged under `RACERBOT_CAR=rb1`)
  - `test_save_runs_refuses_over_push_limit` (a 1.6 GB sparse file total, faked with `truncate`)
  - `test_save_runs_refuses_while_recorder_running` (carried over from the cars' existing tests, if present)
- [ ] **Steps 2–5:** fail → implement → pass → mutation proof (raise the hook limit to 200 MB, and the 96 MB test must fail) → commit.

### Task 18: (dropped)

There's no history rewrite, because data stays in plain git (decision 4). The number is kept so the other references stay valid.

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
  - `README.md`, `docs/git-setup.md` (repo name, how car 2's history came in, Addition 7)
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
- [ ] **Step 3: Publish, with the team lead's go-ahead at this step.**
  - The team lead merges the PR into `main`. It's a normal merge, with no force push.
  - Then the repo is renamed. This is an outward-facing change, so it's confirmed first: `gh api -X PATCH repos/sfu-racerbot/Racerbot-Car-1-Workspace -f name=Racerbot-Car-Workspace`. GitHub redirects the old URL.

### Task 21: Move each car onto the merged repo

Car 1 first, then car 2. There's one paste-ready prompt per car for the Claude Code session on that car, following the standing hand-off procedure. History isn't rewritten, so neither car re-clones.

- **Car 1:**
  - `git remote set-url origin https://github.com/sfu-racerbot/Racerbot-Car-Workspace.git && git pull --ff-only && git submodule update --init --recursive`
  - Leave `src/realsense-ros` uninitialised. It's car 2's driver, and it can't build without librealsense.
- **Car 2:**
  - `git remote set-url origin https://github.com/sfu-racerbot/Racerbot-Car-Workspace.git && git fetch && git merge --ff-only origin/main`. This fast-forwards, because car 2's `9c2c2ed` is the merge commit's second parent.
  - Then `git submodule update --init --recursive`.
  - Car 2's `~/imu-calibration` is retired, since its files are now in `cars/rb2/imu`. Leave the directory on disk until car 2 has driven.
- **Both cars:**
  1. Run `tools/dev/setup_clone.sh`.
  2. Build: `colcon build --symlink-install --parallel-workers 1`. The hand-built `range_libc` Python module is unaffected, because the workspace directory stays the same.
  3. Run the full suites, sourced, on an isolated domain, comparing collected counts against `grep -c '^def test'`.
  4. `ros2 run racerbot_doctor doctor` must print the right car id and serial, report no contradiction, and list the car's tuning overrides.
  5. Car 1 runs `tools/systemd/install.sh` and `tools/udev/install.sh`. Car 2's units stay untouched; check that `racerbot-dashboard.service` and `foxglove-bridge.service` restart cleanly.
  6. Car 2 offers the token move (Q10).
  7. Hardware order: wheels off the ground with LB held, then the floor at low speed, then race speed. A person holds LB throughout. **Car 1 drives car 2's newer gap_follow and SLAM code only after Task 13's simulator A/B passed.**

### Task 22: Retire the old setup

- [ ] Archive `Racerbot-Car-2-Workspace` read-only, done by a GitHub admin with the team lead's go-ahead. Only after both cars have driven on the merged repo.
- [ ] Update memory notes: unified workspace, the new repo URL, `RACERBOT_CAR` on the dev box.
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
- **Implementers:** opencode `openai/gpt-6.1-terra` (OpenAI subscription), free opencode models, or at most one Claude Haiku at max effort at a time. They work on mechanical tasks with disjoint files: Tasks 2, 3, 4, 7 (code part), 14 (guards), 15 (web-dashboards), 16, 17. Each gets its task text and the Global Constraints. The controller reviews every diff inline before the next task starts, and runs the gates itself.
- **Order:** Phases 1–3 run in order. Task 13's approval and Task 20's merge and rename need the team lead. Everything else proceeds on the decisions above.
