# SLAM reliability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

> **Who this is for:** whoever builds or continues this work, human or agent.
> **Read first:** [the spec](../specs/2026-10-30-slam-reliability.md), `CLAUDE.md`, `TEST_QUALITY_STANDARDS.md`.
> **You'll be able to:** build each piece and know from its test that it is done.
> **Time:** about a working day to build; the floor test is one session on the car.

**Goal:** `map_course` closes its laps at speed, builds a usable map on every run with the gyro on, keeps a map even when stopped early, and says plainly whether a map is good enough to race on.

**Architecture:** Two config and logic fixes at the source (the lap counter, the SLAM profile), plus one offline step. The offline step re-localizes the run's scans on its finished map after the car has stopped, so no node that can move the car changes behaviour because of it.

**Tech Stack:** Python, numpy, scipy, rosbag2_py through `tools/run_analysis/runlog.py` and `slam_bag.py`, slam_toolbox 2.8.5.

**Spec:** `docs/superpowers/specs/2026-10-30-slam-reliability.md`

## Global Constraints

- The LB deadman policy is untouched. No new node publishes a drive topic, and `check_start` publishes nothing at all.
- `map_localize.py` imports neither rclpy nor anything ROS, so its tests run unsourced.
- Every new test is seen to fail against a break of the code it covers, before it counts (CLAUDE.md Hard rule 1).
- ROS tests run with `ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.
- A racing line from the rebuild is capped at 1.5 m/s by default, and faster files sit beside it.
- The steering-odometry profile `slam_tracking.yaml` keeps its values: the tight prior goes in `slam_gyro.yaml` only.

## Review Focus

1. **A run with no bag, or a bag with no `/map`** (stopped before SLAM started): the wrapper must say so and still print its report, not raise.
2. **A map too doubled for the start search to find the car:** must be a refusal naming the reason, never a line.
3. **Less than one lap driven:** the map can still be saved, with no line and a statement of why.
4. **A run with no `/odom/imu`** (`imu:=false`): falls back to `/odom`, and says the result is weaker.
5. **Ctrl+C during the rebuild itself:** must not lose what has already been written, and must still print the report.

---

## File structure

| File | Responsibility |
|---|---|
| `src/pure_pursuit/pure_pursuit/auto_map_race_node.py` | `LapRecorder`: a correction is motion odometry did not see |
| `src/racerbot_launch/config/slam_gyro.yaml` | The gyro prior and a reachable loop-closure chain |
| `src/pure_pursuit/pure_pursuit/map_localize.py` | New. ROS-free: distance field, scan matching, tracking, see-through, laps, verdict |
| `tools/run_analysis/rebuild_run.py` | New. Run folder in, racing folder out: map, line, start pose, verdict |
| `src/racerbot_launch/scripts/map_course` | Runs the rebuild after every run, shielded from a second Ctrl+C |
| `src/pure_pursuit/pure_pursuit/check_start.py` | New. Read-only: is the car on the start pose? |

> **Status, 2026-10-30:** tasks 1-6 and 8 are built. Task 7 needs the car and a person: **it is the real acceptance test, and it has not been done.**

### Task 1: Lap counter counts only real corrections

**Done:** commit `949cb6e`. 22 new tests, including run `20261030-185036` as a recorded fixture (4+ laps of 50.4 ± 2 m, at most 20 corrections a lap, where the old code closed none). 13 mutations caught.

### Task 2: SLAM profile

**Files:**
- Modify: `src/racerbot_launch/config/slam_gyro.yaml`
- Test: `src/racerbot_launch/test/test_slam_gyro_profile.py`

- [x] **Step 1: Write the failing tests**, all reading the real YAML files:
  - `test_a_closure_chain_fits_in_the_search_radius`: invariant. `loop_match_minimum_chain_size × 0.57 m` (measured node spacing, 2026-10-30) must be under `2 × loop_search_maximum_distance`. Must fail on chain 20.
  - `test_a_gyro_disagreement_costs_a_match_something`: closed form. `1 - 0.2 × (5°)² / angle_variance_penalty` is at or below 0.9, and `minimum_angle_penalty` is below that value, so the floor does not cancel it.
  - `test_the_coarse_search_still_steps_at_least_ten_times`: `coarse_search_angle_offset / coarse_angle_resolution >= 10`.
  - `test_steering_odometry_does_not_get_the_tight_prior`: none of the prior keys appear in `slam_tracking.yaml`.
- [x] **Step 2: Run, see them fail**
- [x] **Step 3: Set the five values from the spec's table in `slam_gyro.yaml`, with the evidence in comments**
- [x] **Step 4: Run them, and `test_map_course_imu.py`, green**
- [x] **Step 5: Commit**

### Task 3: `map_localize` library

**Files:**
- Create: `src/pure_pursuit/pure_pursuit/map_localize.py`
- Test: `src/pure_pursuit/test/test_map_localize.py`

**Interfaces, produced:**
- `scan_points(ranges, angle_min, angle_increment, laser_x=0.26, stride=3, max_range=12.0) -> (bx, by)`
- `WallField(walls, free, resolution, origin_x, origin_y)`, `WallField.from_occupancy(OccupancyMap)`, `.distance(xs, ys)`
- `match_scan(field, bx, by, pose) -> (x, y, yaw)`, `scan_fit(field, bx, by, pose) -> float`
- `find_start(field, bx, by, near=None) -> (pose, fit)`
- `track_scans(field, times, scans, odom, angle_min, angle_increment, start, laser_x=0.26) -> Track(times, poses, fit)`
- `see_through_rate(field, poses, scans, angle_min, angle_increment, laser_x=0.26) -> rates`
- `find_laps(poses) -> [Lap(start, end, length, gap)]`
- `judge(fit, see_through, window=20) -> Verdict(ok, reason, mean_fit, worst_window_fit, see_through)`, with `MIN_MEAN_FIT` 0.90, `MIN_WINDOW_FIT` 0.70, `MAX_SEE_THROUGH` 0.15

- [x] **Step 1: Tests against an independent ray-cast world.** Edges: blank and NaN scans, empty inputs, a bare corridor stays on odometry, a doubled wall caught by see-through, a spin and a spiral are not laps, the see-through limit's exact boundary.
- [x] **Step 2: Implement, run unsourced, green**
- [x] **Step 3: Mutation check, including sign, clamp and limit breaks**
- [x] **Step 4: Commit**

### Task 4: `rebuild_run.py`

**Files:**
- Create: `tools/run_analysis/rebuild_run.py`
- Test: `tools/run_analysis/tests/test_rebuild_run.py`

**Interfaces:**
- Consumes: Task 3, plus `recorded_path.prepare`, `racing_math.compute_velocity_profile`, `occupancy_map.despeckle_map_file`, `run_commands.write_start_pose`.
- Produces: `rebuild(run_dir, out_root, max_speed=1.5, profile=None) -> RebuildResult(out_dir, verdict, lap_count, raceline_ok, message)`, and a CLI `rebuild_run.py <run> [--out-root DIR] [--max-speed V]`. Exit 0 for a racing folder, 1 for a map refused or no line, 2 for nothing to work from.

Output folder `<out_root>/<run name>-rebuilt/`: `map.yaml`, `map.pgm`, `rebuild.json`, and when accepted `raceline_raw.csv`, `raceline_profiled.csv`, `raceline_2p5.csv`, `raceline_4p0.csv`, `start_pose.yaml`, `commands.txt`.

Decisions:
- **Map:** the last `/map` in the bag (`slam_bag.load`), despeckled with `map_despeckle_max_cells` from `auto_map_race.yaml`.
- **Odometry:** `/odom/imu` if the bag has it, else `/odom`, and the message says the second is weaker.
- **Start of tracking:** `find_start` on a scan 1 s in; if its fit is under 0.8, retry a quarter and halfway through the run; if all fail, the map is refused.
- **Line:** among `find_laps`, the lap `prepare` accepts with the largest wall clearance.
- **Start pose:** the last tracked pose, because that is where the car is parked, as in `map_course` today. `check_start` verifies it.
- **Refused map:** `commands.txt` says why and gives no race command, and no `raceline_profiled.csv` is written, so `race_launch.py` refuses that folder.

- [x] **Step 1: Tests from a synthetic run folder (arrays through a fake loader, closed-form course):** accepted run writes every file; `raceline_profiled.csv` tops out at `max_speed`; doubled map writes no line and names see-through; under one lap writes map but no line and says so; no `/map` exits 2; `/odom` fallback says so; an existing output folder is refused, not overwritten.
- [x] **Step 2: Implement; see them green**
- [x] **Step 3: Run it on the real runs.** Expected: `185036` accepted and `184556` refused on see-through.
- [x] **Step 4: Mutation check, commit**

### Task 5: `map_course` runs the rebuild

**Files:**
- Modify: `src/racerbot_launch/scripts/map_course`
- Test: `src/racerbot_launch/test/test_map_course_rebuild.py`

**Interfaces:** `rebuild_step(run, parent, handle) -> None`, run after `analyse()` with `rebuild:=false` to skip.

Decisions: the reports and the rebuild run with `start_new_session=True` so the terminal's second Ctrl+C reaches only the wrapper. A Ctrl+C there skips what is left, but `report()` still runs, and since the rebuild writes `commands.txt` last, `new_run()` only ever finds a finished folder.

- [x] **Step 1: Tests with a fake subprocess at the process boundary:** the rebuild is invoked after a run with a recording; not invoked without one; `rebuild:=false` skips; an interrupted rebuild still reports; `report()` picks the rebuilt folder when it is newest, and says "no map" when the verdict refused it.
- [x] **Step 2: Implement; green; mutation check; commit**

### Task 6: `check_start`

**Files:**
- Create: `src/pure_pursuit/pure_pursuit/check_start.py`
- Modify: `src/pure_pursuit/setup.py`, `src/pure_pursuit/pure_pursuit/run_commands.py`
- Test: `src/pure_pursuit/test/test_check_start.py`

**Interfaces:** `ros2 run pure_pursuit check_start --ros-args -p run_dir:=<dir>`. Pure function `check_start.verdict(ranges, angle_min, angle_increment, run_dir) -> StartCheck(go, fit, offset_m, offset_deg, pose)`. Limits 0.30 m, 12°, fit at least 0.80. Exit 0 for GO, 1 for NO-GO.

Why: `seed_start_pose` reports success as soon as the filter echoes the seed, which it does wherever the car really is.

- [x] **Step 1: Tests:** GO at the start; NO-GO 2 m along the track, and at 30° turned on the spot, and with a blank scan; a static test that the module creates no publisher.
- [x] **Step 2: Implement; add to `commands_text`; green; mutation check; commit**

### Task 7: Floor validation

This one needs the car and a person; it is the acceptance test for everything above. Hold LB throughout, with someone at the battery switch.

- [ ] **Step 1: Three `map_course` runs at the shipped speed:** `ros2 run racerbot_launch map_course imu:=true gyro_odom:=true`. Success: 3 laps close in each with no Ctrl+C, the verdict is GO on all three, and `slam_report` shows no jump over 1.5 m.
- [ ] **Step 2: One run at `mapping_max_speed:=1.5`.** Success: same, and no false closure. This is the dense-node case replays only approximate.
- [ ] **Step 3: Compare the maps:** all four should agree to within 10 cm on most walls, and `check_start` should say GO on each from where its run ended.
- [ ] **Step 4: Race one at 1.5 m/s,** wheels up first, then the floor.
- [ ] **Step 5: Decide the defaults** with those results: whether `map_course` should default to `imu:=true gyro_odom:=true` on car 2 (a pinned difference from car 1, so it changes `docs/car1-vs-car2.md` and its test), and run the odom wizard for the 3-4% wheel scale.

### Task 8: Docs

**Files:** `docs/localization.md` (closure and chain reasoning, `slam_gyro.yaml`), `docs/racing-autonomy.md` (the 4.8 m/s line), `docs/launchstart.md` and `docs/operations.md` (`check_start`, what a stopped run now keeps), `docs/run-diagnostics.md` (`rebuild_run.py`), `docs/car1-vs-car2.md` (SLAM profile row), `CLAUDE.md` (known facts).

- [ ] **Step 1: Update each, keeping the old reasoning as history where it explains a decision**
- [ ] **Step 2: `python3 .claude/skills/novice-docs/scripts/check_docs.py` on those files**
- [ ] **Step 3: Commit**
