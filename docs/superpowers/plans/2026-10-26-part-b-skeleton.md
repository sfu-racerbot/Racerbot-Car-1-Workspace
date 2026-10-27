# Part B (IMU, observability, recording, overtaking) skeleton plan

> **Who this is for:** whoever expands and builds a Part B item, in this session or another, and the bringup-datalogger session (B1's recording contract).
> **Read first:** [the spec](../specs/2026-10-26-slam-and-gap-follow-plan.md), and [glossary.md](../../glossary.md) for [odometry](../../glossary.md#odometry--odom), [topic](../../glossary.md#topic), [scan](../../glossary.md#scan) and [deadman](../../glossary.md#deadman).
> **You'll be able to:** pick up any Part B item, see what it must deliver and when it may start, and expand it into a full plan.
> **Time:** 10 minutes to read. Each item is days of work once expanded.

> **For agentic workers:** this is a **skeleton**. Each item below must be expanded into its own full plan (superpowers:writing-plans) before anyone builds it. Expand one item at a time, in the order given, in this session or another.

**Goal:** everything from the 2026-10-26 requests that is not built in Part A: SLAM observability, `map_course` IMU flags, a simulated IMU, `gap_follow` IMU features, overtaking, and car 1's course in the simulator.

**Spec:** [docs/superpowers/specs/2026-10-26-slam-and-gap-follow-plan.md](../specs/2026-10-26-slam-and-gap-follow-plan.md), Part B and "Decisions already made". The Part A plan ([2026-10-26-part-a-slam-and-gap-follow.md](2026-10-26-part-a-slam-and-gap-follow.md)) builds the pieces several items reuse:
- `slam_metrics`
- `slam_bag`
- `slam_replay`
- `weave`
- `gap_logic.steering_damping`

## Global Constraints

Everything in Part A's Global Constraints applies unchanged, plus these decisions from 2026-10-26:
- **IMU failure:** each IMU feature switches itself off and `gap_follow` drives as it would without it. A loud warning goes to the log, and `/drive_intent` carries the reason.
- **Flags:** one per feature, plus `imu_all`, with the same names in `map_course` and `gap_follow_launch.py`.
  - All default off.
  - An explicit `false` beats `imu_all`.
  - Any IMU flag with `imu:=false` refuses to start.
- **Analysis runs after the run,** never on the car's CPU while it drives or maps.

## Order and entry criteria

| Item | Starts when | Why this order |
|---|---|---|
| B1 SLAM observability | Part A Tasks 1-3 merged, and the bringup datalogger (another session) has a branch to read | Every later SLAM change is judged with it |
| B2 `map_course` IMU flags | B1's `slam_report` exists | Its effect is measured with B1 |
| B3 Simulated IMU | Any time after Part A | B4's closed-loop tests need it |
| B4 `gap_follow` IMU features | B3 merged | Each feature is proven in the simulator first |
| B6 Overtaking | The team answers the spec's open question 1 | It is a margin trade-off, not a tuning task |
| B7 Car 1's course in the simulator | Part A Task 4 produces a clean `map.pgm` of the course | It needs a map |

B5 (recording every run) is being built by another session as a **bringup datalogger**. This plan only states what SLAM needs from it (the contract under B1).

---

## B1. SLAM observability

**Outcome:** after any run with SLAM, one command (or `map_course` automatically) says:
- which odometry SLAM used;
- how far each odometry source drifted;
- every correction and loop closure, sized at the car;
- per-lap closure and quality numbers;
- how consistent the final map is with the scans;
- when and why SLAM dropped scans.

A replay can **recreate exactly** what SLAM did.

### The recording contract (for the bringup datalogger session)

The datalogger owns recording. SLAM's needs are:

1. **Always in the topic list.** A topic nobody publishes records nothing, so these cost nothing on runs without SLAM:
   - `/scan`, at full rate and never throttled;
   - `/tf`, and `/tf_static` with transient-local durability;
   - `/odom`, `/odom/imu`, `/odom/fused`;
   - `/imu/data`, `/imu/bno/data`, `/imu/vesc/data`, `/imu/bno/raw`, `/sensors/imu/raw`, `/sensors/core`, `/sensors/servo_position_command`;
   - **`/pose`**, slam_toolbox's, published once per scan it processes; **this is the processed-scan record**;
   - `/slam_pose`, `/map`, `/map_metadata`, `/slam_toolbox/graph_visualization`;
   - `/drive`, `/teleop`, `/ackermann_cmd`, `/joy`, `/drive_intent`, `/initialpose`, `/rosout`, `/diagnostics`.
2. **When `slam_toolbox` appears on the graph** (poll the node list), snapshot into `<run>/metadata/`:
   - its full parameter set, as `params/slam_toolbox.yaml`, in the same format `race_diagnostics`' `snapshot_params` writes today;
   - the `odom_frame` it was given;
   - `slam_toolbox`'s package version.
3. **On every run, with or without SLAM:**
   - `tegrastats --interval 1000` to `<run>/system/tegrastats.log`. It works without sudo on car 1, checked 2026-10-26.
   - 1 Hz CPU %, RSS and thread count per ROS node process from `/proc`, to `<run>/system/proc.jsonl`.
4. **Folder layout** stays `record_run.py`'s: `<run>/bag/` (mcap), `<run>/metadata/{params,git,files}/`, `<run>/events.jsonl`, plus the new `<run>/system/`. The replay and report tools read that layout.
5. **`map_course` stops starting its own recorder** once the datalogger records SLAM runs. The `diagnostics:=` default flips. This is one place to record, as decided 2026-10-26.

**Why no end-of-run save is needed:** `/pose` stamps mark exactly which scans SLAM processed. Checked on 2026-10-26's runs: 315 `/pose` against 314 SLAM updates (`-194913`), 229/228 and 257/255. That survives Ctrl+C.

### Tasks to expand

1. **`slam_replay --exact`:** run `sync_slam_toolbox_node` and play only the `/scan` messages whose stamps match `/pose` stamps. The test is that the replayed `/pose` sequence equals the recorded one to 1 mm and 0.1°.
2. **`slam_report.py`** (`tools/run_analysis/`, cached `analysis/slam.npz`, unsourced after the first run). It reports:
   - setup: odometry frame, speeds, distance;
   - heading drift of `/odom`, `/odom/imu` and SLAM against each other;
   - corrections and loop closures, from Part A `slam_metrics`;
   - per-lap closures, from `race_diagnostics.run_events`;
   - map consistency: the share of scan endpoints within one cell of an occupied cell of the final map, per lap;
   - scan drops against CPU, from `system/`;
   - wheel distance against scan-matched distance.
   - Output: `slam_report.txt`, `summary.json` and PNGs. `--compare runA runB` gives a side-by-side table and overlays.
3. **Automatic analysis:** the `map_course` wrapper runs `summarize_run`, `imu_report` and `slam_report` after the launch exits; `analysis:=false` skips them. The wrapper strips the argument before the launch.

**Tests (shape):**
- Every metric against a synthetic run with a closed-form answer, including map consistency with a known scan offset.
- Edge cases: empty, single sample, NaN, missing topics.
- `--exact` against a recorded run.

## B2. IMU flags in `map_course`

**Outcome:**
- `imu_lap_turns:=true` makes the lap counter count turning from the gyro (`auto_map_race_node` `odom_frame: odom_imu`). It is a parameter only, and needs `gyro_odom:=true`.
- `imu_all:=true` turns on `gyro_odom`, `imu_lap_turns` and `fused_speed`, and later every B4 flag. An explicit `false` overrides it.

**Files:** `src/racerbot_launch/launch/map_course_launch.py`, `src/racerbot_launch/test/test_map_course_imu.py`, `docs/launchstart.md`.

**Tests (shape):**
- Each flag sets its parameters.
- `imu_all` expands correctly, and an explicit `false` wins.
- Every refusal is checked with `pytest.raises(RuntimeError, match=...)`.
- The lap counter's frame name equals `vehicle_state.yaml`'s `imu_odom_frame`.

**Known limitation to document:** if `vehicle_state_node` dies mid-run with `gyro_odom` on, SLAM loses odometry and the map is lost. The car stays safe, because `gap_follow` steers from `/scan`. `slam_report` flags it.

## B3. Simulated IMU in `racerbot_sim`

**Outcome:** `gym_bridge_node` publishes `/imu/bno/data`, `/imu/vesc/data`, `/imu/data` and a latched `/imu/status` in `imu_bridge`'s format, all from the gym's true state:
- **Gyro z:** the yaw rate.
- **Accelerometer:** forward from the change in speed, sideways from speed × yaw rate, and gravity on z.
- **Noise:** measured from a stationary stretch of a real bag.

The real `vehicle_state_node` runs on top (`vehicle_state_launch.py hardware:=false` skips the BNO bridge and `imu_bridge_node`).

**Fault injection:** parameter `imu_fault`, one of `none`, `stale`, `status_fault`, `bias`, `scale` or `zero`, settable mid-run.

**Safety:** `imu_bridge_node` and the BNO bridge join `hardware_guard.py`'s refuse list.

**Files:** `src/racerbot_sim/racerbot_sim/{sim_bridge.py,gym_bridge_node.py,hardware_guard.py}`, `src/imu_bridge/launch/vehicle_state_launch.py`, `src/racerbot_launch/scripts/map_course` (it stops forcing `imu:=false` in sim).

**Tests (shape):**
- On a constant circle: the gyro equals v/R and the sideways acceleration equals v²/R.
- On a ramp, the forward acceleration matches.
- The noise spread matches its setting.
- Each fault mode gives exactly its output.
- `/imu/status` parses in `imu_health` and `neural_racer`.
- Reproduction: with `odom_yaw_rate_scale` about 1.1, `map_course` fails without `gyro_odom` and succeeds with it.

**Limit:** the gym's tyre force is linear in slip angle (`.sim/f1tenth_gym/.../single_track.py`), so the car can't slide. Grip features are checked for wiring and fall-back only.

## B4. `gap_follow` IMU features

Each feature gets its own flag, falls back as decided, keeps the four deadman deny-path tests, and has a design approved before it is built.

| Flag | Feature | Main input | Note |
|---|---|---|---|
| `fused_speed` | TTC braking and the acceleration ramp on `/odom/fused` | `/odom/fused` | Exists in `map_course`; add it to `gap_follow_launch.py` |
| `imu_grip_cap` | Slow down as measured sideways acceleration nears a limit | `/imu/data` linear y, speed | Thresholds come from bag replay, not the simulator (B3 limit) |
| `imu_understeer` | Slow down when the car turns much less than commanded | `/steering/measured` against the command | NaN below 0.3 m/s means no action |
| `steering_damping` | Gyro yaw-rate damping | `/imu/data` angular z | **Built in Part A Task 9** if its gate opens |
| `imu_traction` | Hold acceleration while the wheels outrun the car | `/odom` against `/odom/fused` | |
| `imu_impact_stop` | An accelerometer spike stops the car until LB is released and pressed again | `/imu/data` linear | A safety addition; its threshold comes from recorded bumps |

`imu_all` turns them all on, in both `gap_follow_launch.py` and `map_course`.

**Tests (shape):** per feature:
- the logic against closed forms;
- the four IMU failure modes (status `fault`/`waiting`, stale, never received, NaN) give today's command;
- the deadman deny paths with the flag on;
- a simulator run with B3, including a fault injected mid-run.

## B5. Recording every run

Owned by the bringup-datalogger session. Nothing to build here beyond the B1 contract. Part A Task 7 and B1 read whatever folder it writes, as long as it keeps the layout in the contract.

## B6. Overtaking

**Blocked on the team's answer:** how much side margin may the car give up to pass, on a 1.5-1.8 m track where a pass needs about 2.4 m?

**Then:**
- Recognise the other car sooner: today it needs 8 consecutive detections (`opponent_tracking.py`).
- Follow at the other car's speed instead of braking to 0.
- Pass only where the chosen margin fits.

**Evidence base:**
- The 19:51, 19:59 and 20:07 races' decision logs. They have no bags; future races get bags through B5.
- `gap_follow_launch.py opponent_awareness:=true` in the multi-car simulator (`docs/gap-follow.md#several-cars-at-once`).

## B7. Car 1's course in the simulator

**Outcome:** the clean map from Part A Task 4's replay becomes a track for `eval_gap.py` and `racerbot_sim`, beside `indoor_oval`, `indoor_tight`, `indoor_wide` and `asb_10000`. `gap_follow` is then tuned on the course it races.

**Files:** `src/racerbot_sim/racerbot_sim/tracks.py`, `tools/f1tenth_sim/eval_gap.py`.

**Tests (shape):** the track loads, its centreline closes, and its width matches the map's measured corridor width (1.5-1.8 m) within one cell.
