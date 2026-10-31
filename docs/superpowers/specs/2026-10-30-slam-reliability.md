# Why mapping keeps failing on car 2, and the fix

> **Who this is for:** anyone mapping a course with `map_course`, and whoever works on SLAM next.
> **Read first:** [localization.md](../../localization.md) for what SLAM, odometry and a loop closure are, and the [glossary](../../glossary.md).
> **You'll be able to:** see what was wrong on 2026-10-30, the measurement behind each claim, what was tried and rejected, and what is still unproven.
> **Time:** 5 minutes for [the short version](#the-short-version); the rest is reference.

**Status:** investigation finished 2026-10-30. Every number here is from replaying recorded runs. **None of the fix has driven on the floor yet.** Implementation: [the plan](../plans/2026-10-30-slam-reliability.md).

## Contents

- [The short version](#the-short-version)
- [How it was measured](#how-it-was-measured)
- [The problems, in the order they bite](#the-problems-in-the-order-they-bite)
- [What was tested and ruled out](#what-was-tested-and-ruled-out)
- [The replay results](#the-replay-results)
- [The fix](#the-fix)
- [What this does not prove](#what-this-does-not-prove)

## The short version

On 2026-10-30, 21 runs produced no saved map. It looked like one problem, "SLAM is bad". It was four, stacked:

| # | Problem | Evidence | Fix |
|---|---|---|---|
| 1 | The lap counter mistook driving for SLAM corrections above 3 m/s, dragged its own start point away, and never closed a lap | Replaying the real recorder reproduces the log; the start walked 13 m | Count only motion odometry did not see |
| 2 | SLAM ignored the gyro, and its heading drifted 6-10° per lap | Over 25 m: gyro 1.0° off, SLAM 5.3° off | Weight the gyro prior |
| 3 | Loop closure was geometrically impossible at speed | 0.0% of nodes can ever satisfy `chain 20` inside a `3 m` radius | Chain 10 |
| 4 | Stopping a run saved nothing, and nothing could tell a good map from a doubled one | 5 laps of data, no file. A doubled map scores 0.97 on scan fit | Rebuild from the bag after every run; a see-through test |

Problems 2 and 3 together are why results looked random: whether a map came out depended on whether heading drift happened to leave the car inside the closure window at the end of lap 1.

## How it was measured

Everything below needs a trustworthy "truth" that does not come from the SLAM being judged. Here it is:

1. Take one verified map (the final map of run `20261030-185036`).
2. Localize **other runs'** scans on it, with a global search for the start and only odometry steps between scans. Runs `184556` and `184826` fit it at 97.3% and 97.7% of beams within 10 cm, and the global search put both at the same start spot to 3 cm. So the course did not change, and that map is a fair reference.
3. A negative control: the same scans on a wrong map or a mirrored one score 30-40%.

Replays use the existing `tools/run_analysis/slam_replay.py`: the recorded `/scan` and odometry go through a real `slam_toolbox` on its own ROS domain, with the bringup stopped. Each replay is scored by registering its map to the reference and comparing live pose, graph nodes and walls against the truth.

Replays are not deterministic: the same settings on the same data gave wall agreement of 0.31 and 0.64. **So every setting below is judged over repeats, on two runs.**

## The problems, in the order they bite

### 1. The lap counter's false corrections

`LapRecorder` treated a map-frame step over `max_pose_jump_m` (0.12 m) as SLAM correcting itself and applied it to the recording, stored start included. The reasoning in the docs was "0.12 m at 40 Hz is 4.8 m/s". But the transform updates at 50 Hz, so a 40 Hz tick sometimes spans two updates, which reaches 0.12 m at 3 m/s.

| Run `20261030-185036` | Logged | Real `LapRecorder` replayed on `/slam_pose` |
|---|---|---|
| Corrections absorbed at 108 s | 305 | 305 |
| Start distance at 62.8 s | 9.78 m | 9.79 m |
| Laps closed | 0 | 0 |

The stored start walked from (-3.17, +3.11) to (-6.10, -9.70) while the car drove laps. The car passed its real start 5 more times. Each lap also failed `mapping_lap_max_reanchors` (20) at about 70 "corrections".

### 2. SLAM ignored the gyro

| Over 25 m of track, run `185036` (median / p95) | Heading error | Position error |
|---|---|---|
| Gyro odometry (`/odom/imu`) | 1.0° / 3.1° | 0.51 / 0.97 m |
| SLAM's live pose | 5.3° / 8.0° | 0.63 / 1.65 m |
| Steering odometry (`/odom`) | 11.2° / 28.8° | 4.07 / 9.17 m |

Over a full lap (50 m) the gyro drifts 2° and 0.28 m, and most of that position error is a wheel scale problem, not the gyro.

Scan matching should improve on that. It makes it worse, because the prior has almost no weight. As recalled from Karto's source (only headers are installed here), the angle penalty is `1 - 0.2 * dθ² / angle_variance_penalty`, floored at `minimum_angle_penalty`. With the shipped 1.0 and 0.9, a 5° disagreement with the gyro costs a match 0.15%.

The result is a steady one-way drift, the same sign in both runs: about -0.15° per metre through lap 1.

### 3. Loop closure could not happen

`slam_tracking.yaml` set `loop_match_minimum_chain_size: 20`, with the reasoning that a chain's length is `chain_size × minimum_travel_distance` = 5 m. That is true only at low speed. Above about 1.25 m/s the 0.2 s time gate decides the spacing: **0.57 m median** on these runs.

`slam_gyro.yaml` then narrowed the search from 8 m to 3 m, to stop closures landing on a neighbouring stretch. It kept chain 20. A 3 m radius holds about 6 m of path, so about 11 nodes:

| Search radius | Longest older chain, median | Nodes that could ever reach chain 20 | Chain 10 |
|---|---|---|---|
| **3 m (shipped with the gyro)** | 11 | **0.0%** | 67.5% |
| 5 m | 22 | 59.1% | 91.7% |
| 8 m | 35 | 91.2% | 92.2% |

So with correct poses a closure cannot fire. It fires only when drift has bent the path enough to fit, or the car has slowed in a corner. That is the source of the apparent randomness:

| | First return to the start | Outcome |
|---|---|---|
| `185036` | -5.7°, 1.04 m off, **inside** the ±1.5 m window | Closes; error falls to 0.46 m |
| `184556` | -9.6°, 2.33 m off, **outside** it | Never recovers: -14° to -18°, up to 4 m off, doubled walls |

### 4. No map, and no way to judge one

`map_course` saves only after the last lap, so Ctrl+C keeps nothing. And nothing could say whether a map was any good: scan fit does not detect doubling, because a beam ending on either copy of a wall still counts.

**A laser cannot pass through a wall.** So the share of beams that cross a mapped wall well before their own end measures doubling directly, with no reference map:

| Map | Scan fit | See-through |
|---|---|---|
| `185036` live (clean) | 0.978 | 4.9% |
| `185036` with the gyro prior | 0.979 | 8-10% |
| `184556` live (doubled) | see note | 25.7% |
| `184556` shipped settings, replayed | 0.966 | 40.6% |
| `184556` prior only, no closures | **0.975** | **47.5%** |

Note: where the map is badly doubled the global start search can also fail, which is its own refusal.

Also, the live pose cannot be used for a racing line at all on shipped settings: lines recorded from it touch the final map's walls (0.00 m clearance) and sit up to 1.38 m from where the car really drove.

## What was tested and ruled out

| Suspect | Measurement | Verdict |
|---|---|---|
| Gyro timing | Lag within ±6 ms, correlation 0.99+, against LiDAR-derived yaw rate | Not the cause |
| Gyro scale | Gain 0.992-0.999 | Not the cause |
| Motion distortion | Deskewing every scan makes the fit worse: 3.15 → 3.20 cm residual | Not the cause |
| LiDAR mount yaw / sideslip | About -0.7° intercept, repeatable on 3 runs | Not the cause |
| Larger scan buffer (`scan_buffer_size` 30) alone | 0.416 wall agreement | Does not help |
| Scan queue drops | About 4 per 10 s on every run, at 30% CPU | Benign |

**Found on the way, not fixed here:**
- **Wheel distance reads 3-4% long** on all three runs, roughly flat across turning and acceleration. Implied `vesc_to_odom` gain is about 4631; the file says 4447.98 and the driver uses 4614. It explains most of the gyro odometry's position error. Calibrate with the odom wizard; it is not what breaks SLAM.
- **Steering odometry's yaw leads by 138 ms and over-reads by 34%** (gain 0.744). Any run mapped without the gyro inherits that: `184826`'s corrections reached 3.4 m. That is why the gyro should be on for mapping.

## The replay results

Each row is one replay. **Wall** is the share of the map's walls within 10 cm of the reference. **Live** and **graph** are position errors against truth.

| Settings | Run | Wall | Live p95 | Heading p95 | Graph p95 | Solves |
|---|---|---|---|---|---|---|
| Shipped | 184556 | 0.33, 0.30 | 6.49, 5.84 m | 29.8°, 26.3° | 5.40, 4.04 m | 3, 4 |
| Shipped | 185036 | 0.64, 0.31 | 4.01, 4.79 m | 17.3°, 6.5° | 1.31, 4.37 m | 9, 5 |
| No closures | 184556 | 0.34, 0.34 | 4.60, 3.82 m | 25.6°, 22.5° | 3.41, 2.98 m | 0 |
| No closures | 185036 | 0.26, 0.27 | 9.19, 8.08 m | 47.3°, 42.7° | 6.95, 6.06 m | 0 |
| Prior only | 184556 | 0.59, 0.74 | 0.76, 0.34 m | 5.2°, 3.2° | 0.58, 0.25 m | 0 |
| Prior only | 185036 | 0.88, 0.88 | 0.51, 0.34 m | 3.2°, 2.6° | 0.12, 0.12 m | 4, 5 |
| Prior + chain 7 | 184556 | 0.91, 0.91 | 0.18, 0.16 m | 2.2°, 2.1° | 0.10, 0.10 m | 36, 39 |
| Prior + chain 7 | 185036 | 0.67, 0.74 | 0.39, 0.27 m | 2.0°, 2.3° | 0.26, 0.23 m | 52, 52 |
| **Prior + chain 10** | **184556** | **0.91, 0.91, 0.91** | **0.27, 0.17, 0.30 m** | **2.3°, 2.3°, 2.2°** | **0.10, 0.10, 0.09 m** | **32, 34, 30** |
| **Prior + chain 10** | **185036** | **0.92, 0.92, 0.91** | **0.21, 0.16, 0.19 m** | **2.1°, 2.1°, 2.3°** | **0.09, 0.08, 0.09 m** | **51, 50, 52** |
| Same, dense nodes (like slower driving) | 184556 | 0.91, 0.91 | 0.14, 0.15 m | 2.1°, 2.1° | 0.08, 0.08 m | 34, 35 |
| Same, dense nodes | 185036 | 0.83, 0.82 | 0.27, 0.31 m | 2.1°, 2.4° | 0.12, 0.14 m | 52, 40 |
| Same, window ±1 m | 184556 | 0.91, 0.91 | 0.24, 0.15 m | 2.4°, 1.9° | 0.10, 0.09 m | 33, 34 |
| Same, window ±1 m | 185036 | 0.93, 0.92 | 0.16, 0.14 m | 2.1°, 2.1° | 0.09, 0.09 m | 49, 48 |
| Same, plus a distance prior | 184556 | 0.76, 0.75 | 0.34, 0.33 m | 2.8°, 2.7° | 0.25, 0.25 m | 30, 29 |
| Same, plus a distance prior | 185036 | 0.73, 0.74 | 0.38, 0.37 m | 2.6°, 2.4° | 0.26, 0.27 m | 46, 44 |
| **Chain 10 with no prior** | 184556 | **0.22** | 4.95 m | 10.8° | 4.00 m | 26 |

On these two runs, shipped settings gave at best 0.64. Prior + chain 10 gave 0.91-0.92 on all six replays with both runs behaving alike, and live-pose error fell by about 20×.

Three things this table decided:

- **The two changes must ship together.** Chain 10 alone is the worst result here: closures fire easily, but with a drifting pose they land in the wrong place.
- **The ±1 m window is not adopted.** It is no better beyond noise on two runs, so it is not worth moving a value car 1 validated.
- **The distance prior is rejected.** It made every map worse, most likely because wheel distance reads 3-4% long, so it pulls on a wrong number. It may be worth another look after the odom wizard fixes that scale.

## The fix

The prior profile lives in `slam_gyro.yaml`, loaded only with `gyro_odom:=true`, because a tight prior on steering odometry would be wrong:

| Parameter | Shipped | New | Why |
|---|---|---|---|
| `angle_variance_penalty` | 1.0 | 0.01 | A 5° disagreement costs about 15%, not 0.15% |
| `minimum_angle_penalty` | 0.9 | 0.5 | Lets that penalty act: at 0.9 it stops at 10% |
| `coarse_search_angle_offset` | 0.349 | 0.175 | ±10°, not ±20°: the gyro is never 20° out in 0.6 m |
| `coarse_angle_resolution` | 0.0349 | 0.0175 | Keeps the coarse search as fine relative to its width, and costs no extra time |
| `loop_match_minimum_chain_size` | 20 | 10 | About 6 m of evidence at speed, and reachable inside 3 m |

The rest is in [the plan](../plans/2026-10-30-slam-reliability.md): the lap counter fix, a `map_localize` library, a `rebuild_run.py` tool that map_course runs after every run (so Ctrl+C keeps its map), and a read-only `check_start`.

**Why a rebuild step as well as better SLAM?** Because it turns "probably fine" into a measured verdict on every map, and it is the only thing that can salvage a run stopped early. It runs after the car has stopped, so it changes nothing in any node that can move the car.

## What this does not prove

- **Only 2 runs at about 2.5 m/s mean.** Slower driving makes nodes denser, which changes chain length in metres. The dense-node replays approximate that and held up (0.82-0.91); a real slow run is still needed.
- **The verdict's limits come from one course on one evening.** 15% see-through separated these maps, but another course may sit differently.
- **A chain of 10 is less evidence than 20.** Replays showed no false closure on this course. A course with more repetition could be different, and this course's nearest parallel stretch is what the ±1.5 m window protects against.
- **Replays are not the car.** CPU load and timing differ, which is why the [floor test in the plan](../plans/2026-10-30-slam-reliability.md#task-7-floor-validation) is the real acceptance test.
- **The Karto penalty formula is from memory.** The measured improvement stands either way, since it does not depend on the recalled formula.
