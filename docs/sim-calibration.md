# Simulator calibration: making the simulated car drive like the real one

> **Who this is for:** anyone about to trust a simulator result for this car, or about to measure the car again on a new floor.
> **Read first:** [course-analysis.md](course-analysis.md), for how a recorded run is located against a map. [simulator.md](simulator.md) for what the simulator is.
> **You'll be able to:** run the simulator on the measured floor of 2026-10-30, check it against the real car yourself, and measure a new floor the same way.
> **Time:** 15 minutes to read. Re-running the validation takes about 10 minutes.

The simulator now has a measured **floor profile**: a named set of numbers taken from real runs on one floor with one set of tyres. On the run it was checked against, it laps within 1% of the real car and drives the same bend the same way.

## Highlights

- **Laps within 1% on a run it never saw.** Real run `20261030-184826` lapped in 20.70 s. The simulator, driving that run's own launched settings, lapped in 20.56 s median over 24 laps (20.30-20.72). Before calibration it wedged at the bottom-right bend in 6 of 6 seeds.
- **Speed within 0.24 m/s, section by section.** Over 21 sections of 2.5 m, the RMS speed difference is 0.24 m/s, the worst 0.40 m/s.
- **The real stop, to 1 cm.** The real car stopped from 5.41 m/s (on its wheels) in 3.39 m, measured by the wall ahead. The simulator takes 3.38 m. It took 3.67 m before.
- **Grip as a curve, not one number.** The real car follows the request to about 2.5 m/s², then rolls off gradually. A [progressive grip model](#grip) reproduces that curve to 0.17 m/s² RMS, against 0.31 before.
- **Wall fragments that look like cars.** Real scans break walls into pieces that the opponent tracker (the part of `gap_follow` that spots other cars, [gap-follow.md](gap-follow.md)) mistakes for moving cars, 22.5 times per 100 s. The old simulator produced none. Now it produces 17.2.
- **Fit on some runs, judged on others.** Six runs were used to fit, six were held back, and the table below is scored on held-back runs.

**Limits:** one floor, one day, one car. Nothing here is proven until it is driven on the floor. What it still gets wrong is listed in [What it cannot reproduce](#what-it-cannot-reproduce).

### Why it exists

On 2026-10-30 the simulator said settings were fast and safe that the real car could not use, and wedged the car at a bend the real car drove every lap. Tuning against a simulator that disagrees with the floor tunes the simulator, not the car.

## Using it

**Any terminal, drivers not needed:**

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
cd ~/racerbot-ws
python3 tools/f1tenth_sim/eval_gap.py --track course_2026_10_30_scans --floor sfu_course_2026_10_30 --seeds 6
```

**Working when:** each seed prints a line with a lap time near 20 s and no contact.

`--floor` works on `eval_gap.py`, `eval_gap_multi.py`, `sweep_gap.py`, `sweep2.py` and `trace_gap.py`. Without it you get the plain calibration, exactly as before.

Use the floor profile with the track it was measured on, `course_2026_10_30_scans`. Its grip belongs to that floor. On another track it describes a car on the wrong floor.

## What was measured

Every item below came from the runs of 2026-10-30. They use the [LiDAR](glossary.md#lidar) (the spinning laser that measures distance to walls) and [odometry](glossary.md#odometry--odom) (the car's own estimate of its speed and turning). Each run was located against `data/racerbot_auto/20261030-185036-fixed` with `loc.py`.

**The split.** Fit runs: `185036`, `184556`, `175728`, `180355`, `181616`, `183607`. Held-out runs: `184826`, `183134`, `181003`, `181819`, `184316`, `175124`. Both lists are in `sim_fidelity/calibration.py`, and a test checks they do not overlap.

| # | Item | Real car | Before | After |
|---|---|---|---|---|
| 1 | Grip: lateral achieved for 3.0-3.5 asked | 2.61 m/s² | 2.93 (sharp limit at 5.4) | 2.65 |
| 1 | Grip: whole curve, RMS against real | | 0.31 m/s² | 0.17 m/s² |
| 2 | Steering: yaw rate error, real commands replayed, held-out runs | | 0.233 rad/s | 0.166 rad/s |
| 2 | Steering: same, fit runs | | 0.198 rad/s | 0.141 rad/s |
| 2 | Steering trim (rack left of command) | 0.0105 rad | 0 | 0.0105 rad |
| 2 | Full stick 0.436 rad turns | 0.423 rad both sides | 0.436 | 0.423 |
| 3 | Command to speed: delay | 75 ms | 50 ms | 75 ms |
| 3 | Top speed, on the wheels | 5.39-5.44 m/s peaks | 5.04 | 5.23 |
| 3 | Speed, real commands replayed, held-out runs, RMS | | 0.73 m/s | 0.82 m/s (worse) |
| 4 | Stop from 5.41 m/s (wheels), by the wall | 3.39 m, 1.40 s | 3.67 m, 1.40 s | 3.38 m, 1.20 s |
| 4 | Stop from 3.10 m/s (wheels), by the wall | 1.31 m, 0.93 s | | 1.23 m, 0.72 s |
| 5 | Wheel speed over true speed | 1.033 | 1.000 | 1.033 |
| 5 | `/odom` yaw rate over gyro | 1.282 | 1.189 | 1.290 |
| 5 | `/odom` yaw leads the gyro by | 0.112 s | 0.162 s | 0.137 s |
| 6 | LiDAR noise, parked | 3 mm near, 10 mm past 4 m | 10 mm flat | 3 mm + 1.5 mm per m |
| 6 | Beams with no echo | 4 in 10 million | 0.2% | 0 |
| 6 | Beams hitting the car itself | 23, at -135 to -130° | none | the same 23 |
| 6 | Range jumps over 0.3 m, per scan | 7.8 | 3.3 | 10.6 |
| 6 | Lone returns, per scan | 12.0 | 2.7 | 6.1 |
| 8 | Map walls against real scans, held out | | +6.4 cm too close | +1.0 cm (contact walls), +2.4 cm (beam walls) |

"Before" is the plain calibration on the saved map `course_2026_10_30`. Rows 2 and 3 replay real `/drive` commands through the plant on an empty floor; the rest drive the held-out run's settings on the course.

Item 7, geometry and mass, has no error to measure. See [Measured and assumed](#measured-and-assumed).

## Did it drive like the car?

Scored on held-out run `20261030-184826`, a `gap_follow` lap at the 3.5 m/s cap. The simulator drove that run's own launched parameters, read from its `metadata/params`, with the code paths of that day (the four parameters in `OLD_CODE` set to what the code then did). Six seeds, 115 s each.

| Measure | Real | Before | After |
|---|---|---|---|
| Flying lap, median | 20.70 s | no lap | 20.56 s (24 laps) |
| Flying lap, range | 20.65-20.74 | | 20.30-20.72 |
| Seeds with wall contact | 0 | 0 of 6 | 0 of 6 |
| Seeds wedged at the bottom-right bend | 0 | 6 of 6 | 0 of 6 |
| Bend: share of time in `corner_fallback` | 0.31 | 0.08 | 0.48 |
| Bend: mode changes per pass | 2.7 | 1.0 | 3.5 |
| False moving cars per 100 s | 22.5 | 0.0 | 17.2 |
| Section speed, RMS against real | | | 0.24 m/s |
| Section speed, worst / bias | | | 0.40 / -0.05 m/s |
| Stop from 5.41 m/s | 3.39 m | 3.67 m | 3.38 m |

The fit runs score the same way: real laps 20.59-20.75 s.

**To check it yourself, any terminal:**

```bash
R=data/racerbot_runs/20261030; M=data/racerbot_auto/20261030-185036-fixed
python3 tools/run_analysis/loc.py $R-184826 --map $M
python3 tools/run_analysis/car_measure.py $R-184826 $R-183134 $R-181003 $R-181819 $R-184316 $R-175124 --json /tmp/held.json
python3 tools/f1tenth_sim/validate_calibration.py real $R-184826 --line $M/raceline_raw.csv \
    --measured /tmp/held.json --stop-m 3.39 --stop-s 1.40 --out /tmp/real.json
python3 tools/f1tenth_sim/validate_calibration.py sim --params-from $R-184826 --line $M/raceline_raw.csv \
    --out /tmp/before.json
python3 tools/f1tenth_sim/validate_calibration.py sim --params-from $R-184826 --line $M/raceline_raw.csv \
    --floor sfu_course_2026_10_30 --out /tmp/after.json
python3 tools/f1tenth_sim/validate_calibration.py table real=/tmp/real.json before=/tmp/before.json after=/tmp/after.json
```

`loc.py` needs each of the six held-out runs located before `car_measure.py` reads them; pass them all to `loc.py` the first time.

**Working when:** the last command prints the table above, give or take a few hundredths. The simulator side varies with the seed.

## What changed in the simulator

All of it lives in the floor profile `sfu_course_2026_10_30` in `tools/f1tenth_sim/sim_fidelity/calibration.py`, with a source for every number. The plain calibration is unchanged, and a test pins that.

### Grip

The old model was a sharp friction circle: the car turned exactly as asked up to 0.55 × 9.81 = 5.4 m/s², then not at all harder. The real car has no edge like that. It follows the request, then each extra unit asked buys less:

| Asked, m/s² | 2.25 | 3.25 | 4.5 | 5.5 | 7 | 10 |
|---|---|---|---|---|---|---|
| Real, achieved | 2.15 | 2.62 | 3.00 | 3.34 | 3.70 | 4.10 |

`ProgressiveGrip` in `sim_fidelity/grip.py` takes that table. It never delivers more than was asked, and a test checks that for the shipped profile.

The table in the profile is the real one times 1.15, because the simulator's tyres, which are stock F1TENTH ones, understeer by themselves on top of it. The 1.15 was fitted by replaying real commands against the gyro (1.15-1.2 equally good).

Braking does not cost cornering in this model. The real car showed almost none: asked 3-5 m/s², it achieved 2.73 while braking harder than 3 m/s² and 2.71 at steady speed.

### Steering

The trim and the full-stick angles above, with straight lines between. The command reaches the rack after two control ticks, 50 ms, then the servo closes on it at 15 per second.

<details>
<summary>Why not the measured linkage table (safe to skip)</summary>

Binning the rack angle (from yaw at low lateral acceleration) by the command gave a table that read 30% less rack at 0.1-0.2 rad. Replaying real commands through the plant rejected it: yaw rate error 0.24-0.31 rad/s with the table against 0.14 without.

Binning by a command that is lagged and noisy flattens the slope. Statisticians call it regression dilution.

The replay compares whole time series and does not suffer from it. That is why every steering and grip number here was chosen by replay.

`car_measure.py` still prints the table, because it is how anyone will first look at the linkage. Don't fit to it.

</details>

### Speed and braking

The VESC's speed loop is modelled as proportional each way, with a ceiling. Accelerating: gain 7 per second, at most 4 m/s². Braking: gain 7 per second, at most 4.5 m/s². Commands act after 75 ms.

The braking ceiling comes from a fit-run stop (run `180355`, 3.10 m/s, 1.31 m by the wall). It predicts the held-out 5.41 m/s stop of run `175124` at 3.44 m, against 3.39 measured.

### Odometry

`/odom` speed reads 1.033 times true speed, so the simulated `/odom` does too. The speed command is scaled by 4614 / 4447.98 / 1.033 = 1.004. That follows from `vesc.yaml` commanding at 4614 eRPM per m/s while `/odom` reads at 4447.98.

### LiDAR

Noise grows with range, no beam goes missing, and the 23 beams that see the car's own rear right corner read what they read on the real car.

### The map

The saved map's walls are three cells thick at the median, one to six, and stand about 6 cm proud of where the real scans land. `tools/run_analysis/scan_map.py` rebuilds them from the fit runs' scans into two layers:

- `course_2026_10_30_scans.png` is **what beams see.** It holds the cells beams end in, with the gaps the real walls have. The gaps make the fragments the tracker mistakes for cars.
- `course_2026_10_30_scans_contact.png` is **what the car hits.** The saved walls are peeled back to where beams end, never opened, so a car cannot drive through a gap a beam slipped through.

The plant judges contact on the second layer when a track has one.

## Measured and assumed

| Value | Status |
|---|---|
| Wheelbase 0.36 m, track width 0.30 m, LiDAR 0.26 m ahead of the rear axle | measured with a tape, 2026-08-24 |
| Steering limits, trim, full stick | measured: circles 2026-10-16, yaw on 2026-10-30 |
| Grip curve, steering delay, speed loop, braking, odometry, LiDAR | measured, this page |
| Overall length 0.535 m | Traxxas specification, not measured |
| Mass 3.74 kg, yaw inertia, centre of gravity height and position | **assumed**: F1TENTH Gym defaults, never weighed |
| Cornering stiffness | **assumed**: Gym defaults. The fitted grip curve absorbs their error on this floor, which is why the curve is a floor profile and not a fact about the tyres |

`python3 tools/f1tenth_sim/sim_fidelity/calibration.py` prints all 43 values with where each came from. Six are still Gym defaults.

## What it cannot reproduce

- **The soft end of a hard stop.** Real stops slow gently below about 1 m/s and take 1.40 s from 5.41 m/s. The model gets the distance (3.38 m) but stops in 1.20 s. A gain low enough to match the time makes ordinary driving brake too gently, and laps came out 5% fast.
- **The top speed overshoot.** The real wheels peak at 5.39-5.44 m/s, above the 5.23 the VESC limit should allow. The model stops at 5.23.
- **Open-loop speed.** Replaying real commands for 60 s with no controller, the speed error went from 0.73 to 0.82 m/s on held-out runs. In closed loop, the number that matters for lap times, sections are within 0.24 m/s.
- **Fewer false cars than real.** 17.2 per 100 s against 22.5. The rate swings from 16 to 41 between beam-layer thresholds of 25% and 28%, so treat it as good to about ±30%.
- **Different wall texture.** The simulator has more big range jumps than real scans (10.6 against 7.8 per scan) and half the lone returns (6.1 against 12.0). Real lone returns are mostly edge effects of the laser itself, not modelled.
- **A left/right grip difference.** At high asks the right side achieves 5-10% less than the left. The model is symmetric.
- **More bend flicker than real.** The simulator switches between `gap_follow` and `corner_fallback` 3.5 times per pass at the bottom-right bend, against 2.7.
- **LiDAR latency below one tick.** Real scans line up with the gyro to within 10 ms; the simulator delays them one 25 ms tick.
- **Anything about another floor.** The grip curve is this floor and these tyres on 2026-10-30.

## Measuring a new floor

1. Drive at least six runs on it, including some hard corners and one hard straight-line stop from over 3 m/s, and save a map.
2. Locate every run: `python3 tools/run_analysis/loc.py <runs> --map <map>`.
3. Split them: fit runs and held-out runs, with the stop in each if you can.
4. Measure: `python3 tools/run_analysis/car_measure.py <fit runs> --json fit.json`, and `--lidar` on a run where the car sat still for a while.
5. Rebuild the walls: `python3 tools/run_analysis/scan_map.py <fit runs> --map <map> --out tools/f1tenth_sim/maps/<name> --check <held-out runs>`.
6. Add a `Floor` to `FLOORS`, with a source for every number, and fit the grip scale and steering delay with `validate_calibration.py fit-plant`.
7. Score it on a held-out run with `validate_calibration.py`, as in [Did it drive like the car?](#did-it-drive-like-the-car).

<details>
<summary>The tools, one line each (safe to skip)</summary>

| Tool | Does |
|---|---|
| `tools/run_analysis/car_measure.py` | Wheel over-read, `/odom` yaw against the gyro, steering trim and lag, the grip curve by side, the speed loop, hard stops; `--lidar` for noise, dropouts and self hits |
| `tools/run_analysis/scan_map.py` | Rebuilds a map's walls from where real scans landed, in a beam layer and a contact layer |
| `tools/f1tenth_sim/validate_calibration.py real` | Scores located runs: laps, sections, bend flicker, false movers, scan texture |
| `validate_calibration.py sim` | Drives the simulator on a run's launched settings and scores it the same way |
| `validate_calibration.py fit-plant` | Ranks plant settings by replaying real commands against the gyro |
| `validate_calibration.py table` | Prints scored cards side by side |

</details>

## Speed on the calibrated floor

Run on 2026-10-31, after validation passed. Each row is 30 seeds in both directions: 60 laps from a standing start, on `course_2026_10_30_scans` with the floor profile.

"Time trial" is `aim_depth_bias 1.0` with opponent awareness off, as `time_trial:=true` sets.

| Settings | Clean laps | Lap | Closest body to wall |
|---|---|---|---|
| Shipped default (opponent awareness on) | 60/60 | 20.71 s | 9.1 cm |
| Time trial, lateral 3.0, steering 0.349 rad (shipped `time_trial:=true`) | 59/60 | 18.10 s | 1.4 cm |
| Time trial, lateral 3.5, steering 0.349 | 53/60 | 17.86 s | 0.4 cm |
| Time trial, lateral 3.0, steering 0.436 | 59/60 | 17.88 s | 4.3 cm |
| **Time trial, lateral 3.25, steering 0.436** | **60/60** | **17.74 s** | 4.2 cm |
| Time trial, lateral 3.5, steering 0.436 | 59/60 | 17.75 s | 3.3 cm |
| Time trial, lateral 3.75, steering 0.436 | 60/60 | 17.80 s | 3.5 cm |

A first pass at 6 seeds also ran lateral 4.0: 5 of 12 clean at steering 0.349, 9 of 12 at 0.436.

**What it says.** The steering limit is the lever. At 0.349 rad the car runs out of lock in the tight bends when allowed more lateral acceleration.

At 0.436 rad, full stick, lateral 3.25-3.75 all lap in about 17.75 s. Past that, the VESC's 5.04 m/s limit and the line cap the lap, not grip.

Opponent awareness costs about 0.5 s on one car (20.25 s without it, 6-seed pass), from the false cars.

**The fastest settings the measurements support:** time trial with `max_lateral_accel 3.25` and `max_steering_angle 0.436`, 17.74 s in the simulator against 20.6-20.9 s driven on 2026-10-30. Lateral 3.5 and 3.75 are no faster in the simulator, only closer to the wall.

**The risk, stated plainly.** The closest the body came to a wall in 60 laps was 4.2 cm, and the simulator puts walls within about 2 cm of the real ones. The real car may touch a wall. Steering 0.436 rad is full stick: the servo reaches it on both sides, but autonomy has never commanded it. These are not shipped defaults; the floor decides.

## Trying it on the floor

This needs `main` from 2026-11-01 or later built, because `time_trial:=true` arrived then. Put the car on a stand first, so the wheels are off the ground.

**Terminal 1, once:** update and build.

```bash
cd ~/racerbot-ws && git pull
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select gap_follow
```

**Working when:** colcon ends with `Summary: 1 package finished`.

**Terminal 1:** the hardware. Leave it running.

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

**Working when:** the log settles with no `[ERROR]` lines. `ros2 run racerbot_doctor doctor` in another terminal names anything that is not up.

**Terminal 2, step 1:** shipped time trial, capped at the speed the course was driven at on 2026-10-30.

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch gap_follow gap_follow_launch.py time_trial:=true max_speed:=3.5
```

**Working when:** the wheels turn only while LB is held. Then on the floor, it laps.

**Terminal 2, step 2:** the fastest settings above, full speed. Stop step 1 with Ctrl+C first.

```bash
ros2 run gap_follow gap_follow_node --ros-args \
  --params-file $(ros2 pkg prefix --share gap_follow)/config/gap_follow.yaml \
  -p aim_depth_bias:=1.0 -p enable_opponent_awareness:=false \
  -p max_lateral_accel:=3.25 -p max_steering_angle:=0.436
```

**Working when:** the `gap_follow` [node](glossary.md#node) (the running program) logs that it is waiting for LB, and the car laps when LB is held.

**If it doesn't:** a refused parameter stops the node at start with the reason. Check the spelling of each `-p`.

Every run is recorded by bringup's `auto_record`. Compare it with `laps.py` from [course-analysis.md](course-analysis.md): the simulator says 17.74 s.

## Simulator numbers elsewhere that are now out of date

- [gap-follow.md](gap-follow.md), "Speed on the real course", Results: the course lap times (21.3-22.1 s new default, 18.8-19.3 s time trial) and the friction 0.55/0.42 rows were measured on the plain calibration. On the floor profile the same settings lap in 20.71 and 18.10 s (standing start).
- [../tools/f1tenth_sim/README.md](../tools/f1tenth_sim/README.md): the speed loop and friction fitted to run `20261019-200703` still describe that floor; the floor profile overrides them for this one.
- [sim-fidelity-audit.md](sim-fidelity-audit.md): findings F2-F4 (no friction limit, no ellipse) describe the plain calibration and still hold there.

## Safety

The simulator is not the floor. Bring any setting from this page to the car in the order in [writing-your-own-node.md](writing-your-own-node.md#testing-before-its-on-wheels):

1. Wheels off the ground first.
2. Then the floor, at low speed.
3. The driver holds LB the whole time, ready to let go. LB is the deadman button on the F710 controller: releasing it stops every autonomy [node](glossary.md#node) (a program that drives the car), because each one refuses to drive without a live hold. That is what stops the car when the simulator was wrong.
