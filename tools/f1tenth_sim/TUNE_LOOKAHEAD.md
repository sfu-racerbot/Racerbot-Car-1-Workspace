# Pure pursuit lookahead tuner (`tune_lookahead.py`)

One command that tries several pure pursuit lookahead settings on a **saved map and racing line**,
throws out every setting that is unsafe, and recommends the fastest safe one.

It runs entirely in the headless simulator on your PC. It never talks to a car, never publishes a
ROS topic, and never edits a config file the car loads. The recommendation is written to a separate
YAML that you copy by hand after testing it on the car.

## Why it exists

On `indoor_oval` (2026-11-06) the racing line itself was clear of the walls, but pure pursuit drove
up to about 0.20 m wide of it on a corner exit at about 3.6 m/s and touched the wall. A racing line
being safe is not enough; the car has to *follow* it safely. So this tool judges the path the car
actually drove, with the car's full body outline, not the planned line.

## What you need

- This workspace, and the headless simulator image (`docker/sim`, the `sim` service). Everything
  below runs inside it.
- A saved map (`map.yaml` + its image) and a racing-line CSV (`x,y[,speed]`) made from that map.
  `auto_map_race` writes both into every mapping run folder, for example
  `data/rb1/racerbot_sim/sessions/<session>/mapping_<n>/<timestamp>/map.yaml` and
  `.../raceline_profiled.csv`.

## Run it

From the workspace root (`~/racerbot-ws`):

```bash
RUN=data/rb1/racerbot_sim/sessions/20261106-194923/mapping_7/20261106-195353   # a saved run folder

docker compose -f docker/sim/compose.yaml run --rm -T sim \
  python3 tools/f1tenth_sim/tune_lookahead.py \
    --map  $RUN/map.yaml \
    --line $RUN/raceline_profiled.csv \
    --out  .sim/tune/indoor_oval \
    --workers 14
```

That runs 5 lookahead values x 2 speeds x 3 seeds = 30 simulated runs (about 1-2 minutes with 14
workers), re-runs the winner to check it reproduces, prints a summary, and writes the results.
`.sim/` is git-ignored.

### Options

| Option | Default | Meaning |
|---|---|---|
| `--map` | (required) | Saved occupancy map YAML; its image must sit beside it |
| `--line` | (required) | Racing-line CSV. If it is missing or unreadable the tool **stops**; it never falls back to another line |
| `--out` | (required) | Output folder |
| `--min-lookahead` | `0.4 0.5 0.6 0.7 0.8` | Candidate `min_lookahead` values (m) |
| `--seeds` | `0 1 2` | Seeds; every candidate uses the same ones |
| `--cap-speed` | `1.5` | Baseline speed cap (m/s) |
| `--stress-speed` | `auto_map_race.yaml`'s `profile_max_speed` (4.0) | Second, harder speed |
| `--clearance-threshold` | `0.10` | Minimum body-to-wall gap (m) a run must keep |
| `--car` | `rb1` | Which car's calibration the simulator uses |
| `--workers` | `8` | Parallel runs |
| `--no-recheck` | off | Skip re-running the winner |

`max_lookahead` (1.5) and `lookahead_speed_gain` (0.15) are fixed. The lookahead formula is
`racing_math.adaptive_lookahead`: `L = clip(0.15 * speed + min_lookahead, min_lookahead, 1.5)`,
using the car's measured speed. The tool prints the resulting lookahead for every candidate at
each speed, and flags a speed where two candidates would give the same value.

## What one run is

1. The car starts on the line's first point, nudged by the seed (up to +/-5 cm and +/-3 deg).
2. It drives a **warm-up lap** from standstill, then a **timed flying lap**.
3. The run counts as completed only if the flying lap finishes within 3 x (line length / speed cap).

Each seed also drives the simulator's existing noise: pose noise, odometry speed noise, LiDAR
noise and dropouts. A 3-tick steering delay is always on. The same seed always gives exactly the
same result, so you can repeat a sweep and compare.

## How a setting passes, and how the winner is picked

A setting passes at a speed only if **every** seed:

1. completes the flying lap,
2. never triggers the simulator's own body-contact check, and
3. keeps at least `--clearance-threshold` between the car's body and every wall cell.

One failing seed fails the setting. Among the passing settings:

1. Settings that pass at the stress speed win over everything else. If none do, the 1.5 m/s
   results are ranked instead, and the summary says so.
2. Fastest worst-case flying lap wins; laps within 1% of each other count as a tie.
3. Ties go to the lower worst-case cross-track error, then the calmer steering.

There is no combined score, so a setting that crashed can never be picked. If nothing passes, the
tool prints **NO SAFE CONFIGURATION FOUND**, still writes all the results, writes no YAML, and
exits with code 2.

## What it writes (in `--out`)

| File | Contents |
|---|---|
| `lookahead_results.csv` | One row per run (setting x speed x seed) |
| `lookahead_results.json` | Every run, the per-setting summary with pass/fail and the reason, the ranking, the winner (or `null`), the winner re-run check, and where everything came from: map and line paths with their SHA-256, git commit and whether the tree had uncommitted changes, all controller and simulator settings, seeds, speeds, threshold, time |
| `recommended_lookahead.yaml` | Only when something passes. Same structure as `src/pure_pursuit/config/pure_pursuit.yaml` (`pure_pursuit_node: ros__parameters: ...`), with the provenance as comments |

The YAML is a suggestion, not a setting. To use it: test it in the full ROS simulator, then on the
car (wheels off the ground, then slow on the floor), then copy the three values into the car's
config through the normal car/simulator branch workflow.

## How clearance is measured

- The car is the **measured body**: 0.535 m long, 0.30 m wide, centred 0.18 m ahead of the rear
  axle (the point the racing line describes). These come from the simulator calibration and are
  recorded in the JSON.
- Clearance is the **exact** distance between that rotated rectangle and every occupied map cell
  within 0.5 m (zero if they overlap), checked on every simulator tick. It is not the distance from
  the rear axle and not the LiDAR's shortest beam.
- Map cells are read the way the simulator reads them: a pixel of 128 or darker is a wall, anything
  lighter is free. That means a SLAM map's grey "unknown" area counts as **free**.

## Example results (2026-11-06)

- `indoor_oval`, line `mapping_7`: winner `min_lookahead 0.8`, at the 1.5 m/s cap only. At 4.0 m/s
  the best settings missed the 0.10 m threshold by 1-3 mm on one seed.
- `indoor_tight`, line `mapping_82`: **NO SAFE CONFIGURATION FOUND**. The planned line itself only
  leaves 0.089 m of body clearance on that map, so no lookahead can reach 0.10 m without luck.

## Tests

```bash
docker compose -f docker/sim/compose.yaml run --rm -T sim \
  python3 -m pytest tools/f1tenth_sim/tests/test_tune_lookahead.py -q
```

They cover: a crashing or too-close seed can never win, the no-winner path writes no YAML, the 1%
tie rule, the line file is the one recorded (and a missing or broken file stops the run), and the
clearance geometry (a body corner over a wall cell reports zero even though the rear axle is clear).

## Limits

- Results are deterministic per seed; three seeds is screening, not proof of reliability.
- Simulator only. The standalone pure pursuit controller here mirrors the ROS node but is not the
  node itself.
- A SLAM map's walls are drawn slightly thicker than the real ones (about 5 cm on `indoor_oval`), so
  clearance on the saved map is a little pessimistic.
- Where the body sits along the car (front versus rear overhang) has not been measured on the car;
  the body is assumed centred between the axles.
