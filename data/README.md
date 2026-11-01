# data/ — recorded runs

Recorded runs are kept in git so they survive the Jetson's disk and can be
looked at from any clone. **Each car writes into `data/<car>/`: `data/rb1/`
for car 1, `data/rb2/` for car 2.** The workspace defaults to `~/racerbot-ws`;
set `RACERBOT_WS` to use another workspace. The car is resolved by
`racerbot_car` from its serial or `RACERBOT_CAR` (see
[car identity](../src/racerbot_car/README.md)). An unresolved car is an error.

Car 1's existing data has moved into `rb1/`. Before 2026-10-31, the tools
wrote to `~/.ros/`, which git never sees; older runs are still there.

| Folder | Written by | What's in it |
|---|---|---|
| `<car>/racerbot_runs/<timestamp>/` | bringup's automatic recorder (`auto_record`, every algorithm run, off with `record:=false`), `ros2 launch race_diagnostics record_run.py`, and `map_course` / `auto_map_race_launch.py` with diagnostics on | rosbag (`bag/`), `launch.log`, `metadata/` (git state, node parameters). See [run diagnostics](../docs/run-diagnostics.md) |
| `<car>/racerbot_runs/map_course-<timestamp>.log` | `ros2 run racerbot_launch map_course` | that session's terminal output |
| `rb1/racerbot_runs/20261026-*/live_map/` | `tools/run_analysis/extract_map.py`, by hand (2026-10-26) | the last live SLAM map of each 2026-10-26 mapping run, which saved none; which ones are good: `rb1/racerbot_runs/LIVE_MAPS_2026-10-26.txt` |
| `rb1/racerbot_auto/20261026-194514-clean/` | `tools/run_analysis/clean_map.py` | the cleaned map that became the simulator's car 1 course |
| `<car>/imu_calibration/<timestamp>_<step>/` | `tools/imu_calibration/tools/calibrate_imu.py` / `verify_calibration.py` | raw IMU recordings behind `tools/imu_calibration/calib/*.json` (car 1's BNO085, 2026-10-23) |
| `<car>/racerbot_auto/<timestamp>/` | `auto_map_race_node` (via `map_course` or `auto_map_race_launch.py`) on the real car | map, pose graph, racing lines, `commands.txt` |
| `<car>/racerbot_sim/auto/<timestamp>/` | `sim_auto_map_race_launch.py` | the same output from simulator runs |
| `<car>/racerbot_sim/races/<timestamp>/` | `sim_race_launch.py`'s race referee | `race.json`: standings, finish times, contacts |

Folder names are the run's start time, `YYYYMMDD-HHMMSS`. Any tool still takes
`output_directory:=<somewhere else>` if you want a run kept out of git.

## Saving runs to GitHub

After a session on the car, with every recorder stopped:

```bash
# any terminal, on the car
cd ~/racerbot-ws
tools/save_runs.sh            # commits this car's folder and README changes, then pushes
```

It worked if it prints a commit like `data: runs saved 2026-10-25` followed by
git's push summary. It commits nothing, and says why, if a recorder is still
running or a file is too big for GitHub (below). `--no-push` commits without
pushing.

Before committing it lists any new run where the car never moved, and any stub
another recorder replaced at once. They are committed unless you pass
`--skip-idle`, which leaves them uncommitted on disk. Nothing is deleted.
`<car>/racerbot_runs/auto_record.log` is the recorder's log of what it decided,
and is committed too. On a development machine, set `RACERBOT_CAR=rb1` or
`RACERBOT_CAR=rb2` before running the script; it refuses an unresolved car.

## Things to know

- **The repo is public.** Everything here is visible to anyone. Run metadata
  includes the full uncommitted `git diff` of the workspace at run time, so
  don't record a run while a secret is sitting in an uncommitted file.
- **GitHub rejects any single file over 100 MB.** `record_run.py` therefore
  splits each bag into 50 MB files (`bag_0.mcap`, `bag_1.mcap`, ...);
  `ros2 bag play <run>/bag` and the analysis tools read them as one bag.
  The largest non-bag file so far is a 30 MB pose graph. `tools/save_runs.sh`
  checks every file before committing.
- **Car 1's dashboard cannot list or delete runs in `data/`.** Delete a run
  with `git rm -r` followed by its path, then commit. Car 2's dashboard has
  different delete-panel settings; the per-car move does not change them.
- Every file added here stays in git history for good, and every clone
  downloads it, even if it is later deleted. A simulator run is around 30 MB,
  mostly its pose graph.
- `COLCON_IGNORE` stops `colcon build` from searching this folder for packages.
