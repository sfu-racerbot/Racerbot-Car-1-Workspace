# data/ — recorded runs

Car 1's recorded runs, kept in git so they survive the Jetson's disk and can
be looked at from any clone. **The car writes new runs straight into this
folder.** Each tool below defaults to a folder here (since 2026-10-31, ported from car 2;
before that they wrote to `~/.ros/`, which git never sees, and older runs are
still there).

| Folder | Written by | What's in it |
|---|---|---|
| `racerbot_runs/<timestamp>/` | bringup's automatic recorder (`auto_record`, every algorithm run, off with `record:=false`), `ros2 launch race_diagnostics record_run.py`, and `map_course` / `auto_map_race_launch.py` with diagnostics on | rosbag (`bag/`), `launch.log`, `metadata/` (git state, node parameters). See `docs/run-diagnostics.md` |
| `racerbot_runs/map_course-<timestamp>.log` | `ros2 run racerbot_launch map_course` | that session's terminal output |
| `racerbot_runs/20261026-*/live_map/` | `tools/run_analysis/extract_map.py`, by hand (2026-10-26) | the last live SLAM map of each 2026-10-26 mapping run, which saved none; which ones are good: `racerbot_runs/LIVE_MAPS_2026-10-26.txt` |
| `racerbot_auto/20261026-194514-clean/` | `tools/run_analysis/clean_map.py` | the cleaned map that became the simulator's car 1 course |
| `imu_calibration/<timestamp>_<step>/` | `tools/imu_calibration/tools/calibrate_imu.py` / `verify_calibration.py` | raw IMU recordings behind `tools/imu_calibration/calib/*.json` (car 1's BNO085, 2026-10-23) |
| `racerbot_auto/<timestamp>/` | `auto_map_race_node` (via `map_course` or `auto_map_race_launch.py`) on the real car | map, pose graph, racing lines, `commands.txt` |
| `racerbot_sim/auto/<timestamp>/` | `sim_auto_map_race_launch.py` | the same output from simulator runs |
| `racerbot_sim/races/<timestamp>/` | `sim_race_launch.py`'s race referee | `race.json`: standings, finish times, contacts |

Folder names are the run's start time, `YYYYMMDD-HHMMSS`. Any tool still takes
`output_directory:=<somewhere else>` if you want a run kept out of git.

## Saving runs to GitHub

After a session on the car, with every recorder stopped:

```bash
# any terminal, on the car
cd ~/racerbot-ws
tools/save_runs.sh            # commits data/ only, then pushes
```

It worked if it prints a commit like `data: runs saved 2026-10-25` followed by
git's push summary. It commits nothing, and says why, if a recorder is still
running or a file is too big for GitHub (below). `--no-push` commits without
pushing.

Before committing it lists any new run where the car never moved, and any stub
another recorder replaced at once. They are committed unless you pass
`--skip-idle`, which leaves them uncommitted on disk. Nothing is deleted.
`racerbot_runs/auto_record.log` is the recorder's log of what it decided, and is committed too.

## Things to know

- **The repo is public.** Everything here is visible to anyone. Run metadata
  includes the full uncommitted `git diff` of the workspace at run time, so
  don't record a run while a secret is sitting in an uncommitted file.
- **GitHub rejects any single file over 100 MB.** `record_run.py` therefore
  splits each bag into 50 MB files (`bag_0.mcap`, `bag_1.mcap`, ...);
  `ros2 bag play <run>/bag` and the analysis tools read them as one bag.
  The largest non-bag file so far is a 30 MB pose graph. `tools/save_runs.sh`
  checks every file before committing.
- **The dashboard's saved-runs panel lists the maps in `racerbot_auto/` and
  `racerbot_sim/auto/`, and can delete them.** A run deleted there shows in
  `git status` as a deletion, and the next `tools/save_runs.sh` commits that
  deletion; the files stay in git history. Recordings in `racerbot_runs/` are
  not maps and are not in that panel. To delete one, use
  `git rm -r data/racerbot_runs/<timestamp>` and commit.
- Every file added here stays in git history for good, and every clone
  downloads it, even if it is later deleted. A simulator run is around 30 MB,
  mostly its pose graph.
- `COLCON_IGNORE` stops `colcon build` from searching this folder for packages.
