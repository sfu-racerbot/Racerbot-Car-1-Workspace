# Automatic apex line editing

> **Who this is for:** drivers selecting corner points on a saved map.
> **Read first:** [racing autonomy](../../docs/racing-autonomy.md).
> **You will be able to:** build the editor and queue checked lines from the dashboard.
> **Time:** about 15 minutes, plus simulation and car testing.

## Highlights

- Ships inside the `pure_pursuit` ROS package, including its configuration and clearance checks.
- Checks the full chassis against walls and unknown map cells before exporting.
- Writes a new run folder and preserves the original CSV, map and start pose.
- Queues a prepared line until the next LB press while stopped.

The editor converts selected apexes (desired corner points) into a racing line with speeds. Geometry checks cannot prove how accurately the moving car will track it; simulation and staged car testing still matter.

## Build

In a terminal at the workspace root, install the package dependencies and rebuild both changed packages:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select pure_pursuit web_dashboard
source install/setup.bash
ros2 run pure_pursuit apex_line_editor --help
```

Success displays the editor's `--run`, `--apexes` and `--out` options. The dashboard uses this installed Python module automatically; no `tools/f1tenth_sim` files are needed. The matching dashboard PR must also be installed. Restart the controller and dashboard after building.

## Use the dashboard

1. Start pure pursuit on a saved run using the same map shown in the dashboard. The saved run needs `raceline_profiled.csv`, `map.yaml`, its map image, and a valid `start_pose.yaml`.
2. Stop and release LB. Arm tuning in the dashboard.
3. Enable Manual Apex Mode and place corner points. The editor prepares and checks a new line automatically.
4. Wait for the validated-and-queued message. A refusal leaves the current line active; read the displayed reason before trying again.
5. Press LB while stopped to activate the prepared line before driving. Fresh joystick and odometry messages are required; measured speed must be at most 0.05 m/s.

The source run must be inside the dashboard's configured `map_roots`. Undo and clear cancel the previous pending request. Clearing markers does not restore a line that was already activated.

## Optional offline use

In a sourced terminal, use a dashboard JSON export directly:

```bash
ros2 run pure_pursuit apex_line_editor --run /path/to/run --apexes /path/to/apexes.json
```

Success prints a launch command for the new folder. A fingerprint mismatch means the markers belong to a different racing line. The editor refuses unsafe or infeasible points instead of exporting a line.

Validated in simulation only. On the physical car, use the normal test order: static checks, wheels off the ground with LB held, then low-speed floor testing in open space. Keep the LB deadman enabled.
