# disparity_extender (package `disparity_trajectory`)

A reactive driving layer for the car: it reads the LiDAR and steers along the best of 21 short
arcs the car can actually steer, after checking that the car's whole body stays clear along
each one ("method D"). No map needed.

The folder is `src/disparity_extender/`; the ROS package inside is `disparity_trajectory`,
because team A's `src/racerbot_a` already owns the package name `disparity_extender` and colcon
refuses two packages with one name.

## Why

The classic disparity extender aims at the single furthest LiDAR return. That return can sit in
a slot too narrow for the car (a box beside a wall), and the car drives into it. Measured in the
simulator on `ra_compTestTrackObs`, 2026-11-06. Method D scores whole drivable arcs instead.
Simulator results: 0 wall contacts in 36 headless runs (17 for the furthest-ray extender), and
13 clean laps of `indoor_tight` in the full ROS 2 simulator. **Not yet driven on a car.**

## Topics

| Direction | Topic | Type |
|---|---|---|
| in | `/scan` | `sensor_msgs/LaserScan` (the car's Hokuyo) |
| in | `/joy` | `sensor_msgs/Joy` (the F710; **LB must be held** to drive) |
| out | `/drive` | `ackermann_msgs/AckermannDriveStamped`, to `ackermann_mux` (priority 10) |

## Safety

It publishes a non-zero command only while LB is held on a live `/joy` stream. Every other case
publishes an explicit 0.0 / 0.0 stop: no `/joy` yet, `/joy` older than `joy_timeout_sec`, LB
released or missing, a scan with too few real returns (all inf, zero or NaN: a dead LiDAR), no
arc clear for `trajectory_min_score_m`, the LiDAR silent for `scan_timeout_sec`, or any error.

## Run it on the car (in this order, never skip ahead)

```bash
ros2 launch f1tenth_stack bringup_launch.py                       # terminal 1
ros2 launch disparity_trajectory disparity_trajectory_launch.py   # terminal 2: hold LB to drive
```

Do not run `teleop_launch.py` at the same time (its `/teleop` always wins the mux).

1. **Static check**, driver stack off: `ros2 topic echo /drive` shows 0.0 / 0.0 with LB released.
2. **Wheels off the ground**, full stack, LB held: the wheels steer away from a hand held near
   the LiDAR, and stop when LB is released.
3. **Floor, low speed, open space.** `max_speed` is 2.5 m/s in the config; start lower by editing
   it or with `--ros-args -p max_speed:=1.0`.

The steering limit comes from each car's calibration overlay
(`src/racerbot_car/cars/<id>/calibration/disparity_trajectory.yaml`: 0.33 rad on car 1, 0.349 on
car 2), the same values `gap_follow` uses. Every other setting is in
`config/disparity_trajectory.yaml`.

## Tests

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 -m pytest src/disparity_extender/test/ -q
```

`test_trajectory_logic.py` (no ROS) checks the arc choice against closed-form scenes.
`test_disparity_trajectory_node.py` checks the four deadman states and every stop path by the
command actually published, always on `/test_only/drive`.

## Known limits

- It can stop (publish zero) at a tight hairpin when it finds no clear arc, rather than crashing.
- It has no time-to-collision brake or opponent tracking.
