# Opponent perception

Offline-first, tagged-car perception following F1TENTH Lecture 17. This is
support code: it observes images and publishes `/perception/*` only. No
controller consumes it yet. There is no driving behavior change.

## Implemented and pending

Implemented: OpenCV AprilTag 36h11 detection, calibrated planar PnP, tag-mount
removal, timestamped camera-to-fixed-frame conversion, per-vehicle identity
tracking, smoothed velocity, short constant-velocity predictions, observation
aging, synthetic image generation, image-manifest replay, and a ROS 2 adapter.

Offline math and real OpenCV detection are tested on Windows without ROS.
Ubuntu 24.04 / ROS 2 Jazzy are also installed in WSL 2. The package builds
with colcon, and the real ROS adapter tests cover messages, timestamped TF,
CameraInfo arrival order, staleness, marker expiry and clock rewinds. Installed
launch, clean shutdown and an MCAP recording smoke test also pass. These are
synthetic local tests; no actual camera or car has been validated.

Pending: physical calibration, real-image measurements, on-car graph validation,
LiDAR/depth association, untagged-car recognition, track-relative prediction,
and optional controller integration. This version does not fuse LiDAR or raw
IMU measurements. IMU affects it only through the selected timestamped TF
motion estimate, if that TF chain actually uses gyro odometry.

## Offline quick start

Windows, from the workspace root (`uv` must already be available):

```powershell
./tools/perception/setup_offline.ps1
.venv-perception/Scripts/python.exe -m opponent_perception.offline demo --output .venv-perception/demo
Get-Content .venv-perception/demo/summary.json
```

The output directory must not exist. This generates eleven actual rasterized
AprilTag images, a calibrated **synthetic** pinhole camera, and camera poses
that both translate and rotate past a stationary opponent at `(3, 0)` metres.
It runs the same detector/tracker used by the ROS adapter and writes
`tracks.jsonl` and error metrics against the renderer's independent truth.
All sample measurements are invented geometry, never car calibration. At
about 40 pixels per tag, pixel quantization causes appreciable speed jitter;
detecting every frame does not demonstrate readiness for overtaking.

Linux/macOS offline (no ROS required):

```bash
python3 -m venv .venv-perception
.venv-perception/bin/python -m pip install -r src/opponent_perception/requirements-offline.txt
.venv-perception/bin/python -m pip install --no-deps -e src/opponent_perception
.venv-perception/bin/python -m pytest src/opponent_perception/test -m 'not ros' -q
.venv-perception/bin/python -m opponent_perception.offline demo --output .venv-perception/demo
```

Use only one OpenCV package in a Python environment: the offline environment
uses `opencv-contrib-python-headless`; ROS uses Ubuntu's `python3-opencv` and
`cv_bridge`. Do not pip-install these wheels into the car's ROS Python.

## Geometry and calibration contract

`T_parent_child` is a proper finite 4x4 SE(3) transform mapping child points
into the parent frame. Reflections, scale, non-unit quaternions and non-finite
inputs are refused. All lengths are metres, times seconds, yaw radians.

* OpenCV optical frame: x right, y down, z forward.
* Tag frame: origin at the black square's centre, x right, y up, z out of its
  printed front face. Known-size PnP supplies metric scale without depth.
* `size_m` measures the outer black square, excluding the white quiet margin.
* `opponent_from_tag` is the measured tag pose in that car's rear-axle frame.
* `tracking_from_camera` is the camera optical frame's pose in a fixed frame
  **at image acquisition time**, not receipt time or the latest available TF.

The composition is:

```text
T_fixed_opponent = T_fixed_camera * T_camera_tag * inverse(T_opponent_tag)
```

Velocity is computed only after transforming into the fixed frame. Our own
motion must not appear as opponent motion. The default is `odom`; select
`odom_imu` only if the existing IMU/TF launch establishes that frame correctly.
`/odom/fused` changes speed only: it is not a fused heading/pose estimate.
Map localization jumps can corrupt velocities; prefer a continuous local
odometry frame. Start a new tracker after a coordinate-frame reset.

Copy `config/opponent_perception.yaml` and fill the real tag mount/size and
tracking frame. Set `calibrated: true` only after checking camera intrinsics,
camera mount, and opponent tag mount. The flag is human attestation, not an
automatic calibration check. The shipped template is intentionally refused;
even setting its flag true leaves an invalid null mount. Camera intrinsics
come from `CameraInfo` live or `camera.yaml` offline, not guessed focal lengths.

Version 1 allows one configured tag per vehicle, up to 32 vehicles. If two
copies of the same ID appear in one image, neither is accepted. Unknown IDs
are ignored. Multiple tags per car require a future measurement-association
and consistency policy, rather than silently treating them as two cars.

Planar PnP can have ambiguous orientation; low reprojection error is not a
probability of correct pose. Heading is the last measured chassis yaw, not an
angular-motion estimate. Calibrate/test orientation across oblique views
before using it. This is OpenCV's AprilTag dictionary detector, not a wrapper
around the AprilRobotics detector or its decision-margin metadata.

## Offline replay format

Camera YAML has exactly these fields (example values are synthetic):

```yaml
width: 640
height: 480
matrix: [[600, 0, 320], [0, 600, 240], [0, 0, 1]]
distortion: [0, 0, 0, 0, 0]
```

The frame manifest is UTF-8 JSON Lines. Every line has exactly `stamp_sec`,
`image` (relative to the manifest or absolute), and `tracking_from_camera`
(nested 4x4 matrix). Stamps must be nonnegative and strictly increasing,
including frames with no detections. Input records are limited to 64 KiB;
configuration is limited to 64 KiB. Unknown keys, invalid UTF-8/JSON/YAML,
non-finite constants and unreadable or wrong-sized images fail explicitly.
Do not change the camera resolution without changing the intrinsics.

```powershell
.venv-perception/Scripts/python.exe -m opponent_perception.offline replay `
  --config measured-tags.yaml --camera camera.yaml `
  --manifest frames.jsonl --output new-tracks.jsonl
```

Replay validates/processes the manifest before writing output, refuses existing
output files, and does not publish ROS topics. It currently retains all result
records in memory: split long sessions into manageable manifests. A rosbag
extractor is not implemented; use the generated example to see the file format.

## ROS setup and launch when the car is ready

This PC now has Ubuntu-24.04 under WSL 2, with Jazzy, `cv_bridge`, TF, RViz,
rosbag/MCAP, system OpenCV 4.6 and the colcon tools installed. To repeat the
package build and all tests from PowerShell at the workspace root:

```powershell
wsl -d Ubuntu-24.04 -u root -- bash tools/perception/check_ros_wsl.sh
```

The helper isolates ROS on domain 79 with LOCALHOST discovery and stores build,
install and test logs inside Linux at `~/racerbot-perception-validation`, not
in the Windows checkout's generated directories. A fresh Ubuntu 24.04 setup
can install the same prerequisites using `tools/perception/setup_ros_wsl.sh`
as root. That script uses the official ROS apt-source package; it does not
install Python wheels into system ROS Python.

On Ubuntu 24.04 with ROS 2 Jazzy installed, from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/opponent_perception --ignore-src -r -y
colcon build --symlink-install --packages-select opponent_perception
source install/setup.bash
ros2 launch opponent_perception opponent_perception_launch.py \
  config_file:=/absolute/path/to/measured-tags.yaml
```

This launch starts processing only. Start the selected camera and motion/TF
stack separately. It starts no drivers, static transforms, joystick, or
controller. rb2 defaults are raw colour plus its `CameraInfo`. For rb1, first
provide a single ROS webcam owner that also supplies the dashboard; its current
dashboard-only capture does not publish images.

The ROS adapter uses `cv_bridge` mono8, sensor QoS, newest pending image only,
and a bounded eight-message CameraInfo cache. It requires matching image/info
frame IDs, dimensions, and acquisition stamps (zero-stamped static CameraInfo
is also allowed). Supported distortion models: plumb_bob/rational_polynomial.
Cropped/binned images are refused instead of silently using wrong intrinsics.
Use raw images with raw K/D; rectified images need rectified intrinsics and
zero distortion. Lookup TF at the image stamp without blocking the executor;
retry pending frames until their freshness deadline. Delayed/missing TF never
falls back to an identity transform. Backward ROS clock jumps clear tracks.

Inspect outputs:

```bash
ros2 topic echo /perception/opponents
ros2 topic echo /perception/status
# In RViz, set Fixed Frame to the configured tracking_frame and add
# MarkerArray on /perception/opponent_markers.
```

Before live acceptance, replay a recorded scene in an isolated ROS domain
(`ROS_DOMAIN_ID=79`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, simulated time
enabled with launch argument `use_sim_time:=true`). Check stationary, moving,
oblique, occluded,
unknown/duplicate-ID, stale-image and missing-TF cases. Compare tape-measured
positions and independently measured speed. Then test ego translation and
rotation around a stationary opponent; measure errors and latency rather than
only counting detections. Synthetic ROS tests do not replace these physical
measurements or real-image replay checks.

## Version 1 output contract

`/perception/opponents`: `std_msgs/msg/String` containing finite JSON. Like the
existing `/drive_intent` contract, JSON permits offline/ROS consumers without
generating custom message packages. A future typed interface must preserve
the time/frame/validity semantics and bump the version for incompatible changes.

Top level: `schema_version: 1`, `stamp_sec` (state evaluation/publication time),
`frame_id`, `source: apriltag_36h11`, and `tracks` (sorted by vehicle ID).

Each track has:

| Field | Meaning |
|---|---|
| `vehicle_id`, `tag_id` | Configured identity; stable across detections |
| `last_seen_sec`, `age_sec` | Original accepted acquisition stamp and age; never refreshed on a miss |
| `fresh` | Age <= `fresh_sec`; false does not mean free space |
| `hits` | Accepted observations since initialization/expiry |
| `position_m` | `[x,y]`, extrapolated to `stamp_sec` using estimated velocity if valid |
| `yaw_rad` | Last measured chassis yaw in `frame_id`; held during coasting |
| `velocity_valid`, `velocity_mps` | False/null until two accepted poses; then `[vx,vy]` in fixed frame |
| `uncertainty_m` | Uncalibrated heuristic indicator: base + age * growth; not covariance, confidence probability, or certified bound |
| `reprojection_error_px` | RMS pixel residual of the last accepted tag pose |
| `prediction` | Samples with `dt_sec` relative to `stamp_sec` and `position_m`; empty until velocity valid |

Defaults: fresh through 0.25 s inclusive, lost after 0.75 s (boundary retained),
speed gate 8 m/s, velocity EMA gain 0.5, prediction 1 s with 0.1 s steps.
The first velocity uses the first two poses directly. Faster-than-limit
displacements are rejected without refreshing last_seen. Reappearance after
expiry starts from unknown velocity. Empty snapshots are published when no
tracks remain; they mean no current observations, never a cleared obstacle map.

`/perception/opponent_markers`: `visualization_msgs/msg/MarkerArray`, position
spheres plus predicted polylines. Short lifetimes and explicit clearing remove
expired markers. `/perception/status`: JSON `std_msgs/msg/String` with status,
track count and latest processed image age; `ok` describes last processing,
so consumers must also examine image age and each track's `fresh` value.

## Recording and next stages

The automatic recorder does not start for perception-only sessions, and its
default list has no camera/perception topics. Start the existing manual
recorder with an explicit list (substitute the selected car's camera topics):

```bash
ros2 launch race_diagnostics record_run.py \
  topics:="/camera/camera/color/image_raw /camera/camera/color/camera_info /tf /tf_static /odom /odom/imu /odom/fused /scan /perception/opponents /perception/status /perception/opponent_markers"
```

This replaces that recording's default list; use a separate perception session
before any driving integration. It uses existing per-car run paths and 50 MB
bag splitting. Images are much larger than tracks; choose explicit sessions
and review storage before saving them into plain Git. Calibration and dependency
versions must accompany real datasets; copy measured tag config into the run's
metadata because the recorder does not know about this arbitrary config path.

Next: measure actual errors, add LiDAR/depth association and cross-checks,
then track-relative/path-based prediction. Only after validation, add a
controller input adapter. All existing raw-scan safety checks and the mandatory
LB deadman remain authoritative. No part of this package currently sends
commands to a controller.

## Verification and references

Run the 87-case offline suite with
`python -m pytest src/opponent_perception/test -m 'not ros' -q`.
The 11 ROS cases are explicitly deselected by that command; none are skipped.
For all 98 cases, source Jazzy, set domain 79 / LOCALHOST, and run
`python3 -m pytest src/opponent_perception/test -q` or use the WSL helper.
It exercises actual OpenCV and our real modules, not mocked vision. Tests name
closed-form, invariant or specification oracles. Mutation evidence is in
[verification.md](verification.md). Windows dependency versions are captured
in `requirements-offline-lock.txt`; these are not Jetson ROS dependencies.

* [Lecture 17](https://f1tenth-coursekit.readthedocs.io/en/stable/lectures/ModuleE/lecture17.html)
  and its slides 45, 122–130: tags, pose, tracking and prediction. The lecture's
  path-weighted prediction and ten-second display are not implemented here.
* [OpenCV marker detection](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
  and [PnP conventions](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html).
* [Workspace perception research](../../docs/realsense-lidar-perception-research-report.md).
