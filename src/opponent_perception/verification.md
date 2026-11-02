# Offline verification, 2026-11-01

Environment: Windows, CPython 3.12.14, OpenCV contrib headless 4.14.0.94,
NumPy 2.5.4. Exact offline dependency snapshot: `requirements-offline-lock.txt`.

Runner from the workspace root:

```powershell
.venv-perception/Scripts/python.exe -m pytest src/opponent_perception/test -m 'not ros' -q
```

87 collected cases passed. No ROS imports, mocked detectors, skipped cases or
ignored collection errors. Real OpenCV detection and PnP run in this suite.
Compilation of the ROS adapter and launch is a syntax check only.

## Deliberate break evidence

Each mutation was applied alone to an isolated copy of this package under the
ignored `.venv-perception/mutation-audit/` directory, with the matching copied
test suite importing that copy. Original working source was never mutated.
Each row exited with failing assertions/runtime tests, not collection errors.
The complete unmodified suite was rerun after the audit.

| Mutation | Cases run | Failed |
|---|---:|---:|
| Geometry function bodies raise `NotImplementedError` | 14 | 14 |
| Detector function bodies raise `NotImplementedError` | 17 | 17 |
| Tracking function bodies raise `NotImplementedError` | 20 | 20 |
| Config/parser function bodies raise `NotImplementedError` | 33 | 33 |
| Replay function bodies raise `NotImplementedError` | 3 | 3 |
| Freshness boundary `<=` changed to `<` | 20 | 1 |
| Opponent Y velocity sign reversed | 20 | 1 |
| Uncertainty-growth constant increased by 10% | 20 | 1 |
| Camera/tag transform multiplication order swapped | 14 | 3 |
| Pose composition returns camera measurement unchanged | 14 | 4 |
| Kinematic speed gate removed | 20 | 1 |
| Physical tag size increased by 10% inside geometry | 17 | 4 |
| Calibration attestation bypassed | 33 | 2 |

These cover the repository's stub requirement for every new collected case
and numeric mutations M1–M6. They are evidence for these cases, not a claim
that all possible implementation faults are caught. Audit JSON and pytest
failure details are task-local; this table records the durable results.

## Synthetic image experiment

```powershell
.venv-perception/Scripts/python.exe -m opponent_perception.offline demo --output .venv-perception/demo-refined
```

Eleven generated images, eleven detections. With the included moving-camera
geometry and subpixel corner refinement:

* Position RMSE versus stationary-car truth: **0.0366 m**.
* Estimated speed RMSE for the stationary car: **0.2800 m/s**.

These are deterministic synthetic rasterization results, not physical-car
measurements. The speed error shows why differentiation of small, noisy tags
needs real-data tuning; this baseline is not yet adequate evidence for driving.
The unit test's 0.10 m / 0.5 m/s acceptance limits are coarse smoke-test
envelopes, not claimed real-world accuracy or controller tolerances.

## Initially pending verification (superseded below)

ROS 2 Jazzy launch, `cv_bridge`, TF timing, image/info ordering, clock resets,
RViz and rosbag behavior have not been run here. Docker's Linux engine was
unavailable. The Ubuntu/WSL installation was approved but Windows reported a
required reboot. No reboot was performed and no ROS runtime result is claimed.
Actual camera/tag calibration and moving-car recordings remain required.

## ROS verification, 2026-11-02

After the Windows reboot, Ubuntu 24.04 installed successfully under WSL 2.
Jazzy ROS base, cv_bridge, TF, visualization messages, RViz, rosbag/MCAP and
colcon were installed from Ubuntu/the official ROS apt repository using
`tools/perception/setup_ros_wsl.sh`. No camera or driving stack was launched.
`rosdep` was initialized and its Jazzy index updated. The dependency check
now reports all system dependencies satisfied; the package manifest declares
`python3-setuptools` as its actual Python build tool (the initial
`ament_python` entry was not a resolvable rosdep package key).

Runtime versions: system Python 3.12.3, NumPy 1.26.4, OpenCV 4.6.0,
`ros-jazzy-rclpy` 7.1.12 and `ros-jazzy-cv-bridge` 4.1.0.

Linux testing found a real compatibility bug: OpenCV 4.6 exposed
`DetectorParameters()` but setting a property on that binding segfaulted.
The existing real-detector test caught it. The implementation now prefers
`DetectorParameters_create()` where present, retaining the new constructor
for newer OpenCV. The 87 offline cases pass on both Windows 4.14 and Linux 4.6.

Eleven new ROS cases use real pub/sub, cv_bridge, TF and `/clock`, with all
outputs remapped to `/test_only/perception/*`. Missing ROS is not skipped;
offline users select `-m 'not ros'` (87 passed, 11 explicitly deselected).
All ROS runs require domain 79 and LOCALHOST discovery. Cases cover:

* Image-first, CameraInfo-first and zero-stamped static calibration.
* Metric world pose from a real rendered tag and timestamped dynamic TF.
* Missing TF and an older TF that must not substitute for acquisition-time TF.
* Mismatched calibration frames and stale acquisition stamps.
* Stale/expired track publications and clearing expired visualization markers.
* A backward ROS clock jump, clearing the old replay epoch.
* Ego motion compensation, velocity and predicted marker paths.
* A real processing subprocess reaching readiness and shutting down on SIGINT.

The process check exposed a shutdown race while rclpy lazily constructed its
global executor. The entry point now initializes its own executor before
announcing readiness, handles ROS shutdown exceptions, and cleans up explicitly.
The regression test first failed, then passed without loosening its assertions.

### ROS mutation evidence

Mutants ran in isolated package copies under
`/root/racerbot-perception-validation/ros-mutation-audit/`; original working
source was untouched. No row had collection/setup errors.

| Mutation | Cases run | Failed |
|---|---:|---:|
| Image processing returns immediately | 10 | 10 |
| Entry point returns immediately | 1 | 1 |
| Lookup latest TF instead of acquisition-time TF | 1 | 1 |
| Bypass camera frame agreement | 1 | 1 |
| Bypass image age rejection | 1 | 1 |
| Publish ADD instead of DELETEALL for clearing markers | 1 | 1 |
| Stop handling KeyboardInterrupt on shutdown | 1 | 1 |

### Build, launch and recording

Repeat from PowerShell at the workspace root:

```powershell
wsl -d Ubuntu-24.04 -u root -- bash tools/perception/check_ros_wsl.sh
```

The helper runs a symlink colcon build, colcon test and verbose test results
for this package only. Generated directories live in Linux at
`/root/racerbot-perception-validation/`. Result: **98 tests, 0 errors,
0 failures, 0 skipped**. Core flake8 E/F/W checks use the same E226/W503
defaults as Windows; optional ROS import-order/docstring plugins are outside
that check. No assertion or test tolerance was weakened.

The installed launch with a synthetic config reached readiness, stayed alive,
and exited cleanly after SIGINT. A simultaneous MCAP recorder captured
51 `/perception/opponents` and 51 `/perception/status` messages (102 total),
read back by `ros2 bag info`. No images were supplied in this launch smoke
test, so its track arrays were empty; image-to-track behavior is covered by
the real ROS tests above. Smoke logs and bag are under the same Linux directory.

The Linux synthetic-image demo detected all eleven frames, with position RMSE
0.0370 m and stationary speed RMSE 0.2797 m/s. RViz is installed; visual GUI
inspection has not been performed. No real camera calibration, physical-car
measurements, learned detector, LiDAR/depth fusion or driving integration is
claimed by this verification.
