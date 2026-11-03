# Offline opponent perception

The new [opponent_perception package](../src/opponent_perception/README.md)
implements an offline-first AprilTag pose/tracking baseline. Its README is
the procedure and interface reference: local dependency setup, synthetic image
demo, manifest replay, calibration contracts, ROS setup, recording and the
versioned `/perception/opponents` output.

This is support code. Existing controllers do not subscribe to it. The current
milestone is to measure opponent pose and velocity while compensating for ego
motion; LiDAR/depth fusion and overtaking integration are later stages.

The 87-case offline suite runs without ROS or cars. Ubuntu/Jazzy are installed
in WSL 2; the full 98-case suite, package build, installed launch and MCAP
recording smoke test are validated locally. Physical camera calibration,
real-image accuracy and on-car behavior remain unverified. No real calibration
is guessed or installed. See the package guide for the repeatable WSL helper.
