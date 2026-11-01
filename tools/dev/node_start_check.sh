#!/bin/bash
# Does a node start with these arguments, and what does it say? Run it for a few seconds
# on an isolated ROS domain and print how it ended.
#
#   tools/dev/node_start_check.sh 6 ros2 run gap_follow gap_follow_node --ros-args \
#       --params-file src/gap_follow/config/gap_follow.yaml -p max_steering_angle:=0.436
#
# Use it before handing anyone a floor-test command: a refused parameter shows up here,
# not on the floor. The node runs on domain 79 with localhost discovery only, so it never
# joins the car's graph, and without LB (no /joy on that domain) a driving node commands
# nothing. Hardware drivers are refused: a domain does not separate a serial port.
set -e
if [ $# -lt 2 ]; then
    echo "usage: $0 <seconds> <command...>" >&2
    exit 2
fi
SECONDS_TO_RUN=$1; shift
case "$*" in
    *bringup*|*vesc*|*urg_node*|*joy_node*|*teleop*|*realsense*)
        echo "node_start_check.sh: refusing to start a hardware driver: $*" >&2
        exit 2 ;;
esac
if [ "${ROSTEST_DOMAIN:-79}" = "0" ]; then
    echo "node_start_check.sh: refusing ROS domain 0, the car's own graph" >&2
    exit 2
fi
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
INSTALL=$ROOT/install
[ -f "$INSTALL/setup.bash" ] || INSTALL=$(git -C "$ROOT" worktree list --porcelain | awk 'NR==1{print $2}')/install
set +u
source /opt/ros/jazzy/setup.bash
[ -f "$INSTALL/setup.bash" ] && source "$INSTALL/setup.bash"
export ROS_DOMAIN_ID=${ROSTEST_DOMAIN:-79} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
for package in "$ROOT"/src/*/; do
    name=$(basename "$package")
    [ -f "$package/setup.py" ] && [ -d "$package/$name" ] && PYTHONPATH="$package:$PYTHONPATH"
done
export PYTHONPATH
cd "$ROOT"
set +e
OUT=$(timeout -s INT "$SECONDS_TO_RUN" "$@" 2>&1)
CODE=$?
set -e
echo "$OUT" | tail -n 15
if [ $CODE -eq 124 ] || [ $CODE -eq 130 ]; then
    echo "node_start_check: still running after ${SECONDS_TO_RUN} s (started fine)"
    exit 0
fi
echo "node_start_check: exited by itself with code $CODE"
exit 1
