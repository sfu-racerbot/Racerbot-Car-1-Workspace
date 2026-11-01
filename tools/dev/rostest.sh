#!/bin/bash
# Run pytest the way CLAUDE.md requires: ROS sourced, on an isolated ROS domain, with
# this checkout's own Python sources ahead of the installed ones.
#
#   tools/dev/rostest.sh src/gap_follow/test/ -q
#   tools/dev/rostest.sh tools/f1tenth_sim/tests/ tools/racerbot_sim/tests/ tools/run_analysis/tests/ -q
#
# Works from a git worktree too: the worktree has no install/, so it sources the main
# checkout's install and puts the worktree's src/<package>/ first on PYTHONPATH, so the
# code under test is the code in front of you, not what was last built.
set -e
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
DOMAIN=${ROSTEST_DOMAIN:-79}
if [ "$DOMAIN" = "0" ]; then
    echo "rostest.sh: refusing ROS domain 0, the car's own graph" >&2
    exit 2
fi
INSTALL=$ROOT/install
[ -f "$INSTALL/setup.bash" ] || INSTALL=$(git -C "$ROOT" worktree list --porcelain | awk 'NR==1{print $2}')/install
set +u
source /opt/ros/jazzy/setup.bash
[ -f "$INSTALL/setup.bash" ] && source "$INSTALL/setup.bash"
export ROS_DOMAIN_ID=$DOMAIN ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
for package in "$ROOT"/src/*/; do
    name=$(basename "$package")
    [ -f "$package/setup.py" ] && [ -d "$package/$name" ] && PYTHONPATH="$package:$PYTHONPATH"
done
export PYTHONPATH
cd "$ROOT"
exec python3 -m pytest -p no:cacheprovider "$@"
