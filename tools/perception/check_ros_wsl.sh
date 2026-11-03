#!/usr/bin/env bash
# Build and test only this support package in an isolated local ROS graph.
set -eo pipefail
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
validation_dir="${RACERBOT_PERCEPTION_VALIDATION:-$HOME/racerbot-perception-validation}"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
cd "$workspace"
colcon --log-base "$validation_dir/log" build \
    --base-paths src/opponent_perception \
    --build-base "$validation_dir/build" --install-base "$validation_dir/install" \
    --symlink-install --packages-select opponent_perception
source "$validation_dir/install/setup.bash"
colcon --log-base "$validation_dir/log" test \
    --build-base "$validation_dir/build" --install-base "$validation_dir/install" \
    --packages-select opponent_perception --return-code-on-test-failure \
    --event-handlers console_direct+
colcon test-result --test-result-base "$validation_dir/build" --verbose
# Same core checks as default flake8, independent of optional ROS lint plugins.
python3 -m flake8 src/opponent_perception --select E,F,W --extend-ignore E226,W503 \
    --exclude='*.egg-info,__pycache__,.pytest_cache' --max-line-length=99
