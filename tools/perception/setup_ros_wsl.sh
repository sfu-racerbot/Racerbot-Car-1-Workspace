#!/usr/bin/env bash
# Run as root in Ubuntu 24.04 (WSL or a disposable development machine).
# Installs development dependencies only; starts no car nodes or services.
set -euo pipefail
if [[ "$(id -u)" != 0 ]]; then
    echo 'Run with sudo, or wsl -d Ubuntu-24.04 -u root -- bash <this-script>.' >&2
    exit 1
fi
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 ]]; then
    echo 'This setup supports Ubuntu 24.04 / ROS 2 Jazzy only.' >&2
    exit 1
fi
export DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8
apt-get update -qq
apt-get install -y --no-install-recommends ca-certificates curl python3
if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'install ok installed'; then
    release=$(curl --retry 3 -fsSL \
        https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
    package=$(mktemp --suffix=.deb)
    trap 'rm -f -- "$package"' EXIT
    curl --retry 3 -fsSL -o "$package" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${release}/ros2-apt-source_${release}.noble_all.deb"
    dpkg -i "$package"
fi
apt-get update -qq
apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base ros-jazzy-cv-bridge ros-jazzy-tf2-ros \
    ros-jazzy-visualization-msgs ros-jazzy-rosbag2-transport \
    ros-jazzy-rosbag2-storage-mcap ros-jazzy-rviz2 \
    python3-colcon-common-extensions python3-rosdep python3-pytest python3-flake8 \
    python3-opencv python3-numpy python3-yaml python3-setuptools
echo 'ROS 2 Jazzy installed. Source /opt/ros/jazzy/setup.bash in ROS terminals.'
