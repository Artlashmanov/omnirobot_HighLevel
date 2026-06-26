#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
cd "${PROJECT_DIR}/src/ros2_ws"

if [[ -f omni.repos && ! -d src/sllidar_ros2 ]]; then
  vcs import src < omni.repos
fi

colcon build --symlink-install
