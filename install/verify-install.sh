#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "${PROJECT_DIR}/src/ros2_ws/install/setup.bash" ]]; then
  source "${PROJECT_DIR}/src/ros2_ws/install/setup.bash"
fi

echo "== systemd =="
systemctl is-enabled omni-can.service omni-bridge.service omni-mux.service teleop-web.service || true
systemctl is-active omni-can.service omni-bridge.service omni-mux.service teleop-web.service || true

echo "== CAN =="
ip -details -statistics link show can0 || true

echo "== ROS2 nodes =="
timeout 8 ros2 node list || true

echo "== ROS2 topics =="
timeout 8 ros2 topic list -t || true

echo "== Base telemetry sample =="
timeout 3 candump can0,190:7FF,191:7FF || true
