#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${PROJECT_DIR}/inventory"
mkdir -p "${OUT}"

uname -a > "${OUT}/uname.txt"
lsusb > "${OUT}/lsusb.txt" || true
ip addr > "${OUT}/ip-addr.txt" || true
ip -details -statistics link show can0 > "${OUT}/can0-link.txt" || true
apt-mark showmanual > "${OUT}/apt-manual-packages.txt" || true
python -m pip freeze > "${OUT}/python-venv-ros-freeze.txt" || true

source /opt/ros/jazzy/setup.bash
if [[ -f "${PROJECT_DIR}/src/ros2_ws/install/setup.bash" ]]; then
  source "${PROJECT_DIR}/src/ros2_ws/install/setup.bash"
fi
ros2 node list > "${OUT}/ros2-nodes.txt" || true
ros2 topic list -t > "${OUT}/ros2-topics.txt" || true
apt list --installed 'ros-jazzy-*' > "${OUT}/ros-jazzy-packages.txt" 2>/dev/null || true

echo "Inventory written to ${OUT}"
