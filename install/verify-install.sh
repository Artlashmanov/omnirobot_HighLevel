#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${PROJECT_DIR}/tools/omni_env.sh"

source_ros
activate_venv

echo "== runtime config =="
echo "OMNI_HOME=${OMNI_HOME}"
echo "OMNI_ROS_WS=${OMNI_ROS_WS}"
echo "OMNI_BRIDGE_PARAMS=${OMNI_BRIDGE_PARAMS}"
echo "ROBOT_PLATFORM=${ROBOT_PLATFORM}"
echo "OMNI_PLATFORM_CONFIG=${OMNI_PLATFORM_CONFIG}"
echo "CAN_IFACE=${CAN_IFACE}"
echo "CAN_BITRATE=${CAN_BITRATE}"

echo "== systemd =="
systemctl is-enabled omni-can.service omni-bridge.service omni-mux.service teleop-web.service || true
systemctl is-active omni-can.service omni-bridge.service omni-mux.service teleop-web.service || true

echo "== service exec commands =="
systemctl show -p ExecStart omni-bridge.service omni-mux.service teleop-web.service --no-pager || true

echo "== CAN =="
ip -details -statistics link show "${CAN_IFACE}" || true

echo "== ROS2 nodes =="
timeout 8 ros2 node list || true

echo "== ROS2 topics =="
timeout 8 ros2 topic list -t || true

echo "== Platform profile =="
python - <<'PY' || true
from omni_pi.platforms import load_platform_profile
profile = load_platform_profile()
print(profile.as_public_dict())
PY

echo "== Bridge executable =="
ros2 pkg executables omni_bridge || true

echo "== Base telemetry topics =="
timeout 8 ros2 topic echo --once /omni/base_status || true
timeout 8 ros2 topic echo --once /omni/wheel_states || true

echo "== Base CAN sample =="
timeout 3 candump "${CAN_IFACE},190:7FF,191:7FF" || true
