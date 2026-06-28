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
echo "OMNI_FETCH_ROS_DEPS=${OMNI_FETCH_ROS_DEPS}"
echo "OMNI_ENABLE_LIDAR=${OMNI_ENABLE_LIDAR}"
echo "OMNI_ROS_REPOS_FILE=${OMNI_ROS_REPOS_FILE}"
echo "OMNI_ODOM_PARAMS=${OMNI_ODOM_PARAMS}"
echo "OMNI_ENABLE_TF_LUNA=${OMNI_ENABLE_TF_LUNA}"
echo "TF_LUNA_SERIAL_PORT=${TF_LUNA_SERIAL_PORT}"
echo "TF_LUNA_BAUDRATE=${TF_LUNA_BAUDRATE}"
echo "TF_LUNA_FRAME_ID=${TF_LUNA_FRAME_ID}"
echo "TF_LUNA_RANGE_TOPIC=${TF_LUNA_RANGE_TOPIC}"
echo "LIDAR_MODEL=${LIDAR_MODEL}"
echo "LIDAR_SERIAL_PORT=${LIDAR_SERIAL_PORT}"
echo "LIDAR_FALLBACK_SERIAL_PORT=${LIDAR_FALLBACK_SERIAL_PORT}"
echo "LIDAR_SERIAL_BAUDRATE=${LIDAR_SERIAL_BAUDRATE}"
echo "LIDAR_FRAME_ID=${LIDAR_FRAME_ID}"
echo "LIDAR_SCAN_MODE=${LIDAR_SCAN_MODE}"
echo "LIDAR_USB_SERIAL_SHORT=${LIDAR_USB_SERIAL_SHORT:-}"
echo "OMNI_ENABLE_SLAM=${OMNI_ENABLE_SLAM}"
echo "SLAM_PARAMS=${SLAM_PARAMS}"
echo "SLAM_USE_SIM_TIME=${SLAM_USE_SIM_TIME}"

echo "== systemd =="
systemctl is-enabled omni-can.service omni-bridge.service omni-odom.service omni-tfluna.service omni-mux.service teleop-web.service omni-lidar.service omni-slam.service || true
systemctl is-active omni-can.service omni-bridge.service omni-odom.service omni-tfluna.service omni-mux.service teleop-web.service omni-lidar.service omni-slam.service || true

echo "== service exec commands =="
systemctl show -p ExecStart omni-bridge.service omni-odom.service omni-tfluna.service omni-mux.service teleop-web.service omni-lidar.service omni-slam.service --no-pager || true

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

if [[ "${OMNI_ENABLE_LIDAR}" != "0" ]]; then
  echo "== LIDAR ROS package =="
  test -d "${OMNI_ROS_WS}/src/sllidar_ros2" && echo "source: ${OMNI_ROS_WS}/src/sllidar_ros2" || echo "missing source: ${OMNI_ROS_WS}/src/sllidar_ros2"
  ros2 pkg prefix sllidar_ros2 || true
  ros2 pkg executables sllidar_ros2 || true
fi

echo "== Base telemetry topics =="
timeout 8 ros2 topic echo --once /omni/base_status || true
timeout 8 ros2 topic echo --once /omni/wheel_states || true

echo "== INA228 power telemetry =="
timeout 8 ros2 topic echo --once /omni/power_status || true

echo "== Odometry and TF =="
ros2 pkg executables omni_bridge | grep wheel_odometry || true
timeout 8 ros2 topic echo --once /odom || true
timeout 8 ros2 topic echo --once /tf_static || true

echo "== TF-Luna front range sensor =="
ros2 pkg executables omni_bridge | grep tf_luna || true
ls -l "${TF_LUNA_SERIAL_PORT}" 2>/dev/null || true
timeout 8 ros2 topic echo --once "${TF_LUNA_RANGE_TOPIC}" || true
timeout 8 ros2 topic echo --once "${TF_LUNA_STATUS_TOPIC}" || true

echo "== LIDAR device =="
ls -l "${LIDAR_SERIAL_PORT}" "${LIDAR_FALLBACK_SERIAL_PORT}" 2>/dev/null || true
if [[ -e "${LIDAR_SERIAL_PORT}" ]]; then
  udevadm info -q property -n "${LIDAR_SERIAL_PORT}" | grep -E '^(ID_VENDOR|ID_MODEL|ID_SERIAL|ID_SERIAL_SHORT|DEVLINKS)=' || true
elif [[ -n "${LIDAR_FALLBACK_SERIAL_PORT:-}" && -e "${LIDAR_FALLBACK_SERIAL_PORT}" ]]; then
  udevadm info -q property -n "${LIDAR_FALLBACK_SERIAL_PORT}" | grep -E '^(ID_VENDOR|ID_MODEL|ID_SERIAL|ID_SERIAL_SHORT|DEVLINKS)=' || true
fi

echo "== LIDAR scan sample =="
timeout 8 ros2 topic echo --once /scan || true
echo "== SLAM toolbox =="
if [[ "${OMNI_ENABLE_SLAM}" != "0" ]]; then
  test -f "${SLAM_PARAMS}" && echo "params: ${SLAM_PARAMS}" || echo "missing params: ${SLAM_PARAMS}"
  ros2 pkg prefix slam_toolbox || true
  ros2 pkg executables slam_toolbox || true
  timeout 8 ros2 topic echo --once /map || true
  timeout 8 ros2 run tf2_ros tf2_echo map odom || true
fi


echo "== Base CAN sample =="
timeout 3 candump "${CAN_IFACE},190:7FF,191:7FF,192:7FF" || true
