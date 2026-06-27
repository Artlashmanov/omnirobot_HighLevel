#!/usr/bin/env bash
# Shared runtime environment for omnirobot Pi5 scripts.
# Defaults are suitable for the current live robot; /etc/omni-robot/omni.env can override them.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_OMNI_HOME="$(cd "${SCRIPT_DIR}/.." && pwd)"

OMNI_ENV_FILE="${OMNI_ENV_FILE:-/etc/omni-robot/omni.env}"
if [[ -f "${OMNI_ENV_FILE}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${OMNI_ENV_FILE}"
  set +a
fi

export OMNI_HOME="${OMNI_HOME:-${DEFAULT_OMNI_HOME}}"
export OMNI_USER="${OMNI_USER:-noob}"
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
export ROBOT_PLATFORM="${ROBOT_PLATFORM:-omni4}"
export OMNI_PLATFORM_CONFIG="${OMNI_PLATFORM_CONFIG:-${OMNI_HOME}/config/platforms/${ROBOT_PLATFORM}.json}"
export CAN_IFACE="${CAN_IFACE:-can0}"
export CAN_BITRATE="${CAN_BITRATE:-500000}"
export TELEOP_HOST="${TELEOP_HOST:-0.0.0.0}"
export TELEOP_PORT="${TELEOP_PORT:-8080}"
export OMNI_ROS_WS="${OMNI_ROS_WS:-${OMNI_HOME}/src/ros2_ws}"
export OMNI_VENV="${OMNI_VENV:-${OMNI_HOME}/.venv_ros}"
export OMNI_BRIDGE_PARAMS="${OMNI_BRIDGE_PARAMS:-${OMNI_ROS_WS}/src/omni_bridge/config/omni_bridge.params.yaml}"
export OMNI_FETCH_ROS_DEPS="${OMNI_FETCH_ROS_DEPS:-1}"
export OMNI_ENABLE_LIDAR="${OMNI_ENABLE_LIDAR:-1}"
export OMNI_ROS_REPOS_FILE="${OMNI_ROS_REPOS_FILE:-${OMNI_ROS_WS}/omni.repos}"
export OMNI_ODOM_PARAMS="${OMNI_ODOM_PARAMS:-${OMNI_BRIDGE_PARAMS}}"
export OMNI_ENABLE_TF_LUNA="${OMNI_ENABLE_TF_LUNA:-1}"
export TF_LUNA_PARAMS="${TF_LUNA_PARAMS:-${OMNI_BRIDGE_PARAMS}}"
export TF_LUNA_SERIAL_PORT="${TF_LUNA_SERIAL_PORT:-/dev/ttyAMA0}"
export TF_LUNA_BAUDRATE="${TF_LUNA_BAUDRATE:-115200}"
export TF_LUNA_FRAME_ID="${TF_LUNA_FRAME_ID:-tf_luna_front}"
export TF_LUNA_RANGE_TOPIC="${TF_LUNA_RANGE_TOPIC:-/range/front}"
export TF_LUNA_STATUS_TOPIC="${TF_LUNA_STATUS_TOPIC:-/sensors/tf_luna/front}"
export LIDAR_MODEL="${LIDAR_MODEL:-rplidar_c1}"
export LIDAR_SERIAL_PORT="${LIDAR_SERIAL_PORT:-/dev/rplidar}"
export LIDAR_FALLBACK_SERIAL_PORT="${LIDAR_FALLBACK_SERIAL_PORT:-/dev/ttyUSB0}"
export LIDAR_SERIAL_BAUDRATE="${LIDAR_SERIAL_BAUDRATE:-460800}"
export LIDAR_FRAME_ID="${LIDAR_FRAME_ID:-laser}"
export LIDAR_WAIT_TIMEOUT_SEC="${LIDAR_WAIT_TIMEOUT_SEC:-20}"
export LIDAR_SCAN_MODE="${LIDAR_SCAN_MODE:-Standard}"
export LIDAR_INVERTED="${LIDAR_INVERTED:-false}"
export LIDAR_ANGLE_COMPENSATE="${LIDAR_ANGLE_COMPENSATE:-true}"
export LIDAR_USB_SERIAL_SHORT="${LIDAR_USB_SERIAL_SHORT:-}"
export OMNI_ENABLE_SLAM="${OMNI_ENABLE_SLAM:-1}"
export SLAM_PARAMS="${SLAM_PARAMS:-${OMNI_HOME}/config/slam/slam_toolbox_online_async.yaml}"
export SLAM_USE_SIM_TIME="${SLAM_USE_SIM_TIME:-false}"

with_nounset_disabled() {
  local restore_nounset=0
  case "$-" in
    *u*)
      restore_nounset=1
      set +u
      ;;
  esac

  "$@"

  if [[ "${restore_nounset}" -eq 1 ]]; then
    set -u
  fi
}

source_ros_impl() {
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  if [[ -f "${OMNI_ROS_WS}/install/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "${OMNI_ROS_WS}/install/setup.bash"
  fi
}

source_ros() {
  with_nounset_disabled source_ros_impl
}

activate_venv_impl() {
  if [[ -f "${OMNI_VENV}/bin/activate" ]]; then
    # shellcheck disable=SC1090
    source "${OMNI_VENV}/bin/activate"
  fi
}

activate_venv() {
  with_nounset_disabled activate_venv_impl
}

export PYTHONPATH="${OMNI_HOME}/src:${PYTHONPATH:-}"
