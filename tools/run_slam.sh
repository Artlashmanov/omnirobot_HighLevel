#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/omni_env.sh"

if [[ "${OMNI_ENABLE_SLAM}" == "0" ]]; then
  echo "SLAM service disabled by OMNI_ENABLE_SLAM=0." >&2
  exit 0
fi

unset AMENT_TRACE_SETUP_FILES || true
source_ros
activate_venv

if [[ ! -f "${SLAM_PARAMS}" ]]; then
  echo "SLAM params file not found: ${SLAM_PARAMS}" >&2
  exit 2
fi

if ! ros2 pkg prefix slam_toolbox >/dev/null 2>&1; then
  echo "slam_toolbox ROS package is missing. Install package: ros-${ROS_DISTRO}-slam-toolbox" >&2
  exit 2
fi

echo "Starting slam_toolbox online_async: params=${SLAM_PARAMS}, use_sim_time=${SLAM_USE_SIM_TIME}." >&2

exec ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="${SLAM_PARAMS}" \
  use_sim_time:="${SLAM_USE_SIM_TIME}"
