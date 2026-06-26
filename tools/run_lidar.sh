#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/omni_env.sh"

unset AMENT_TRACE_SETUP_FILES || true
source_ros
activate_venv

serial_port="${LIDAR_SERIAL_PORT}"
if [[ ! -e "${serial_port}" && -e "${LIDAR_FALLBACK_SERIAL_PORT}" ]]; then
  serial_port="${LIDAR_FALLBACK_SERIAL_PORT}"
fi

exec ros2 launch sllidar_ros2 sllidar_c1_launch.py \
  serial_port:="${serial_port}" \
  serial_baudrate:="${LIDAR_SERIAL_BAUDRATE}" \
  frame_id:="${LIDAR_FRAME_ID}"
