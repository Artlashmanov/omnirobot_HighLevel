#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/omni_env.sh"

unset AMENT_TRACE_SETUP_FILES || true
source_ros
activate_venv

wait_timeout="${LIDAR_WAIT_TIMEOUT_SEC:-20}"
if ! [[ "${wait_timeout}" =~ ^[0-9]+$ ]]; then
  echo "Invalid LIDAR_WAIT_TIMEOUT_SEC=${wait_timeout}; expected integer seconds." >&2
  exit 2
fi

wait_for_lidar_port() {
  local elapsed

  for ((elapsed = 0; elapsed <= wait_timeout; elapsed++)); do
    if [[ -e "${LIDAR_SERIAL_PORT}" ]]; then
      printf '%s\n' "${LIDAR_SERIAL_PORT}"
      return 0
    fi

    if [[ -n "${LIDAR_FALLBACK_SERIAL_PORT:-}" && -e "${LIDAR_FALLBACK_SERIAL_PORT}" ]]; then
      echo "Using fallback LIDAR port ${LIDAR_FALLBACK_SERIAL_PORT}; preferred ${LIDAR_SERIAL_PORT} is not present." >&2
      printf '%s\n' "${LIDAR_FALLBACK_SERIAL_PORT}"
      return 0
    fi

    if [[ "${elapsed}" -lt "${wait_timeout}" ]]; then
      echo "Waiting for LIDAR serial port ${LIDAR_SERIAL_PORT} (${elapsed}/${wait_timeout}s)..." >&2
      sleep 1
    fi
  done

  echo "LIDAR serial port not found: ${LIDAR_SERIAL_PORT}; fallback: ${LIDAR_FALLBACK_SERIAL_PORT:-<disabled>}." >&2
  return 1
}

serial_port="$(wait_for_lidar_port)"

echo "Starting RPLIDAR C1 on ${serial_port} at ${LIDAR_SERIAL_BAUDRATE} baud, frame_id=${LIDAR_FRAME_ID}, scan_mode=${LIDAR_SCAN_MODE}." >&2

exec ros2 launch sllidar_ros2 sllidar_c1_launch.py \
  serial_port:="${serial_port}" \
  serial_baudrate:="${LIDAR_SERIAL_BAUDRATE}" \
  frame_id:="${LIDAR_FRAME_ID}" \
  inverted:="${LIDAR_INVERTED}" \
  angle_compensate:="${LIDAR_ANGLE_COMPENSATE}" \
  scan_mode:="${LIDAR_SCAN_MODE}"
