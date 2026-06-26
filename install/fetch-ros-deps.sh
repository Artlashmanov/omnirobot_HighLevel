#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="${OMNI_ROS_WS:-${PROJECT_DIR}/src/ros2_ws}"
REPOS_FILE="${OMNI_ROS_REPOS_FILE:-${ROS_WS}/omni.repos}"
FETCH_ROS_DEPS="${OMNI_FETCH_ROS_DEPS:-1}"
ENABLE_LIDAR="${OMNI_ENABLE_LIDAR:-1}"

is_false() {
  case "${1:-}" in
    0|false|FALSE|no|NO|off|OFF) return 0 ;;
    *) return 1 ;;
  esac
}

if is_false "${FETCH_ROS_DEPS}"; then
  echo "Skipping external ROS source fetch because OMNI_FETCH_ROS_DEPS=${FETCH_ROS_DEPS}."
  exit 0
fi

if [[ ! -f "${REPOS_FILE}" ]]; then
  echo "External ROS repos file is missing: ${REPOS_FILE}" >&2
  exit 1
fi

if ! command -v vcs >/dev/null 2>&1; then
  echo "vcstool is not installed. Install package: python3-vcstool" >&2
  exit 1
fi

install -d "${ROS_WS}/src"

cd "${ROS_WS}"

echo "Fetching external ROS source dependencies from ${REPOS_FILE}..."
# omni.repos stores paths relative to the ROS workspace root, for example:
#   src/sllidar_ros2
# Therefore the import base path is '.', not 'src'.
vcs import --input "${REPOS_FILE}" --skip-existing --recursive --retry 3 .

if ! is_false "${ENABLE_LIDAR}"; then
  if [[ ! -d "${ROS_WS}/src/sllidar_ros2" ]]; then
    echo "RPLIDAR C1 is enabled, but ${ROS_WS}/src/sllidar_ros2 was not fetched." >&2
    echo "Check ${REPOS_FILE} and network access to GitHub." >&2
    exit 1
  fi

  if [[ -d "${ROS_WS}/src/sllidar_ros2/.git" ]]; then
    echo "sllidar_ros2: $(git -C "${ROS_WS}/src/sllidar_ros2" rev-parse --short HEAD)"
  else
    echo "sllidar_ros2 present at ${ROS_WS}/src/sllidar_ros2"
  fi
fi
