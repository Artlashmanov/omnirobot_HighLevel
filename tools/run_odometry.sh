#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/omni_env.sh"

source_ros
activate_venv

installed_odom="${OMNI_ROS_WS}/install/omni_bridge/lib/omni_bridge/wheel_odometry"

if [[ ! -x "${installed_odom}" ]]; then
  echo "Missing installed odometry executable: ${installed_odom}" >&2
  echo "Build the workspace first: ${OMNI_HOME}/install/build-workspace.sh" >&2
  exit 1
fi

exec "${OMNI_VENV}/bin/python" "${installed_odom}" \
  --ros-args \
  --params-file "${OMNI_ODOM_PARAMS}"
