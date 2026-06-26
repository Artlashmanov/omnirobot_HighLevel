#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/omni_env.sh"

source_ros
activate_venv

installed_bridge="${OMNI_ROS_WS}/install/omni_bridge/lib/omni_bridge/can_bridge"

if [[ ! -x "${installed_bridge}" ]]; then
  echo "Missing installed bridge executable: ${installed_bridge}" >&2
  echo "Build the workspace first: ${OMNI_HOME}/install/build-workspace.sh" >&2
  exit 1
fi

exec "${OMNI_VENV}/bin/python" "${installed_bridge}" \
  --ros-args \
  --params-file "${OMNI_BRIDGE_PARAMS}" \
  -p platform_name:="${ROBOT_PLATFORM}" \
  -p platform_config:="${OMNI_PLATFORM_CONFIG}"
