#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/omni_env.sh"

if [[ "${OMNI_ENABLE_TF_LUNA}" == "0" ]]; then
  echo "TF-Luna service disabled by OMNI_ENABLE_TF_LUNA=0." >&2
  exit 0
fi

source_ros
activate_venv

installed_node="${OMNI_ROS_WS}/install/omni_bridge/lib/omni_bridge/tf_luna"

if [[ ! -x "${installed_node}" ]]; then
  echo "Missing installed TF-Luna executable: ${installed_node}" >&2
  echo "Build the workspace first: ${OMNI_HOME}/install/build-workspace.sh" >&2
  exit 1
fi

exec "${OMNI_VENV}/bin/python" "${installed_node}" \
  --ros-args \
  --params-file "${TF_LUNA_PARAMS}" \
  -p serial_port:="${TF_LUNA_SERIAL_PORT}" \
  -p baudrate:="${TF_LUNA_BAUDRATE}" \
  -p frame_id:="${TF_LUNA_FRAME_ID}" \
  -p range_topic:="${TF_LUNA_RANGE_TOPIC}" \
  -p status_topic:="${TF_LUNA_STATUS_TOPIC}"
