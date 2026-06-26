#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/omni_env.sh"

source_ros
activate_venv

exec ros2 launch "${OMNI_HOME}/launch/omni_robot.launch.py" \
  params_file:="${OMNI_BRIDGE_PARAMS}"
