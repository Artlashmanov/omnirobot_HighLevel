#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/omni_env.sh"

unset AMENT_TRACE_SETUP_FILES || true
source_ros
activate_venv

exec python "${OMNI_HOME}/teleop_web/app.py"
