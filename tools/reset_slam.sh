#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/omni_env.sh"

source_ros
activate_venv

if ! ros2 service list | grep -Fxq "/slam_toolbox/reset"; then
  echo "slam_toolbox reset service is not available" >&2
  exit 4
fi

echo "Resetting slam_toolbox map state..." >&2
timeout "${SLAM_RESET_TIMEOUT_SEC:-10}" ros2 service call /slam_toolbox/reset slam_toolbox/srv/Reset "{pause_new_measurements: false}" >&2
python3 - <<'PY'
import json
print(json.dumps({"ok": True, "action": "reset_slam"}, ensure_ascii=False))
PY
