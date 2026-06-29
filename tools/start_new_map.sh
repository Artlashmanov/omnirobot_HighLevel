#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/omni_env.sh"

source_ros
activate_venv

# Stop is best-effort: the map reset should not fail only because teleop/mux is offline.
ros2 topic pub --once /omni/manual_cmd std_msgs/msg/String "{data: '{\"mode\":\"STOP\",\"speed_pct\":0}'}" >/dev/null 2>&1 || true
ros2 topic pub --once /omni/odom_reset std_msgs/msg/String "{data: 'start_new_map'}" >/dev/null 2>&1 || true

"${SCRIPT_DIR}/reset_slam.sh" >/dev/null

python3 - <<'PY'
import json
print(json.dumps({"ok": True, "action": "start_new_map"}, ensure_ascii=False))
PY
