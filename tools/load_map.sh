#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/omni_env.sh"

source_ros
activate_venv

raw_name="${1:-}"
name="$(printf '%s' "${raw_name}" | tr -cs 'A-Za-z0-9_.-' '_' | sed -E 's/^[_\.\-]+//; s/[_\.\-]+$//; s/(.{64}).*/\1/')"
if [[ -z "${name}" || ! "${name}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$ ]]; then
  echo "Invalid map name: ${raw_name}" >&2
  exit 2
fi

maps_dir="${OMNI_MAPS_DIR:-${OMNI_HOME}/maps}"
map_dir="${maps_dir}/${name}"
posegraph_prefix="${map_dir}/slam_posegraph"

if [[ ! -d "${map_dir}" ]]; then
  echo "Saved map not found: ${map_dir}" >&2
  exit 2
fi

if [[ ! -f "${posegraph_prefix}.posegraph" && ! -f "${posegraph_prefix}.data" ]]; then
  echo "Saved map has no slam_toolbox pose graph: ${map_dir}" >&2
  exit 5
fi

if ! ros2 service list | grep -Fxq "/slam_toolbox/deserialize_map"; then
  echo "slam_toolbox deserialize service is not available" >&2
  exit 4
fi

# Loading a pose graph is a mapping/resume action, not localization mode yet.
ros2 topic pub --once /omni/manual_cmd std_msgs/msg/String "{data: '{\"mode\":\"STOP\",\"speed_pct\":0}'}" >/dev/null 2>&1 || true
ros2 topic pub --once /omni/odom_reset std_msgs/msg/String "{data: 'load_map'}" >/dev/null 2>&1 || true

echo "Loading slam_toolbox pose graph: ${posegraph_prefix}" >&2
timeout "${MAP_LOAD_TIMEOUT_SEC:-20}" ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph "{filename: '${posegraph_prefix}', match_type: 1, initial_pose: {x: 0.0, y: 0.0, theta: 0.0}}" >&2
python3 - "${map_dir}" "${name}" <<'PY'
import json
import sys
from pathlib import Path

map_dir = Path(sys.argv[1])
name = sys.argv[2]
files = sorted(item.name for item in map_dir.iterdir() if item.is_file())
print(json.dumps({"ok": True, "action": "load_map", "name": name, "path": str(map_dir), "files": files}, ensure_ascii=False))
PY
