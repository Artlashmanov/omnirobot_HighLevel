#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/omni_env.sh"

source_ros
activate_venv

raw_name="${1:-}"
if [[ -z "${raw_name}" ]]; then
  raw_name="map_$(date -u +%Y%m%d_%H%M%S)"
fi

name="$(printf '%s' "${raw_name}" | tr -cs 'A-Za-z0-9_.-' '_' | sed -E 's/^[_\.\-]+//; s/[_\.\-]+$//; s/(.{64}).*/\1/')"
if [[ -z "${name}" || ! "${name}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$ ]]; then
  echo "Invalid map name: ${raw_name}" >&2
  exit 2
fi

maps_dir="${OMNI_MAPS_DIR:-${OMNI_HOME}/maps}"
map_dir="${maps_dir}/${name}"
mkdir -p "${maps_dir}"

if [[ -e "${map_dir}" ]]; then
  echo "Map already exists: ${map_dir}" >&2
  exit 2
fi
mkdir -p "${map_dir}"

echo "Waiting for one /map message..." >&2
timeout "${MAP_SAVE_WAIT_TIMEOUT_SEC:-10}" ros2 topic echo --once /map >/dev/null

if ! ros2 pkg prefix nav2_map_server >/dev/null 2>&1; then
  echo "Missing ROS package: ros-${ROS_DISTRO}-nav2-map-server" >&2
  exit 3
fi

echo "Saving occupancy grid to ${map_dir}/map.yaml + map.pgm" >&2
ros2 run nav2_map_server map_saver_cli -f "${map_dir}/map" >&2

posegraph_saved=false
if ros2 service list | grep -Fxq "/slam_toolbox/serialize_map"; then
  echo "Serializing slam_toolbox pose graph to ${map_dir}/slam_posegraph*" >&2
  if timeout "${POSEGRAPH_SAVE_TIMEOUT_SEC:-15}" ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '${map_dir}/slam_posegraph'}" >&2; then
    posegraph_saved=true
  else
    echo "Warning: slam_toolbox pose graph serialization failed; occupancy grid was still saved." >&2
  fi
else
  echo "slam_toolbox serialize service is not available; saved occupancy grid only." >&2
fi

python3 - "${map_dir}" "${name}" "${posegraph_saved}" <<'PY'
import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

map_dir = Path(sys.argv[1])
name = sys.argv[2]
posegraph_saved = sys.argv[3].lower() == 'true'
files = sorted(item.name for item in map_dir.iterdir() if item.is_file())
metadata = {
    "name": name,
    "created_utc": datetime.now(timezone.utc).isoformat(),
    "robot_platform": os.environ.get("ROBOT_PLATFORM", "omni4"),
    "lidar_model": os.environ.get("LIDAR_MODEL", "rplidar_c1"),
    "slam_params": os.environ.get("SLAM_PARAMS"),
    "source_map_topic": "/map",
    "occupancy_grid": "map.yaml",
    "posegraph_saved": posegraph_saved,
    "files": files,
}
(map_dir / "metadata.json").write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
print(json.dumps({"ok": True, "name": name, "path": str(map_dir), "files": files}, ensure_ascii=False))
PY
