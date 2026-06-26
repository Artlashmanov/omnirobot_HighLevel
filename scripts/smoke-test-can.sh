#!/usr/bin/env bash
set -euo pipefail

CAN_IFACE="${CAN_IFACE:-can0}"
SECONDS_TO_SAMPLE="${1:-6}"
TMP="$(mktemp)"
trap 'rm -f "${TMP}"' EXIT

timeout "${SECONDS_TO_SAMPLE}" candump -L "${CAN_IFACE},181:7FF,182:7FF,183:7FF,190:7FF,191:7FF" > "${TMP}" || true
python3 - "${TMP}" <<'PY'
from collections import Counter
from pathlib import Path
import sys

path = Path(sys.argv[1])
ids = Counter()
wheels = Counter()
base = []
for line in path.read_text(errors='replace').splitlines():
    frame = next((part for part in line.split() if '#' in part), None)
    if not frame:
        continue
    can_id, data = frame.split('#', 1)
    can_id = can_id.upper().lstrip('0') or '0'
    ids[can_id] += 1
    if can_id == '190' and len(data) >= 16:
        b = bytes.fromhex(data[:16])
        base.append({
            'seq': b[1], 'motion_mode': b[2], 'speed_pct': b[3],
            'wheel_count': b[4], 'status_flags': b[5], 'error_flags': b[6],
        })
    if can_id == '191' and len(data) >= 16:
        wheels[bytes.fromhex(data[:16])[0]] += 1

print('id_counts', dict(sorted(ids.items())))
print('wheel_index_counts', dict(sorted(wheels.items())))
print('last_base_status', base[-5:])
PY
