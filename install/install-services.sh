#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run through sudo: sudo ./install/install-services.sh" >&2
  exit 1
fi

cp "${PROJECT_DIR}"/services/*.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable omni-can.service omni-bridge.service omni-mux.service teleop-web.service
systemctl enable omni-lidar.service || true

echo "Services installed. Start with: sudo systemctl start omni-can omni-bridge omni-mux teleop-web"
echo "Note: current service files assume /home/noob/omni-pi and user noob."
