#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_OWNER="$(stat -c '%U' "${PROJECT_DIR}")"
OMNI_USER_VALUE="${OMNI_USER:-${SUDO_USER:-${PROJECT_OWNER}}}"
if [[ "${OMNI_USER_VALUE}" == "root" && "${PROJECT_OWNER}" != "root" ]]; then
  OMNI_USER_VALUE="${PROJECT_OWNER}"
fi

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run through sudo: sudo ./install/install-services.sh" >&2
  exit 1
fi

install -d -m 0755 /etc/omni-robot
if [[ ! -f /etc/omni-robot/omni.env ]]; then
  sed \
    -e "s|^OMNI_HOME=.*|OMNI_HOME=${PROJECT_DIR}|" \
    -e "s|^OMNI_USER=.*|OMNI_USER=${OMNI_USER_VALUE}|" \
    "${PROJECT_DIR}/config/omni.env.example" > /etc/omni-robot/omni.env
  chmod 0644 /etc/omni-robot/omni.env
else
  cp -a /etc/omni-robot/omni.env "/etc/omni-robot/omni.env.bak.$(date +%Y%m%d-%H%M%S)"
  sed -i \
    -e "s|^OMNI_HOME=.*|OMNI_HOME=${PROJECT_DIR}|" \
    -e "s|^OMNI_USER=.*|OMNI_USER=${OMNI_USER_VALUE}|" \
    /etc/omni-robot/omni.env
  grep -q '^OMNI_HOME=' /etc/omni-robot/omni.env || echo "OMNI_HOME=${PROJECT_DIR}" >> /etc/omni-robot/omni.env
  grep -q '^OMNI_USER=' /etc/omni-robot/omni.env || echo "OMNI_USER=${OMNI_USER_VALUE}" >> /etc/omni-robot/omni.env
fi

SERVICE_TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${SERVICE_TMP_DIR}"' EXIT

for service_file in "${PROJECT_DIR}"/services/*.service; do
  service_name="$(basename "${service_file}")"
  sed \
    -e "s|^User=.*|User=${OMNI_USER_VALUE}|" \
    -e "s|/home/noob/omni-pi|${PROJECT_DIR}|g" \
    -e "s|/home/noob/omnirobot_HighLevel|${PROJECT_DIR}|g" \
    "${service_file}" > "${SERVICE_TMP_DIR}/${service_name}"
done

cp "${SERVICE_TMP_DIR}"/*.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable omni-can.service omni-bridge.service omni-mux.service teleop-web.service
systemctl enable omni-lidar.service || true

echo "Services installed. Runtime config: /etc/omni-robot/omni.env"
echo "Services run as user: ${OMNI_USER_VALUE}"
echo "Start core stack with: sudo systemctl start omni-can omni-bridge omni-mux teleop-web"
