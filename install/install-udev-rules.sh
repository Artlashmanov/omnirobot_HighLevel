#!/usr/bin/env bash
set -euo pipefail

OMNI_ENV_FILE="${OMNI_ENV_FILE:-/etc/omni-robot/omni.env}"
UDEV_RULE_FILE="/etc/udev/rules.d/99-omni-rplidar.rules"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run through sudo: sudo ./install/install-udev-rules.sh" >&2
  exit 1
fi

if [[ -f "${OMNI_ENV_FILE}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${OMNI_ENV_FILE}"
  set +a
fi

update_env_key_if_empty() {
  local key="$1"
  local value="$2"

  [[ -f "${OMNI_ENV_FILE}" ]] || return 0

  local current=""
  current="$(grep -E "^${key}=" "${OMNI_ENV_FILE}" | tail -n 1 | cut -d= -f2- || true)"
  if [[ -z "${current}" ]]; then
    if grep -q -E "^${key}=" "${OMNI_ENV_FILE}"; then
      sed -i "s|^${key}=.*|${key}=${value}|" "${OMNI_ENV_FILE}"
    else
      printf '%s=%s\n' "${key}" "${value}" >> "${OMNI_ENV_FILE}"
    fi
  fi
}

detect_rplidar_serial() {
  local dev props serial

  for dev in /dev/ttyUSB*; do
    [[ -e "${dev}" ]] || continue
    props="$(udevadm info -q property -n "${dev}" 2>/dev/null || true)"
    if grep -q '^ID_VENDOR_ID=10c4$' <<< "${props}" && grep -q '^ID_MODEL_ID=ea60$' <<< "${props}"; then
      serial="$(sed -n 's/^ID_SERIAL_SHORT=//p' <<< "${props}" | head -n 1)"
      if [[ -n "${serial}" ]]; then
        printf '%s\n' "${serial}"
        return 0
      fi
    fi
  done

  return 1
}

install -d -m 0755 /etc/udev/rules.d
install -d -m 0755 "$(dirname "${OMNI_ENV_FILE}")"

lidar_serial="${LIDAR_USB_SERIAL_SHORT:-}"
if [[ -z "${lidar_serial}" ]]; then
  lidar_serial="$(detect_rplidar_serial || true)"
  if [[ -n "${lidar_serial}" ]]; then
    update_env_key_if_empty "LIDAR_USB_SERIAL_SHORT" "${lidar_serial}"
  fi
fi

if [[ -n "${lidar_serial}" && ! "${lidar_serial}" =~ ^[A-Za-z0-9._:-]+$ ]]; then
  echo "Refusing unsafe LIDAR_USB_SERIAL_SHORT value: ${lidar_serial}" >&2
  exit 1
fi

serial_match=""
if [[ -n "${lidar_serial}" ]]; then
  serial_match=", ENV{ID_SERIAL_SHORT}==\"${lidar_serial}\""
fi

cat > "${UDEV_RULE_FILE}" <<EOF
# Installed by omnirobot_HighLevel/install/install-udev-rules.sh
# Slamtec RPLIDAR C1 USB UART bridge (Silicon Labs CP210x, 10c4:ea60).
# If LIDAR_USB_SERIAL_SHORT is known, the rule is pinned to this exact device.
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60"${serial_match}, GROUP="dialout", MODE="0660", SYMLINK+="rplidar"
EOF

target_user="${OMNI_USER:-${SUDO_USER:-noob}}"
if id "${target_user}" >/dev/null 2>&1 && getent group dialout >/dev/null 2>&1; then
  usermod -aG dialout "${target_user}" || true
fi

udevadm control --reload-rules
udevadm trigger --subsystem-match=tty || true

echo "Installed udev rule: ${UDEV_RULE_FILE}"
if [[ -n "${lidar_serial}" ]]; then
  echo "RPLIDAR rule pinned to USB serial: ${lidar_serial}"
else
  echo "RPLIDAR was not connected; installed generic CP210x rule."
fi
echo "If /dev/rplidar does not appear immediately, replug the LIDAR USB cable."
