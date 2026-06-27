#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run through sudo: sudo ./install/configure-pi-hardware.sh" >&2
  exit 1
fi

OMNI_ENABLE_TF_LUNA="${OMNI_ENABLE_TF_LUNA:-1}"

find_boot_config() {
  if [[ -f /boot/firmware/config.txt ]]; then
    printf '%s\n' /boot/firmware/config.txt
    return 0
  fi
  if [[ -f /boot/config.txt ]]; then
    printf '%s\n' /boot/config.txt
    return 0
  fi
  return 1
}

ensure_line() {
  local file="$1"
  local line="$2"
  grep -qxF "${line}" "${file}" || printf '%s\n' "${line}" >> "${file}"
}

boot_config="$(find_boot_config)" || {
  echo "Cannot find Raspberry Pi boot config (/boot/firmware/config.txt or /boot/config.txt)." >&2
  exit 1
}

backup="${boot_config}.omni-backup-$(date +%Y%m%d-%H%M%S)"
cp "${boot_config}" "${backup}"

changed=0
before_hash="$(sha256sum "${boot_config}" | awk '{print $1}')"

if [[ "${OMNI_ENABLE_TF_LUNA}" != "0" ]]; then
  if ! grep -qxF '# Omni Robot: enable GPIO UART0 on Pi5 pins 8/10 for TF-Luna' "${boot_config}"; then
    printf '\n# Omni Robot: enable GPIO UART0 on Pi5 pins 8/10 for TF-Luna\n' >> "${boot_config}"
  fi
  ensure_line "${boot_config}" 'enable_uart=1'
  ensure_line "${boot_config}" 'dtoverlay=uart0-pi5'
fi

after_hash="$(sha256sum "${boot_config}" | awk '{print $1}')"
if [[ "${before_hash}" != "${after_hash}" ]]; then
  changed=1
fi

echo "Boot config: ${boot_config}"
echo "Backup: ${backup}"
echo "Changed: ${changed}"
if [[ "${changed}" -eq 1 ]]; then
  echo "Reboot required for Pi hardware overlay changes."
fi
