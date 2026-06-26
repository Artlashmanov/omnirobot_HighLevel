#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run through sudo: sudo ./install/install.sh" >&2
  exit 1
fi

cd "${PROJECT_DIR}"

apt-get update
xargs -a install/apt-packages.txt apt-get install -y

if [[ ! -d .venv_ros ]]; then
  python3 -m venv --system-site-packages .venv_ros
fi

. .venv_ros/bin/activate
python -m pip install --upgrade pip
python -m pip install -r install/requirements-venv.txt

sudo -u "${SUDO_USER:-noob}" bash install/build-workspace.sh
bash install/install-services.sh
bash install/install-udev-rules.sh

echo "Install finished. Run: ./install/verify-install.sh"
