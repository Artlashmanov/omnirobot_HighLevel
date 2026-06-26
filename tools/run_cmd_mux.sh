#!/usr/bin/env bash
set -eo pipefail

unset AMENT_TRACE_SETUP_FILES || true
source /opt/ros/jazzy/setup.bash
source /home/noob/omni-pi/.venv_ros/bin/activate

exec python /home/noob/omni-pi/tools/cmd_mux_node.py
