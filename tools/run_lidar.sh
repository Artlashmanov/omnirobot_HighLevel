#!/usr/bin/env bash
set -eo pipefail

unset AMENT_TRACE_SETUP_FILES || true
source /opt/ros/jazzy/setup.bash
source /home/noob/omni-pi/src/ros2_ws/install/setup.bash
source /home/noob/omni-pi/.venv_ros/bin/activate

exec ros2 launch sllidar_ros2 sllidar_c1_launch.py
