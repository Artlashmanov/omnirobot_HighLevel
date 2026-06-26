#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
if [[ -f /home/noob/omni-pi/src/ros2_ws/install/setup.bash ]]; then
  source /home/noob/omni-pi/src/ros2_ws/install/setup.bash
fi

export PYTHONPATH=/home/noob/omni-pi/src:${PYTHONPATH:-}

exec /home/noob/omni-pi/.venv_ros/bin/python   /home/noob/omni-pi/src/ros2_ws/build/omni_bridge/omni_bridge/can_bridge_node.py   --ros-args   --params-file /home/noob/omni-pi/src/ros2_ws/src/omni_bridge/config/omni_bridge.params.yaml
