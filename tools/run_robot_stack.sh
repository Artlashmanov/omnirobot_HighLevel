#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

exec ros2 launch /home/noob/omni-pi/launch/omni_robot.launch.py
