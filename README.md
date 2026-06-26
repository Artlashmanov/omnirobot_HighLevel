# omnirobot HighLevel

High-level Raspberry Pi 5 project for the omnirobot platform.

This repository contains the Pi-side software stack:

- ROS2 Jazzy workspace;
- SocketCAN bridge to the STM32 base controller;
- MANUAL/AUTO command mux;
- Flask web teleop UI;
- Slamtec RPLIDAR C1 ROS2 driver integration;
- systemd service definitions;
- install, verify, recovery, and inventory helpers.

The low-level STM32 firmware lives in a separate repository: `omnirobot`.
This repository is intentionally Pi5/high-level only.

## Current robot target

Default target path on the robot:

```bash
/home/noob/omni-pi
```

Runtime service config is read from:

```bash
/etc/omni-robot/omni.env
```

Default SSH user on the current robot:

```bash
noob
```

Default CAN interface:

```bash
can0 @ 500000 bit/s
```

## Live ROS graph

Core nodes:

- `/omni_can_bridge`
- `/command_mux`
- `/teleop_web_node`

Core command topics:

- `/omni/control_mode` (`std_msgs/String`): `AUTO` or `MANUAL`
- `/omni/manual_cmd` (`std_msgs/String` JSON)
- `/omni/auto_cmd` (`std_msgs/String` JSON)
- `/omni/motion_cmd` (`std_msgs/String` JSON after mux)
- `/cmd_vel` (`geometry_msgs/Twist`, currently supported directly by bridge)

Core telemetry topics:

- `/omni/status_text` (`std_msgs/String`)
- `/omni/status_json` (`std_msgs/String` JSON)
- `/omni/rx_raw` (`std_msgs/String` JSON)
- `/omni/base_status` (`std_msgs/String` JSON, decoded CAN `0x190`)
- `/omni/wheel_states` (`std_msgs/String` JSON, aggregated decoded CAN `0x191`)

## Quick operator commands

Read CAN telemetry:

```bash
candump can0,181:7FF,182:7FF,183:7FF,190:7FF,191:7FF
```

Check ROS topics:

```bash
source /opt/ros/jazzy/setup.bash
source /home/noob/omni-pi/src/ros2_ws/install/setup.bash
ros2 topic list -t
```

Open teleop UI:

```text
http://<pi-ip>:8080
```

## External ROS sources

Third-party ROS packages are pinned in `src/ros2_ws/omni.repos`. The local checkout may contain `src/ros2_ws/src/sllidar_ros2`, but it is intentionally ignored by git and should be restored with:

```bash
cd src/ros2_ws
vcs import src < omni.repos
```

## Install on a fresh Pi5

The current first-pass installer assumes Ubuntu 24.04 + ROS2 Jazzy and the project checked out to `/home/noob/omni-pi`.

```bash
cd /home/noob/omni-pi
sudo ./install/install.sh
./install/verify-install.sh
```

See [docs/recovery.md](docs/recovery.md) before using this on a completely fresh Pi.

## Hardware note

LIDAR and RealSense D415 are not required for the base/CAN/teleop layer. Connect them when working on lidar service, SLAM, navigation, or perception. The current base telemetry and command stack can be developed and tested with only CAN connected.


## Runtime model

The systemd units do not run Python files out of `src/ros2_ws/build`. They call small scripts in `tools/`, and those scripts:

1. load `/etc/omni-robot/omni.env`;
2. source ROS2 Jazzy;
3. source the workspace `install/` overlay;
4. activate `.venv_ros`;
5. launch installed ROS executables from `install/` using the project virtualenv where needed, for example the CAN bridge executable.

This keeps build artifacts disposable and makes a fresh Pi install reproducible.
