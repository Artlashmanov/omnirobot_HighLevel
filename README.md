# omnirobot HighLevel

High-level Raspberry Pi 5 project for the omnirobot platform.

This repository contains the Pi-side software stack:

- ROS2 Jazzy workspace;
- SocketCAN bridge to the STM32 base controller;
- MANUAL/AUTO command mux;
- Flask web teleop UI;
- Slamtec RPLIDAR C1 ROS2 driver integration;
- slam_toolbox mapping integration;
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

- `/odom` (`nav_msgs/Odometry`, wheel dead-reckoning)
- `/scan` (`sensor_msgs/msg/LaserScan`, RPLIDAR C1)
- `/map` (`nav_msgs/msg/OccupancyGrid`, when `omni-slam.service` is running)
- `/tf`, `/tf_static` (currently `odom -> base_link -> laser`)
- `/omni/status_text` (`std_msgs/String`)
- `/omni/status_json` (`std_msgs/String` JSON)
- `/omni/rx_raw` (`std_msgs/String` JSON)
- `/omni/base_status` (`std_msgs/String` JSON, decoded CAN `0x190`)
- `/omni/wheel_states` (`std_msgs/String` JSON, aggregated decoded CAN `0x191`)


## Platform layer

The high-level stack now selects a robot base through a platform profile. The current implemented platform is `omni4`, configured by `ROBOT_PLATFORM=omni4` and `config/platforms/omni4.json`. See [docs/architecture.md](docs/architecture.md) for the layer model and how future platforms should be added.

## STM32 CAN protocol contract

The byte-level Pi5 <-> STM32 CAN contract is documented in [docs/can-protocol.md](docs/can-protocol.md). Update that file together with `src/omni_pi/protocol.py` whenever the STM32 CAN frame layout changes.

Wheel odometry and the robot frame tree are documented in [docs/odometry.md](docs/odometry.md). SLAM/map runtime is documented in [docs/slam.md](docs/slam.md).

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

## External ROS sources and hardware drivers

Third-party ROS packages are pinned in `src/ros2_ws/omni.repos`. The Slamtec RPLIDAR C1 driver is intentionally not vendored into this repository; the installer restores it automatically before building the ROS workspace:

```bash
./install/fetch-ros-deps.sh
```

Manual equivalent:

```bash
cd src/ros2_ws
vcs import --input omni.repos .
```

The current hardware profile for the LIDAR is `config/hardware/lidars/rplidar_c1.yaml`. Future installer UI checkboxes should update/select these hardware profiles, then run the same fetch/build/service install pipeline.

## Install on a fresh Pi5

The installer assumes Ubuntu 24.04 + ROS2 Jazzy and the project checked out to `/home/noob/omni-pi`. It installs apt packages, restores pinned external ROS sources such as `sllidar_ros2`, builds the workspace, installs udev rules, and enables systemd services.

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
