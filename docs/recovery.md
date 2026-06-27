# Pi5 recovery / reinstall notes

This document describes the intended direction for rebuilding a robot Pi5.

## Current assumption

The current live robot uses:

```bash
/home/noob/omni-pi
```

Service files are installed through `install/install-services.sh`, which renders the runtime user and `OMNI_HOME` into `/etc/systemd/system` and keeps `/etc/omni-robot/omni.env` aligned with the checkout path.

## Fresh install outline

1. Install Ubuntu 24.04 for Raspberry Pi 5.
2. Configure network and SSH.
3. Clone this repository to `/home/noob/omni-pi`.
4. Install ROS2 Jazzy packages.
5. Fetch pinned external ROS sources with `install/fetch-ros-deps.sh`.
6. Build ROS workspace.
7. Install systemd services.
8. Install udev rules for stable hardware device names.
9. Verify CAN, ROS graph, web UI, telemetry, and optional sensors.

Short version once OS and SSH are ready:

```bash
cd /home/noob
git clone <omnirobot_HighLevel-url> omni-pi
cd omni-pi
sudo ./install/install.sh
./install/verify-install.sh
```

## Service order

- `omni-can.service`: brings up `can0`.
- `omni-bridge.service`: ROS2 CAN bridge.
- `omni-odom.service`: wheel odometry and TF (`odom -> base_link -> laser`).
- `omni-mux.service`: MANUAL/AUTO command mux.
- `teleop-web.service`: Flask teleop UI.
- `omni-lidar.service`: RPLIDAR node, optional for base bringup.
- `omni-slam.service`: slam_toolbox mapping (`map -> odom`), requires LIDAR and odometry.

## Base telemetry smoke test

```bash
./scripts/smoke-test-can.sh 6
```

Expected on the known-good STM32 firmware:

```text
0x190: about 30 frames
0x191: about 240 frames
wheel 0/1/2/3: about 60 each
```

## TODO for fully generic installer

- Keep extending `/etc/omni-robot/omni.env` for new hardware/runtime options.
- Add optional RealSense install path when perception is introduced.
