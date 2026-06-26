# Pi5 recovery / reinstall notes

This document describes the intended direction for rebuilding a robot Pi5.

## Current assumption

The current live robot uses:

```bash
/home/noob/omni-pi
```

Most existing service files still assume this path and user `noob`.

## Fresh install outline

1. Install Ubuntu 24.04 for Raspberry Pi 5.
2. Configure network and SSH.
3. Clone this repository to `/home/noob/omni-pi`.
4. Install ROS2 Jazzy packages.
5. Build ROS workspace.
6. Install systemd services.
7. Verify CAN, ROS graph, web UI, and telemetry.

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
- `omni-mux.service`: MANUAL/AUTO command mux.
- `teleop-web.service`: Flask teleop UI.
- `omni-lidar.service`: RPLIDAR node, optional for base bringup.

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

- Remove hardcoded `/home/noob/omni-pi` from service and launch files.
- Source `/etc/omni-robot/omni.env` from services.
- Use `ros2 run omni_bridge can_bridge` instead of running the build artifact directly.
- Add udev rule install for `/dev/rplidar`.
- Add optional RealSense install path when perception is introduced.
