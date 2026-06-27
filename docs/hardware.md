# Hardware notes

## Current Pi5 system

- Raspberry Pi 5, Ubuntu 24.04 / ROS2 Jazzy.
- CAN adapter: `gs_usb` / OpenMoko Geschwister Schneider CAN adapter.
- CAN interface: `can0`, bitrate `500000`.
- Robot base controller: STM32 NUCLEO-G474RE.

## STM32 wheel mapping

| Wheel | Name | Encoder pins |
| --- | --- | --- |
| 0 | front_left | PA0 / PA1, TIM5 |
| 1 | front_right | PC6 / PC7, TIM8 |
| 2 | rear_left | PB6 / PB7, TIM4 |
| 3 | rear_right | PA6 / PA7, TIM3 |

Encoder signs are calibrated in STM32 so `FORWARD` produces positive `delta_ticks` and positive `speed_ticks_per_sec` for all wheels.

## LIDAR

The high-level base/CAN work does not require LIDAR to be connected.

Current service target is Slamtec RPLIDAR C1 through `sllidar_ros2`. The driver source is pinned in `src/ros2_ws/omni.repos` and fetched automatically by `install/fetch-ros-deps.sh` during a fresh install.

Runtime defaults:

- USB bridge: Silicon Labs CP2102N / CP210x, USB ID `10c4:ea60`.
- Preferred device path: `/dev/rplidar`.
- Fallback device path: `/dev/ttyUSB0`.
- Baudrate: `460800`.
- ROS frame: `laser`.
- ROS topic: `/scan`.
- Scan mode: `Standard`.

Install the stable device path through:

```bash
sudo ./install/install-udev-rules.sh
```

If the LIDAR is connected during install, the generated rule is pinned to that device's `ID_SERIAL_SHORT` and creates `/dev/rplidar` with group `dialout`, mode `0660`. If the LIDAR is not connected, the installer writes a generic CP210x fallback rule; re-run it later with the LIDAR connected to pin the rule to the exact sensor.

Check the live state with:

```bash
ls -l /dev/rplidar /dev/ttyUSB0
systemctl status omni-lidar --no-pager
ros2 topic echo --once /scan
```

On the current known-good robot, the RPLIDAR C1 reports health OK and publishes `sensor_msgs/msg/LaserScan` on `/scan`.

## TF-Luna front range sensor

The robot has a Benewake TF-Luna mounted on the front nose under the RealSense D415. It is connected directly to Raspberry Pi 5 UART0 on GPIO14/15 and publishes a standard ROS range topic.

Runtime defaults:

- Device: `/dev/ttyAMA0`.
- Baudrate: `115200`.
- ROS frame: `tf_luna_front`.
- ROS topic: `/range/front` (`sensor_msgs/msg/Range`).
- JSON status topic: `/sensors/tf_luna/front`.
- systemd service: `omni-tfluna.service`.

Pi 5 UART0 must be enabled in `/boot/firmware/config.txt`:

```text
enable_uart=1
dtoverlay=uart0-pi5
```

Fresh installs run `install/configure-pi-hardware.sh`, which ensures these lines are present and prints whether a reboot is required.

See `docs/tf-luna.md` for wiring and calibration notes.

## RealSense D415

D415 is not required for the current base/teleop/encoder telemetry layer. Connect it later for perception/depth/navigation work. When connected, check USB bandwidth and power budget before enabling camera services at boot.
