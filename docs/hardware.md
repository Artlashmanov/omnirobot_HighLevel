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

Current service target is Slamtec RPLIDAR C1 through `sllidar_ros2`. Prefer a stable udev symlink `/dev/rplidar` before relying on the service in navigation.

## RealSense D415

D415 is not required for the current base/teleop/encoder telemetry layer. Connect it later for perception/depth/navigation work. When connected, check USB bandwidth and power budget before enabling camera services at boot.
