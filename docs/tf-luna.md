# TF-Luna front range sensor

The front TF-Luna is mounted under the Intel RealSense D415 and faces forward.
It is used as a simple close-range safety and calibration distance sensor.

## Electrical connection

The sensor is connected to Raspberry Pi 5 UART0 on the 40-pin header:

| TF-Luna pin | Raspberry Pi 5 pin |
| --- | --- |
| `5V` | physical pin 2 or 4 |
| `GND` | physical pin 6 |
| `SCL / TxD` | physical pin 10, GPIO15/RX |
| `SDA / RxD` | physical pin 8, GPIO14/TX |
| `CFG` | not connected |
| `MUX` | not connected |

Do not move these wires while the Pi is powered. GPIO pins are 3.3 V logic.

## Raspberry Pi UART setup

Pi 5 needs UART0 enabled on GPIO14/15:

```text
enable_uart=1
dtoverlay=uart0-pi5
```

The live robot has this in `/boot/firmware/config.txt`.

## ROS interfaces

The `tf_luna` node publishes:

- `/range/front` (`sensor_msgs/msg/Range`)
- `/sensors/tf_luna/front` (`std_msgs/msg/String` JSON)

Default parameters are in `src/ros2_ws/src/omni_bridge/config/omni_bridge.params.yaml`
under `tf_luna_front`.

## systemd

The service is `omni-tfluna.service` and is started through:

```bash
tools/run_tf_luna.sh
```

Quick check on the robot:

```bash
systemctl status omni-tfluna.service
ros2 topic echo /range/front --once
ros2 topic echo /sensors/tf_luna/front --once
```

## Notes for calibration

TF-Luna measures distance to the object in front of the robot, not robot travel
directly. For encoder calibration, place the robot square to a flat wall:

```text
real_travel = distance_before - distance_after
```

This is most useful for 50-100 cm runs. Very short runs are dominated by
manual placement, wall angle, and sensor noise.
