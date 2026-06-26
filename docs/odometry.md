# Odometry and TF

The mapping/navigation stack needs a ROS frame tree before SLAM can build a usable map.
The current first layer is `omni_odometry`, launched by `omni-odom.service`.

Published interfaces:

- `/odom` (`nav_msgs/msg/Odometry`)
- `/tf`: `odom -> base_link`
- `/tf_static`: `base_link -> laser`

Input topics:

- `/omni/base_status` (`std_msgs/msg/String` JSON): active STM32 motion mode.
- `/omni/wheel_states` (`std_msgs/msg/String` JSON): round-robin encoder deltas from STM32.

## Current odometry model

The STM32 currently exposes a discrete motion mode (`FORWARD`, `BACKWARD`, `LEFT`, `RIGHT`, `ROTATE_CCW`, `ROTATE_CW`) and wheel encoder deltas. Until full omni-wheel kinematics are calibrated, the node projects encoder magnitude through the active motion mode:

- `FORWARD/BACKWARD` -> robot X
- `LEFT/RIGHT` -> robot Y
- `ROTATE_CCW/ROTATE_CW` -> robot yaw

This is good enough to bring up the ROS frame tree and start SLAM experiments, but the scale must be calibrated before relying on map accuracy.

## Calibration parameters

Configured in `src/ros2_ws/src/omni_bridge/config/omni_bridge.params.yaml` under `omni_odometry`:

- `meters_per_tick`: linear distance per averaged encoder tick.
- `radians_per_tick`: yaw angle per averaged encoder tick.
- `laser_xyz`: LIDAR position relative to `base_link`, meters.
- `laser_rpy`: LIDAR orientation relative to `base_link`, radians.

Initial defaults are placeholders. Calibrate them by commanding measured motion:

1. Drive forward a known distance, for example 1 meter.
2. Compare `/odom.pose.pose.position.x` with the measured distance.
3. Adjust `meters_per_tick` proportionally.
4. Rotate the robot a known angle, for example 360 degrees.
5. Compare `/odom.pose.pose.orientation` / yaw with the measured angle.
6. Adjust `radians_per_tick` proportionally.

## Why this comes before the map

SLAM needs both laser scans and the transform tree:

```text
map -> odom -> base_link -> laser
```

`map -> odom` will be produced later by SLAM/localization. This project now provides the robot-side part:

```text
odom -> base_link -> laser
```
