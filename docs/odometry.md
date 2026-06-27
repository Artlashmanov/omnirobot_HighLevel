# Odometry and TF

The mapping/navigation stack needs a ROS frame tree before SLAM can build a usable map.
The current first layer is `omni_odometry`, launched by `omni-odom.service`.

Published interfaces:

- `/odom` (`nav_msgs/msg/Odometry`)
- `/tf`: `odom -> base_link`
- `/tf_static`: `base_link -> laser`

Input topics:

- `/omni/base_status` (`std_msgs/msg/String` JSON): active STM32 motion mode fallback/status.
- `/omni/motion_cmd` (`std_msgs/msg/String` JSON): commanded discrete motion mode, used for low-latency odometry direction.
- `/cmd_vel` (`geometry_msgs/msg/Twist`): velocity command fallback for ROS-native control.
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

Current linear calibration:

| Date | Test | Odom total | Real total | Scale | `meters_per_tick` |
| --- | --- | ---: | ---: | ---: | ---: |
| 2026-06-27 | 6 x `FORWARD 40%` for 3 seconds | 3.723725 m | 4.553 m | 1.222700387 | 0.00012227 |
| 2026-06-27 | 3 x `FORWARD 30%` for 1 second | 0.701891 m | 0.783 m | 1.115557932 | 0.00013640 |
| 2026-06-27 | 3 robust `FORWARD 30%` for 1 second | 0.547032 m | 0.567 m | 1.036501891 | 0.00014138 |
| 2026-06-27 | 3 fixed `FORWARD 30%` for 0.7 seconds, measured by TF-Luna | 0.402862 m | 0.420000 m | 1.042540157 | 0.00014739 |
| 2026-06-27 | 1 fixed `FORWARD 30%` for 0.7 seconds, TF-Luna control after correction | 0.140794 m | 0.140000 m | 0.994358776 | 0.00014656 |
| 2026-06-27 | 3 fixed `FORWARD 30%` for 0.7 seconds, TF-Luna control after correction | 0.417623 m | 0.445000 m | 1.065555322 | 0.00015617 |
| 2026-06-27 | 3 fixed `FORWARD 30%` for 0.7 seconds, TF-Luna final control | 0.449887 m | 0.445000 m | 0.989138120 | kept 0.00015617 |

Current lateral validation:

| Date | Test | Odom total | Real total | Error | Decision |
| --- | --- | ---: | ---: | ---: | --- |
| 2026-06-28 | 3 x `LEFT/RIGHT 35%` for 0.5 seconds on smooth floor, measured by side-mounted TF-Luna | 0.589229 m | 0.595000 m | -0.97% | kept `meters_per_tick = 0.00015617` |

Current rotation calibration:

| Date | Test | Odom yaw | Real yaw | Scale | `radians_per_tick` |
| --- | --- | ---: | ---: | ---: | ---: |
| 2026-06-28 | `ROTATE_CW 35%` for 1.0 second | 9.513 deg | ~35 deg | ~3.679 | rough short probe |
| 2026-06-28 | `ROTATE_CW 35%` for 5.0 seconds | 34.658 deg | ~200 deg | 5.770491 | 0.00057705 |
| 2026-06-28 | `ROTATE_CW 35%` for 2.3 seconds control | 138.796 deg | ~170 deg | 1.224815 | 0.00070678 |
| 2026-06-28 | `ROTATE_CW 35%` for 2.44 seconds control | 176.469 deg | ~170 deg | 0.963340 | 0.00068087 |
| 2026-06-28 | `ROTATE_CW 35%` for 5.31 seconds 360-control | 366.030 deg | ~341 deg | 0.931617 | 0.00063431 |
| 2026-06-28 | `ROTATE_CW 35%` for 5.51 seconds 360-control | 367.222 deg | ~360 deg | 0.980334 | 0.00062184 |
| 2026-06-28 | `ROTATE_CW 35%` for 2.71 seconds 180-control | 172.043 deg | ~180 deg | timing validation | kept 0.00062184 |

Practical rotation timing on smooth floor at 35% speed:

| Motion | Duration | Notes |
| --- | ---: | --- |
| `ROTATE_CW 35%` | ~5.51 s | approximately 360 degrees, physical error about +/-1 degree in the best control run |
| `ROTATE_CW 35%` | ~2.71 s | approximately 180 degrees, physical error about +/-1 degree in the best control run |

Continue calibration by commanding measured motion:

1. Drive forward/lateral a known distance or timed run.
2. Compare `/odom.pose.pose.position.x/y` with the measured distance.
3. Adjust `meters_per_tick` proportionally only if both forward and lateral agree.
4. Rotate the robot a known angle, for example 180 or 360 degrees.
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
