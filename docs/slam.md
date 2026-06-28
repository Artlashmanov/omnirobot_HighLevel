# LIDAR SLAM and map runtime

The mapping layer is a normal ROS2 stack, not a side-channel integration.

Runtime graph:

```text
RPLIDAR C1 -> /scan  (sensor_msgs/msg/LaserScan, frame_id=laser)
omni_odometry -> /odom
omni_odometry -> /tf        odom -> base_link
omni_odometry -> /tf_static base_link -> laser
slam_toolbox -> /map
slam_toolbox -> /tf         map -> odom
```

The full transform tree during mapping is:

```text
map -> odom -> base_link -> laser
```

## Why the project uses a custom slam_toolbox config

The default `slam_toolbox` online config uses `base_frame: base_footprint`.
This robot currently publishes `odom -> base_link` and `base_link -> laser`, so
SLAM must use:

```yaml
base_frame: base_link
odom_frame: odom
map_frame: map
scan_topic: /scan
```

Without this, `slam_toolbox` starts but logs `Failed to compute odom pose` and
cannot build a usable map.

## Runtime files

- SLAM params: `config/slam/slam_toolbox_online_async.yaml`
- Run script: `tools/run_slam.sh`
- systemd service: `services/omni-slam.service`
- Runtime env keys:
  - `OMNI_ENABLE_SLAM=1`
  - `SLAM_PARAMS=${OMNI_HOME}/config/slam/slam_toolbox_online_async.yaml`
  - `SLAM_USE_SIM_TIME=false`

## Manual checks

```bash
source /opt/ros/jazzy/setup.bash
source /home/noob/omni-pi/src/ros2_ws/install/setup.bash

ros2 topic echo --once /scan
ros2 topic echo --once /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
```

Start mapping:

```bash
sudo systemctl start omni-lidar omni-slam
```

Check map output:

```bash
ros2 topic echo --once /map
ros2 run tf2_ros tf2_echo map odom
```

## Current status

On the known-good robot, `omni-lidar.service` publishes `/scan` at about 10 Hz.
A temporary `slam_toolbox` launch using `base_frame=base_link` successfully
published `/map` and `map -> odom`, confirming that the ROS graph is compatible.

## Map persistence

The live `/map` topic is not enough for a reusable robot map: it exists while the SLAM process is running. Persistent maps are saved under `OMNI_MAPS_DIR`, which defaults to `${OMNI_HOME}/maps`.

Save the current SLAM map from the command line:

```bash
/home/noob/omni-pi/tools/save_map.sh apartment_001
```

The web cockpit also exposes a `Save map` button. It calls the same script and stores the result as:

```text
maps/<map_name>/
  map.yaml          # occupancy grid metadata
  map.pgm           # 2D occupancy grid image
  metadata.json     # robot/sensor/runtime metadata
  slam_posegraph*   # optional, if slam_toolbox serialization is available
```

`map.yaml + map.pgm` are the navigation map. The optional `slam_posegraph*` artifact is useful later when we want to continue or refine a mapping session instead of only using the flattened occupancy grid.
