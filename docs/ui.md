# Web UI telemetry

The teleop web UI is also the first lightweight operator dashboard. It keeps
manual control working and adds read-only telemetry panels.

## Fast telemetry

`/api/state` is intentionally small and is polled by the browser about every
500 ms. It contains cached values already reduced by `teleop_web_node`:

- `/omni/base_status` — STM32 base state, motion mode, speed, CAN error flags.
- `/omni/wheel_states` — last known encoder telemetry for all wheels.
- `/range/front` and `/sensors/tf_luna/front` — front TF-Luna distance/status.
- `/scan` — reduced LIDAR summary only: nearest/front/left/right distances.
- `/sensors/ina228` or `/omni/power_status` — future INA228 power JSON topic.

The fast state API does not stream raw laser scans to the browser.

## Live LIDAR preview

`/api/scan` is separate from `/api/state`. It returns a downsampled point preview
from `/scan` so the operator can see what the robot sees right now. The browser
polls it about every 500 ms. The maximum point count is controlled by
`TELEOP_SCAN_PREVIEW_MAX_POINTS` and defaults to `360`.

## Map telemetry

`/api/map` is separate from `/api/state`. It returns a downsampled cached
`nav_msgs/OccupancyGrid` preview from `/map`, and the browser polls it about
every 2 seconds when auto-refresh is enabled.

This keeps the control/status channel responsive even when the map grows.
The maximum preview dimension is controlled by `TELEOP_MAP_MAX_DIM` and defaults
to `220` cells.

The UI also displays the map timestamp, age, and content hash. If the timestamp
changes but the hash stays the same, SLAM is publishing a fresh map message but
the actual occupancy cells did not change. This is expected when the robot is
stationary or sees the same area from the same pose.

## INA228 status

There is no live INA228 ROS topic on the current Pi5 system yet. The UI already
subscribes to the future JSON topics below and will show data automatically once
one of them is published:

- `/sensors/ina228`
- `/omni/power_status`

Recommended future JSON fields:

```json
{
  "type": "ina228",
  "bus_voltage_v": 12.4,
  "current_a": 1.2,
  "power_w": 14.9,
  "energy_j": 1234.5,
  "temperature_c": 36.0
}
```

## Cockpit layout

The UI now uses a cockpit-style operator layout inspired by an inside-rover
view:

- the top/center camera area is reserved for the future RealSense D415 feed;
- the right `MAP` panel switches between `SLAM` map and `LIVE` LIDAR preview;
- the bottom row uses equal-height instruments for speed, motors, INA228 power,
  TF-Luna, and LIDAR sectors;
- missing telemetry is shown as `No data` or `Waiting for ...`;
- warnings are grouped behind the yellow dashboard-style warning indicator.
