# CAN protocol contract: Pi5 high level <-> STM32 base

This is the current `omni4` / `stm32_omni_v1` wire-level contract between the Raspberry Pi 5 high-level stack and the STM32 base controller.

Source of truth in code:

- Pi encoder/decoder: `src/omni_pi/protocol.py`
- ROS2 bridge mapping: `src/ros2_ws/src/omni_bridge/omni_bridge/can_bridge_node.py`

If STM32 changes any CAN frame layout, update this document and the Pi decoder/encoder in the same commit.

## Bus format

- CAN bus: SocketCAN `can0`
- Bitrate: `500000` bit/s
- CAN identifiers: standard 11-bit IDs, not extended IDs
- Protocol version byte: `1`
- Expected payload size: 8 bytes per protocol frame
- Byte order for multi-byte integers: little-endian
- Signed integer format: two's complement

The current Pi decoder pads shorter received frames to 8 bytes before decoding, but the STM32 contract should still send DLC 8 for all frames below.

## Frame summary

### Pi5 -> STM32

| CAN ID | Name | Purpose |
| --- | --- | --- |
| `0x101` | `CMD_MOTION` | Command base motion mode and speed |
| `0x102` | `CMD_STOP` | Stop base motion |
| `0x103` | `CMD_PING` | Connectivity check |
| `0x104` | `CMD_STATUS_REQ` | Request current base status |

### STM32 -> Pi5

| CAN ID | Name | Purpose |
| --- | --- | --- |
| `0x181` | `ACK` | Acknowledgement for commands |
| `0x182` | `STATUS` | Reply to `CMD_STATUS_REQ` |
| `0x183` | `PONG` | Reply to `CMD_PING` |
| `0x184` | `TELEMETRY` | Legacy/raw telemetry, currently only forwarded as raw/status text |
| `0x190` | `BASE_STATUS` | Periodic base status, expected about 5 Hz |
| `0x191` | `WHEEL_STATE` | Round-robin wheel telemetry, one wheel per frame |
| `0x192` | `INA228_STATUS` | INA228 power telemetry, expected about 5 Hz |
| `0x1FF` | `ERROR` | Legacy/raw error, currently only forwarded as raw/status text |

## Common fields

`proto_version` must be `1`.

`seq` is an unsigned 8-bit sequence counter. Pi commands use a rolling sequence. STM32 replies should copy the command sequence for direct replies when possible. Periodic telemetry may use STM32's own rolling sequence.

Reserved bytes should be sent as `0`.

## Pi5 -> STM32 payloads

### `CMD_MOTION` `0x101`

Payload:

| Byte | Type | Meaning |
| --- | --- | --- |
| 0 | `uint8` | `proto_version = 1` |
| 1 | `uint8` | `seq` |
| 2 | `uint8` | `motion_mode` |
| 3 | `uint8` | `speed_pct`, range `0..100` |
| 4 | `uint8` | reserved, `0` |
| 5 | `uint8` | reserved, `0` |
| 6 | `uint8` | reserved, `0` |
| 7 | `uint8` | reserved, `0` |

Motion modes:

| Value | Name | Meaning |
| --- | --- | --- |
| `0` | `STOP` | Stop |
| `1` | `FORWARD` | Move forward |
| `2` | `BACKWARD` | Move backward |
| `3` | `LEFT` | Strafe left |
| `4` | `RIGHT` | Strafe right |
| `5` | `ROTATE_CCW` | Rotate counter-clockwise |
| `6` | `ROTATE_CW` | Rotate clockwise |

Example:

```bash
# proto=1, seq=1, FORWARD=1, speed=30%
cansend can0 101#0101011E00000000
```

### `CMD_STOP` `0x102`

Payload:

| Byte | Type | Meaning |
| --- | --- | --- |
| 0 | `uint8` | `proto_version = 1` |
| 1 | `uint8` | `seq` |
| 2-7 | `uint8` | reserved, `0` |

Example:

```bash
cansend can0 102#01EE000000000000
```

Expected reply: `ACK` `0x181`.

### `CMD_PING` `0x103`

Payload:

| Byte | Type | Meaning |
| --- | --- | --- |
| 0 | `uint8` | `proto_version = 1` |
| 1 | `uint8` | `seq` |
| 2-7 | `uint8` | reserved, `0` |

Example:

```bash
cansend can0 103#01EF000000000000
```

Expected reply: `PONG` `0x183`.

### `CMD_STATUS_REQ` `0x104`

Payload:

| Byte | Type | Meaning |
| --- | --- | --- |
| 0 | `uint8` | `proto_version = 1` |
| 1 | `uint8` | `seq` |
| 2-7 | `uint8` | reserved, `0` |

Example:

```bash
cansend can0 104#01F0000000000000
```

Expected reply: `STATUS` `0x182`.

## STM32 -> Pi5 payloads

### `ACK` `0x181`

Pi expects this after commands such as `CMD_STOP`.

Payload:

| Byte | Type | Meaning |
| --- | --- | --- |
| 0 | `uint8` | `proto_version = 1` |
| 1 | `uint8` | `seq`, preferably copied from command |
| 2 | `uint8` | low byte of acknowledged command ID, for example `0x02` for `0x102` |
| 3 | `uint8` | `result_code`, `0` means OK |
| 4 | `uint8` | current `motion_mode` |
| 5 | `uint8` | current `speed_pct` |
| 6 | `uint8` | reserved, `0` |
| 7 | `uint8` | reserved, `0` |

Pi publishes this to `/omni/status_text` and `/omni/status_json`.

### `STATUS` `0x182`

Pi expects this as a reply to `CMD_STATUS_REQ`.

Payload:

| Byte | Type | Meaning |
| --- | --- | --- |
| 0 | `uint8` | `proto_version = 1` |
| 1 | `uint8` | `seq`, preferably copied from request |
| 2 | `uint8` | current `motion_mode` |
| 3 | `uint8` | current `speed_pct` |
| 4 | `uint8` | reserved, `0` |
| 5 | `uint8` | reserved, `0` |
| 6 | `uint8` | reserved, `0` |
| 7 | `uint8` | reserved, `0` |

Pi publishes this to `/omni/status_text` and `/omni/status_json`.

### `PONG` `0x183`

Pi expects this as a reply to `CMD_PING`.

Payload:

| Byte | Type | Meaning |
| --- | --- | --- |
| 0 | `uint8` | `proto_version = 1` |
| 1 | `uint8` | `seq`, preferably copied from request |
| 2-7 | `uint8` | reserved, `0` |

Pi publishes this to `/omni/status_text` and `/omni/status_json`.

### `BASE_STATUS` `0x190`

Periodic base status. Expected rate: about 5 Hz.

Payload:

| Byte | Type | Meaning |
| --- | --- | --- |
| 0 | `uint8` | `proto_version = 1` |
| 1 | `uint8` | STM32 rolling sequence counter |
| 2 | `uint8` | current `motion_mode` |
| 3 | `uint8` | current `speed_pct` |
| 4 | `uint8` | `wheel_count`, currently `4` |
| 5 | `uint8` | `status_flags` |
| 6 | `uint8` | `error_flags` |
| 7 | `uint8` | reserved, `0` |

`status_flags`:

| Bit | Name | Meaning |
| --- | --- | --- |
| 0 | `encoders_ready` | Encoder telemetry is ready |
| 1 | `encoder_debug_enabled` | STM32 encoder/CAN debug mode is enabled |

`error_flags`:

| Bit | Name | Meaning |
| --- | --- | --- |
| 0 | `can_tx_errors_seen` | STM32 saw at least one CAN TX error since reset/clear |
| 1 | `can_rx_errors_seen` | STM32 saw at least one CAN RX error since reset/clear |

Pi publishes decoded JSON to `/omni/base_status`.

Example raw frame:

```text
can0  190   [8]  01 A1 00 28 04 01 01 00
```

Decoded meaning:

- protocol v1
- seq `0xA1`
- mode `STOP`
- speed `40%`
- 4 wheels
- encoders ready
- CAN TX error was seen at least once

### `WHEEL_STATE` `0x191`

Round-robin wheel telemetry. STM32 sends one wheel per frame:

```text
wheel 0 -> wheel 1 -> wheel 2 -> wheel 3 -> repeat
```

Expected scheduling: one `0x191` frame about every 25 ms, so about 40 total wheel frames per second and about 10 frames per second per wheel.

Payload:

| Byte | Type | Meaning |
| --- | --- | --- |
| 0 | `uint8` | `wheel_index`, currently `0..3` |
| 1 | `uint8` | `flags` |
| 2-3 | `int16_le` | `delta_ticks` since previous wheel update |
| 4-7 | `int32_le` | `speed_ticks_per_sec` |

`flags`:

| Bit | Name | Meaning |
| --- | --- | --- |
| 0 | `present` | Wheel/encoder exists and is available |
| 1 | `updated` | This frame contains a fresh update |
| 2 | `moving` | STM32 considers this wheel moving |

Current wheel mapping:

| Index | Name |
| --- | --- |
| `0` | `front_left` |
| `1` | `front_right` |
| `2` | `rear_left` |
| `3` | `rear_right` |

Direction convention:

- For `FORWARD`, all four wheels should report positive `delta_ticks` and positive `speed_ticks_per_sec`.
- After `STOP`, `speed_ticks_per_sec` should fall back toward `0`.

Pi aggregates the last known state of each wheel and publishes JSON to `/omni/wheel_states`.

### `INA228_STATUS` `0x192`

Periodic power telemetry from the STM32-side INA228 reader. Expected rate: about 5 Hz.

Payload:

| Byte | Type | Meaning |
| --- | --- | --- |
| 0 | `uint8` | `proto_version = 1` |
| 1 | `uint8` | STM32 rolling sequence counter |
| 2-3 | `uint16_le` | `bus_voltage_mv` |
| 4-5 | `int16_le` | `current_ma` |
| 6 | `uint8` | `power_w`, saturated `0..255`; Pi also calculates precise `V * A` |
| 7 | `uint8` | `flags` |

`flags`:

| Bit | Name | Meaning |
| --- | --- | --- |
| 0 | `ina228_present` | INA228 was detected by STM32 |
| 1 | `measurement_valid` | Voltage/current measurement is valid |
| 2 | `over_current` | Over-current condition |
| 3 | `over_voltage` | Over-voltage condition |
| 4 | `under_voltage` | Under-voltage condition |
| 5 | `sensor_error` | STM32 could not read INA228 over I2C |
| 6 | `power_saturated` | Byte 6 saturated at `255 W` |
| 7 | reserved | Reserved for future use |

Normal flags value is `0x03`: `ina228_present` + `measurement_valid`.

If flags are `0x20`, STM32 firmware and CAN are alive, but INA228 is not being read successfully over I2C. Check INA228 power, GND, SDA/SCL, I2C address `0x40`, and STM32 I2C1 wiring/config.

Pi publishes decoded JSON to `/omni/power_status`.

Example raw frame:

```text
can0  192   [8]  01 28 98 2D D4 00 02 03
```

Decoded meaning:

- protocol v1
- seq `0x28`
- bus voltage `0x2D98` = `11672 mV` = `11.672 V`
- current `0x00D4` = `212 mA` = `0.212 A`
- raw power byte `2 W`
- Pi calculated power about `2.474 W`
- flags `0x03`: INA228 present and measurement valid

### `TELEMETRY` `0x184` and `ERROR` `0x1FF`

These frames are currently treated as legacy/raw frames by the Pi bridge.

The Pi does not decode a stable field layout for them yet. It publishes the raw bytes to:

- `/omni/rx_raw`
- `/omni/status_text`
- `/omni/status_json`

If either frame becomes part of the stable STM32 contract, define its byte layout here and update the Pi decoder.

## ROS topics produced by Pi from STM32 CAN

| CAN input | ROS topic | Type | Notes |
| --- | --- | --- | --- |
| any received frame | `/omni/rx_raw` | `std_msgs/String` JSON | Raw CAN ID, DLC, padded data bytes |
| `0x181`, `0x182`, `0x183`, `0x184`, `0x1FF` | `/omni/status_text` | `std_msgs/String` | Human-readable status |
| `0x181`, `0x182`, `0x183`, `0x184`, `0x1FF` | `/omni/status_json` | `std_msgs/String` JSON | Structured status |
| `0x190` | `/omni/base_status` | `std_msgs/String` JSON | Decoded base state |
| `0x191` | `/omni/wheel_states` | `std_msgs/String` JSON | Aggregated wheel states |
| `0x192` | `/omni/power_status` | `std_msgs/String` JSON | Decoded INA228 power telemetry |

## ROS commands accepted by Pi before CAN encoding

The STM32 does not receive ROS directly, but these are the high-level inputs that the Pi bridge converts to CAN:

- `/omni/motion_cmd`, `std_msgs/String` JSON:

```json
{"mode":"FORWARD","speed_pct":30}
```

- `/cmd_vel`, `geometry_msgs/Twist`:
  - positive `linear.x` -> `FORWARD`
  - negative `linear.x` -> `BACKWARD`
  - positive `angular.z` -> `ROTATE_CCW`
  - negative `angular.z` -> `ROTATE_CW`

The current bridge does not map `/cmd_vel` sideways motion to `LEFT`/`RIGHT`; side motion is available through `/omni/motion_cmd`.

## Smoke tests

Listen to STM32 traffic:

```bash
candump can0,181:7FF,182:7FF,183:7FF,190:7FF,191:7FF,192:7FF
```

Direct command checks:

```bash
cansend can0 103#01EF000000000000
cansend can0 104#01F0000000000000
cansend can0 102#01EE000000000000
```

Expected replies:

- `0x103` -> `0x183 PONG`
- `0x104` -> `0x182 STATUS`
- `0x102` -> `0x181 ACK`

Telemetry rate check over 6 seconds on known-good STM32 firmware:

- `0x190`: about 30 frames
- `0x191`: about 240 frames
- `0x192`: about 30 frames
- `wheel_index 0/1/2/3`: about 60 frames each

Known-good STM32 reference:

```text
2e677ab Fix CAN wheel telemetry scheduling
```

Observed after that STM32 firmware:

- `0x190`: about 30 frames per 6 seconds
- `0x191`: about 231-240 frames per 6 seconds
- wheel indices distributed evenly, about 57-60 each
- `FORWARD 30%`: all 4 wheels positive, around 2230-2280 ticks/sec on the lifted robot
