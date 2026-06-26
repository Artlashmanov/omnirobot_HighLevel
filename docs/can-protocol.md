# CAN protocol

Bus: SocketCAN `can0`, standard 11-bit identifiers, bitrate `500000`.

## Pi5 -> STM32

| CAN ID | Name | Payload |
| --- | --- | --- |
| `0x101` | `CMD_MOTION` | byte0 proto, byte1 seq, byte2 motion mode, byte3 speed_pct |
| `0x102` | `CMD_STOP` | byte0 proto, byte1 seq |
| `0x103` | `CMD_PING` | byte0 proto, byte1 seq |
| `0x104` | `CMD_STATUS_REQ` | byte0 proto, byte1 seq |

Motion modes:

| Value | Name |
| --- | --- |
| 0 | STOP |
| 1 | FORWARD |
| 2 | BACKWARD |
| 3 | LEFT |
| 4 | RIGHT |
| 5 | ROTATE_CCW |
| 6 | ROTATE_CW |

## STM32 -> Pi5

| CAN ID | Name | Notes |
| --- | --- | --- |
| `0x181` | `ACK` | Response to commands |
| `0x182` | `STATUS` | Response to status request |
| `0x183` | `PONG` | Response to ping |
| `0x190` | `BASE_STATUS` | Periodic, about 5 Hz |
| `0x191` | `WHEEL_STATE` | Round-robin wheel telemetry, one wheel every ~25 ms |

## BASE_STATUS `0x190`

Payload:

| Byte | Meaning |
| --- | --- |
| 0 | protocol_version |
| 1 | sequence counter |
| 2 | motion_mode |
| 3 | speed_pct |
| 4 | wheel_count |
| 5 | status_flags |
| 6 | error_flags |
| 7 | reserved |

`status_flags`:

- bit0: encoders_ready
- bit1: encoder_debug_enabled

`error_flags`:

- bit0: CAN TX errors happened
- bit1: CAN RX errors happened

## WHEEL_STATE `0x191`

Payload:

| Byte | Meaning |
| --- | --- |
| 0 | wheel_index |
| 1 | flags |
| 2-3 | delta_ticks, int16 little-endian |
| 4-7 | speed_ticks_per_sec, int32 little-endian |

`flags`:

- bit0: present
- bit1: updated
- bit2: moving

Expected current robot mapping:

| Index | Name |
| --- | --- |
| 0 | front_left |
| 1 | front_right |
| 2 | rear_left |
| 3 | rear_right |

## Known-good STM32 firmware

`2e677ab Fix CAN wheel telemetry scheduling`

Observed after flashing:

- `0x190`: about 30 frames per 6 seconds;
- `0x191`: about 231-240 frames per 6 seconds;
- wheel indices distributed evenly, about 57-60 each;
- `FORWARD 30%`: all 4 wheels positive, around 2230-2280 ticks/sec.
