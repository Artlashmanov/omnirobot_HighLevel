from enum import IntEnum
import can

PROTO_VERSION = 1

ID_CMD_MOTION = 0x101
ID_CMD_STOP = 0x102
ID_CMD_PING = 0x103
ID_CMD_STATUS_REQ = 0x104

ID_EVT_ACK = 0x181
ID_EVT_STATUS = 0x182
ID_EVT_PONG = 0x183
ID_EVT_TELEMETRY = 0x184
ID_EVT_BASE_STATUS = 0x190
ID_EVT_WHEEL_STATE = 0x191
ID_EVT_ERROR = 0x1FF

BASE_STATUS_FLAG_ENCODERS_READY = 1 << 0
BASE_STATUS_FLAG_ENCODER_DEBUG_ENABLED = 1 << 1

BASE_ERROR_FLAG_CAN_TX = 1 << 0
BASE_ERROR_FLAG_CAN_RX = 1 << 1

WHEEL_STATE_FLAG_PRESENT = 1 << 0
WHEEL_STATE_FLAG_UPDATED = 1 << 1
WHEEL_STATE_FLAG_MOVING = 1 << 2


class MotionMode(IntEnum):
    STOP = 0
    FORWARD = 1
    BACKWARD = 2
    LEFT = 3
    RIGHT = 4
    ROTATE_CCW = 5
    ROTATE_CW = 6


def make_ping(seq: int) -> can.Message:
    return can.Message(
        arbitration_id=ID_CMD_PING,
        is_extended_id=False,
        data=bytes([PROTO_VERSION, seq & 0xFF, 0, 0, 0, 0, 0, 0]),
    )


def make_stop(seq: int) -> can.Message:
    return can.Message(
        arbitration_id=ID_CMD_STOP,
        is_extended_id=False,
        data=bytes([PROTO_VERSION, seq & 0xFF, 0, 0, 0, 0, 0, 0]),
    )


def make_motion(seq: int, mode: MotionMode, speed_pct: int) -> can.Message:
    if not 0 <= speed_pct <= 100:
        raise ValueError("speed_pct must be 0..100")
    return can.Message(
        arbitration_id=ID_CMD_MOTION,
        is_extended_id=False,
        data=bytes([PROTO_VERSION, seq & 0xFF, int(mode), speed_pct, 0, 0, 0, 0]),
    )


def make_status_req(seq: int) -> can.Message:
    return can.Message(
        arbitration_id=ID_CMD_STATUS_REQ,
        is_extended_id=False,
        data=bytes([PROTO_VERSION, seq & 0xFF, 0, 0, 0, 0, 0, 0]),
    )


def motion_mode_name(value: int) -> str:
    try:
        return MotionMode(value).name
    except ValueError:
        return f"UNKNOWN_{value}"


def _int16_le(lo: int, hi: int) -> int:
    value = (lo & 0xFF) | ((hi & 0xFF) << 8)
    if value >= 0x8000:
        value -= 0x10000
    return value


def _int32_le(b0: int, b1: int, b2: int, b3: int) -> int:
    value = (
        (b0 & 0xFF)
        | ((b1 & 0xFF) << 8)
        | ((b2 & 0xFF) << 16)
        | ((b3 & 0xFF) << 24)
    )
    if value >= 0x80000000:
        value -= 0x100000000
    return value


def decode_base_status_data(data: list[int]) -> dict:
    status_flags = data[5]
    error_flags = data[6]

    return {
        "type": "base_status",
        "proto_version": data[0],
        "seq": data[1],
        "motion_mode": data[2],
        "motion_mode_name": motion_mode_name(data[2]),
        "speed_pct": data[3],
        "wheel_count": data[4],
        "status_flags": status_flags,
        "encoders_ready": bool(status_flags & BASE_STATUS_FLAG_ENCODERS_READY),
        "encoder_debug_enabled": bool(status_flags & BASE_STATUS_FLAG_ENCODER_DEBUG_ENABLED),
        "error_flags": error_flags,
        "can_tx_errors_seen": bool(error_flags & BASE_ERROR_FLAG_CAN_TX),
        "can_rx_errors_seen": bool(error_flags & BASE_ERROR_FLAG_CAN_RX),
        "reserved": data[7],
    }


def decode_wheel_state_data(data: list[int]) -> dict:
    flags = data[1]

    return {
        "type": "wheel_state",
        "wheel_index": data[0],
        "flags": flags,
        "present": bool(flags & WHEEL_STATE_FLAG_PRESENT),
        "updated": bool(flags & WHEEL_STATE_FLAG_UPDATED),
        "moving": bool(flags & WHEEL_STATE_FLAG_MOVING),
        "delta_ticks": _int16_le(data[2], data[3]),
        "speed_ticks_per_sec": _int32_le(data[4], data[5], data[6], data[7]),
    }


def decode_message(msg: can.Message) -> dict:
    data = list(msg.data)
    while len(data) < 8:
        data.append(0)

    return {
        "can_id": hex(msg.arbitration_id),
        "dlc": msg.dlc,
        "data": data[:8],
    }
