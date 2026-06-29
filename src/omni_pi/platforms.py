from __future__ import annotations

from dataclasses import dataclass, field
import json
import os
from pathlib import Path
from typing import Any

DEFAULT_MOTION_MODES = ["STOP", "FORWARD", "BACKWARD", "LEFT", "RIGHT", "ROTATE_CCW", "ROTATE_CW"]
DEFAULT_WHEEL_NAMES = ["front_left", "front_right", "rear_left", "rear_right"]
DEFAULT_TELEOP_BUTTON_MODES = {
    "forward": "FORWARD",
    "backward": "BACKWARD",
    "left": "LEFT",
    "right": "RIGHT",
    "rotate_ccw": "ROTATE_CCW",
    "rotate_cw": "ROTATE_CW",
}
DEFAULT_CAN_MOTION_MODES = {
    "STOP": "STOP",
    "FORWARD": "FORWARD",
    "BACKWARD": "BACKWARD",
    "LEFT": "RIGHT",
    "RIGHT": "LEFT",
    "ROTATE_CCW": "ROTATE_CW",
    "ROTATE_CW": "ROTATE_CCW",
}


@dataclass(frozen=True)
class PlatformProfile:
    name: str
    display_name: str
    motion_interface: str
    can_protocol: str
    motion_modes: tuple[str, ...]
    wheel_names: tuple[str, ...]
    cmd_vel: dict[str, Any] = field(default_factory=dict)
    teleop_button_modes: dict[str, str] = field(default_factory=dict)
    can_motion_modes: dict[str, str] = field(default_factory=dict)
    config_path: str | None = None

    def supports_motion_mode(self, mode: str) -> bool:
        return mode.strip().upper() in self.motion_modes

    def stop_command(self) -> dict[str, int | str]:
        return {"mode": "STOP", "speed_pct": 0}

    def as_public_dict(self) -> dict[str, Any]:
        return {
            "platform": self.name,
            "display_name": self.display_name,
            "motion_interface": self.motion_interface,
            "can_protocol": self.can_protocol,
            "motion_modes": list(self.motion_modes),
            "wheel_names": list(self.wheel_names),
            "cmd_vel": dict(self.cmd_vel),
            "teleop_button_modes": dict(self.teleop_button_modes),
            "can_motion_modes": dict(self.can_motion_modes),
            "config_path": self.config_path,
        }


def builtin_omni4_profile(config_path: str | None = None) -> PlatformProfile:
    return PlatformProfile(
        name="omni4",
        display_name="4x4 omnidirectional base",
        motion_interface="discrete_mode_speed_v1",
        can_protocol="stm32_omni_v1",
        motion_modes=tuple(DEFAULT_MOTION_MODES),
        wheel_names=tuple(DEFAULT_WHEEL_NAMES),
        cmd_vel={
            "linear_x_positive": "FORWARD",
            "linear_x_negative": "BACKWARD",
            "angular_z_positive": "ROTATE_CCW",
            "angular_z_negative": "ROTATE_CW",
            "linear_y_supported": False,
        },
        teleop_button_modes=dict(DEFAULT_TELEOP_BUTTON_MODES),
        can_motion_modes=dict(DEFAULT_CAN_MOTION_MODES),
        config_path=config_path,
    )


def _as_upper_string_list(value: Any, default: list[str]) -> tuple[str, ...]:
    if not isinstance(value, list):
        return tuple(default)
    result = [item.strip().upper() for item in value if isinstance(item, str) and item.strip()]
    return tuple(result or default)


def _as_string_list(value: Any, default: list[str]) -> tuple[str, ...]:
    if not isinstance(value, list):
        return tuple(default)
    result = [item.strip() for item in value if isinstance(item, str) and item.strip()]
    return tuple(result or default)


def profile_from_dict(data: dict[str, Any], config_path: str | None = None) -> PlatformProfile:
    name = str(data.get("platform", "omni4")).strip().lower()
    display_name = str(data.get("display_name", name)).strip() or name
    motion_interface = str(data.get("motion_interface", "discrete_mode_speed_v1")).strip()
    can_protocol = str(data.get("can_protocol", "stm32_omni_v1")).strip()
    motion_modes = _as_upper_string_list(data.get("motion_modes"), DEFAULT_MOTION_MODES)
    wheel_names = _as_string_list(data.get("wheel_names"), DEFAULT_WHEEL_NAMES)

    cmd_vel = data.get("cmd_vel") if isinstance(data.get("cmd_vel"), dict) else {}
    teleop_button_modes = dict(DEFAULT_TELEOP_BUTTON_MODES)
    raw_buttons = data.get("teleop_button_modes")
    if isinstance(raw_buttons, dict):
        for key, value in raw_buttons.items():
            if isinstance(key, str) and isinstance(value, str):
                teleop_button_modes[key.strip()] = value.strip().upper()

    can_motion_modes = dict(DEFAULT_CAN_MOTION_MODES)
    raw_can_modes = data.get("can_motion_modes")
    if isinstance(raw_can_modes, dict):
        for key, value in raw_can_modes.items():
            if isinstance(key, str) and isinstance(value, str):
                can_motion_modes[key.strip().upper()] = value.strip().upper()

    if "STOP" not in motion_modes:
        motion_modes = ("STOP", *motion_modes)

    return PlatformProfile(
        name=name,
        display_name=display_name,
        motion_interface=motion_interface,
        can_protocol=can_protocol,
        motion_modes=motion_modes,
        wheel_names=wheel_names,
        cmd_vel=dict(cmd_vel),
        teleop_button_modes=teleop_button_modes,
        can_motion_modes=can_motion_modes,
        config_path=config_path,
    )


def load_platform_profile(platform_name: str | None = None, config_path: str | None = None) -> PlatformProfile:
    name = (platform_name or os.environ.get("ROBOT_PLATFORM") or "omni4").strip().lower()
    path = (config_path or os.environ.get("OMNI_PLATFORM_CONFIG") or "").strip()

    if path:
        try:
            with Path(path).expanduser().open("r", encoding="utf-8") as f:
                profile = profile_from_dict(json.load(f), config_path=path)
        except FileNotFoundError:
            if name != "omni4":
                raise
            profile = builtin_omni4_profile(config_path=path)
    else:
        if name != "omni4":
            raise ValueError(f"platform config is required for unsupported built-in platform: {name}")
        profile = builtin_omni4_profile()

    if profile.name != name:
        raise ValueError(f"platform mismatch: requested {name}, config contains {profile.name}")
    return profile


def normalize_motion_payload(payload: dict[str, Any], profile: PlatformProfile) -> dict[str, int | str]:
    if not isinstance(payload, dict):
        raise ValueError("payload_not_object")
    mode = payload.get("mode")
    if not isinstance(mode, str):
        raise ValueError("mode_missing_or_not_string")
    mode = mode.strip().upper()
    if not profile.supports_motion_mode(mode):
        raise ValueError(f"unsupported_mode_for_{profile.name}: {mode}")
    try:
        speed_pct = int(payload.get("speed_pct", 0))
    except (TypeError, ValueError) as exc:
        raise ValueError("speed_pct_not_int") from exc
    speed_pct = max(0, min(100, speed_pct))
    if mode == "STOP":
        speed_pct = 0
    return {"mode": mode, "speed_pct": speed_pct}
