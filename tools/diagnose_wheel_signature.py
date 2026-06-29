#!/usr/bin/env python3
"""Capture per-wheel encoder signatures for each high-level motion mode.

The current robot can be commanded only by base-level modes such as FORWARD,
LEFT or ROTATE_CW.  That is enough to discover the real wheel telemetry signs:
for each short movement this tool records new `/omni/wheel_states` samples and
sums `delta_ticks` independently for wheel0..wheel3.

Run in passive mode first:

    python tools/diagnose_wheel_signature.py

To actually command the robot, use --drive after the robot is safely lifted or
placed with enough free space:

    python tools/diagnose_wheel_signature.py --drive --speed-pct 20 --duration-sec 0.7
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_MODES = (
    "FORWARD",
    "BACKWARD",
    "LEFT",
    "RIGHT",
    "ROTATE_CW",
    "ROTATE_CCW",
)


@dataclass
class WheelStats:
    samples: int = 0
    delta_sum: int = 0
    abs_delta_sum: int = 0
    min_delta: int | None = None
    max_delta: int | None = None
    speed_sum: float = 0.0
    speed_abs_sum: float = 0.0
    speed_samples: int = 0

    def add(self, delta_ticks: int, speed_ticks_per_sec: float | None) -> None:
        self.samples += 1
        self.delta_sum += delta_ticks
        self.abs_delta_sum += abs(delta_ticks)
        self.min_delta = delta_ticks if self.min_delta is None else min(self.min_delta, delta_ticks)
        self.max_delta = delta_ticks if self.max_delta is None else max(self.max_delta, delta_ticks)
        if speed_ticks_per_sec is not None and math.isfinite(speed_ticks_per_sec):
            self.speed_sum += speed_ticks_per_sec
            self.speed_abs_sum += abs(speed_ticks_per_sec)
            self.speed_samples += 1

    def as_dict(self) -> dict[str, Any]:
        avg_speed = self.speed_sum / self.speed_samples if self.speed_samples else 0.0
        avg_abs_speed = self.speed_abs_sum / self.speed_samples if self.speed_samples else 0.0
        sign = "+"
        if self.delta_sum < 0:
            sign = "-"
        elif self.delta_sum == 0:
            sign = "0"
        return {
            "samples": self.samples,
            "delta_sum": self.delta_sum,
            "abs_delta_sum": self.abs_delta_sum,
            "sign": sign,
            "min_delta": self.min_delta,
            "max_delta": self.max_delta,
            "avg_speed_ticks_per_sec": round(avg_speed, 3),
            "avg_abs_speed_ticks_per_sec": round(avg_abs_speed, 3),
        }


@dataclass
class CaptureWindow:
    label: str
    mode: str
    speed_pct: int
    started_wall_time: float = field(default_factory=time.time)
    wheels: dict[int, WheelStats] = field(default_factory=dict)

    def add_wheel(self, wheel_index: int, delta_ticks: int, speed_ticks_per_sec: float | None) -> None:
        self.wheels.setdefault(wheel_index, WheelStats()).add(delta_ticks, speed_ticks_per_sec)

    def as_dict(self) -> dict[str, Any]:
        return {
            "label": self.label,
            "mode": self.mode,
            "speed_pct": self.speed_pct,
            "started_wall_time": self.started_wall_time,
            "wheels": {str(index): stats.as_dict() for index, stats in sorted(self.wheels.items())},
        }


class WheelSignatureDiag(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("wheel_signature_diag")
        self.args = args
        self.latest_payload: dict[str, Any] | None = None
        self.active_window: CaptureWindow | None = None
        self.last_update_times: dict[int, float] = {}

        self.control_pub = self.create_publisher(String, args.control_topic, 10)
        self.motion_pub = self.create_publisher(String, args.command_topic, 10)
        self.create_subscription(String, args.wheel_topic, self.on_wheel_states, 50)

    def on_wheel_states(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("Ignoring invalid wheel_states JSON")
            return

        if not isinstance(payload, dict):
            return

        self.latest_payload = payload
        window = self.active_window
        if window is None:
            self.remember_seen_updates(payload)
            return

        wheels = payload.get("wheels") or []
        if not isinstance(wheels, list):
            return

        for wheel in wheels:
            if not isinstance(wheel, dict):
                continue
            if not bool(wheel.get("present", True)):
                continue
            try:
                wheel_index = int(wheel.get("wheel_index"))
            except (TypeError, ValueError):
                continue

            update_time = self.safe_float(wheel.get("last_update_monotonic_sec"))
            if update_time is None:
                continue
            if self.last_update_times.get(wheel_index) == update_time:
                continue
            self.last_update_times[wheel_index] = update_time

            try:
                delta_ticks = int(wheel.get("delta_ticks", 0) or 0)
            except (TypeError, ValueError):
                delta_ticks = 0

            speed_ticks = self.safe_float(wheel.get("speed_ticks_per_sec"))
            window.add_wheel(wheel_index, delta_ticks, speed_ticks)

    def remember_seen_updates(self, payload: dict[str, Any]) -> None:
        wheels = payload.get("wheels") or []
        if not isinstance(wheels, list):
            return
        for wheel in wheels:
            if not isinstance(wheel, dict):
                continue
            try:
                wheel_index = int(wheel.get("wheel_index"))
            except (TypeError, ValueError):
                continue
            update_time = self.safe_float(wheel.get("last_update_monotonic_sec"))
            if update_time is not None:
                self.last_update_times[wheel_index] = update_time

    @staticmethod
    def safe_float(value: Any) -> float | None:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        return result if math.isfinite(result) else None

    def publish_control_mode(self, mode: str) -> None:
        msg = String()
        msg.data = mode
        self.control_pub.publish(msg)

    def publish_motion(self, mode: str, speed_pct: int) -> None:
        msg = String()
        msg.data = json.dumps({"mode": mode, "speed_pct": int(speed_pct)}, separators=(",", ":"))
        self.motion_pub.publish(msg)

    def stop(self, repeats: int = 3, delay_sec: float = 0.08) -> None:
        for _ in range(repeats):
            self.publish_motion("STOP", 0)
            rclpy.spin_once(self, timeout_sec=delay_sec)

    def wait_for_wheel_states(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_payload is not None:
                return True
        return False

    def capture_passive(self, duration_sec: float) -> CaptureWindow:
        window = CaptureWindow(label="passive", mode="PASSIVE", speed_pct=0)
        self.active_window = window
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.active_window = None
        return window

    def capture_motion(self, label: str, mode: str, speed_pct: int, duration_sec: float) -> CaptureWindow:
        window = CaptureWindow(label=label, mode=mode, speed_pct=speed_pct)
        self.active_window = window
        self.publish_motion(mode, speed_pct)
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)
        self.stop()
        self.active_window = None
        return window


def parse_modes(raw_modes: str) -> list[str]:
    modes = [item.strip().upper() for item in raw_modes.split(",") if item.strip()]
    if not modes:
        raise argparse.ArgumentTypeError("at least one mode is required")
    return modes


def default_output_path() -> Path:
    omni_home = Path(os.environ.get("OMNI_HOME", str(Path.home() / "omni-pi")))
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return omni_home / "logs" / f"wheel_signature_{stamp}.json"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drive", action="store_true", help="Actually publish movement commands.")
    parser.add_argument("--modes", type=parse_modes, default=list(DEFAULT_MODES), help="Comma-separated modes to test.")
    parser.add_argument("--speed-pct", type=int, default=20, help="Speed percentage for each motion test.")
    parser.add_argument("--duration-sec", type=float, default=0.7, help="Motion duration for each test.")
    parser.add_argument("--repeat", type=int, default=1, help="Repeat each mode this many times.")
    parser.add_argument("--between-sec", type=float, default=1.2, help="Pause between tests.")
    parser.add_argument("--initial-wait-sec", type=float, default=3.0, help="Wait for initial wheel telemetry.")
    parser.add_argument("--passive-sec", type=float, default=2.0, help="Passive capture duration when --drive is not used.")
    parser.add_argument("--command-topic", default="/omni/manual_cmd", help="Motion command topic.")
    parser.add_argument("--control-topic", default="/omni/control_mode", help="Control mode topic.")
    parser.add_argument("--wheel-topic", default="/omni/wheel_states", help="Wheel telemetry topic.")
    parser.add_argument("--output", type=Path, default=None, help="Where to save JSON report.")
    return parser


def print_window_summary(window: CaptureWindow) -> None:
    data = window.as_dict()
    print(f"\n{data['label']}: mode={data['mode']} speed={data['speed_pct']}%")
    print("wheel | samples | delta_sum | sign | abs_delta | avg_speed_ticks/s")
    print("------+---------+-----------+------+-----------+------------------")
    for index_text, wheel in data["wheels"].items():
        print(
            f"{int(index_text):>5} | "
            f"{wheel['samples']:>7} | "
            f"{wheel['delta_sum']:>9} | "
            f"{wheel['sign']:^4} | "
            f"{wheel['abs_delta_sum']:>9} | "
            f"{wheel['avg_speed_ticks_per_sec']:>16}"
        )


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    args.speed_pct = max(0, min(100, int(args.speed_pct)))
    args.repeat = max(1, int(args.repeat))
    args.output = args.output or default_output_path()

    rclpy.init()
    node = WheelSignatureDiag(args)
    results: list[dict[str, Any]] = []

    try:
        print(f"Waiting for {args.wheel_topic} telemetry...")
        if not node.wait_for_wheel_states(args.initial_wait_sec):
            print(f"ERROR: no wheel telemetry on {args.wheel_topic} within {args.initial_wait_sec:.1f}s")
            return 2

        if not args.drive:
            print("Passive mode: not sending movement commands. Use --drive to run the motion test.")
            window = node.capture_passive(args.passive_sec)
            print_window_summary(window)
            results.append(window.as_dict())
        else:
            print("Switching command mux to MANUAL and sending initial STOP.")
            node.publish_control_mode("MANUAL")
            node.stop()
            time.sleep(0.2)

            for repeat_index in range(1, args.repeat + 1):
                for mode in args.modes:
                    label = f"repeat_{repeat_index}_{mode}"
                    print(f"\nRunning {label}: {args.speed_pct}% for {args.duration_sec:.2f}s")
                    window = node.capture_motion(label, mode, args.speed_pct, args.duration_sec)
                    print_window_summary(window)
                    results.append(window.as_dict())
                    deadline = time.monotonic() + args.between_sec
                    while rclpy.ok() and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.05)

            node.stop()

        report = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "args": {
                "drive": args.drive,
                "modes": args.modes,
                "speed_pct": args.speed_pct,
                "duration_sec": args.duration_sec,
                "repeat": args.repeat,
                "command_topic": args.command_topic,
                "wheel_topic": args.wheel_topic,
            },
            "results": results,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"\nSaved report: {args.output}")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted, sending STOP.")
        node.stop()
        return 130
    except ExternalShutdownException:
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
