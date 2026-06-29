#!/usr/bin/env python3
"""Run a short operator-facing motion direction check.

By default this tests human/UI labels such as "left" or "rotate_ccw" and maps
them through the active platform profile `teleop_button_modes`, exactly like the
web cockpit does.  With `--raw`, labels are sent directly as STM32 motion mode
names, so "left" publishes raw `LEFT`, "rotate_ccw" publishes raw `ROTATE_CCW`,
and so on.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from omni_pi.platforms import load_platform_profile


DEFAULT_SEQUENCE = (
    "forward",
    "backward",
    "left",
    "right",
    "rotate_ccw",
    "rotate_cw",
)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def normalize_angle(value: float) -> float:
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def pose_from_odom(msg: Odometry) -> Pose2D:
    p = msg.pose.pose.position
    return Pose2D(float(p.x), float(p.y), yaw_from_odom(msg))


def pose_delta(start: Pose2D, end: Pose2D) -> dict[str, float]:
    return {
        "dx_m": end.x - start.x,
        "dy_m": end.y - start.y,
        "dyaw_rad": normalize_angle(end.yaw - start.yaw),
        "dyaw_deg": math.degrees(normalize_angle(end.yaw - start.yaw)),
    }


def default_speed_for_label(label: str) -> int:
    if label in {"left", "right", "rotate_ccw", "rotate_cw"}:
        return 30
    return 20


def default_duration_for_label(label: str) -> float:
    if label in {"rotate_ccw", "rotate_cw"}:
        return 0.30
    return 0.35


class MotionDirectionCheck(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("motion_direction_check")
        self.args = args
        self.platform = load_platform_profile()
        self.latest_odom: Odometry | None = None
        self.motion_pub = self.create_publisher(String, args.command_topic, 10)
        self.control_pub = self.create_publisher(String, args.control_topic, 10)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, 20)

    def on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def wait_for_odom(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_odom is not None:
                return True
        return False

    def spin_for(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)

    def publish_control_mode(self, control_mode: str) -> None:
        msg = String()
        msg.data = control_mode
        self.control_pub.publish(msg)

    def publish_motion(self, raw_mode: str, speed_pct: int) -> None:
        msg = String()
        msg.data = json.dumps(
            {"mode": raw_mode, "speed_pct": int(speed_pct)},
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.motion_pub.publish(msg)

    def stop(self, repeats: int = 3) -> None:
        for _ in range(repeats):
            self.publish_motion("STOP", 0)
            self.spin_for(0.08)

    def raw_mode_for_label(self, label: str) -> str:
        label = label.strip().lower()
        if self.args.raw:
            return label.upper()
        mapped = self.platform.teleop_button_modes.get(label)
        if not mapped:
            raise ValueError(f"label has no teleop mapping for platform {self.platform.name}: {label}")
        return mapped.strip().upper()

    def run_test(self, label: str) -> dict[str, Any]:
        label = label.strip().lower()
        raw_mode = self.raw_mode_for_label(label)
        speed_pct = int(self.args.speed_pct) if self.args.speed_pct is not None else default_speed_for_label(label)
        duration_sec = float(self.args.duration_sec) if self.args.duration_sec is not None else default_duration_for_label(label)

        self.spin_for(self.args.before_each_sec)
        start_msg = self.latest_odom
        if start_msg is None:
            raise RuntimeError("no odom before test")
        start = pose_from_odom(start_msg)

        print(
            f"TEST {label.upper():>10} -> raw {raw_mode:<10} "
            f"speed={speed_pct}% duration={duration_sec:.2f}s",
            flush=True,
        )
        self.publish_motion(raw_mode, speed_pct)
        self.spin_for(duration_sec)
        self.stop()
        self.spin_for(self.args.settle_sec)

        end_msg = self.latest_odom
        if end_msg is None:
            raise RuntimeError("no odom after test")
        end = pose_from_odom(end_msg)
        delta = pose_delta(start, end)
        print(
            "  odom delta: "
            f"dx={delta['dx_m']:+.3f} m, "
            f"dy={delta['dy_m']:+.3f} m, "
            f"dyaw={delta['dyaw_deg']:+.1f} deg",
            flush=True,
        )
        return {
            "label": label,
            "raw_mode": raw_mode,
            "speed_pct": speed_pct,
            "duration_sec": duration_sec,
            "start": {"x_m": start.x, "y_m": start.y, "yaw_rad": start.yaw},
            "end": {"x_m": end.x, "y_m": end.y, "yaw_rad": end.yaw},
            "delta": delta,
        }


def parse_sequence(value: str) -> list[str]:
    result = [item.strip().lower() for item in value.split(",") if item.strip()]
    if not result:
        raise argparse.ArgumentTypeError("empty sequence")
    return result


def default_output_path() -> Path:
    omni_home = Path(os.environ.get("OMNI_HOME", str(Path.home() / "omni-pi")))
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return omni_home / "logs" / f"motion_direction_check_{stamp}.json"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drive", action="store_true", help="Actually publish motion commands.")
    parser.add_argument("--raw", action="store_true", help="Send labels directly as raw STM32 motion modes.")
    parser.add_argument("--sequence", type=parse_sequence, default=list(DEFAULT_SEQUENCE))
    parser.add_argument("--speed-pct", type=int, default=None, help="Override speed for every test.")
    parser.add_argument("--duration-sec", type=float, default=None, help="Override duration for every test.")
    parser.add_argument("--between-sec", type=float, default=3.0, help="Pause after each test.")
    parser.add_argument("--before-each-sec", type=float, default=0.4, help="Short odom sampling delay before each test.")
    parser.add_argument("--settle-sec", type=float, default=0.35, help="Wait after STOP before reading odom.")
    parser.add_argument("--initial-wait-sec", type=float, default=3.0)
    parser.add_argument("--command-topic", default="/omni/manual_cmd")
    parser.add_argument("--control-topic", default="/omni/control_mode")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--output", type=Path, default=None)
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    args.output = args.output or default_output_path()

    if not args.drive:
        print("Dry run only. Add --drive to publish motion commands.")
        return 0

    rclpy.init()
    node = MotionDirectionCheck(args)
    results: list[dict[str, Any]] = []
    try:
        print(f"Waiting for {args.odom_topic}...")
        if not node.wait_for_odom(args.initial_wait_sec):
            print(f"ERROR: no odom on {args.odom_topic}")
            return 2

        print(f"Platform: {node.platform.name}")
        print("Switching command mux to MANUAL and sending STOP.")
        node.publish_control_mode("MANUAL")
        node.stop()

        for index, label in enumerate(args.sequence, start=1):
            print(f"\n[{index}/{len(args.sequence)}]")
            results.append(node.run_test(label))
            node.spin_for(args.between_sec)

        node.stop()
        report = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "platform": node.platform.as_public_dict(),
            "args": {
                "sequence": args.sequence,
                "raw": args.raw,
                "command_topic": args.command_topic,
                "odom_topic": args.odom_topic,
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
