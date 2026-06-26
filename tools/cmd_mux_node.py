#!/usr/bin/env python3
import json
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

VALID_MOTION_MODES = {
    "STOP",
    "FORWARD",
    "BACKWARD",
    "LEFT",
    "RIGHT",
    "ROTATE_CCW",
    "ROTATE_CW",
}

VALID_CONTROL_MODES = {"MANUAL", "AUTO"}


class CommandMuxNode(Node):
    def __init__(self) -> None:
        super().__init__("command_mux")

        self.active_source = "AUTO"

        self.pub_motion = self.create_publisher(String, "/omni/motion_cmd", 10)
        self.pub_mode_status = self.create_publisher(String, "/omni/control_mode_status", 10)
        self.pub_arbiter_status = self.create_publisher(String, "/omni/arbiter_status", 10)

        self.sub_manual = self.create_subscription(
            String, "/omni/manual_cmd", self.on_manual_cmd, 10
        )
        self.sub_auto = self.create_subscription(
            String, "/omni/auto_cmd", self.on_auto_cmd, 10
        )
        self.sub_control_mode = self.create_subscription(
            String, "/omni/control_mode", self.on_control_mode, 10
        )

        self.publish_mode_status("startup")
        self.get_logger().info("command_mux started, default control mode = AUTO")

    def normalize_text(self, value: str) -> str:
        return value.strip().upper()

    def parse_control_mode(self, raw: str) -> Optional[str]:
        raw = raw.strip()
        if not raw:
            return None

        if raw.startswith("{"):
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                return None
            mode = data.get("mode") or data.get("control_mode") or data.get("source")
            if not isinstance(mode, str):
                return None
            mode = self.normalize_text(mode)
        else:
            mode = self.normalize_text(raw)

        if mode not in VALID_CONTROL_MODES:
            return None

        return mode

    def parse_motion_cmd(self, raw: str) -> Tuple[Optional[dict], Optional[str]]:
        try:
            data = json.loads(raw)
        except json.JSONDecodeError as exc:
            return None, f"invalid_json: {exc}"

        if not isinstance(data, dict):
            return None, "payload_not_object"

        mode = data.get("mode")
        speed_pct = data.get("speed_pct", 0)

        if not isinstance(mode, str):
            return None, "mode_missing_or_not_string"

        mode = self.normalize_text(mode)
        if mode not in VALID_MOTION_MODES:
            return None, f"unsupported_mode: {mode}"

        try:
            speed_pct = int(speed_pct)
        except (TypeError, ValueError):
            return None, "speed_pct_not_int"

        if speed_pct < 0:
            speed_pct = 0
        if speed_pct > 100:
            speed_pct = 100

        return {"mode": mode, "speed_pct": speed_pct}, None

    def publish_json(self, pub, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
        pub.publish(msg)

    def publish_mode_status(self, reason: str) -> None:
        self.publish_json(
            self.pub_mode_status,
            {
                "active_source": self.active_source,
                "reason": reason,
            },
        )

    def publish_arbiter_status(
        self,
        event: str,
        forwarded: bool,
        source: Optional[str],
        command: Optional[dict],
        reason: str,
    ) -> None:
        self.publish_json(
            self.pub_arbiter_status,
            {
                "event": event,
                "forwarded": forwarded,
                "active_source": self.active_source,
                "source": source,
                "command": command,
                "reason": reason,
            },
        )

    def publish_stop(self, reason: str) -> None:
        stop_cmd = {"mode": "STOP", "speed_pct": 0}
        self.publish_json(self.pub_motion, stop_cmd)
        self.publish_arbiter_status(
            event="publish_stop",
            forwarded=True,
            source="MUX",
            command=stop_cmd,
            reason=reason,
        )
        self.get_logger().info(f"published STOP, reason={reason}")

    def maybe_forward(self, source: str, cmd: dict) -> None:
        if source != self.active_source:
            self.publish_arbiter_status(
                event="command_ignored",
                forwarded=False,
                source=source,
                command=cmd,
                reason="inactive_source",
            )
            return

        self.publish_json(self.pub_motion, cmd)
        self.publish_arbiter_status(
            event="command_forwarded",
            forwarded=True,
            source=source,
            command=cmd,
            reason="active_source",
        )
        self.get_logger().info(
            f"forwarded source={source} mode={cmd['mode']} speed_pct={cmd['speed_pct']}"
        )

    def on_control_mode(self, msg: String) -> None:
        new_mode = self.parse_control_mode(msg.data)
        if new_mode is None:
            self.publish_arbiter_status(
                event="mode_rejected",
                forwarded=False,
                source=None,
                command=None,
                reason="invalid_mode_payload",
            )
            self.get_logger().warning(f"invalid control mode payload: {msg.data}")
            return

        if new_mode == self.active_source:
            self.publish_mode_status("mode_unchanged")
            self.publish_arbiter_status(
                event="mode_unchanged",
                forwarded=False,
                source=None,
                command=None,
                reason=f"already_{new_mode}",
            )
            self.get_logger().info(f"control mode already {new_mode}")
            return

        old_mode = self.active_source
        self.active_source = new_mode

        self.publish_stop(f"mode_switch_{old_mode}_to_{new_mode}")
        self.publish_mode_status("mode_switched")
        self.publish_arbiter_status(
            event="mode_switched",
            forwarded=False,
            source=None,
            command=None,
            reason=f"{old_mode}_to_{new_mode}",
        )
        self.get_logger().info(f"control mode changed: {old_mode} -> {new_mode}")

    def on_manual_cmd(self, msg: String) -> None:
        cmd, error = self.parse_motion_cmd(msg.data)
        if error:
            self.publish_arbiter_status(
                event="manual_rejected",
                forwarded=False,
                source="MANUAL",
                command=None,
                reason=error,
            )
            self.get_logger().warning(f"manual cmd rejected: {error}; payload={msg.data}")
            return

        self.maybe_forward("MANUAL", cmd)

    def on_auto_cmd(self, msg: String) -> None:
        cmd, error = self.parse_motion_cmd(msg.data)
        if error:
            self.publish_arbiter_status(
                event="auto_rejected",
                forwarded=False,
                source="AUTO",
                command=None,
                reason=error,
            )
            self.get_logger().warning(f"auto cmd rejected: {error}; payload={msg.data}")
            return

        self.maybe_forward("AUTO", cmd)


def main() -> None:
    rclpy.init()
    node = CommandMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
