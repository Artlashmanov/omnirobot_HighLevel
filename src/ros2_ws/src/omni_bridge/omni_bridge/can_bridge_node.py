import json
import os
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from omni_pi.can_transport import CanTransport
from omni_pi import protocol
from omni_pi.protocol import MotionMode
from omni_pi.platforms import load_platform_profile, normalize_motion_payload


MODE_FROM_NAME = {
    'STOP': MotionMode.STOP,
    'FORWARD': MotionMode.FORWARD,
    'BACKWARD': MotionMode.BACKWARD,
    'LEFT': MotionMode.LEFT,
    'RIGHT': MotionMode.RIGHT,
    'ROTATE_CCW': MotionMode.ROTATE_CCW,
    'ROTATE_CW': MotionMode.ROTATE_CW,
}

DEFAULT_WHEEL_NAMES = [
    'front_left',
    'front_right',
    'rear_left',
    'rear_right',
]


class OmniCanBridgeNode(Node):
    def __init__(self):
        super().__init__('omni_can_bridge')

        self.declare_parameter('platform_name', os.environ.get('ROBOT_PLATFORM', 'omni4'))
        self.declare_parameter('platform_config', os.environ.get('OMNI_PLATFORM_CONFIG', ''))
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('status_poll_period_sec', 0.5)
        self.declare_parameter('watchdog_timeout_sec', 0.8)
        self.declare_parameter('cmd_vel_timeout_stop', True)

        platform_name = str(self.get_parameter('platform_name').value or 'omni4')
        platform_config = str(self.get_parameter('platform_config').value or '')
        self.platform = load_platform_profile(platform_name, platform_config)
        if self.platform.can_protocol != 'stm32_omni_v1':
            raise RuntimeError(
                f"omni_bridge currently supports can_protocol=stm32_omni_v1, "
                f"platform {self.platform.name} requests {self.platform.can_protocol}"
            )

        self.declare_parameter('wheel_names', list(self.platform.wheel_names))
        self.declare_parameter('max_linear_x', 1.0)
        self.declare_parameter('max_angular_z', 1.0)
        self.declare_parameter('linear_deadband', 0.05)
        self.declare_parameter('angular_deadband', 0.05)
        self.declare_parameter('min_speed_pct', 30)
        self.declare_parameter('max_speed_pct', 100)

        self.can_channel = self.get_parameter('can_channel').value
        self.status_poll_period_sec = float(self.get_parameter('status_poll_period_sec').value)
        self.watchdog_timeout_sec = float(self.get_parameter('watchdog_timeout_sec').value)
        self.cmd_vel_timeout_stop = bool(self.get_parameter('cmd_vel_timeout_stop').value)
        self.wheel_names = list(self.get_parameter('wheel_names').value)

        self.max_linear_x = float(self.get_parameter('max_linear_x').value)
        self.max_angular_z = float(self.get_parameter('max_angular_z').value)
        self.linear_deadband = float(self.get_parameter('linear_deadband').value)
        self.angular_deadband = float(self.get_parameter('angular_deadband').value)
        self.min_speed_pct = int(self.get_parameter('min_speed_pct').value)
        self.max_speed_pct = int(self.get_parameter('max_speed_pct').value)

        self.transport = CanTransport(channel=self.can_channel)

        self.seq = 0
        self.last_cmd_time = time.monotonic()
        self.last_sent_motion = None
        self.last_sent_speed = None
        self.last_base_status = None
        self.wheel_states = {}
        self.last_power_status = None

        self.status_text_pub = self.create_publisher(String, '/omni/status_text', 10)
        self.status_json_pub = self.create_publisher(String, '/omni/status_json', 10)
        self.rx_raw_pub = self.create_publisher(String, '/omni/rx_raw', 20)
        self.base_status_pub = self.create_publisher(String, '/omni/base_status', 10)
        self.wheel_states_pub = self.create_publisher(String, '/omni/wheel_states', 10)
        self.power_status_pub = self.create_publisher(String, '/omni/power_status', 10)

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.motion_cmd_sub = self.create_subscription(String, '/omni/motion_cmd', self.on_motion_cmd, 10)

        self.status_timer = self.create_timer(self.status_poll_period_sec, self.poll_status)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)

        self.rx_thread = threading.Thread(target=self.recv_loop, daemon=True)
        self.rx_thread.start()

        self.get_logger().info(
            f'omni_can_bridge started on {self.can_channel}, '
            f'platform={self.platform.name}, can_protocol={self.platform.can_protocol}'
        )

    def next_seq(self) -> int:
        self.seq = (self.seq + 1) & 0xFF
        return self.seq

    def scale_to_speed_pct(self, value_abs: float, value_max: float) -> int:
        if value_max <= 0.0:
            return self.min_speed_pct

        ratio = value_abs / value_max
        ratio = max(0.0, min(1.0, ratio))

        speed = int(round(self.min_speed_pct + ratio * (self.max_speed_pct - self.min_speed_pct)))
        speed = max(self.min_speed_pct, min(self.max_speed_pct, speed))
        return speed

    def can_mode_name_for_semantic(self, semantic_mode_name: str) -> str:
        semantic = str(semantic_mode_name or 'STOP').strip().upper()
        return str(self.platform.can_motion_modes.get(semantic, semantic)).strip().upper()

    def semantic_mode_name_for_can(self, can_mode_name: str) -> str:
        raw = str(can_mode_name or 'STOP').strip().upper()
        for semantic, mapped_raw in self.platform.can_motion_modes.items():
            if str(mapped_raw).strip().upper() == raw:
                return str(semantic).strip().upper()
        return raw

    def motion_mode_for_semantic(self, semantic_mode_name: str) -> tuple[MotionMode, str]:
        raw_mode_name = self.can_mode_name_for_semantic(semantic_mode_name)
        if raw_mode_name not in MODE_FROM_NAME:
            raise ValueError(f'Motion mode {semantic_mode_name} maps to unsupported STM32 mode {raw_mode_name}')
        return MODE_FROM_NAME[raw_mode_name], raw_mode_name

    def twist_to_motion(self, linear_x: float, angular_z: float):
        if abs(linear_x) < self.linear_deadband and abs(angular_z) < self.angular_deadband:
            mode, _ = self.motion_mode_for_semantic('STOP')
            return mode, 0

        if abs(linear_x) >= abs(angular_z):
            if linear_x > 0:
                mode, _ = self.motion_mode_for_semantic('FORWARD')
                return mode, self.scale_to_speed_pct(abs(linear_x), self.max_linear_x)
            mode, _ = self.motion_mode_for_semantic('BACKWARD')
            return mode, self.scale_to_speed_pct(abs(linear_x), self.max_linear_x)

        if angular_z > 0:
            mode, _ = self.motion_mode_for_semantic('ROTATE_CCW')
            return mode, self.scale_to_speed_pct(abs(angular_z), self.max_angular_z)
        mode, _ = self.motion_mode_for_semantic('ROTATE_CW')
        return mode, self.scale_to_speed_pct(abs(angular_z), self.max_angular_z)

    def send_stop(self, reason: str = 'manual'):
        seq = self.next_seq()
        msg = protocol.make_stop(seq)
        self.transport.send(msg)
        self.last_sent_motion = MotionMode.STOP
        self.last_sent_speed = 0
        self.get_logger().info(f'TX STOP seq={seq} reason={reason}')

    def send_motion(self, mode: MotionMode, speed_pct: int, reason: str = 'motion'):
        if mode == MotionMode.STOP:
            self.send_stop(reason=reason)
            return

        seq = self.next_seq()
        msg = protocol.make_motion(seq, mode, speed_pct)
        self.transport.send(msg)
        self.last_sent_motion = mode
        self.last_sent_speed = speed_pct
        self.get_logger().info(f'TX MOTION seq={seq} mode={mode.name} speed_pct={speed_pct} reason={reason}')

    def on_cmd_vel(self, msg: Twist):
        self.last_cmd_time = time.monotonic()
        mode, speed_pct = self.twist_to_motion(msg.linear.x, msg.angular.z)

        if self.last_sent_motion == mode and self.last_sent_speed == speed_pct:
            return

        self.send_motion(mode, speed_pct, reason='cmd_vel')

    def on_motion_cmd(self, msg: String):
        self.last_cmd_time = time.monotonic()

        try:
            payload = json.loads(msg.data)
            command = normalize_motion_payload(payload, self.platform)
        except Exception as e:
            self.get_logger().error(f'Invalid /omni/motion_cmd for platform {self.platform.name}: {e}')
            return

        mode_name = str(command['mode'])
        speed_pct = int(command['speed_pct'])

        try:
            mode, raw_mode_name = self.motion_mode_for_semantic(mode_name)
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        if self.last_sent_motion == mode and self.last_sent_speed == speed_pct:
            return

        reason = f'motion_cmd semantic={mode_name} raw={raw_mode_name}' if raw_mode_name != mode_name else 'motion_cmd'
        self.send_motion(mode, speed_pct, reason=reason)

    def poll_status(self):
        seq = self.next_seq()
        msg = protocol.make_status_req(seq)
        self.transport.send(msg)

    def watchdog_check(self):
        if not self.cmd_vel_timeout_stop:
            return

        elapsed = time.monotonic() - self.last_cmd_time
        if elapsed > self.watchdog_timeout_sec:
            if self.last_sent_motion is not None and self.last_sent_motion != MotionMode.STOP:
                self.send_stop(reason='cmd_vel_watchdog')

    def recv_loop(self):
        while rclpy.ok():
            try:
                msg = self.transport.recv(timeout=0.2)
                if msg is None:
                    continue
                self.handle_rx(msg)
            except Exception as e:
                self.get_logger().error(f'CAN recv error: {e}')
                time.sleep(0.2)

    def wheel_name_for_index(self, wheel_index: int) -> str:
        if 0 <= wheel_index < len(self.wheel_names):
            return str(self.wheel_names[wheel_index])
        return f'wheel_{wheel_index}'

    def publish_json_msg(self, publisher, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        publisher.publish(msg)

    def publish_base_status(self, decoded: dict, raw: dict) -> None:
        payload = protocol.decode_base_status_data(decoded['data'])
        raw_mode_name = str(payload.get('motion_mode_name') or 'STOP').upper()
        semantic_mode_name = self.semantic_mode_name_for_can(raw_mode_name)
        payload['raw_motion_mode_name'] = raw_mode_name
        payload['semantic_motion_mode_name'] = semantic_mode_name
        payload['motion_mode_name'] = semantic_mode_name
        payload['raw'] = raw
        payload['received_time_sec'] = time.time()
        self.last_base_status = payload
        self.publish_json_msg(self.base_status_pub, payload)

    def publish_power_status(self, decoded: dict, raw: dict) -> None:
        payload = protocol.decode_ina228_status_data(decoded['data'])
        payload['raw'] = raw
        payload['received_time_sec'] = time.time()
        self.last_power_status = payload
        self.publish_json_msg(self.power_status_pub, payload)

    def publish_wheel_state(self, decoded: dict, raw: dict) -> None:
        now_monotonic = time.monotonic()
        payload = protocol.decode_wheel_state_data(decoded['data'])
        wheel_index = int(payload['wheel_index'])
        payload['name'] = self.wheel_name_for_index(wheel_index)
        payload['raw'] = raw
        payload['last_update_monotonic_sec'] = now_monotonic
        self.wheel_states[wheel_index] = payload

        wheels = []
        for index in sorted(self.wheel_states):
            wheel = dict(self.wheel_states[index])
            wheel['last_update_age_sec'] = round(now_monotonic - float(wheel['last_update_monotonic_sec']), 3)
            wheels.append(wheel)

        aggregate = {
            'type': 'wheel_states',
            'wheel_count': self.last_base_status.get('wheel_count') if self.last_base_status else len(wheels),
            'wheels': wheels,
            'received_time_sec': time.time(),
        }
        self.publish_json_msg(self.wheel_states_pub, aggregate)

    def handle_rx(self, msg):
        decoded = protocol.decode_message(msg)

        raw_text = String()
        raw_text.data = json.dumps(decoded, ensure_ascii=False)
        self.rx_raw_pub.publish(raw_text)

        can_id = msg.arbitration_id
        data = decoded['data']

        if can_id == protocol.ID_EVT_BASE_STATUS:
            self.publish_base_status(decoded, decoded)
            return

        if can_id == protocol.ID_EVT_WHEEL_STATE:
            self.publish_wheel_state(decoded, decoded)
            return

        if can_id == protocol.ID_EVT_INA228_STATUS:
            self.publish_power_status(decoded, decoded)
            return

        if can_id == protocol.ID_EVT_ACK:
            text = (
                f"ACK proto={data[0]} seq={data[1]} "
                f"cmd_low=0x{data[2]:02X} result={data[3]} "
                f"motion={data[4]} speed_pct={data[5]}"
            )
            self.publish_status(text, {
                'type': 'ack',
                'proto_version': data[0],
                'seq': data[1],
                'command_id_low': data[2],
                'result_code': data[3],
                'current_motion_mode': data[4],
                'raw_motion_mode_name': protocol.motion_mode_name(data[4]),
                'current_motion_mode_name': self.semantic_mode_name_for_can(protocol.motion_mode_name(data[4])),
                'current_speed_pct': data[5],
                'raw': decoded,
            })
            return

        if can_id == protocol.ID_EVT_STATUS:
            text = (
                f"STATUS proto={data[0]} seq={data[1]} "
                f"motion={data[2]} speed_pct={data[3]}"
            )
            self.publish_status(text, {
                'type': 'status',
                'proto_version': data[0],
                'seq': data[1],
                'current_motion_mode': data[2],
                'raw_motion_mode_name': protocol.motion_mode_name(data[2]),
                'current_motion_mode_name': self.semantic_mode_name_for_can(protocol.motion_mode_name(data[2])),
                'current_speed_pct': data[3],
                'raw': decoded,
            })
            return

        if can_id == protocol.ID_EVT_PONG:
            text = f"PONG proto={data[0]} seq={data[1]}"
            self.publish_status(text, {
                'type': 'pong',
                'proto_version': data[0],
                'seq': data[1],
                'raw': decoded,
            })
            return

        if can_id == protocol.ID_EVT_TELEMETRY:
            text = f"TELEMETRY data={data}"
            self.publish_status(text, {
                'type': 'telemetry',
                'raw': decoded,
            })
            return

        if can_id == protocol.ID_EVT_ERROR:
            text = f"ERROR data={data}"
            self.publish_status(text, {
                'type': 'error',
                'raw': decoded,
            })
            return

    def publish_status(self, text_value: str, json_value: dict):
        text_msg = String()
        text_msg.data = text_value
        self.status_text_pub.publish(text_msg)

        json_msg = String()
        json_msg.data = json.dumps(json_value, ensure_ascii=False)
        self.status_json_pub.publish(json_msg)

    def destroy_node(self):
        try:
            self.transport.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OmniCanBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
