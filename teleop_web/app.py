#!/usr/bin/env python3
import copy
import json
import os
import threading
import time
from flask import Flask, jsonify, render_template, request

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


VALID_MOTION_MODES = {
    'STOP',
    'FORWARD',
    'BACKWARD',
    'LEFT',
    'RIGHT',
    'ROTATE_CCW',
    'ROTATE_CW',
}


class TeleopWebNode(Node):
    def __init__(self) -> None:
        super().__init__('teleop_web_node')
        self.manual_publisher = self.create_publisher(String, '/omni/manual_cmd', 10)
        self.mode_publisher = self.create_publisher(String, '/omni/control_mode', 10)

        self.state_lock = threading.Lock()
        self.current_mode = 'AUTO'
        self.last_base_status = None
        self.last_wheel_states = None
        self.last_status_json = None

        self.create_subscription(String, '/omni/base_status', self.on_base_status, 10)
        self.create_subscription(String, '/omni/wheel_states', self.on_wheel_states, 10)
        self.create_subscription(String, '/omni/status_json', self.on_status_json, 10)
        self.create_subscription(String, '/omni/control_mode_status', self.on_control_mode_status, 10)

    def publish_motion(self, mode: str, speed_pct: int) -> None:
        mode = str(mode).strip().upper()
        if mode not in VALID_MOTION_MODES:
            raise ValueError(f'unsupported motion mode: {mode}')

        speed_pct = max(0, min(100, int(speed_pct)))
        if mode == 'STOP':
            speed_pct = 0

        msg = String()
        msg.data = json.dumps({
            'mode': mode,
            'speed_pct': speed_pct,
        }, separators=(',', ':'))
        self.manual_publisher.publish(msg)

    def publish_control_mode(self, control_mode: str) -> None:
        control_mode = str(control_mode).strip().upper()
        if control_mode not in ('AUTO', 'MANUAL'):
            raise ValueError(f'unsupported control mode: {control_mode}')

        msg = String()
        msg.data = control_mode
        self.mode_publisher.publish(msg)
        with self.state_lock:
            self.current_mode = control_mode

    def on_base_status(self, msg: String) -> None:
        self._store_json('last_base_status', msg.data)

    def on_wheel_states(self, msg: String) -> None:
        self._store_json('last_wheel_states', msg.data)

    def on_status_json(self, msg: String) -> None:
        self._store_json('last_status_json', msg.data)

    def on_control_mode_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        active_source = payload.get('active_source')
        if isinstance(active_source, str):
            active_source = active_source.strip().upper()
            if active_source in ('AUTO', 'MANUAL'):
                with self.state_lock:
                    self.current_mode = active_source

    def _store_json(self, attribute: str, raw: str) -> None:
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            return

        payload['_web_received_time_sec'] = time.time()
        with self.state_lock:
            setattr(self, attribute, payload)

    def get_state_snapshot(self) -> dict:
        with self.state_lock:
            return {
                'control_mode': self.current_mode,
                'base_status': copy.deepcopy(self.last_base_status),
                'wheel_states': copy.deepcopy(self.last_wheel_states),
                'status': copy.deepcopy(self.last_status_json),
            }


teleop_node = None
app = Flask(__name__)


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/move', methods=['POST'])
def api_move():
    global teleop_node
    if teleop_node is None:
        return jsonify({'ok': False, 'error': 'teleop node not initialized'}), 500

    payload = request.get_json(force=True, silent=True) or {}
    mode = str(payload.get('mode', 'STOP')).strip().upper()

    try:
        speed_pct = int(payload.get('speed_pct', 30))
        teleop_node.publish_motion(mode, speed_pct)
    except (TypeError, ValueError) as exc:
        return jsonify({'ok': False, 'error': str(exc)}), 400

    if mode == 'STOP':
        speed_pct = 0

    return jsonify({
        'ok': True,
        'mode': mode,
        'speed_pct': max(0, min(100, speed_pct)),
        'control_mode': teleop_node.get_state_snapshot()['control_mode'],
    })


@app.route('/api/control_mode', methods=['GET'])
def api_get_control_mode():
    global teleop_node
    if teleop_node is None:
        return jsonify({'ok': False, 'error': 'teleop node not initialized'}), 500

    return jsonify({
        'ok': True,
        'control_mode': teleop_node.get_state_snapshot()['control_mode'],
    })


@app.route('/api/control_mode', methods=['POST'])
def api_set_control_mode():
    global teleop_node
    if teleop_node is None:
        return jsonify({'ok': False, 'error': 'teleop node not initialized'}), 500

    payload = request.get_json(force=True, silent=True) or {}
    control_mode = str(payload.get('control_mode', '')).strip().upper()

    if control_mode not in ('AUTO', 'MANUAL'):
        return jsonify({'ok': False, 'error': 'unsupported control mode'}), 400

    teleop_node.publish_control_mode(control_mode)
    return jsonify({
        'ok': True,
        'control_mode': teleop_node.get_state_snapshot()['control_mode'],
    })


@app.route('/api/state', methods=['GET'])
def api_state():
    global teleop_node
    if teleop_node is None:
        return jsonify({'ok': False, 'error': 'teleop node not initialized'}), 500

    return jsonify({
        'ok': True,
        'state': teleop_node.get_state_snapshot(),
    })


def get_web_host_port() -> tuple[str, int]:
    host = os.environ.get('TELEOP_HOST', '0.0.0.0')
    port = int(os.environ.get('TELEOP_PORT', '8080'))
    return host, port


def run_web() -> None:
    host, port = get_web_host_port()
    app.run(host=host, port=port, debug=False, threaded=True, use_reloader=False)


def main() -> None:
    global teleop_node

    rclpy.init()
    teleop_node = TeleopWebNode()

    web_thread = threading.Thread(target=run_web, daemon=True)
    web_thread.start()

    host, port = get_web_host_port()
    teleop_node.get_logger().info(f'teleop web started on {host}:{port}')

    try:
        rclpy.spin(teleop_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        teleop_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
