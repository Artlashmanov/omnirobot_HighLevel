#!/usr/bin/env python3
import copy
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import threading
import time
from flask import Flask, jsonify, render_template, request

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String

from omni_pi.platforms import load_platform_profile, normalize_motion_payload



PROJECT_ROOT = Path(os.environ.get('OMNI_HOME', Path(__file__).resolve().parents[1])).resolve()
MAP_NAME_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$')


def get_maps_dir() -> Path:
    return Path(os.environ.get('OMNI_MAPS_DIR', str(PROJECT_ROOT / 'maps'))).resolve()


def sanitize_map_name(raw_name: str | None) -> str:
    name = (raw_name or '').strip()
    if not name:
        name = dt.datetime.now().strftime('map_%Y%m%d_%H%M%S')
    name = re.sub(r'[^A-Za-z0-9_.-]+', '_', name).strip('._-')
    if not name:
        name = dt.datetime.now().strftime('map_%Y%m%d_%H%M%S')
    if len(name) > 64:
        name = name[:64].rstrip('._-')
    if not MAP_NAME_RE.fullmatch(name):
        raise ValueError('map name must use letters, digits, dot, dash or underscore')
    return name


def read_json_file(path: Path) -> dict | None:
    try:
        with path.open('r', encoding='utf-8') as fh:
            payload = json.load(fh)
        return payload if isinstance(payload, dict) else None
    except (OSError, json.JSONDecodeError):
        return None

def project_script_path(env_name: str, default_name: str) -> Path:
    return Path(os.environ.get(env_name, str(PROJECT_ROOT / 'tools' / default_name))).resolve()


def parse_json_from_stdout(stdout: str) -> dict | None:
    for line in reversed((stdout or '').splitlines()):
        stripped = line.strip()
        if not stripped.startswith('{'):
            continue
        try:
            payload = json.loads(stripped)
        except json.JSONDecodeError:
            continue
        if isinstance(payload, dict):
            return payload
    return None


def run_project_script(
    env_name: str,
    default_name: str,
    args: list[str] | None = None,
    timeout_env: str = 'TELEOP_MAP_ACTION_TIMEOUT_SEC',
    default_timeout_sec: int = 30,
) -> dict:
    script = project_script_path(env_name, default_name)
    if not script.exists():
        raise RuntimeError(f'map action script not found: {script}')

    env = os.environ.copy()
    env.setdefault('OMNI_HOME', str(PROJECT_ROOT))
    env.setdefault('OMNI_MAPS_DIR', str(get_maps_dir()))

    timeout_sec = int(os.environ.get(timeout_env, str(default_timeout_sec)))
    completed = subprocess.run(
        [str(script), *(args or [])],
        cwd=str(PROJECT_ROOT),
        env=env,
        text=True,
        capture_output=True,
        timeout=timeout_sec,
        check=False,
    )

    stdout = completed.stdout.strip()
    stderr = completed.stderr.strip()
    if completed.returncode != 0:
        detail = stderr or stdout or f'exit code {completed.returncode}'
        raise RuntimeError(f'{default_name} failed: {detail}')

    return {
        'stdout': stdout,
        'stderr': stderr,
        'tool': parse_json_from_stdout(stdout),
    }



def map_entry(map_dir: Path) -> dict:
    yaml_file = map_dir / 'map.yaml'
    pgm_file = map_dir / 'map.pgm'
    png_file = map_dir / 'map.png'
    metadata_file = map_dir / 'metadata.json'
    posegraph_files = sorted(
        item.name
        for item in map_dir.iterdir()
        if item.is_file() and item.name.startswith('slam_posegraph')
    )
    files = [item for item in map_dir.iterdir() if item.is_file()]
    modified = max((item.stat().st_mtime for item in files), default=map_dir.stat().st_mtime)
    total_size = sum(item.stat().st_size for item in files)

    return {
        'name': map_dir.name,
        'path': str(map_dir),
        'modified_time_sec': modified,
        'total_size_bytes': total_size,
        'has_occupancy_grid': yaml_file.exists() and (pgm_file.exists() or png_file.exists()),
        'has_posegraph': bool(posegraph_files),
        'files': sorted(item.name for item in files),
        'posegraph_files': posegraph_files,
        'metadata': read_json_file(metadata_file),
    }


def list_saved_maps() -> list[dict]:
    maps_dir = get_maps_dir()
    if not maps_dir.exists():
        return []
    entries = [
        map_entry(item)
        for item in maps_dir.iterdir()
        if item.is_dir() and not item.name.startswith('.')
    ]
    entries.sort(key=lambda item: item['modified_time_sec'], reverse=True)
    return entries


def save_current_map(raw_name: str | None) -> dict:
    name = sanitize_map_name(raw_name)
    maps_dir = get_maps_dir()
    result = run_project_script(
        'OMNI_SAVE_MAP_SCRIPT',
        'save_map.sh',
        [name],
        timeout_env='TELEOP_MAP_SAVE_TIMEOUT_SEC',
        default_timeout_sec=60,
    )

    saved_dir = maps_dir / name
    return {
        'name': name,
        **result,
        'map': map_entry(saved_dir) if saved_dir.exists() else None,
    }


def reset_slam_map() -> dict:
    return run_project_script(
        'OMNI_RESET_SLAM_SCRIPT',
        'reset_slam.sh',
        timeout_env='TELEOP_SLAM_RESET_TIMEOUT_SEC',
        default_timeout_sec=20,
    )


def start_new_map() -> dict:
    return run_project_script(
        'OMNI_START_NEW_MAP_SCRIPT',
        'start_new_map.sh',
        timeout_env='TELEOP_START_NEW_MAP_TIMEOUT_SEC',
        default_timeout_sec=25,
    )


def load_saved_map(raw_name: str | None) -> dict:
    name = sanitize_map_name(raw_name)
    map_dir = get_maps_dir() / name
    if not map_dir.exists() or not map_dir.is_dir():
        raise ValueError(f'saved map not found: {name}')

    result = run_project_script(
        'OMNI_LOAD_MAP_SCRIPT',
        'load_map.sh',
        [name],
        timeout_env='TELEOP_MAP_LOAD_TIMEOUT_SEC',
        default_timeout_sec=35,
    )
    return {
        'name': name,
        **result,
        'map': map_entry(map_dir),
    }


class TeleopWebNode(Node):
    def __init__(self) -> None:
        super().__init__('teleop_web_node')
        self.platform = load_platform_profile()
        self.manual_publisher = self.create_publisher(String, '/omni/manual_cmd', 10)
        self.mode_publisher = self.create_publisher(String, '/omni/control_mode', 10)

        self.state_lock = threading.Lock()
        self.current_mode = 'AUTO'
        self.last_base_status = None
        self.last_wheel_states = None
        self.last_status_json = None
        self.last_range_front = None
        self.last_tf_luna_status = None
        self.last_power_status = None
        self.last_scan_summary = None
        self.last_scan_preview = None
        self.last_map_preview = None
        self.last_scan_summary_monotonic_sec = 0.0
        self.map_max_dim = int(os.environ.get('TELEOP_MAP_MAX_DIM', '220'))
        self.scan_preview_max_points = int(os.environ.get('TELEOP_SCAN_PREVIEW_MAX_POINTS', '360'))

        self.create_subscription(String, '/omni/base_status', self.on_base_status, 10)
        self.create_subscription(String, '/omni/wheel_states', self.on_wheel_states, 10)
        self.create_subscription(String, '/omni/status_json', self.on_status_json, 10)
        self.create_subscription(String, '/omni/control_mode_status', self.on_control_mode_status, 10)
        self.create_subscription(Range, '/range/front', self.on_range_front, 10)
        self.create_subscription(String, '/sensors/tf_luna/front', self.on_tf_luna_status, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, 1)

        # slam_toolbox publishes /map as TRANSIENT_LOCAL, so this subscription
        # receives the last map immediately after the web UI restarts. The
        # volatile subscription above keeps compatibility with other map sources.
        map_latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_latched_qos)

        for topic in self.power_status_topics():
            self.create_subscription(
                String,
                topic,
                lambda msg, source_topic=topic: self.on_power_status(msg, source_topic),
                10,
            )

    def publish_motion(self, mode: str, speed_pct: int) -> dict:
        command = normalize_motion_payload({
            'mode': mode,
            'speed_pct': speed_pct,
        }, self.platform)

        msg = String()
        msg.data = json.dumps(command, separators=(',', ':'))
        self.manual_publisher.publish(msg)
        return command

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

    def on_tf_luna_status(self, msg: String) -> None:
        self._store_json('last_tf_luna_status', msg.data)

    def on_power_status(self, msg: String, source_topic: str) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        payload.setdefault('type', 'power_status')
        payload['_source_topic'] = source_topic
        payload['_web_received_time_sec'] = time.time()
        with self.state_lock:
            self.last_power_status = payload

    def on_range_front(self, msg: Range) -> None:
        payload = {
            'type': 'range',
            'frame_id': msg.header.frame_id,
            'stamp': self.stamp_to_dict(msg.header.stamp),
            'radiation_type': int(msg.radiation_type),
            'field_of_view_rad': float(msg.field_of_view),
            'min_range_m': float(msg.min_range),
            'max_range_m': float(msg.max_range),
            'range_m': float(msg.range),
            '_web_received_time_sec': time.time(),
        }
        with self.state_lock:
            self.last_range_front = payload

    def on_scan(self, msg: LaserScan) -> None:
        now = time.monotonic()
        if now - self.last_scan_summary_monotonic_sec < 0.25:
            return
        self.last_scan_summary_monotonic_sec = now

        summary = self.build_scan_summary(msg)
        preview = self.build_scan_preview(msg)
        received_time_sec = time.time()
        summary['_web_received_time_sec'] = received_time_sec
        preview['_web_received_time_sec'] = received_time_sec
        with self.state_lock:
            self.last_scan_summary = summary
            self.last_scan_preview = preview

    def on_map(self, msg: OccupancyGrid) -> None:
        payload = self.build_map_preview(msg)
        payload['_web_received_time_sec'] = time.time()
        with self.state_lock:
            self.last_map_preview = payload

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

    @staticmethod
    def stamp_to_dict(stamp) -> dict:
        return {
            'sec': int(stamp.sec),
            'nanosec': int(stamp.nanosec),
        }

    @staticmethod
    def power_status_topics() -> list[str]:
        configured = os.environ.get(
            'TELEOP_POWER_STATUS_TOPICS',
            os.environ.get('TELEOP_POWER_STATUS_TOPIC', '/sensors/ina228,/omni/power_status'),
        )
        topics = []
        for item in configured.split(','):
            topic = item.strip()
            if topic and topic not in topics:
                topics.append(topic)
        return topics

    @staticmethod
    def min_sector_range(msg: LaserScan, center_rad: float, width_rad: float) -> float | None:
        best = None
        half_width = width_rad / 2.0
        angle = float(msg.angle_min)
        increment = float(msg.angle_increment)
        range_min = float(msg.range_min)
        range_max = float(msg.range_max)

        for value in msg.ranges:
            distance = float(value)
            if math.isfinite(distance) and range_min <= distance <= range_max:
                diff = math.atan2(math.sin(angle - center_rad), math.cos(angle - center_rad))
                if abs(diff) <= half_width and (best is None or distance < best):
                    best = distance
            angle += increment

        return best

    def build_scan_summary(self, msg: LaserScan) -> dict:
        valid_values = [
            float(value)
            for value in msg.ranges
            if math.isfinite(float(value)) and float(msg.range_min) <= float(value) <= float(msg.range_max)
        ]
        nearest = min(valid_values) if valid_values else None
        sector_width = math.radians(30.0)

        return {
            'type': 'laser_scan_summary',
            'frame_id': msg.header.frame_id,
            'stamp': self.stamp_to_dict(msg.header.stamp),
            'range_min_m': float(msg.range_min),
            'range_max_m': float(msg.range_max),
            'sample_count': len(msg.ranges),
            'valid_sample_count': len(valid_values),
            'nearest_m': nearest,
            'front_m': self.min_sector_range(msg, 0.0, sector_width),
            'left_m': self.min_sector_range(msg, math.pi / 2.0, sector_width),
            'right_m': self.min_sector_range(msg, -math.pi / 2.0, sector_width),
            'back_m': self.min_sector_range(msg, math.pi, sector_width),
        }

    def build_scan_preview(self, msg: LaserScan) -> dict:
        max_points = max(36, int(self.scan_preview_max_points))
        step = max(1, math.ceil(len(msg.ranges) / max_points))
        points = []
        angle = float(msg.angle_min)
        increment = float(msg.angle_increment)
        range_min = float(msg.range_min)
        range_max = float(msg.range_max)

        for index, value in enumerate(msg.ranges):
            if index % step != 0:
                angle += increment
                continue

            distance = float(value)
            if math.isfinite(distance) and range_min <= distance <= range_max:
                points.append({
                    'x': round(math.cos(angle) * distance, 3),
                    'y': round(math.sin(angle) * distance, 3),
                    'r': round(distance, 3),
                })
            angle += increment

        return {
            'type': 'laser_scan_preview',
            'frame_id': msg.header.frame_id,
            'stamp': self.stamp_to_dict(msg.header.stamp),
            'range_min_m': range_min,
            'range_max_m': range_max,
            'source_sample_count': len(msg.ranges),
            'downsample_step': step,
            'point_count': len(points),
            'points': points,
        }

    def build_map_preview(self, msg: OccupancyGrid) -> dict:
        source_width = int(msg.info.width)
        source_height = int(msg.info.height)
        if source_width <= 0 or source_height <= 0:
            return {
                'type': 'occupancy_grid_preview',
                'available': False,
                'error': 'empty map',
            }

        max_dim = max(32, int(self.map_max_dim))
        step = max(1, math.ceil(max(source_width, source_height) / max_dim))
        width = math.ceil(source_width / step)
        height = math.ceil(source_height / step)
        data = msg.data
        cells = []

        for out_y in range(height):
            y0 = out_y * step
            y1 = min(source_height, y0 + step)
            for out_x in range(width):
                x0 = out_x * step
                x1 = min(source_width, x0 + step)
                has_occupied = False
                has_free = False

                for source_y in range(y0, y1):
                    row_offset = source_y * source_width
                    for source_x in range(x0, x1):
                        value = int(data[row_offset + source_x])
                        if value >= 65:
                            has_occupied = True
                        elif value >= 0:
                            has_free = True

                if has_occupied:
                    cells.append(100)
                elif has_free:
                    cells.append(0)
                else:
                    cells.append(-1)

        content_hash = hashlib.sha1(bytes((cell + 1) & 0xFF for cell in cells)).hexdigest()[:12]

        return {
            'type': 'occupancy_grid_preview',
            'available': True,
            'frame_id': msg.header.frame_id,
            'stamp': self.stamp_to_dict(msg.header.stamp),
            'source_width': source_width,
            'source_height': source_height,
            'width': width,
            'height': height,
            'downsample_step': step,
            'resolution_m': float(msg.info.resolution) * step,
            'source_resolution_m': float(msg.info.resolution),
            'origin': {
                'x': float(msg.info.origin.position.x),
                'y': float(msg.info.origin.position.y),
                'z': float(msg.info.origin.position.z),
            },
            'content_hash': content_hash,
            'cells': cells,
        }

    def get_state_snapshot(self) -> dict:
        with self.state_lock:
            return {
                'server_time_sec': time.time(),
                'control_mode': self.current_mode,
                'base_status': copy.deepcopy(self.last_base_status),
                'wheel_states': copy.deepcopy(self.last_wheel_states),
                'status': copy.deepcopy(self.last_status_json),
                'range_front': copy.deepcopy(self.last_range_front),
                'tf_luna_status': copy.deepcopy(self.last_tf_luna_status),
                'power_status': copy.deepcopy(self.last_power_status),
                'scan_summary': copy.deepcopy(self.last_scan_summary),
                'platform': self.platform.as_public_dict(),
            }

    def get_map_snapshot(self) -> dict | None:
        with self.state_lock:
            return copy.deepcopy(self.last_map_preview)

    def clear_map_snapshot(self) -> None:
        with self.state_lock:
            self.last_map_preview = None

    def get_scan_snapshot(self) -> dict | None:
        with self.state_lock:
            return copy.deepcopy(self.last_scan_preview)


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
        command = teleop_node.publish_motion(mode, speed_pct)
    except (TypeError, ValueError) as exc:
        return jsonify({'ok': False, 'error': str(exc)}), 400

    return jsonify({
        'ok': True,
        'mode': command['mode'],
        'speed_pct': command['speed_pct'],
        'control_mode': teleop_node.get_state_snapshot()['control_mode'],
    })


@app.route('/api/platform', methods=['GET'])
def api_platform():
    global teleop_node
    if teleop_node is None:
        return jsonify({'ok': False, 'error': 'teleop node not initialized'}), 500

    return jsonify({
        'ok': True,
        'platform': teleop_node.platform.as_public_dict(),
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


@app.route('/api/map', methods=['GET'])
def api_map():
    global teleop_node
    if teleop_node is None:
        return jsonify({'ok': False, 'error': 'teleop node not initialized'}), 500

    response = jsonify({
        'ok': True,
        'server_time_sec': time.time(),
        'map': teleop_node.get_map_snapshot(),
    })
    response.headers['Cache-Control'] = 'no-store, max-age=0'
    return response



@app.route('/api/maps', methods=['GET'])
def api_maps():
    return jsonify({
        'ok': True,
        'maps_dir': str(get_maps_dir()),
        'maps': list_saved_maps(),
    })


@app.route('/api/maps/save', methods=['POST'])
def api_save_map():
    payload = request.get_json(force=True, silent=True) or {}
    try:
        result = save_current_map(payload.get('name'))
    except ValueError as exc:
        return jsonify({'ok': False, 'error': str(exc)}), 400
    except subprocess.TimeoutExpired:
        return jsonify({'ok': False, 'error': 'map save timed out'}), 504
    except RuntimeError as exc:
        return jsonify({'ok': False, 'error': str(exc)}), 500

    return jsonify({
        'ok': True,
        'result': result,
        'maps_dir': str(get_maps_dir()),
        'maps': list_saved_maps(),
    })


@app.route('/api/maps/start_new', methods=['POST'])
def api_start_new_map():
    global teleop_node
    try:
        result = start_new_map()
    except subprocess.TimeoutExpired:
        return jsonify({'ok': False, 'error': 'start new map timed out'}), 504
    except RuntimeError as exc:
        return jsonify({'ok': False, 'error': str(exc)}), 500

    if teleop_node is not None:
        teleop_node.clear_map_snapshot()

    return jsonify({
        'ok': True,
        'result': result,
        'maps_dir': str(get_maps_dir()),
        'maps': list_saved_maps(),
    })


@app.route('/api/slam/reset', methods=['POST'])
def api_reset_slam():
    global teleop_node
    try:
        result = reset_slam_map()
    except subprocess.TimeoutExpired:
        return jsonify({'ok': False, 'error': 'SLAM reset timed out'}), 504
    except RuntimeError as exc:
        return jsonify({'ok': False, 'error': str(exc)}), 500

    if teleop_node is not None:
        teleop_node.clear_map_snapshot()

    return jsonify({
        'ok': True,
        'result': result,
        'maps_dir': str(get_maps_dir()),
        'maps': list_saved_maps(),
    })


@app.route('/api/maps/load', methods=['POST'])
def api_load_map():
    global teleop_node
    payload = request.get_json(force=True, silent=True) or {}
    try:
        result = load_saved_map(payload.get('name'))
    except ValueError as exc:
        return jsonify({'ok': False, 'error': str(exc)}), 400
    except subprocess.TimeoutExpired:
        return jsonify({'ok': False, 'error': 'map load timed out'}), 504
    except RuntimeError as exc:
        return jsonify({'ok': False, 'error': str(exc)}), 500

    if teleop_node is not None:
        teleop_node.clear_map_snapshot()

    return jsonify({
        'ok': True,
        'result': result,
        'maps_dir': str(get_maps_dir()),
        'maps': list_saved_maps(),
    })


@app.route('/api/scan', methods=['GET'])
def api_scan():
    global teleop_node
    if teleop_node is None:
        return jsonify({'ok': False, 'error': 'teleop node not initialized'}), 500

    response = jsonify({
        'ok': True,
        'server_time_sec': time.time(),
        'scan': teleop_node.get_scan_snapshot(),
    })
    response.headers['Cache-Control'] = 'no-store, max-age=0'
    return response


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
