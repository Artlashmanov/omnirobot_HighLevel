import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String

try:
    import serial
    from serial import SerialException
except ImportError:  # pragma: no cover - handled at runtime on robot
    serial = None

    class SerialException(Exception):
        pass


FRAME_HEADER = b'\x59\x59'
FRAME_SIZE = 9


class TfLunaNode(Node):
    """ROS2 reader for a front-mounted Benewake TF-Luna UART range sensor."""

    def __init__(self):
        super().__init__('tf_luna_front')

        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'tf_luna_front')
        self.declare_parameter('range_topic', '/range/front')
        self.declare_parameter('status_topic', '/sensors/tf_luna/front')
        self.declare_parameter('min_range_m', 0.2)
        self.declare_parameter('max_range_m', 8.0)
        self.declare_parameter('field_of_view_rad', 0.0349066)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('read_timeout_sec', 0.1)
        self.declare_parameter('reconnect_period_sec', 2.0)
        self.declare_parameter('publish_json_status', True)

        self.serial_port = str(self.get_parameter('serial_port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.range_topic = str(self.get_parameter('range_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.min_range_m = float(self.get_parameter('min_range_m').value)
        self.max_range_m = float(self.get_parameter('max_range_m').value)
        self.field_of_view_rad = float(self.get_parameter('field_of_view_rad').value)
        self.publish_period_sec = 1.0 / max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.read_timeout_sec = float(self.get_parameter('read_timeout_sec').value)
        self.reconnect_period_sec = float(self.get_parameter('reconnect_period_sec').value)
        self.publish_json_status = bool(self.get_parameter('publish_json_status').value)

        self.range_pub = self.create_publisher(Range, self.range_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.serial_handle = None
        self.rx_buffer = bytearray()
        self.last_publish_monotonic_sec = 0.0
        self.last_connect_attempt_sec = 0.0
        self.last_error = ''

        self.create_timer(0.02, self.poll)

        self.get_logger().info(
            f'tf_luna_front starting: port={self.serial_port}, baudrate={self.baudrate}, '
            f'range_topic={self.range_topic}, frame_id={self.frame_id}'
        )

    def open_serial(self) -> bool:
        if serial is None:
            self.last_error = 'pyserial is not installed'
            self.get_logger().error(self.last_error)
            return False

        now = time.monotonic()
        if now - self.last_connect_attempt_sec < self.reconnect_period_sec:
            return False
        self.last_connect_attempt_sec = now

        try:
            self.serial_handle = serial.Serial(
                self.serial_port,
                self.baudrate,
                timeout=self.read_timeout_sec,
            )
            self.rx_buffer.clear()
            self.last_error = ''
            self.get_logger().info(f'Connected TF-Luna on {self.serial_port}')
            return True
        except SerialException as exc:
            self.serial_handle = None
            self.last_error = str(exc)
            self.get_logger().warning(f'Cannot open TF-Luna serial port {self.serial_port}: {exc}')
            return False

    def close_serial(self) -> None:
        if self.serial_handle is not None:
            try:
                self.serial_handle.close()
            except Exception:
                pass
        self.serial_handle = None

    def poll(self) -> None:
        if self.serial_handle is None:
            self.open_serial()
            return

        try:
            data = self.serial_handle.read(64)
        except SerialException as exc:
            self.last_error = str(exc)
            self.get_logger().warning(f'TF-Luna read failed: {exc}')
            self.close_serial()
            return

        if data:
            self.rx_buffer.extend(data)

        while len(self.rx_buffer) >= FRAME_SIZE:
            index = self.rx_buffer.find(FRAME_HEADER)
            if index < 0:
                del self.rx_buffer[:-1]
                return
            if index > 0:
                del self.rx_buffer[:index]
            if len(self.rx_buffer) < FRAME_SIZE:
                return

            frame = bytes(self.rx_buffer[:FRAME_SIZE])
            checksum = sum(frame[:8]) & 0xFF
            if checksum != frame[8]:
                del self.rx_buffer[0]
                continue

            del self.rx_buffer[:FRAME_SIZE]
            self.publish_frame(frame)

    def publish_frame(self, frame: bytes) -> None:
        now = time.monotonic()
        if now - self.last_publish_monotonic_sec < self.publish_period_sec:
            return
        self.last_publish_monotonic_sec = now

        distance_cm = frame[2] | (frame[3] << 8)
        strength = frame[4] | (frame[5] << 8)
        temperature_c = ((frame[6] | (frame[7] << 8)) / 8.0) - 256.0
        distance_m = distance_cm / 100.0

        stamp = self.get_clock().now().to_msg()

        range_msg = Range()
        range_msg.header.stamp = stamp
        range_msg.header.frame_id = self.frame_id
        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = self.field_of_view_rad
        range_msg.min_range = self.min_range_m
        range_msg.max_range = self.max_range_m
        range_msg.range = distance_m
        self.range_pub.publish(range_msg)

        if self.publish_json_status:
            status = {
                'type': 'tf_luna',
                'frame_id': self.frame_id,
                'distance_cm': distance_cm,
                'distance_m': distance_m,
                'strength': strength,
                'temperature_c': round(temperature_c, 2),
                'valid_range': self.min_range_m <= distance_m <= self.max_range_m,
                'raw': frame.hex(),
            }
            msg = String()
            msg.data = json.dumps(status, separators=(',', ':'))
            self.status_pub.publish(msg)

    def destroy_node(self):
        self.close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TfLunaNode()
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
