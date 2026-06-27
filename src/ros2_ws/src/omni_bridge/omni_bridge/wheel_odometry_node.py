import json
import math
import time
from typing import Iterable

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


MOTION_STOP = 'STOP'
MOTION_FORWARD = 'FORWARD'
MOTION_BACKWARD = 'BACKWARD'
MOTION_LEFT = 'LEFT'
MOTION_RIGHT = 'RIGHT'
MOTION_ROTATE_CCW = 'ROTATE_CCW'
MOTION_ROTATE_CW = 'ROTATE_CW'


def normalize_angle(value: float) -> float:
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def parameter_list(value: object, length: int, default: Iterable[float]) -> list[float]:
    if isinstance(value, (list, tuple)) and len(value) == length:
        return [float(v) for v in value]
    return [float(v) for v in default]


class WheelOdometryNode(Node):
    """Dead-reckoning odometry from STM32 wheel telemetry.

    The current STM32 protocol exposes discrete base motion mode plus per-wheel
    encoder deltas. Until full omni wheel kinematics are calibrated, this node
    projects encoder magnitude through the active motion mode. The scale factors
    are intentionally parameters, because they must be calibrated on the real
    robot before SLAM/navigation relies on metric accuracy.
    """

    def __init__(self):
        super().__init__('omni_odometry')

        self.declare_parameter('wheel_states_topic', '/omni/wheel_states')
        self.declare_parameter('base_status_topic', '/omni/base_status')
        self.declare_parameter('motion_cmd_topic', '/omni/motion_cmd')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('laser_frame_id', 'laser')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_laser_tf', True)
        self.declare_parameter('wheel_count', 4)
        self.declare_parameter('meters_per_tick', 0.00010)
        self.declare_parameter('radians_per_tick', 0.00010)
        self.declare_parameter('publish_period_sec', 0.05)
        self.declare_parameter('velocity_timeout_sec', 0.35)
        self.declare_parameter('stale_wheel_age_sec', 0.6)
        self.declare_parameter('min_delta_ticks', 0)
        self.declare_parameter('laser_xyz', [0.0, 0.0, 0.12])
        self.declare_parameter('laser_rpy', [0.0, 0.0, 0.0])

        self.wheel_states_topic = str(self.get_parameter('wheel_states_topic').value)
        self.base_status_topic = str(self.get_parameter('base_status_topic').value)
        self.motion_cmd_topic = str(self.get_parameter('motion_cmd_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.laser_frame_id = str(self.get_parameter('laser_frame_id').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.publish_laser_tf = bool(self.get_parameter('publish_laser_tf').value)
        self.wheel_count = max(1, int(self.get_parameter('wheel_count').value))
        self.meters_per_tick = float(self.get_parameter('meters_per_tick').value)
        self.radians_per_tick = float(self.get_parameter('radians_per_tick').value)
        self.publish_period_sec = float(self.get_parameter('publish_period_sec').value)
        self.velocity_timeout_sec = float(self.get_parameter('velocity_timeout_sec').value)
        self.stale_wheel_age_sec = float(self.get_parameter('stale_wheel_age_sec').value)
        self.min_delta_ticks = int(self.get_parameter('min_delta_ticks').value)
        self.laser_xyz = parameter_list(self.get_parameter('laser_xyz').value, 3, [0.0, 0.0, 0.12])
        self.laser_rpy = parameter_list(self.get_parameter('laser_rpy').value, 3, [0.0, 0.0, 0.0])

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0
        self.motion_mode_name = MOTION_STOP
        self.last_wheel_update_times: dict[int, float] = {}
        self.last_motion_monotonic_sec = 0.0

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.create_subscription(String, self.base_status_topic, self.on_base_status, 20)
        self.create_subscription(String, self.motion_cmd_topic, self.on_motion_cmd, 20)
        self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 20)
        self.create_subscription(String, self.wheel_states_topic, self.on_wheel_states, 30)
        self.create_timer(self.publish_period_sec, self.publish_odometry)

        if self.publish_laser_tf:
            self.publish_static_laser_transform()

        self.get_logger().info(
            f'omni_odometry started: wheel_states={self.wheel_states_topic}, '
            f'motion_cmd={self.motion_cmd_topic}, cmd_vel={self.cmd_vel_topic}, odom={self.odom_topic}, '
            f'frames={self.odom_frame_id}->{self.base_frame_id}->{self.laser_frame_id}, '
            f'meters_per_tick={self.meters_per_tick}, radians_per_tick={self.radians_per_tick}'
        )

    def set_motion_mode(self, mode: str, source: str) -> None:
        normalized = str(mode or MOTION_STOP).upper()
        if normalized not in {
            MOTION_STOP,
            MOTION_FORWARD,
            MOTION_BACKWARD,
            MOTION_LEFT,
            MOTION_RIGHT,
            MOTION_ROTATE_CCW,
            MOTION_ROTATE_CW,
        }:
            self.get_logger().warning(f'Ignoring unknown motion mode from {source}: {mode}')
            return

        self.motion_mode_name = normalized
        if normalized == MOTION_STOP:
            self.vx = 0.0
            self.vy = 0.0
            self.vyaw = 0.0

    def on_motion_cmd(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f'Invalid motion_cmd JSON: {exc}')
            return

        self.set_motion_mode(str(payload.get('mode') or MOTION_STOP), source='motion_cmd')

    def on_cmd_vel(self, msg: Twist) -> None:
        linear_x = float(msg.linear.x)
        linear_y = float(msg.linear.y)
        angular_z = float(msg.angular.z)
        deadband = 1e-4

        if abs(linear_x) < deadband and abs(linear_y) < deadband and abs(angular_z) < deadband:
            self.set_motion_mode(MOTION_STOP, source='cmd_vel')
            return

        if abs(linear_x) >= abs(linear_y) and abs(linear_x) >= abs(angular_z):
            self.set_motion_mode(MOTION_FORWARD if linear_x > 0.0 else MOTION_BACKWARD, source='cmd_vel')
        elif abs(linear_y) >= abs(angular_z):
            self.set_motion_mode(MOTION_LEFT if linear_y > 0.0 else MOTION_RIGHT, source='cmd_vel')
        else:
            self.set_motion_mode(MOTION_ROTATE_CCW if angular_z > 0.0 else MOTION_ROTATE_CW, source='cmd_vel')

    def on_base_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f'Invalid base_status JSON: {exc}')
            return

        self.set_motion_mode(str(payload.get('motion_mode_name') or MOTION_STOP), source='base_status')

    def on_wheel_states(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f'Invalid wheel_states JSON: {exc}')
            return

        wheels = payload.get('wheels') or []
        if not isinstance(wheels, list):
            return

        wheel_count = int(payload.get('wheel_count') or self.wheel_count or len(wheels) or 1)
        wheel_count = max(1, wheel_count)

        new_tick_magnitude = 0.0
        new_update_count = 0
        fresh_speed_sum = 0.0
        fresh_speed_count = 0

        for wheel in wheels:
            if not isinstance(wheel, dict):
                continue
            if not bool(wheel.get('present', True)):
                continue

            try:
                wheel_index = int(wheel.get('wheel_index'))
            except (TypeError, ValueError):
                continue

            try:
                update_time = float(wheel.get('last_update_monotonic_sec', 0.0))
            except (TypeError, ValueError):
                update_time = 0.0

            age = float(wheel.get('last_update_age_sec', 0.0) or 0.0)
            if age <= self.stale_wheel_age_sec:
                try:
                    fresh_speed_sum += abs(float(wheel.get('speed_ticks_per_sec', 0.0) or 0.0))
                    fresh_speed_count += 1
                except (TypeError, ValueError):
                    pass

            if self.last_wheel_update_times.get(wheel_index) == update_time:
                continue
            self.last_wheel_update_times[wheel_index] = update_time

            try:
                delta_ticks = int(wheel.get('delta_ticks', 0) or 0)
            except (TypeError, ValueError):
                delta_ticks = 0

            if abs(delta_ticks) <= self.min_delta_ticks:
                continue

            new_tick_magnitude += abs(float(delta_ticks))
            new_update_count += 1

        if new_update_count == 0:
            return

        # WHEEL_STATE is sent round-robin, so each aggregate usually contains
        # one new wheel sample. Dividing by wheel_count makes one full cycle of
        # four similar wheel deltas integrate as one robot motion delta.
        ticks = new_tick_magnitude / float(wheel_count)
        speed_ticks = fresh_speed_sum / float(fresh_speed_count) if fresh_speed_count else 0.0
        self.integrate_motion_mode_projection(ticks, speed_ticks)

    def integrate_motion_mode_projection(self, ticks: float, speed_ticks_per_sec: float) -> None:
        mode = self.motion_mode_name
        dx_body = 0.0
        dy_body = 0.0
        dyaw = 0.0
        vx_body = 0.0
        vy_body = 0.0
        vyaw = 0.0

        distance = ticks * self.meters_per_tick
        angle = ticks * self.radians_per_tick
        linear_speed = speed_ticks_per_sec * self.meters_per_tick
        angular_speed = speed_ticks_per_sec * self.radians_per_tick

        if mode == MOTION_FORWARD:
            dx_body = distance
            vx_body = linear_speed
        elif mode == MOTION_BACKWARD:
            dx_body = -distance
            vx_body = -linear_speed
        elif mode == MOTION_LEFT:
            dy_body = distance
            vy_body = linear_speed
        elif mode == MOTION_RIGHT:
            dy_body = -distance
            vy_body = -linear_speed
        elif mode == MOTION_ROTATE_CCW:
            dyaw = angle
            vyaw = angular_speed
        elif mode == MOTION_ROTATE_CW:
            dyaw = -angle
            vyaw = -angular_speed
        else:
            self.vx = 0.0
            self.vy = 0.0
            self.vyaw = 0.0
            return

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += cos_yaw * dx_body - sin_yaw * dy_body
        self.y += sin_yaw * dx_body + cos_yaw * dy_body
        self.yaw = normalize_angle(self.yaw + dyaw)
        self.vx = vx_body
        self.vy = vy_body
        self.vyaw = vyaw
        self.last_motion_monotonic_sec = time.monotonic()

    def publish_static_laser_transform(self) -> None:
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame_id
        msg.child_frame_id = self.laser_frame_id
        msg.transform.translation.x = self.laser_xyz[0]
        msg.transform.translation.y = self.laser_xyz[1]
        msg.transform.translation.z = self.laser_xyz[2]
        qx, qy, qz, qw = quaternion_from_rpy(*self.laser_rpy)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        self.static_tf_broadcaster.sendTransform(msg)

    def publish_odometry(self) -> None:
        if time.monotonic() - self.last_motion_monotonic_sec > self.velocity_timeout_sec:
            self.vx = 0.0
            self.vy = 0.0
            self.vyaw = 0.0

        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vyaw
        odom.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.20,
        ]
        odom.twist.covariance = [
            0.10, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.10, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.40,
        ]
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame_id
            tf_msg.child_frame_id = self.base_frame_id
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
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
