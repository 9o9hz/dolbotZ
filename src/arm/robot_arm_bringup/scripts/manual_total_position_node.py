#!/usr/bin/env python3
import math
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, Float64MultiArray


class ManualTotalPositionNode(Node):
    JOINTS = [
        'shoulder_lift_joint', 'elbow_joint', 'wrist_joint',
        'gripper_joint', 'base_rotate_joint']

    def __init__(self):
        super().__init__('manual_total_position_node')
        self.declare_parameter('command_topic', '/position_controller/commands')
        self.declare_parameter('publish_rate', 20.0)
        # 관절 순서는 JOINTS 및 controller YAML과 동일해야 합니다 (rad/s, rad).
        # Operation rates (RMD: shoulder/elbow/wrist, Dynamixel: gripper/base).
        # self.declare_parameter('rates', [0.17, 0.26, 0.35, 0.25, 0.25])
        # Low-speed calibration/test rates. RMD: 0.03 rad/s (about 1.72 deg/s).
        self.declare_parameter('rates', [0.03, 0.03, 0.03, 0.25, 0.25])
        # Operation angle limits.
        # self.declare_parameter('lower_limits', [0.0, -1.5708, -2.0944, -2.3562, -3.1416])
        # self.declare_parameter('upper_limits', [2.0944, 0.3491, 2.0944, 2.3562, 3.1416])
        # Calibration/test limits: widen only the three RMD joints to +/-720 deg;
        # keep the Dynamixel gripper/base limits unchanged.
        self.declare_parameter('lower_limits', [-12.5664, -12.5664, -12.5664, -2.3562, -3.1416])
        self.declare_parameter('upper_limits', [12.5664, 12.5664, 12.5664, 2.3562, 3.1416])
        self.declare_parameter('shoulder_axis', 3)
        self.declare_parameter('elbow_axis', 4)
        self.declare_parameter('wrist_axis', 1)
        self.declare_parameter('axis_threshold', 0.5)
        self.declare_parameter('gripper_positive_button', 1)
        self.declare_parameter('gripper_negative_button', 3)
        self.declare_parameter('base_yaw_positive_button', 2)
        self.declare_parameter('base_yaw_negative_button', 0)
        self.declare_parameter('joy_timeout', 0.5)
        self.declare_parameter('protective_stop_deceleration_time', 0.5)    #0.5초간 선형 감소 (1초 동안 감속을 원하면 이거 값 0.5->1로 변경)

        self.rates = list(map(float, self.get_parameter('rates').value))
        self.lower = list(map(float, self.get_parameter('lower_limits').value))
        self.upper = list(map(float, self.get_parameter('upper_limits').value))
        if not (len(self.rates) == len(self.lower) == len(self.upper) == len(self.JOINTS)):
            raise ValueError('rates/lower_limits/upper_limits must follow the controller joint order')
        self.shoulder_axis = int(self.get_parameter('shoulder_axis').value)
        self.elbow_axis = int(self.get_parameter('elbow_axis').value)
        self.wrist_axis = int(self.get_parameter('wrist_axis').value)
        self.axis_threshold = float(self.get_parameter('axis_threshold').value)
        self.button_indices = [int(self.get_parameter(n).value) for n in
                               ('gripper_positive_button', 'gripper_negative_button',
                                'base_yaw_positive_button', 'base_yaw_negative_button')]
        self.joy_timeout = float(self.get_parameter('joy_timeout').value)
        self.protective_stop_deceleration_time = max(
            float(self.get_parameter('protective_stop_deceleration_time').value), 0.0)
        self.positions: Dict[str, float] = {}
        self.velocities: Dict[str, float] = {}
        self.target: Optional[list] = None
        self.joy: Optional[Joy] = None
        self.manual_enabled = False
        self.protective_stop = False
        self.protective_stop_reported = False
        self.protective_stop_start = None
        self.protective_stop_initial_velocity = [0.0] * len(self.JOINTS)
        self.last_joy_time = self.get_clock().now()
        self.last_time = self.get_clock().now()
        self.publisher = self.create_publisher(Float64MultiArray, self.get_parameter('command_topic').value, 10)
        self.create_subscription(JointState, '/joint_states', self.on_state, 10)
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        mode_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, '/control/manual_enabled', self.on_manual_enabled, mode_qos)
        self.create_subscription(
            Bool, '/control/protective_stop', self.on_protective_stop, mode_qos)
        self.create_timer(1.0 / float(self.get_parameter('publish_rate').value), self.on_timer)

    def on_state(self, msg):
        self.positions.update(zip(msg.name, msg.position))
        self.velocities.update(zip(msg.name, msg.velocity))
        if self.target is None and all(j in self.positions for j in self.JOINTS):
            self.target = [min(max(self.positions[j], self.lower[i]), self.upper[i]) for i, j in enumerate(self.JOINTS)]
            self.get_logger().info('All joint targets initialized from /joint_states; manual commands are now safe to publish.')

    def on_joy(self, msg):
        self.joy = msg
        self.last_joy_time = self.get_clock().now()

    def on_manual_enabled(self, msg):
        enabled = bool(msg.data)
        if enabled and not self.manual_enabled:
            # Always discard the old manual target when entering MANUAL. The next
            # command is initialized from the newest measured joint positions.
            self.target = None
            self.get_logger().info('MANUAL mode enabled; synchronizing targets from /joint_states.')
        elif not enabled and self.manual_enabled:
            self.get_logger().info('MANUAL mode disabled; command publishing stopped.')
        self.manual_enabled = enabled

    def on_protective_stop(self, msg):
        if not msg.data or self.protective_stop:
            return
        self.protective_stop = True
        self.protective_stop_reported = False
        self.protective_stop_start = self.get_clock().now()
        # Begin the stop trajectory at measured position. Shoulder, elbow, and
        # base-yaw retain their measured velocity and ramp it linearly to zero;
        # the gripper immediately holds its measured position.
        if all(j in self.positions for j in self.JOINTS):
            self.target = [self.positions[j] for j in self.JOINTS]
        self.protective_stop_initial_velocity = [
            0.0 if joint == 'gripper_joint' else self.velocities.get(joint, 0.0)
            for joint in self.JOINTS]
        self.get_logger().fatal(
            'Protective E-stop received: operator commands blocked; holding position with torque enabled.')

    def axis(self, index):
        if self.joy is None or index < 0 or index >= len(self.joy.axes):
            return 0.0
        value = self.joy.axes[index]
        return 1.0 if value > self.axis_threshold else (-1.0 if value < -self.axis_threshold else 0.0)

    def button(self, index):
        return self.joy is not None and 0 <= index < len(self.joy.buttons) and self.joy.buttons[index] != 0

    def on_timer(self):
        now = self.get_clock().now()
        dt = min(max((now - self.last_time).nanoseconds * 1e-9, 0.0), 0.1)
        self.last_time = now
        if self.target is None:
            return
        if self.protective_stop:
            elapsed = (now - self.protective_stop_start).nanoseconds * 1e-9
            if self.protective_stop_deceleration_time > 0.0:
                scale = max(1.0 - elapsed / self.protective_stop_deceleration_time, 0.0)
            else:
                scale = 0.0
            for i, initial_velocity in enumerate(self.protective_stop_initial_velocity):
                self.target[i] = min(max(
                    self.target[i] + initial_velocity * scale * dt,
                    self.lower[i]), self.upper[i])
            self.publisher.publish(Float64MultiArray(data=self.target))
            if (not self.protective_stop_reported
                    and scale == 0.0
                    and all(j in self.velocities for j in self.JOINTS)
                    and max(abs(self.velocities[j]) for j in self.JOINTS) <= 0.02):
                self.protective_stop_reported = True
                self.get_logger().fatal(
                    'Protective E-stop settled: all measured joint velocities are <= 0.02 rad/s.')
            return
        if not self.manual_enabled:
            return
        directions = [0.0] * len(self.JOINTS)
        joy_fresh = (now - self.last_joy_time).nanoseconds * 1e-9 <= self.joy_timeout
        if self.joy is not None and joy_fresh:
            directions = [
                self.axis(self.shoulder_axis),
                self.axis(self.elbow_axis),
                self.axis(self.wrist_axis),
                float(self.button(self.button_indices[0])) - float(self.button(self.button_indices[1])),
                float(self.button(self.button_indices[2])) - float(self.button(self.button_indices[3])),
            ]
        for i, direction in enumerate(directions):
            self.target[i] = min(max(self.target[i] + direction * self.rates[i] * dt, self.lower[i]), self.upper[i])
        self.publisher.publish(Float64MultiArray(data=self.target))

def main(args=None):
    rclpy.init(args=args)
    node = ManualTotalPositionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
