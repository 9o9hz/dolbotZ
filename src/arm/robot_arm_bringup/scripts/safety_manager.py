#!/usr/bin/env python3
"""Operator mode selection and latched controlled protective stop."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String


class SafetyManager(Node):
    OFF = 'OFF'
    SEMIAUTO = 'SEMIAUTO'
    MANUAL = 'MANUAL'
    ESTOP_LATCHED = 'ESTOP_LATCHED'

    def __init__(self):
        super().__init__('safety_manager')
        self.declare_parameter('control_toggle_button', 9)
        # R1 in the standard PlayStation /joy mapping.
        self.declare_parameter('manual_mode_button', 5)
        self.declare_parameter('emergency_stop_button', 10)
        self.control_button = int(self.get_parameter('control_toggle_button').value)
        self.manual_button = int(self.get_parameter('manual_mode_button').value)
        self.estop_button = int(self.get_parameter('emergency_stop_button').value)

        self.mode = self.OFF
        self.last_buttons = []

        latched_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub = self.create_publisher(String, '/control/mode', latched_qos)
        self.control_enabled_pub = self.create_publisher(Bool, '/control/enabled', latched_qos)
        self.manual_enabled_pub = self.create_publisher(Bool, '/control/manual_enabled', latched_qos)
        self.semiauto_enabled_pub = self.create_publisher(Bool, '/control/semiauto_enabled', latched_qos)
        self.protective_stop_pub = self.create_publisher(
            Bool, '/control/protective_stop', latched_qos)
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', latched_qos)
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.estop_pub.publish(Bool(data=False))
        self.protective_stop_pub.publish(Bool(data=False))
        self.publish_mode()

    def pressed(self, msg, index):
        return 0 <= index < len(msg.buttons) and msg.buttons[index] != 0

    def rising(self, msg, index):
        previous = 0 <= index < len(self.last_buttons) and self.last_buttons[index] != 0
        return self.pressed(msg, index) and not previous

    def on_joy(self, msg):
        if self.mode != self.ESTOP_LATCHED and self.rising(msg, self.estop_button):
            self.trigger_estop()
        elif self.mode != self.ESTOP_LATCHED and self.rising(msg, self.control_button):
            self.mode = self.SEMIAUTO if self.mode == self.OFF else self.OFF
            self.publish_mode()
        elif self.mode != self.ESTOP_LATCHED and self.mode != self.OFF and self.rising(msg, self.manual_button):
            self.mode = self.MANUAL if self.mode == self.SEMIAUTO else self.SEMIAUTO
            self.publish_mode()
        self.last_buttons = list(msg.buttons)

    def publish_mode(self):
        enabled = self.mode in (self.SEMIAUTO, self.MANUAL)
        self.mode_pub.publish(String(data=self.mode))
        self.control_enabled_pub.publish(Bool(data=enabled))
        self.manual_enabled_pub.publish(Bool(data=self.mode == self.MANUAL))
        self.semiauto_enabled_pub.publish(Bool(data=self.mode == self.SEMIAUTO))
        self.get_logger().info(f'Operator control mode: {self.mode}')

    def trigger_estop(self):
        self.mode = self.ESTOP_LATCHED
        self.publish_mode()
        self.protective_stop_pub.publish(Bool(data=True))
        self.estop_pub.publish(Bool(data=True))
        self.get_logger().fatal(
            'PROTECTIVE E-STOP LATCHED: blocking operator commands and holding position '
            'with motor torque enabled. Restart bringup to recover.')


def main(args=None):
    rclpy.init(args=args)
    node = SafetyManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
