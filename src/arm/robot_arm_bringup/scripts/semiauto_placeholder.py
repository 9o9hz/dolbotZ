#!/usr/bin/env python3
"""Empty placeholder for the future MoveIt-based semiautomatic controller."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class SemiautoPlaceholder(Node):
    def __init__(self):
        super().__init__('semiauto_placeholder')
        mode_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, '/control/semiauto_enabled', self.on_enabled, mode_qos)

    def on_enabled(self, _msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = SemiautoPlaceholder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
