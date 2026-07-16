import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray


class ManualJoyControlNode(Node):
    def __init__(self):
        super().__init__('manual_joy_control_node')

        # ---- 파라미터 (실측 후 조정) ----
        self.declare_parameter('axis_left_stick_x', 0)   # 좌우(회전)
        self.declare_parameter('axis_left_stick_y', 1)   # 전후(직진)
        self.declare_parameter('button_l1', 4)
        self.declare_parameter('button_r1', 5)
        self.declare_parameter('button_ps', 10)

        self.declare_parameter('default_speed_dps', 400.0)
        self.declare_parameter('speed_step_dps', 10.0)
        self.declare_parameter('min_speed_dps', 0.0)
        self.declare_parameter('max_speed_dps', 800.0)  # 모터 스펙 확인 후 수정
        self.declare_parameter('stick_deadzone', 0.05)

        # CAN ID 1 = left, CAN ID 2 = right.
        self.declare_parameter('left_motor_sign', 1.0)
        self.declare_parameter('right_motor_sign', -1.0)

        self.declare_parameter('cmd_topic', '/motor_speed_cmd')
        self.declare_parameter('cmd_publish_rate_hz', 50.0)

        p = self.get_parameter
        self.axis_x = p('axis_left_stick_x').value
        self.axis_y = p('axis_left_stick_y').value
        self.btn_l1 = p('button_l1').value
        self.btn_r1 = p('button_r1').value
        self.btn_ps = p('button_ps').value

        self.max_speed_limit_dps = p('default_speed_dps').value
        self.step = p('speed_step_dps').value
        self.min_speed = p('min_speed_dps').value
        self.max_speed = p('max_speed_dps').value
        self.deadzone = p('stick_deadzone').value

        self.left_sign = p('left_motor_sign').value
        self.right_sign = p('right_motor_sign').value

        self.estop_active = False

        self._prev_l1 = 0
        self._prev_r1 = 0
        self._prev_ps = 0

        self._latest_left_cmd = 0.0
        self._latest_right_cmd = 0.0

        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        cmd_topic = p('cmd_topic').value
        self.cmd_pub = self.create_publisher(Float32MultiArray, cmd_topic, 10)

        rate = p('cmd_publish_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.publish_cmd)

        self.get_logger().info(
            f'Manual joy control started. default_speed={self.max_speed_limit_dps}dps, '
            f'estop_button={self.btn_ps}, cmd_topic={cmd_topic}'
        )

    def joy_callback(self, msg: Joy):
        ps = msg.buttons[self.btn_ps] if len(msg.buttons) > self.btn_ps else 0
        if ps == 1 and self._prev_ps == 0:
            self.estop_active = not self.estop_active
            if self.estop_active:
                self.get_logger().warn('EMERGENCY STOP ACTIVATED')
            else:
                self.get_logger().warn('Emergency stop RELEASED')
        self._prev_ps = ps

        if self.estop_active:
            self._latest_left_cmd = 0.0
            self._latest_right_cmd = 0.0
            return

        r1 = msg.buttons[self.btn_r1] if len(msg.buttons) > self.btn_r1 else 0
        l1 = msg.buttons[self.btn_l1] if len(msg.buttons) > self.btn_l1 else 0

        if r1 == 1 and self._prev_r1 == 0:
            self.max_speed_limit_dps = min(self.max_speed_limit_dps + self.step, self.max_speed)
            self.get_logger().info(f'Max speed -> {self.max_speed_limit_dps} dps')
        if l1 == 1 and self._prev_l1 == 0:
            self.max_speed_limit_dps = max(self.max_speed_limit_dps - self.step, self.min_speed)
            self.get_logger().info(f'Max speed -> {self.max_speed_limit_dps} dps')

        self._prev_r1 = r1
        self._prev_l1 = l1

        fwd = msg.axes[self.axis_y] if len(msg.axes) > self.axis_y else 0.0
        turn = msg.axes[self.axis_x] if len(msg.axes) > self.axis_x else 0.0

        if abs(fwd) < self.deadzone:
            fwd = 0.0
        if abs(turn) < self.deadzone:
            turn = 0.0

        left_raw = fwd + turn
        right_raw = fwd - turn

        # 둘 중 하나라도 -1~1을 벗어나면, 좌우 비율(회전 반경)을 유지한 채
        # 두 값을 동일한 비율로 스케일 다운 (saturation으로 인한 조향 왜곡 방지)
        max_mag = max(abs(left_raw), abs(right_raw), 1.0)
        left_ratio = left_raw / max_mag
        right_ratio = right_raw / max_mag

        self._latest_left_cmd = self.left_sign * left_ratio * self.max_speed_limit_dps
        self._latest_right_cmd = self.right_sign * right_ratio * self.max_speed_limit_dps

    def publish_cmd(self):
        msg = Float32MultiArray()
        msg.data = [self._latest_left_cmd, self._latest_right_cmd]
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ManualJoyControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()