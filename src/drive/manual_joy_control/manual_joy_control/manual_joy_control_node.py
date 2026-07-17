import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, Float32


class ManualJoyControlNode(Node):
    def __init__(self):
        super().__init__('manual_joy_control_node')

        # ---- 파라미터 (실측 후 조정) ----
        self.declare_parameter('axis_left_stick_x', 0)   # 좌우(회전)
        self.declare_parameter('axis_left_stick_y', 1)   # 전후(직진)
        self.declare_parameter('button_l1', 4)
        self.declare_parameter('button_r1', 5)

        self.declare_parameter('default_speed_dps', 200.0)
        self.declare_parameter('speed_step_dps', 10.0)
        self.declare_parameter('min_speed_dps', 0.0)
        self.declare_parameter('max_speed_dps', 800.0)
        self.declare_parameter('stick_deadzone', 0.05)

        self.declare_parameter('left_motor_sign', 1.0)
        self.declare_parameter('right_motor_sign', -1.0)

        self.declare_parameter('cmd_topic', '/motor_speed_cmd')
        self.declare_parameter('cmd_publish_rate_hz', 50.0)

        p = self.get_parameter
        self.axis_x = p('axis_left_stick_x').value
        self.axis_y = p('axis_left_stick_y').value
        self.btn_l1 = p('button_l1').value
        self.btn_r1 = p('button_r1').value

        self.max_speed_limit_dps = p('default_speed_dps').value
        self.step = p('speed_step_dps').value
        self.min_speed = p('min_speed_dps').value
        self.max_speed = p('max_speed_dps').value
        self.deadzone = p('stick_deadzone').value

        self.left_sign = p('left_motor_sign').value
        self.right_sign = p('right_motor_sign').value

        self._prev_l1 = 0
        self._prev_r1 = 0

        self._latest_left_cmd = 0.0
        self._latest_right_cmd = 0.0

        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        cmd_topic = p('cmd_topic').value
        self.cmd_pub = self.create_publisher(Float32MultiArray, cmd_topic, 10)
        self.max_speed_pub = self.create_publisher(Float32, 'max_speed_dps', 10)

        rate = p('cmd_publish_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.publish_cmd)

        self.get_logger().info(
            f'Manual joy control started. default_speed={self.max_speed_limit_dps}dps, '
            f'cmd_topic={cmd_topic}'
        )

    def joy_callback(self, msg: Joy):
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

        # ---- 3구간 discrete 조향 로직 ----
        if turn == 0.0:
            # 순수 전진/후진: 좌우 동일 ratio (최종 부호는 left_sign/right_sign가 갈라줌)
            left_ratio = fwd
            right_ratio = fwd
        elif fwd == 0.0:
            # 순수 좌우 -> 제자리 회전: 우측 ratio 부호를 반전시켜서 최종 부호가 같아지게
            left_ratio = turn
            right_ratio = -turn
        else:
            # 대각선 입력 -> 한쪽 바퀴만 살리고 반대쪽은 0 (피벗 회전)
            if turn > 0:
                left_ratio = fwd
                right_ratio = 0.0
            else:
                left_ratio = 0.0
                right_ratio = fwd

        self._latest_left_cmd = self.left_sign * left_ratio * self.max_speed_limit_dps
        self._latest_right_cmd = self.right_sign * right_ratio * self.max_speed_limit_dps

    def publish_cmd(self):
        msg = Float32MultiArray()
        msg.data = [self._latest_left_cmd, self._latest_right_cmd]
        self.cmd_pub.publish(msg)

        max_speed_msg = Float32()
        max_speed_msg.data = self.max_speed_limit_dps
        self.max_speed_pub.publish(max_speed_msg)


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