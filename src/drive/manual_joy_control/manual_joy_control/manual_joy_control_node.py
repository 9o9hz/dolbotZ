import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, String


class ManualJoyControlNode(Node):
    def __init__(self):
        super().__init__('manual_joy_control_node')

        # ---- 파라미터 (실측 후 조정) ----
        # 탱크 조종: 왼쪽 스틱 상하 -> 좌모터, 오른쪽 스틱 상하 -> 우모터.
        # [하림 수정] 실측값: 왼쪽 스틱 상하=축1. 오른쪽 스틱 상하는 축4가 맞음
        # (축3은 오른쪽 스틱 좌우 - PS 계열 표준 매핑 기준).
        self.declare_parameter('axis_left_motor', 1)
        self.declare_parameter('axis_right_motor', 4)
        # [하림 수정] 직진/후진 홀드 버튼 - 세모(전진)/엑스(후진). arm 코드
        # (base_yaw_negative_button=0=X, base_yaw_positive_button=2=삼각형)에서
        # 이미 검증된 버튼 인덱스라 D-pad 축 추정보다 신뢰도 높음.
        # 양쪽 모터를 동일 속도로 구동, 스틱 입력보다 우선.
        self.declare_parameter('button_forward', 2)   # 세모(Triangle)
        self.declare_parameter('button_backward', 0)  # 엑스(X)
        self.declare_parameter('button_l1', 4)
        self.declare_parameter('button_r1', 5)
        # 제자리 회전 홀드 버튼 - 좌우 모터를 반대 부호로 구동해 자리에서 회전.
        # arm 코드에서 이미 검증된 버튼 인덱스(1=Circle, 3=Square)를 재사용.
        self.declare_parameter('button_rotate_left', 3)   # 네모(Square)
        self.declare_parameter('button_rotate_right', 1)  # 동그라미(Circle)

        # [하림 수정] 우측 모터가 힘이 덜 실려서 좌/우 초기 속도를 다르게 보정.
        # L1/R1은 이 둘을 동시에(같은 step만큼) 올리고 내려서 좌우 간 오프셋은 유지한다.
        self.declare_parameter('default_speed_dps_left', 200.0)
        self.declare_parameter('default_speed_dps_right', 200.0)
        self.declare_parameter('speed_step_dps', 10.0)
        self.declare_parameter('min_speed_dps', 0.0)
        self.declare_parameter('max_speed_dps', 800.0)
        self.declare_parameter('stick_deadzone', 0.05)

        self.declare_parameter('left_motor_sign', 1.0)
        self.declare_parameter('right_motor_sign', -1.0)

        self.declare_parameter('cmd_topic', '/motor_speed_cmd')
        self.declare_parameter('cmd_publish_rate_hz', 50.0)
        self.declare_parameter('joy_topic', 'joy')
        # [하림 수정] 조이스틱 연결이 끊겨도 publish_cmd 타이머는 계속 마지막 값을
        # 재발행하기 때문에(can_driver 워치독은 값이 "계속 오기만" 하면 안 걸림),
        # /joy 수신 자체가 오래됐으면 여기서 직접 0으로 강제한다. arm의
        # manual_total_position_node.py와 동일한 joy_timeout 패턴.
        self.declare_parameter('joy_timeout', 0.5)

        p = self.get_parameter
        self.axis_left_motor = p('axis_left_motor').value
        self.axis_right_motor = p('axis_right_motor').value
        self.btn_forward = p('button_forward').value
        self.btn_backward = p('button_backward').value
        self.btn_l1 = p('button_l1').value
        self.btn_r1 = p('button_r1').value
        self.btn_rotate_left = p('button_rotate_left').value
        self.btn_rotate_right = p('button_rotate_right').value

        self.max_speed_limit_left = p('default_speed_dps_left').value
        self.max_speed_limit_right = p('default_speed_dps_right').value
        self.step = p('speed_step_dps').value
        self.min_speed = p('min_speed_dps').value
        self.max_speed = p('max_speed_dps').value
        self.deadzone = p('stick_deadzone').value

        self.left_sign = p('left_motor_sign').value
        self.right_sign = p('right_motor_sign').value
        self.joy_timeout = p('joy_timeout').value

        self._prev_l1 = 0
        self._prev_r1 = 0

        self._latest_left_cmd = 0.0
        self._latest_right_cmd = 0.0
        self.last_joy_time = self.get_clock().now()
        self._joy_stale_reported = False

        # [하림 수정] arm의 safety_manager가 발행하는 조이스틱 포커스 토글 구독.
        # arm이 없거나 아직 메시지를 못 받았을 때는 drive가 기본으로 조종 가능해야
        # 하므로 'drive'로 fail-open. safety_manager가 'arm'을 발행하면(토글 버튼
        # 9번으로 arm이 켜지면) 즉시 조종을 멈추고 정지 명령을 낸다.
        self.active_target = 'drive'
        active_target_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, '/control/active_target', self.on_active_target, active_target_qos)

        joy_topic = p('joy_topic').value
        self.sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)

        cmd_topic = p('cmd_topic').value
        self.cmd_pub = self.create_publisher(Float32MultiArray, cmd_topic, 10)
        # [하림 수정] 좌우 속도 프리셋이 분리되면서 단일 Float32로는 둘 다 못 담아
        # Float32MultiArray([left, right])로 변경.
        self.max_speed_pub = self.create_publisher(Float32MultiArray, 'max_speed_dps', 10)

        rate = p('cmd_publish_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.publish_cmd)

        self.get_logger().info(
            f'Manual joy control started. default_speed left={self.max_speed_limit_left}dps '
            f'right={self.max_speed_limit_right}dps, joy_topic={joy_topic}, cmd_topic={cmd_topic}'
        )

    def on_active_target(self, msg: String):
        # [하림 수정] drive 포커스를 잃는 순간 즉시 정지 명령으로 스냅 (다음 /joy
        # 메시지를 기다리지 않음).
        self.active_target = msg.data
        if self.active_target != 'drive':
            self._latest_left_cmd = 0.0
            self._latest_right_cmd = 0.0

    def joy_callback(self, msg: Joy):
        self.last_joy_time = self.get_clock().now()

        if self.active_target != 'drive':
            # arm이 조이스틱 포커스를 가진 동안은 입력을 무시하고 정지 상태 유지
            self._latest_left_cmd = 0.0
            self._latest_right_cmd = 0.0
            return

        r1 = msg.buttons[self.btn_r1] if len(msg.buttons) > self.btn_r1 else 0
        l1 = msg.buttons[self.btn_l1] if len(msg.buttons) > self.btn_l1 else 0

        if r1 == 1 and self._prev_r1 == 0:
            self.max_speed_limit_left = min(self.max_speed_limit_left + self.step, self.max_speed)
            self.max_speed_limit_right = min(self.max_speed_limit_right + self.step, self.max_speed)
            self.get_logger().info(
                f'Max speed -> left={self.max_speed_limit_left} right={self.max_speed_limit_right} dps')
        if l1 == 1 and self._prev_l1 == 0:
            self.max_speed_limit_left = max(self.max_speed_limit_left - self.step, self.min_speed)
            self.max_speed_limit_right = max(self.max_speed_limit_right - self.step, self.min_speed)
            self.get_logger().info(
                f'Max speed -> left={self.max_speed_limit_left} right={self.max_speed_limit_right} dps')

        self._prev_r1 = r1
        self._prev_l1 = l1

        forward_held = msg.buttons[self.btn_forward] if len(msg.buttons) > self.btn_forward else 0
        backward_held = msg.buttons[self.btn_backward] if len(msg.buttons) > self.btn_backward else 0
        rotate_left_held = msg.buttons[self.btn_rotate_left] if len(msg.buttons) > self.btn_rotate_left else 0
        rotate_right_held = msg.buttons[self.btn_rotate_right] if len(msg.buttons) > self.btn_rotate_right else 0

        if forward_held and not backward_held:
            # 직진/후진 버튼이 스틱보다 우선 - 정확한 직진이 필요할 때 사용
            left_ratio = 1.0
            right_ratio = 1.0
        elif backward_held and not forward_held:
            left_ratio = -1.0
            right_ratio = -1.0
        elif rotate_left_held and not rotate_right_held:
            # 좌우 모터를 반대 부호로 구동 -> 제자리 좌회전
            left_ratio = -1.0
            right_ratio = 1.0
        elif rotate_right_held and not rotate_left_held:
            left_ratio = 1.0
            right_ratio = -1.0
        else:
            left_val = msg.axes[self.axis_left_motor] if len(msg.axes) > self.axis_left_motor else 0.0
            right_val = msg.axes[self.axis_right_motor] if len(msg.axes) > self.axis_right_motor else 0.0

            if abs(left_val) < self.deadzone:
                left_val = 0.0
            if abs(right_val) < self.deadzone:
                right_val = 0.0

            left_ratio = left_val
            right_ratio = right_val

        self._latest_left_cmd = self.left_sign * left_ratio * self.max_speed_limit_left
        self._latest_right_cmd = self.right_sign * right_ratio * self.max_speed_limit_right

    def publish_cmd(self):
        elapsed = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        if elapsed > self.joy_timeout:
            self._latest_left_cmd = 0.0
            self._latest_right_cmd = 0.0
            if not self._joy_stale_reported:
                self._joy_stale_reported = True
                self.get_logger().warn(
                    f'No /joy for {elapsed:.2f}s (timeout={self.joy_timeout}s). Forcing stop.')
        elif self._joy_stale_reported:
            self._joy_stale_reported = False
            self.get_logger().info('/joy reception recovered.')

        msg = Float32MultiArray()
        msg.data = [self._latest_left_cmd, self._latest_right_cmd]
        self.cmd_pub.publish(msg)

        max_speed_msg = Float32MultiArray()
        max_speed_msg.data = [self.max_speed_limit_left, self.max_speed_limit_right]
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
