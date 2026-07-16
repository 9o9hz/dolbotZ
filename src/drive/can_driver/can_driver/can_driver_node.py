import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import can


class CanDriverNode(Node):
    def __init__(self):
        super().__init__('can_driver_node')

        # ---- 파라미터 ----
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('left_can_id', 1)   # -> 0x141
        self.declare_parameter('right_can_id', 2)  # -> 0x142
        self.declare_parameter('cmd_topic', '/motor_speed_cmd')
        self.declare_parameter('cmd_timeout_sec', 0.3)  # 이 시간 동안 명령 없으면 안전정지
        self.declare_parameter('watchdog_check_hz', 20.0)

        p = self.get_parameter
        self.left_id = 0x140 + p('left_can_id').value
        self.right_id = 0x140 + p('right_can_id').value
        channel = p('can_channel').value
        cmd_topic = p('cmd_topic').value
        self.cmd_timeout = p('cmd_timeout_sec').value

        # ---- CAN 버스 연결 ----
        try:
            self.bus = can.interface.Bus(channel=channel, bustype='socketcan')
            self.get_logger().info(f'CAN bus opened on {channel}')
        except Exception as e:
            self.get_logger().error(f'Failed to open CAN bus {channel}: {e}')
            raise

        self._last_cmd_time = self.get_clock().now()
        self._has_received_cmd = False

        self.sub = self.create_subscription(
            Float32MultiArray, cmd_topic, self.cmd_callback, 10
        )

        # 워치독: 조이스틱/상위 노드가 죽거나 연결이 끊기면 자동으로 모터 정지
        watchdog_hz = p('watchdog_check_hz').value
        self.watchdog_timer = self.create_timer(1.0 / watchdog_hz, self.watchdog_check)

        self.get_logger().info(
            f'CAN driver started. left_id=0x{self.left_id:X}, right_id=0x{self.right_id:X}, '
            f'listening on {cmd_topic}'
        )

    def cmd_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn('motor_speed_cmd expects [left_dps, right_dps], got fewer values')
            return

        left_dps, right_dps = msg.data[0], msg.data[1]
        self._last_cmd_time = self.get_clock().now()
        self._has_received_cmd = True

        self.send_speed_command(self.left_id, left_dps)
        self.send_speed_command(self.right_id, right_dps)

    def send_speed_command(self, arb_id: int, speed_dps: float):
        # 0.01dps/LSB 단위로 스케일링 (프로토콜 스펙)
        speed_control = int(round(speed_dps * 100.0))

        # int32_t 범위 클램프 (안전상 과도한 값 방지)
        speed_control = max(min(speed_control, 2_147_483_647), -2_147_483_648)

        data = bytearray(8)
        data[0] = 0xA2
        data[1] = 0x00
        data[2] = 0x00
        data[3] = 0x00
        data[4:8] = struct.pack('<i', speed_control)  # little-endian int32

        self._send_frame(arb_id, data)

    def send_stop_command(self, arb_id: int):
        # 0x81: 모터 정지 (closed-loop 유지, 속도만 0으로)
        data = bytearray(8)
        data[0] = 0x81
        self._send_frame(arb_id, data)

    def _send_frame(self, arb_id: int, data: bytearray):
        msg = can.Message(arbitration_id=arb_id, data=bytes(data), is_extended_id=False)
        try:
            self.bus.send(msg)
        except can.CanError as e:
            self.get_logger().error(f'CAN send failed (id=0x{arb_id:X}): {e}')

    def watchdog_check(self):
        if not self._has_received_cmd:
            return  # 아직 한 번도 명령을 못 받았으면 대기 (조기 정지 명령 스팸 방지)

        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            self.get_logger().warn(
                f'No /motor_speed_cmd for {elapsed:.2f}s (timeout={self.cmd_timeout}s). '
                f'Sending stop command.'
            )
            self.send_stop_command(self.left_id)
            self.send_stop_command(self.right_id)

    def destroy_node(self):
        # 노드 종료 시 안전하게 정지 명령 전송 후 CAN 버스 닫기
        try:
            self.send_stop_command(self.left_id)
            self.send_stop_command(self.right_id)
        except Exception:
            pass
        try:
            self.bus.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CanDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()