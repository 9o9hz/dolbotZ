#!/usr/bin/env python3
"""
rmd_x8_driver_node

Drives two RMD-X8-120 actuators (left/right) of a skid-steer /
tracked vehicle over CAN, using MYACTUATOR's RMD-X protocol
(Speed Closed-loop Control Command, 0xA2).

Subscribes:
    /cmd_vel               (geometry_msgs/Twist)   -- desired body v, w

Publishes:
    /wheel/odom             (nav_msgs/Odometry)      -- vx only is meaningful;
                                                         see robot_localization
                                                         odom0_config (this
                                                         project fuses vx only,
                                                         wheel yaw rate is
                                                         intentionally NOT
                                                         trusted -- see the
                                                         control guide, 4.1)
    /wheel/joint_states      (sensor_msgs/JointState) -- position(rad),
                                                         velocity(rad/s),
                                                         effort(A, torque
                                                         current -- reused
                                                         field, see README)
    /wheel/motor_status      (diagnostic_msgs/DiagnosticArray)
                                                       -- temp / voltage /
                                                          error flags, for
                                                          the current-monitor
                                                          / stability layers
                                                          described in the
                                                          control guide (3.3)

IMPORTANT SAFETY NOTE:
    The RMD-X drive itself has a 500 ms heartbeat timeout: if it does not
    receive a new control frame within 500 ms it stops the motor on its
    own. This node ALSO enforces a shorter, configurable cmd_vel timeout
    so the robot decelerates smoothly well before that hardware cutoff
    (see control_guide.md section 2.3 "never hard-stop on a slope").
"""

import math
import struct
import threading
import time

import can
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# CMakeLists.txt가 이 파일을 install(PROGRAMS ...)로 그대로 복사해 실행 파일로
# 등록하기 때문에 ros2 run이 이걸 __main__으로 직접 실행한다 (패키지로 import되지
# 않음) -> 상대 임포트(from . import ...)는 __package__가 없어 항상 실패한다.
# ament_python_install_package(rmd_x8_driver)로 패키지 자체는 정상 설치되어
# PYTHONPATH에 잡히므로 절대 임포트를 사용한다.
from rmd_x8_driver import rmd_x8_protocol as proto


class MotorChannel:
    """Per-motor runtime state (one instance for left, one for right)."""

    def __init__(self, name: str, can_id: int, direction_sign: float,
                 wheel_radius_m: float, external_gear_ratio: float):
        self.name = name
        self.can_id = can_id
        self.direction_sign = direction_sign
        self.wheel_radius_m = wheel_radius_m
        self.external_gear_ratio = external_gear_ratio

        self.unwrapper = proto.AngleUnwrapper()
        self.lock = threading.Lock()

        # latest feedback (updated from CAN RX thread)
        self.last_feedback_time = None
        self.wheel_angle_rad = 0.0      # unwrapped, actual wheel frame
        self.wheel_speed_rad_s = 0.0    # actual wheel frame
        self.torque_current_a = 0.0
        self.temperature_c = None
        self.status1 = None            # proto.MotorStatus1 or None

    def apply_feedback(self, fb: proto.MotorFeedback):
        with self.lock:
            # Manual values are OUTPUT SHAFT values (internal gearbox
            # already applied). external_gear_ratio only accounts for
            # any *additional* gearing between the actuator's output
            # shaft and the actual wheel/sprocket (1.0 if directly
            # coupled).
            unwrapped_output_deg = self.unwrapper.update(fb.angle_deg)
            wheel_deg = (unwrapped_output_deg / self.external_gear_ratio) * self.direction_sign
            wheel_dps = (fb.speed_dps / self.external_gear_ratio) * self.direction_sign

            self.wheel_angle_rad = math.radians(wheel_deg)
            self.wheel_speed_rad_s = math.radians(wheel_dps)
            self.torque_current_a = fb.torque_current_a * self.direction_sign
            self.temperature_c = fb.temperature_c
            self.last_feedback_time = time.monotonic()

    def apply_status1(self, st: proto.MotorStatus1):
        with self.lock:
            self.status1 = st

    def wheel_linear_speed_m_s(self) -> float:
        with self.lock:
            return self.wheel_speed_rad_s * self.wheel_radius_m

    def snapshot(self):
        with self.lock:
            return {
                "angle_rad": self.wheel_angle_rad,
                "speed_rad_s": self.wheel_speed_rad_s,
                "current_a": self.torque_current_a,
                "temperature_c": self.temperature_c,
                "status1": self.status1,
                "stale": (self.last_feedback_time is None or
                          (time.monotonic() - self.last_feedback_time) > 0.5),
            }


class RmdX8DriverNode(Node):
    def __init__(self):
        super().__init__("rmd_x8_driver_node")

        # ---- parameters -------------------------------------------------
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("left_motor_can_id", 1)
        self.declare_parameter("right_motor_can_id", 2)
        self.declare_parameter("left_direction_sign", 1.0)
        self.declare_parameter("right_direction_sign", -1.0)  # mirrored mounting, verify on bench!
        self.declare_parameter("wheel_radius_m", 0.1125)
        self.declare_parameter("external_gear_ratio", 1.0)
        self.declare_parameter("effective_track_width_m", 0.50)  # MUST calibrate, see control_guide 3.1
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("cmd_vel_timeout_s", 0.3)
        self.declare_parameter("cmd_vel_safety_timeout_s", 0.5)
        self.declare_parameter("max_wheel_speed_dps", 3000.0)
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_odom_tf", False)  # normally False: robot_localization publishes odom->base_link

        can_interface = self.get_parameter("can_interface").value
        left_id = int(self.get_parameter("left_motor_can_id").value)
        right_id = int(self.get_parameter("right_motor_can_id").value)
        left_dir = float(self.get_parameter("left_direction_sign").value)
        right_dir = float(self.get_parameter("right_direction_sign").value)
        wheel_radius = float(self.get_parameter("wheel_radius_m").value)
        gear_ratio = float(self.get_parameter("external_gear_ratio").value)

        self.track_width = float(self.get_parameter("effective_track_width_m").value)
        self.control_period = 1.0 / float(self.get_parameter("control_rate_hz").value)
        self.cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout_s").value)
        self.cmd_vel_safety_timeout = float(self.get_parameter("cmd_vel_safety_timeout_s").value)
        self.max_wheel_speed_dps = float(self.get_parameter("max_wheel_speed_dps").value)
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.publish_odom_tf = bool(self.get_parameter("publish_odom_tf").value)

        self.left = MotorChannel("left", left_id, left_dir, wheel_radius, gear_ratio)
        self.right = MotorChannel("right", right_id, right_dir, wheel_radius, gear_ratio)
        self.channels = [self.left, self.right]

        # ---- CAN bus ------------------------------------------------------
        try:
            self.bus = can.interface.Bus(channel=can_interface, bustype="socketcan")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().fatal(
                f"Failed to open CAN interface '{can_interface}': {exc}. "
                f"Check `ip link show {can_interface}` and that the bus is up "
                f"(e.g. `sudo ip link set {can_interface} up type can bitrate 1000000`)."
            )
            raise

        self._reply_id_map = {
            proto.motor_reply_id(left_id): self.left,
            proto.motor_reply_id(right_id): self.right,
        }

        self._notifier = can.Notifier(self.bus, [self._on_can_message])

        # ---- ROS interfaces -------------------------------------------------
        best_effort_qos = QoSProfile(depth=10,
                                      reliability=ReliabilityPolicy.BEST_EFFORT,
                                      history=HistoryPolicy.KEEP_LAST)

        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, 10)

        # stability_monitor_node의 Level-2(CRITICAL) 비상 명령을 조이스틱 MUX를
        # 거치지 않고 여기서 직접 받아 최우선 처리한다 (joy_mux_node가 죽어도
        # 안전 개입은 살아있도록, 실제 모터 명령 생성 지점에 가장 가깝게 배치).
        self.cmd_vel_safety_sub = self.create_subscription(
            Twist, "/cmd_vel_safety", self._on_cmd_vel_safety, 10)

        self.odom_pub = self.create_publisher(Odometry, "/wheel/odom", best_effort_qos)
        self.joint_state_pub = self.create_publisher(JointState, "/wheel/joint_states", best_effort_qos)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/wheel/motor_status", 10)

        self._tf_broadcaster = None
        if self.publish_odom_tf:
            from tf2_ros import TransformBroadcaster
            self._tf_broadcaster = TransformBroadcaster(self)

        # ---- runtime state --------------------------------------------------
        self._desired_v = 0.0
        self._desired_w = 0.0
        self._last_cmd_vel_time = self.get_clock().now()
        self._safety_v = 0.0
        self._safety_w = 0.0
        self._last_safety_time = None  # None = /cmd_vel_safety로부터 아직 아무 메시지도 못 받음
        self._cmd_lock = threading.Lock()

        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_theta = 0.0
        self._last_odom_time = self.get_clock().now()

        self._status1_poll_counter = 0

        self.control_timer = self.create_timer(self.control_period, self._control_loop)

        self.get_logger().info(
            f"rmd_x8_driver_node started: can={can_interface}, "
            f"left_id={left_id}, right_id={right_id}, "
            f"track_width={self.track_width} m (VERIFY VIA CALIBRATION), "
            f"wheel_radius={wheel_radius} m"
        )

    # ------------------------------------------------------------------ #
    # CAN RX
    # ------------------------------------------------------------------ #
    def _on_can_message(self, msg: can.Message):
        channel = self._reply_id_map.get(msg.arbitration_id)
        if channel is None:
            return
        if len(msg.data) != 8:
            return

        cmd = msg.data[0]
        try:
            if cmd in (proto.CMD_SPEED_CONTROL, proto.CMD_READ_STATUS_2):
                fb = proto.parse_a2_9c_reply(bytes(msg.data))
                channel.apply_feedback(fb)
            elif cmd == proto.CMD_READ_STATUS_1:
                st = proto.parse_status1_reply(bytes(msg.data))
                channel.apply_status1(st)
                if st.error_flags:
                    self.get_logger().warn(
                        f"[{channel.name}] motor error flags: {st.error_flags}"
                    )
        except (struct.error, ValueError) as exc:
            self.get_logger().warn(f"[{channel.name}] failed to parse CAN reply: {exc}")

    # ------------------------------------------------------------------ #
    # cmd_vel in
    # ------------------------------------------------------------------ #
    def _on_cmd_vel(self, msg: Twist):
        with self._cmd_lock:
            self._desired_v = msg.linear.x
            self._desired_w = msg.angular.z
            self._last_cmd_vel_time = self.get_clock().now()

    def _on_cmd_vel_safety(self, msg: Twist):
        with self._cmd_lock:
            self._safety_v = msg.linear.x
            self._safety_w = msg.angular.z
            self._last_safety_time = self.get_clock().now()

    # ------------------------------------------------------------------ #
    # main control loop
    # ------------------------------------------------------------------ #
    def _control_loop(self):
        now = self.get_clock().now()
        with self._cmd_lock:
            safety_age_s = (
                (now - self._last_safety_time).nanoseconds * 1e-9
                if self._last_safety_time is not None
                else None
            )

            if safety_age_s is not None and safety_age_s <= self.cmd_vel_safety_timeout:
                # /cmd_vel_safety가 최근에 수신됨 -> /cmd_vel 완전히 무시하고
                # 이 값을 그대로 사용. 기존 cmd_vel_timeout 로직과는 별개(이쪽은
                # /cmd_vel의 나이를 아예 보지 않는다).
                v, w = self._safety_v, self._safety_w
            else:
                age_s = (now - self._last_cmd_vel_time).nanoseconds * 1e-9
                v, w = self._desired_v, self._desired_w
                if age_s > self.cmd_vel_timeout:
                    # Safety: no recent cmd_vel -> command zero speed rather than
                    # relying solely on the drive's own 500ms heartbeat cutoff.
                    v, w = 0.0, 0.0

        v_left, v_right = self._skid_steer_inverse(v, w)

        self._send_speed_command(self.left, v_left)
        self._send_speed_command(self.right, v_right)

        # occasionally poll Motor Status 1 (temp/voltage/error flags) --
        # not needed every cycle, ~2 Hz is plenty for diagnostics
        self._status1_poll_counter += 1
        if self._status1_poll_counter >= max(1, int(round(1.0 / self.control_period / 2.0))):
            self._status1_poll_counter = 0
            self._poll_status1(self.left)
            self._poll_status1(self.right)

        self._publish_feedback(now)

    def _skid_steer_inverse(self, v: float, w: float):
        """v [m/s], w [rad/s] -> (v_left, v_right) wheel linear speed [m/s]."""
        half_track = self.track_width / 2.0
        v_left = v - w * half_track
        v_right = v + w * half_track
        return v_left, v_right

    def _send_speed_command(self, channel: MotorChannel, wheel_linear_speed_m_s: float):
        wheel_angular_dps = math.degrees(wheel_linear_speed_m_s / channel.wheel_radius_m)
        # convert wheel-frame dps to the actuator's OUTPUT SHAFT dps
        # (accounting for any external gear stage) and this motor's
        # physical mounting direction sign
        output_shaft_dps = wheel_angular_dps * channel.external_gear_ratio * channel.direction_sign

        # clamp for safety regardless of what Nav2/EKF asked for
        output_shaft_dps = max(-self.max_wheel_speed_dps,
                                min(self.max_wheel_speed_dps, output_shaft_dps))

        data = proto.build_speed_command(output_shaft_dps)
        message = can.Message(
            arbitration_id=proto.motor_send_id(channel.can_id),
            data=data,
            is_extended_id=False,
        )
        try:
            self.bus.send(message)
        except can.CanError as exc:
            self.get_logger().warn(f"[{channel.name}] CAN send failed: {exc}")

    def _poll_status1(self, channel: MotorChannel):
        data = proto.build_read_status1_command()
        message = can.Message(
            arbitration_id=proto.motor_send_id(channel.can_id),
            data=data,
            is_extended_id=False,
        )
        try:
            self.bus.send(message)
        except can.CanError as exc:
            self.get_logger().warn(f"[{channel.name}] status1 poll send failed: {exc}")

    # ------------------------------------------------------------------ #
    # publishing
    # ------------------------------------------------------------------ #
    def _publish_feedback(self, now):
        left = self.left.snapshot()
        right = self.right.snapshot()

        # ---- joint states ----
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ["left_wheel_joint", "right_wheel_joint"]
        js.position = [left["angle_rad"], right["angle_rad"]]
        js.velocity = [left["speed_rad_s"], right["speed_rad_s"]]
        # NOTE: 'effort' is reused here to carry torque current in Amps
        # (not Nm) for convenience -- see README. The 3.3 current-protection
        # logic in the control guide should subscribe to this topic.
        js.effort = [left["current_a"], right["current_a"]]
        self.joint_state_pub.publish(js)

        # ---- wheel odometry (dead-reckoning for debug/rviz; robot_localization
        # is configured to trust ONLY vx from this topic, see control_guide 4.2) ----
        v_left = left["speed_rad_s"] * self.left.wheel_radius_m
        v_right = right["speed_rad_s"] * self.right.wheel_radius_m
        v = (v_left + v_right) / 2.0
        w = (v_right - v_left) / self.track_width if self.track_width > 1e-6 else 0.0

        dt = (now - self._last_odom_time).nanoseconds * 1e-9
        self._last_odom_time = now
        if 0.0 < dt < 1.0:  # guard against startup / clock jump artifacts
            self._odom_theta += w * dt
            self._odom_x += v * math.cos(self._odom_theta) * dt
            self._odom_y += v * math.sin(self._odom_theta) * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        qz = math.sin(self._odom_theta / 2.0)
        qw = math.cos(self._odom_theta / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        # Large covariances on everything except vx: this is a DEBUG /
        # dead-reckoning estimate. robot_localization's odom0_config
        # (control_guide 4.2) is what actually decides what gets trusted --
        # these covariance numbers are not authoritative on their own, but
        # set them sanely in case anything else consumes this topic directly.
        odom.pose.covariance[0] = 1e6
        odom.pose.covariance[7] = 1e6
        odom.pose.covariance[35] = 1e6
        odom.twist.covariance[0] = 0.01       # vx: reasonably trusted
        odom.twist.covariance[35] = 1e6       # wz: NOT trusted (track slip)
        self.odom_pub.publish(odom)

        if self._tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = self._odom_x
            t.transform.translation.y = self._odom_y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(t)

        # ---- diagnostics ----
        diag = DiagnosticArray()
        diag.header.stamp = now.to_msg()
        for name, ch_snapshot in (("left", left), ("right", right)):
            status = DiagnosticStatus()
            status.name = f"rmd_x8/{name}"
            status.hardware_id = name
            if ch_snapshot["stale"]:
                status.level = DiagnosticStatus.STALE
                status.message = "no feedback received recently"
            elif ch_snapshot["status1"] and ch_snapshot["status1"].error_flags:
                status.level = DiagnosticStatus.ERROR
                status.message = ",".join(ch_snapshot["status1"].error_flags)
            else:
                status.level = DiagnosticStatus.OK
                status.message = "ok"
            status.values.append(KeyValue(key="torque_current_a", value=f'{ch_snapshot["current_a"]:.3f}'))
            if ch_snapshot["temperature_c"] is not None:
                status.values.append(KeyValue(key="temperature_c", value=str(ch_snapshot["temperature_c"])))
            if ch_snapshot["status1"] is not None:
                status.values.append(KeyValue(key="voltage_v", value=f'{ch_snapshot["status1"].voltage_v:.1f}'))
                status.values.append(KeyValue(key="mos_temperature_c", value=str(ch_snapshot["status1"].mos_temperature_c)))
            diag.status.append(status)
        self.diag_pub.publish(diag)

    def destroy_node(self):
        # best-effort: stop motors before shutting down the node
        try:
            for ch in self.channels:
                data = proto.build_stop_command()
                msg = can.Message(arbitration_id=proto.motor_send_id(ch.can_id),
                                   data=data, is_extended_id=False)
                self.bus.send(msg)
        except Exception:  # noqa: BLE001
            pass
        try:
            self._notifier.stop()
            self.bus.shutdown()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RmdX8DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()