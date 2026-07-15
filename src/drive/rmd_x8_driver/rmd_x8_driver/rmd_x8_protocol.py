"""
RMD-X8 CAN protocol helpers.

Reference: MYACTUATOR "Servo Motor Control Protocol V4.01" (RMD-X series).

CAN identifiers (standard frame, DLC=8):
    Single motor command send : 0x140 + ID   (ID = 1..32)
    Single motor command reply: 0x240 + ID

Only the commands actually needed for a velocity-controlled skid-steer
drive motor are implemented here:
    0xA2 - Speed closed-loop control command (send + reply)
    0x81 - Motor stop command (keeps closed-loop mode, speed -> 0)
    0x80 - Motor shutdown command (clears running state entirely)
    0x9A - Read Motor Status 1 (temperature / voltage / error flags)

All multi-byte fields are little-endian, as specified in the manual
(DATA[n] is the lowest byte, DATA[n+k] is the highest byte).
"""

import struct
from dataclasses import dataclass


CMD_SPEED_CONTROL = 0xA2
CMD_MOTOR_STOP = 0x81
CMD_MOTOR_SHUTDOWN = 0x80
CMD_READ_STATUS_1 = 0x9A
CMD_READ_STATUS_2 = 0x9C

# System_errorState bit meanings (see manual section 2.14.3)
ERROR_FLAGS = {
    0x0002: "motor_stall",
    0x0004: "low_voltage",
    0x0008: "over_voltage",
    0x0010: "over_current",
    0x0040: "power_overrun",
    0x0080: "calibration_write_error",
    0x0100: "over_speed",
    0x1000: "over_temperature",
    0x2000: "encoder_calibration_error",
}


def motor_send_id(motor_can_id: int) -> int:
    """Arbitration ID used to *send* a command to a given motor."""
    return 0x140 + motor_can_id


def motor_reply_id(motor_can_id: int) -> int:
    """Arbitration ID a given motor's *reply* will arrive on."""
    return 0x240 + motor_can_id


def build_speed_command(speed_dps: float) -> bytes:
    """
    Build the 8-byte data field for the 0xA2 speed closed-loop command.

    speed_dps: desired OUTPUT SHAFT speed in degrees/second (signed).
               Positive = the direction the manufacturer defines as
               "forward" for that motor's mounting orientation -- this
               is handled by the caller (driver node), not here.
    """
    speed_control = int(round(speed_dps * 100.0))  # 0.01 dps/LSB
    # int32_t range check / clamp defensively
    speed_control = max(-2_147_483_648, min(2_147_483_647, speed_control))
    return struct.pack("<Bxxxi", CMD_SPEED_CONTROL, speed_control)


def build_stop_command() -> bytes:
    """0x81 - stop the motor speed but stay in closed-loop mode."""
    return bytes([CMD_MOTOR_STOP, 0, 0, 0, 0, 0, 0, 0])


def build_shutdown_command() -> bytes:
    """0x80 - fully shut down motor output and clear running state."""
    return bytes([CMD_MOTOR_SHUTDOWN, 0, 0, 0, 0, 0, 0, 0])


def build_read_status1_command() -> bytes:
    return bytes([CMD_READ_STATUS_1, 0, 0, 0, 0, 0, 0, 0])


@dataclass
class MotorFeedback:
    command: int
    temperature_c: int
    torque_current_a: float
    speed_dps: float
    angle_deg: int  # raw int16, wraps at +-32767 deg (see manual 2.20.3)


def parse_a2_9c_reply(data: bytes) -> MotorFeedback:
    """
    Parse the common reply layout shared by 0xA1 / 0xA2 / 0xA4 / 0xA6 / 0xA8
    / 0x9C:
        DATA[0] command byte
        DATA[1] temperature          int8,  1 C/LSB
        DATA[2:4] torque current iq  int16, 0.01 A/LSB
        DATA[4:6] output shaft speed int16, 1 dps/LSB
        DATA[6:8] output shaft angle int16, 1 degree/LSB (+-32767 deg)
    """
    if len(data) != 8:
        raise ValueError(f"expected 8 byte CAN frame, got {len(data)}")
    cmd, temp, iq, speed, angle = struct.unpack("<Bbhhh", data[:8])
    return MotorFeedback(
        command=cmd,
        temperature_c=temp,
        torque_current_a=iq * 0.01,
        speed_dps=float(speed),
        angle_deg=angle,
    )


@dataclass
class MotorStatus1:
    temperature_c: int
    mos_temperature_c: int
    brake_released: bool
    voltage_v: float
    error_flags: list


def parse_status1_reply(data: bytes) -> MotorStatus1:
    if len(data) != 8:
        raise ValueError(f"expected 8 byte CAN frame, got {len(data)}")
    _, temp, mos_temp, brake, voltage, err = struct.unpack("<BbbBHH", data[:8])
    active_errors = [name for bit, name in ERROR_FLAGS.items() if err & bit]
    return MotorStatus1(
        temperature_c=temp,
        mos_temperature_c=mos_temp,
        brake_released=bool(brake),
        voltage_v=voltage * 0.1,
        error_flags=active_errors,
    )


class AngleUnwrapper:
    """
    The int16 angle field in the 0xA2/0x9C reply wraps around at +-32767
    degrees (~ +-91 output-shaft revolutions). For a competition-length
    run this is usually enough headroom, but we defensively unwrap it so
    odometry integration doesn't glitch if it's ever exceeded.

    NOTE: this assumes the control loop runs fast enough that the wheel
    cannot physically travel more than half the wrap range (32768 deg)
    between two consecutive samples -- true for any realistic speed at
    a normal 20-50 Hz control rate.
    """

    WRAP_RANGE = 65536  # int16 full range in degrees

    def __init__(self):
        self._last_raw = None
        self._unwrapped_deg = 0.0

    def update(self, raw_angle_deg: int) -> float:
        if self._last_raw is None:
            self._last_raw = raw_angle_deg
            self._unwrapped_deg = float(raw_angle_deg)
            return self._unwrapped_deg

        delta = raw_angle_deg - self._last_raw
        if delta > self.WRAP_RANGE // 2:
            delta -= self.WRAP_RANGE
        elif delta < -self.WRAP_RANGE // 2:
            delta += self.WRAP_RANGE

        self._unwrapped_deg += delta
        self._last_raw = raw_angle_deg
        return self._unwrapped_deg