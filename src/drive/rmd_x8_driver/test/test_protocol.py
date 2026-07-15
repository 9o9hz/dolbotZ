"""
Sanity tests against the worked examples in the RMD-X manual
(Servo Motor Control Protocol V4.01), section 2.20.4.
"""
import sys
import os

sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), "..", "rmd_x8_driver")
)
import rmd_x8_protocol as proto  # noqa: E402


def test_can_ids():
    assert proto.motor_send_id(1) == 0x141
    assert proto.motor_send_id(2) == 0x142
    assert proto.motor_reply_id(1) == 0x241
    assert proto.motor_reply_id(2) == 0x242


def test_build_speed_command_matches_manual_example_1():
    # Manual: 100 dps -> DATA = A2 00 00 00 10 27 00 00
    frame = proto.build_speed_command(100.0)
    assert frame == bytes([0xA2, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00])


def test_build_speed_command_matches_manual_example_2():
    # Manual: -100 dps -> DATA = A2 00 00 00 F0 D8 FF FF
    frame = proto.build_speed_command(-100.0)
    assert frame == bytes([0xA2, 0x00, 0x00, 0x00, 0xF0, 0xD8, 0xFF, 0xFF])


def test_parse_reply_matches_manual_example():
    # Manual reply: 32 64 00 F4 01 2D 00
    # -> temp=50C, iq=1.00A, speed=500dps, angle=45deg
    data = bytes([0xA2, 0x32, 0x64, 0x00, 0xF4, 0x01, 0x2D, 0x00])
    fb = proto.parse_a2_9c_reply(data)
    assert fb.temperature_c == 50
    assert abs(fb.torque_current_a - 1.00) < 1e-6
    assert abs(fb.speed_dps - 500.0) < 1e-6
    assert fb.angle_deg == 45


def test_parse_reply_negative_matches_manual_example():
    # Manual reply: 32 9C FF 0C FE D3 FF
    # -> temp=50C, iq=-1.00A, speed=-500dps, angle=-45deg
    data = bytes([0xA2, 0x32, 0x9C, 0xFF, 0x0C, 0xFE, 0xD3, 0xFF])
    fb = proto.parse_a2_9c_reply(data)
    assert fb.temperature_c == 50
    assert abs(fb.torque_current_a - (-1.00)) < 1e-6
    assert abs(fb.speed_dps - (-500.0)) < 1e-6
    assert fb.angle_deg == -45


def test_parse_status1_matches_manual_example():
    # Manual reply: 9A 32 50 01 E5 01 04 00
    # -> temp=50C, mos_temp=80C, brake_released=True, voltage=48.5V,
    #    error_flags contains low_voltage (0x0004)
    data = bytes([0x9A, 0x32, 0x50, 0x01, 0xE5, 0x01, 0x04, 0x00])
    st = proto.parse_status1_reply(data)
    assert st.temperature_c == 50
    assert st.mos_temperature_c == 80
    assert st.brake_released is True
    assert abs(st.voltage_v - 48.5) < 1e-6
    assert "low_voltage" in st.error_flags


def test_angle_unwrap_basic():
    unwrapper = proto.AngleUnwrapper()
    assert unwrapper.update(100) == 100.0
    assert unwrapper.update(200) == 200.0
    assert unwrapper.update(150) == 150.0


def test_angle_unwrap_positive_wrap():
    unwrapper = proto.AngleUnwrapper()
    unwrapper.update(32760)
    # jumps past +32767 -> wraps to a very negative raw value
    result = unwrapper.update(-32760)
    # -32760 raw == 32776 - 65536, so the true unwrapped motion was +16 deg
    assert abs(result - 32776.0) < 1e-6


if __name__ == "__main__":
    tests = [v for k, v in list(globals().items()) if k.startswith("test_")]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"PASS {t.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL {t.__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)