"""Tests for /arm/detection Float32MultiArray encode/decode helpers."""

import pytest
from std_msgs.msg import Float32MultiArray

from dolbotz.utils.detection import Detection2D, decode_detection, encode_detection


def test_encode_detection_orders_fields_as_x1_y1_x2_y2_u_v_confidence():
    det = Detection2D(x1=1.0, y1=2.0, x2=3.0, y2=4.0, u=2.0, v=3.0, confidence=0.75)

    msg = encode_detection(det)

    assert isinstance(msg, Float32MultiArray)
    assert list(msg.data) == pytest.approx([1.0, 2.0, 3.0, 4.0, 2.0, 3.0, 0.75])


def test_decode_detection_is_inverse_of_encode():
    det = Detection2D(x1=10.5, y1=20.5, x2=30.5, y2=40.5, u=20.0, v=30.0, confidence=0.9)

    decoded = decode_detection(encode_detection(det))

    assert decoded.__dict__ == pytest.approx(det.__dict__)


def test_decode_detection_rejects_wrong_length_payload():
    msg = Float32MultiArray()
    msg.data = [1.0, 2.0, 3.0]

    with pytest.raises(ValueError, match='길이가 예상과 다릅니다'):
        decode_detection(msg)
