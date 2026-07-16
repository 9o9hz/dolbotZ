"""Tests for compressed image transport decoding helpers."""

import struct
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from dolbotz.utils.compressed_image import decode_compressed_depth


def _message(
    format_text: str, image: np.ndarray, header: bytes,
) -> SimpleNamespace:
    ok, encoded = cv2.imencode('.png', image)
    assert ok
    return SimpleNamespace(
        format=format_text,
        data=header + encoded.tobytes(),
    )


def test_decode_16uc1_compressed_depth_preserves_values():
    depth_mm = np.array([[0, 500, 1234], [4000, 65535, 42]], dtype=np.uint16)
    msg = _message(
        '16UC1; compressedDepth png',
        depth_mm,
        struct.pack('=iff', 0, 0.0, 0.0),
    )

    decoded = decode_compressed_depth(msg)

    assert decoded.dtype == np.uint16
    np.testing.assert_array_equal(decoded, depth_mm)


def test_decode_32fc1_inverse_depth_uses_header_parameters():
    inverse_depth = np.array([[0, 1000], [2000, 4000]], dtype=np.uint16)
    depth_quant_a = 10000.0
    depth_quant_b = 100.0
    msg = _message(
        '32FC1; compressedDepth png',
        inverse_depth,
        struct.pack('=iff', 0, depth_quant_a, depth_quant_b),
    )

    decoded = decode_compressed_depth(msg)

    expected = np.zeros((2, 2), dtype=np.float32)
    valid = inverse_depth != 0
    expected[valid] = depth_quant_a / (inverse_depth[valid] + depth_quant_b)
    np.testing.assert_allclose(decoded, expected)


def test_rejects_regular_compressed_image():
    msg = SimpleNamespace(format='jpeg', data=b'not a depth image')

    with pytest.raises(ValueError, match='compressedDepth'):
        decode_compressed_depth(msg)
