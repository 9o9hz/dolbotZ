"""Decode ROS image_transport compressed image payloads."""

import struct

import cv2
import numpy as np


_CONFIG_HEADER = struct.Struct('=iff')
_PNG_SIGNATURE = b'\x89PNG\r\n\x1a\n'


def decode_compressed_depth(msg) -> np.ndarray:
    """Decode a compressedDepth message into its original depth array.

    RealSense normally publishes 16UC1 depth, which compressedDepth stores as
    a lossless 16-bit PNG after a 12-byte transport header. 32FC1 inverse-depth
    payloads are reconstructed using the quantization parameters in that
    header.
    """
    format_text = str(msg.format)
    if 'compressedDepth' not in format_text:
        raise ValueError(f'compressedDepth 형식이 아닙니다: {format_text!r}')

    payload = bytes(msg.data)
    png_offset = payload.find(_PNG_SIGNATURE, _CONFIG_HEADER.size)
    if png_offset < 0:
        raise ValueError('compressedDepth PNG 시그니처를 찾을 수 없습니다.')

    encoded = np.frombuffer(payload[png_offset:], dtype=np.uint8)
    decoded = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
    if decoded is None:
        raise ValueError('compressedDepth PNG 디코딩에 실패했습니다.')
    if decoded.ndim != 2 or decoded.dtype != np.uint16:
        raise ValueError(
            f'예상하지 못한 compressedDepth 이미지: '
            f'shape={decoded.shape}, dtype={decoded.dtype}'
        )

    source_encoding = format_text.split(';', 1)[0].strip()
    if source_encoding in ('16UC1', 'mono16'):
        return decoded
    if source_encoding == '32FC1':
        if len(payload) < _CONFIG_HEADER.size:
            raise ValueError('compressedDepth 설정 헤더가 손상되었습니다.')
        _compression_format, depth_quant_a, depth_quant_b = \
            _CONFIG_HEADER.unpack_from(payload)
        depth = np.zeros(decoded.shape, dtype=np.float32)
        valid = decoded != 0
        depth[valid] = depth_quant_a / (
            decoded[valid].astype(np.float32) + depth_quant_b
        )
        return depth

    raise ValueError(f'지원하지 않는 Depth 인코딩입니다: {source_encoding!r}')
