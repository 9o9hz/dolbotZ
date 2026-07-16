"""/arm/detection 페이로드(std_msgs/Float32MultiArray) 인코딩/디코딩.

필드 순서를 arm_pickup(발행)과 arm_visualizer(구독) 양쪽에서 매직 인덱스로
중복 정의하지 않도록 이 모듈 하나로 공유한다.
"""

from dataclasses import dataclass

from std_msgs.msg import Float32MultiArray

_NUM_FIELDS = 7


@dataclass
class Detection2D:
    x1: float
    y1: float
    x2: float
    y2: float
    u: float
    v: float
    confidence: float


def encode_detection(det: Detection2D) -> Float32MultiArray:
    msg = Float32MultiArray()
    msg.data = [det.x1, det.y1, det.x2, det.y2, det.u, det.v, det.confidence]
    return msg


def decode_detection(msg: Float32MultiArray) -> Detection2D:
    if len(msg.data) != _NUM_FIELDS:
        raise ValueError(
            f'/arm/detection payload 길이가 예상과 다릅니다: {len(msg.data)}')
    return Detection2D(*msg.data)
