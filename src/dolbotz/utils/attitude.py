"""
IMU 자세(roll/pitch) 추정 공용 유틸리티 — ROS 의존성 없음.

elevation_map.py와 slope_drive.py가 둘 다 RealSense 카메라에 내장된 동일한
IMU('/camera/camera/imu', frame_id 'camera_*_optical_frame')를 구독해서 자세를
추정하므로, 이 모듈로 뽑아 공유한다.

설계 노트:
  * 이 IMU는 섀시에 별도로 장착된 것이 아니라 카메라 자체 내장 센서이다.
    따라서 원시 가속도/자이로가 측정하는 기울기는 고정 장착 기울기와 섀시의
    동적 기울어짐이 *결합된* 총 기울기이다. 총 기울기를 한 번만 되돌리면
    이미 world-level에 도달하며, 장착 기울기를 별도로 한 번 더 빼면
    존재하지 않는 성분을 중복 제거하는 오차가 생긴다 (elevation_map.py의
    camera_body_to_level_matrix()에서 실제로 발견/수정된 버그 — 마운트
    피치 10도 + 섀시 수평 상태에서 실제 경사 15도가 약 37도로 과대 계산됨을
    독립 물리 시뮬레이션으로 확인). 이 모듈의 함수를 사용하는 모든 코드는
    이 원칙을 지켜야 한다.
  * yaw는 추정하지 않는다 (마그네토미터도 자이로 Z축 적분도 없음) — roll/pitch만
    두 개의 독립 스칼라로 추적한다.
"""

import numpy as np

# ---------------------------------------------------------------------------
# 고정 축 규약 상수
# ---------------------------------------------------------------------------

# Body 프레임(x=전방, y=왼쪽, z=위, gradient_map.py/slope_drive.py와 일치)에서
# 카메라 optical 프레임(x=오른쪽, y=아래, z=전방)으로. slope_drive.py의
# R_body_to_optical과 동일한 상수이며, 그곳에서 축 치환으로 검증됨.
R_BODY_TO_OPTICAL = np.array([
    [0., -1., 0.],
    [0., 0., -1.],
    [1., 0., 0.],
])
R_OPTICAL_TO_BODY = R_BODY_TO_OPTICAL.T  # rotation matrix is orthogonal

# 실측 전 임시값 — 실제 하드웨어에서 상보필터 튜닝 후 조정할 것.
COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER = 0.97


def roll_pitch_from_accel_body(accel_body: np.ndarray) -> tuple[float, float]:
    """body 프레임(x-전방,y-왼쪽,z-위) 가속도계 값으로부터 (roll, pitch) [rad]를 추정한다.

    표준 2축 기울기 공식: 정지 상태의 가속도계는 현재 "위"를 향하는
    로컬 축을 따라 대략 +g를 읽는다.
    roll은 전방(x) 축을 중심으로 한 회전(양수 = 오른쪽이 아래로);
    pitch는 왼쪽(y) 축을 중심으로 한 회전(양수 = 기수가 아래로).
    구성 회전 Rotation.from_euler('xyz', [roll, pitch, 0])에 대한 왕복
    합성 테스트로 검증됨.
    """
    ax, ay, az = accel_body
    roll = np.arctan2(ay, az)
    pitch = np.arctan2(-ax, np.hypot(ay, az))
    return float(roll), float(pitch)


def update_complementary_filter(
    prev_roll: float | None,
    prev_pitch: float | None,
    gyro_body: np.ndarray,
    accel_body: np.ndarray,
    dt: float,
    alpha: float = COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER,
) -> tuple[float, float]:
    """(roll, pitch)에 대한 상보 필터 1스텝, yaw 없음.

    angle = alpha * (prev_angle + gyro_rate * dt) + (1 - alpha) * accel_angle

    gyro_body[0]/[1]은 body x/y 축을 중심으로 한 roll-rate/pitch-rate이다.
    첫 호출 시(prev_roll이 None), 아직 블렌딩할 자이로 적분값이 없으므로
    가속도계만으로 추정한 값을 반환한다.
    """
    accel_roll, accel_pitch = roll_pitch_from_accel_body(accel_body)

    if prev_roll is None or prev_pitch is None:
        return accel_roll, accel_pitch

    roll = alpha * (prev_roll + gyro_body[0] * dt) + (1.0 - alpha) * accel_roll
    pitch = alpha * (prev_pitch + gyro_body[1] * dt) + (1.0 - alpha) * accel_pitch
    return float(roll), float(pitch)


def optical_vector_to_body(v) -> np.ndarray:
    """카메라 optical 축(x=오른쪽,y=아래,z=전방) 벡터를 body 축(x=전방,y=왼쪽,z=위)으로 변환한다.

    RealSense 내장 IMU는 frame_id가 'camera_*_optical_frame'이라 원시
    gyro/accel 축이 depth 포인트와 동일한 optical 프레임을 따른다고 가정한다
    — 따라서 역투영 포인트에 쓰는 것과 동일한 R_OPTICAL_TO_BODY 재매핑을
    그대로 적용한다. geometry_msgs/Vector3 등 .x/.y/.z 속성을 가진 아무
    객체나 받는다.
    """
    return R_OPTICAL_TO_BODY @ np.array([v.x, v.y, v.z], dtype=np.float64)
