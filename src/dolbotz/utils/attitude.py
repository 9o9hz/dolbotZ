"""
IMU 자세(roll/pitch) 추정 공용 유틸리티 — ROS 의존성 없음.

elevation_map.py와 flat_drive.py가 둘 다 RealSense 카메라에 내장된 동일한
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

마운트 파라미터 상수(MOUNT_*_PLACEHOLDER, COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER):
  카메라 높이/장착 각도/상보필터 alpha는 대회장에서 실측 후 자주 바뀔 수 있는
  값이라, config/*.yaml 파일 경로를 거치지 않고 의도적으로 코드에 상수로
  둔다 (config/ 파일을 고쳐서 재빌드/재배포하는 것보다 이 파일 한 곳의 숫자를
  바로 고치는 편이 대회 중 실측-반영 사이클에 더 알맞다). elevation_map.py와
  flat_drive.py가 예전에는 각각 리터럴 숫자(0.5/10.0/0.0/0.97)로 중복
  선언하고 있어서, 값 하나를 바꿀 때 두 파일을 다 고쳐야 하고 하나만 놓치면
  두 노드가 서로 다른 값으로 계산하는 위험이 있었다 — 이제 두 파일 모두 이
  상수를 import해서 declare_parameter 기본값으로 쓴다.
"""

import numpy as np

from dolbotz.utils.paths import load_calibration

# ---------------------------------------------------------------------------
# 고정 축 규약 상수
# ---------------------------------------------------------------------------

# Body 프레임(x=전방, y=왼쪽, z=위, gradient_map.py/flat_drive.py와 일치)에서
# 카메라 optical 프레임(x=오른쪽, y=아래, z=전방)으로. flat_drive.py의
# R_body_to_optical과 동일한 상수이며, 그곳에서 축 치환으로 검증됨.
R_BODY_TO_OPTICAL = np.array([
    [0., -1., 0.],
    [0., 0., -1.],
    [1., 0., 0.],
])
R_OPTICAL_TO_BODY = R_BODY_TO_OPTICAL.T  # rotation matrix is orthogonal

# 실측 전 임시값 — 실제 하드웨어에서 상보필터 튜닝 후 조정할 것.
COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER = 0.97

# 실측 전 임시값 — 지면 위 카메라 높이 [m]. 대회장에서 실측 후 갱신할 것.
MOUNT_CAMERA_HEIGHT_M_PLACEHOLDER = 0.5

# 실측 전 임시값 — 고정 카메라 장착 피치 [deg] (기수 하향이 양수). 실측 후 갱신할 것.
MOUNT_PITCH_OFFSET_DEG_PLACEHOLDER = 10.0

# 실측 전 임시값 — 고정 카메라 장착 롤 [deg]. 실측 후 갱신할 것.
MOUNT_ROLL_OFFSET_DEG_PLACEHOLDER = 0.0


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


def resolve_mount_defaults(serial_no: str) -> dict:
    """마운트 파라미터의 declare_parameter 기본값으로 쓸 dict를 만든다.

    우선순위: dolbotz.utils.paths.load_calibration(serial_no)가 값을 반환하면
    (해당 키가 피클에 있는 경우) 그 값을 쓰고, 없으면(피클 자체가 없거나
    특정 키가 빠져 있으면) 이 모듈의 MOUNT_*_PLACEHOLDER /
    COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER 상수로 폴백한다. 지금은 캘리브레이션
    피클이 하나도 없으므로(config/calibration/README.md 참고) 항상 상수가
    반환된다.

    이 함수가 반환한 값은 declare_parameter의 기본값일 뿐이므로, 사용자가
    실행 시 --ros-args -p camera_height_m:=X 등으로 명시적으로 넘기면 ROS
    파라미터 시스템이 그 값을 항상 최우선으로 쓴다 — 이 함수는 그 경우를
    신경 쓸 필요가 없다.
    """
    calibration = load_calibration(serial_no) or {}
    return {
        'camera_height_m': calibration.get('camera_height_m', MOUNT_CAMERA_HEIGHT_M_PLACEHOLDER),
        'camera_pitch_offset_deg': calibration.get(
            'camera_pitch_offset_deg', MOUNT_PITCH_OFFSET_DEG_PLACEHOLDER),
        'camera_roll_offset_deg': calibration.get(
            'camera_roll_offset_deg', MOUNT_ROLL_OFFSET_DEG_PLACEHOLDER),
        'complementary_filter_alpha': calibration.get(
            'complementary_filter_alpha', COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER),
    }
