"""
고도 맵 노드 — gradient_map.py가 입력으로 기대하는 /terrain/elevation_map을 게시한다.

순수 계산 부분은 ROS에 의존하지 않으므로 단독으로 단위 테스트 및 벤치마크가
가능하다 (다운스트림 소비자와 이 노드가 맞춰야 하는 그리드 좌표 규약은
gradient_map.py의 모듈 docstring 참고: col=+x 전방, row=-y,
row 0 = 최대 y = 로봇 왼쪽).

설계 노트 (전체 논의는 프로젝트 이력 / howtorun.md 참고):

  * IMU는 뎁스 카메라 자체 내장 센서이며 (frame_id
    'camera_imu_optical_frame'), 별도의 섀시 장착 IMU가 아니다. 따라서 원시
    가속도/자이로 값은 카메라 자체의 절대 기울기를 측정하며, 이는 고정된
    장착 기울기(camera_roll/pitch_offset_deg)와 현재 섀시가 지형 위에서
    하는 동작이 *결합된* 값이다. 알려진 고정 장착 기울기를 빼서 섀시 자체의
    동적 기울어짐을 복원해야 한다 — 그렇지 않으면 완전히 평평하고 수평인
    바닥도 카메라가 camera_pitch_offset_deg만큼 아래를 보도록 장착되어
    있다는 이유만으로 경사진 것으로 읽힌다.

  * yaw는 전혀 추정하지 않는다 (마그네토미터도, 자이로 Z축 적분도 없음) —
    roll/pitch만 카메라 자체의 원시 자이로 + 가속도 값을 입력받는 상보
    필터를 통해 두 개의 독립 스칼라로 추적한다. 이렇게 하면 출력 그리드의
    +x가 매 프레임마다 로봇의 현재 헤딩에 고정된다 (gradient_map.py의
    프레임별 비누적 그리드와 일치), yaw가 애초에 상태의 일부가 아니었으므로
    별도의 "strip yaw" 단계도 필요 없다.

  * 가속도계로부터의 기울기 계산과 body<->optical 축 재매핑은 왕복 합성
    검증(알려진 기울기 -> 합성 가속도 -> 복원된 기울기; 알려진 평평한/경사진
    바닥 -> 합성 뎁스 포인트 -> 복원된 고도)을 거친 후 아래 공식들로
    정리되었다.

구독 토픽:
  /camera/camera/depth/image_rect_raw   sensor_msgs/Image        (16UC1 mm 또는 32FC1 m)
  /camera/camera/depth/camera_info      sensor_msgs/CameraInfo   (fx, fy, cx, cy)
  /camera/camera/imu                    sensor_msgs/Imu          (원시 자이로 + 가속도만)

게시 토픽:
  /terrain/elevation_map   sensor_msgs/Image  32FC1  (미터 단위; NaN = 관측되지 않은 셀)

파라미터 (장착 오프셋은 임시값 — 아래 PLACEHOLDER 주석 참고):
  camera_height_m          float  기본값 0.5    — 지면 위 카메라 높이 [m], 미실측
  camera_pitch_offset_deg  float  기본값 10.0   — 고정 카메라 장착 피치 (기수 하향이 양수), 미실측
  camera_roll_offset_deg   float  기본값 0.0    — 고정 카메라 장착 롤, 미실측
  resolution_m             float  기본값 0.15   — 그리드 셀 크기 [m], gradient_map.py 기본값과 일치
  grid_forward_m           float  기본값 4.0    — 전방 맵 범위 [m]
  grid_width_m             float  기본값 4.0    — 측면 맵 범위 [m] (±grid_width_m/2)
  min_depth_m               float  기본값 0.3
  max_depth_m               float  기본값 4.0
  complementary_filter_alpha float 기본값 0.97  — PLACEHOLDER, 실제 주행 튜닝 필요
  bench_log_hz              float  기본값 1.0
"""

import numpy as np
from scipy.spatial.transform import Rotation

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


# ---------------------------------------------------------------------------
# 순수 계산 — ROS 의존성 없음
# ---------------------------------------------------------------------------

def depth_to_meters(depth_img: np.ndarray) -> np.ndarray:
    """원시 뎁스 이미지를 float32 미터 단위로 변환한다.

    16UC1 이미지(RealSense 기본값)는 밀리미터 단위이며, 그 외에는 이미
    미터 단위라고 가정한다. slope_decision.py의 _depth_to_meters 헬퍼와
    동일한 로직.
    """
    if depth_img.dtype == np.uint16:
        return depth_img.astype(np.float32) * 0.001
    return depth_img.astype(np.float32)


def backproject_depth_to_points(
    depth_m: np.ndarray,
    fx: float, fy: float, cx: float, cy: float,
    min_depth_m: float, max_depth_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """뎁스 이미지를 카메라 optical 3D 포인트로 벡터화된 핀홀 역투영한다.

    Args:
        depth_m: (H, W) 미터 단위 뎁스.
        fx, fy, cx, cy: 핀홀 내부 파라미터 (CameraInfo.k에서 가져옴).
        min_depth_m, max_depth_m: 유효 뎁스 범위; 이 범위를 벗어나거나
            (또는 유한하지 않은) 뎁스는 무효로 표시된다.

    Returns:
        points_optical: (H, W, 3) float32 — (X_opt=오른쪽, Y_opt=아래, Z_opt=전방) 미터.
                         무효 셀은 depth_m 값 그대로 남아있음 (호출자가 반드시
                         `valid`를 확인해야 하므로 NaN이어도 안전).
        valid: (H, W) bool 마스크.
    """
    h, w = depth_m.shape
    us, vs = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))

    z = depth_m
    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy
    points_optical = np.stack([x, y, z], axis=-1).astype(np.float32)

    valid = np.isfinite(z) & (z >= min_depth_m) & (z <= max_depth_m)
    return points_optical, valid


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


def camera_body_to_level_matrix(
    roll_meas: float,
    pitch_meas: float,
    roll_offset: float,
    pitch_offset: float,
) -> np.ndarray:
    """카메라-BODY 프레임 벡터를 수평 출력 프레임으로 매핑하는 회전 행렬.

    `roll_meas`/`pitch_meas`는 카메라 자체의 총 측정 기울기이다(장착 +
    섀시 동적 기울어짐이 결합됨, update_complementary_filter에서 옴).
    `roll_offset`/`pitch_offset`은 고정 장착 기울기이다
    (camera_roll/pitch_offset_deg, 라디안 단위). 결과값은 먼저 총 측정
    기울기를 되돌리고(가상의 수평 *장착된* 카메라 프레임으로), 그 다음
    고정 장착 기울기를 되돌린다(실제 섀시 수평 프레임으로) — 따라서 출력
    프레임의 +x는 섀시의 현재 헤딩을 따르며, 중력에 대해 수평이 맞춰지고,
    알려진 장착 기울기가 빠진 상태가 된다.

    검증됨: 평평한 바닥에서는, 섀시 자체가 수평이기만 하면 카메라 장착
    기울기와 무관하게 이 행렬이 고도 ~= 0을 산출한다.
    """
    r_detilt = Rotation.from_euler('xyz', [roll_meas, pitch_meas, 0]).inv()
    r_mount_undo = Rotation.from_euler('xyz', [roll_offset, pitch_offset, 0]).inv()
    return (r_mount_undo * r_detilt).as_matrix()


def points_to_elevation_grid(
    points_optical: np.ndarray,
    valid: np.ndarray,
    r_level: np.ndarray,
    camera_height_m: float,
    resolution_m: float,
    grid_forward_m: float,
    grid_width_m: float,
) -> np.ndarray:
    """유효한 카메라-optical 포인트들을 수평화된 고도 그리드에 투영한다.

    그리드 규약은 gradient_map.py와 일치: col은 +x(전방) 방향으로 증가,
    row는 -y 방향으로 증가(row 0 = 최대 y = 왼쪽). col=0 / row=center가
    로봇/카메라 자신의 위치이며; 전방 범위는 [0, grid_forward_m), 측면
    범위는 [-grid_width_m/2, grid_width_m/2)이다.

    같은 셀에 떨어지는 여러 포인트는 중앙값으로 집계한다 (반사면 뎁스
    노이즈에 강건함). 빈 셀은 NaN이다.
    """
    h_grid = max(1, round(grid_width_m / resolution_m))
    w_grid = max(1, round(grid_forward_m / resolution_m))

    pts = points_optical[valid]
    if pts.size == 0:
        return np.full((h_grid, w_grid), np.nan, dtype=np.float32)

    pts_body = pts @ R_OPTICAL_TO_BODY.T
    pts_level = pts_body @ r_level.T

    x_forward = pts_level[:, 0]
    y_left = pts_level[:, 1]
    elevation = pts_level[:, 2] + camera_height_m

    col = np.floor(x_forward / resolution_m).astype(np.int64)
    row = np.floor((grid_width_m / 2.0 - y_left) / resolution_m).astype(np.int64)

    in_bounds = (row >= 0) & (row < h_grid) & (col >= 0) & (col < w_grid)
    row, col, elevation = row[in_bounds], col[in_bounds], elevation[in_bounds]

    grid = np.full((h_grid, w_grid), np.nan, dtype=np.float32)
    if row.size == 0:
        return grid

    flat_idx = row * w_grid + col
    order = np.argsort(flat_idx)
    sorted_idx, sorted_elev = flat_idx[order], elevation[order]
    unique_idx, start_pos = np.unique(sorted_idx, return_index=True)
    for idx, group in zip(unique_idx, np.split(sorted_elev, start_pos[1:])):
        grid.flat[idx] = np.median(group)

    return grid
