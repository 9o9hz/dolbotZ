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

파라미터 (마운트 오프셋/상보필터 alpha 기본값은 dolbotz.utils.attitude의
MOUNT_*_PLACEHOLDER / COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER 상수 참고 —
실측 전 임시값이며 대회장에서 자주 바뀔 수 있어 의도적으로 config/*.yaml이
아니라 코드에 상수로 둔다. camera_serial_no로 config/calibration/의
캘리브레이션 피클을 지정하면 그 값이 이 상수보다 우선 적용된다):
  camera_serial_no         str    기본값 ''     — config/calibration/{model}_{serial}.pkl 조회용
  camera_height_m          float  attitude.MOUNT_CAMERA_HEIGHT_M_PLACEHOLDER
  camera_pitch_offset_deg  float  attitude.MOUNT_PITCH_OFFSET_DEG_PLACEHOLDER
  camera_roll_offset_deg   float  attitude.MOUNT_ROLL_OFFSET_DEG_PLACEHOLDER
  resolution_m             float  기본값 0.15   — 그리드 셀 크기 [m], gradient_map.py 기본값과 일치
  grid_forward_m           float  기본값 4.0    — 전방 맵 범위 [m]
  grid_width_m             float  기본값 4.0    — 측면 맵 범위 [m] (±grid_width_m/2)
  min_depth_m               float  기본값 0.5    — D435i 최소 인식거리 고려해 상향, PLACEHOLDER
  max_depth_m               float  기본값 4.0
  blind_fill_forward_m      float  기본값 0.6    — 사각지대 0.0 채움 범위 [m], PLACEHOLDER
  complementary_filter_alpha float attitude.COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER
  bench_log_hz              float  기본값 1.0
"""

import math
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, Imu

from dolbotz.utils.attitude import (
    R_BODY_TO_OPTICAL,
    R_OPTICAL_TO_BODY,
    optical_vector_to_body,
    resolve_mount_defaults,
    roll_pitch_from_accel_body,
    update_complementary_filter,
)

# ---------------------------------------------------------------------------
# 고정 임계값 상수
# ---------------------------------------------------------------------------

# PLACEHOLDER — D435i의 최소 인식거리(~0.5~0.6m)와 FOV 하단 한계를 고려해 상향된
# 임시값. 로봇 발밑(그리드 원점 근방)은 depth로 실측이 불가능한 사각지대이므로
# 이 범위 안의 NaN 셀은 "발밑 지면 = 0"으로 채운다. 실측 후 실제 카메라
# 사각지대 크기에 맞춰 조정할 것.
BLIND_FILL_FORWARD_M_PLACEHOLDER = 0.6


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


def camera_body_to_level_matrix(
    roll_meas: float,
    pitch_meas: float,
    roll_offset: float,
    pitch_offset: float,
) -> np.ndarray:
    """카메라-BODY 프레임(현재 카메라의 물리적 기울어진 프레임) 좌표를
    world-level 프레임(중력에 대해 수평, +x는 섀시 현재 헤딩) 좌표로
    매핑하는 회전 행렬.

    `roll_meas`/`pitch_meas`는 상보필터가 추정한 카메라의 총 기울기이며,
    이는 이미 고정 장착 기울기와 섀시의 동적 기울어짐이 결합된 값이다 —
    가속도계가 물리적으로 측정하는 것이 바로 이 결합된 총 기울기이기
    때문이다 (roll_pitch_from_accel_body/update_complementary_filter 참고).
    따라서 이 총 기울기를 한 번만 되돌리면 이미 world-level 프레임에
    도달하며, roll_offset/pitch_offset을 별도로 한 번 더 빼면 실제로는
    존재하지 않는 성분을 중복으로 제거하는 셈이 되어 오차가 생긴다
    (예: 마운트 피치 10도 + 섀시 수평 상태에서 실제 경사 15도인 지형이
    약 37도로 과대 계산됨을 독립적인 물리 시뮬레이션으로 확인함).

    roll_pitch_from_accel_body의 라운드트립 관계
    (accel = R.inv().apply([0,0,g]) 로 합성하면 roll_pitch_from_accel_body(accel)가
    R의 오일러각을 복원함, TestRollPitchFromAccelBody 참고)로부터,
    R_meas := Rotation.from_euler('xyz', [roll_meas, pitch_meas, 0])는
    world-level 프레임에서 카메라 현재 프레임으로의 회전이다
    (accel_camera_frame = R_meas.inv().apply(accel_world_level)). 따라서
    카메라-현재-프레임 좌표를 world-level 좌표로 되돌리려면 R_meas를
    "정방향"으로 적용해야 한다 — .inv()가 아니다:
    p_level = R_meas.apply(p_camera_frame).

    roll_offset/pitch_offset은 위 이유로 이 계산에 더 이상 쓰이지 않는다.
    총 측정 기울기가 이미 마운트 기울기를 포함하므로 별도로 뺄 필요가
    없기 때문이다. 호출 시그니처는 마운트 오프셋 값을 호출부에 명시적으로
    남겨두기 위해(및 호환성을 위해) 그대로 유지한다.
    """
    del roll_offset, pitch_offset  # 의도적으로 미사용 — 위 docstring 참고
    return Rotation.from_euler('xyz', [roll_meas, pitch_meas, 0]).as_matrix()


def points_to_elevation_grid(
    points_optical: np.ndarray,
    valid: np.ndarray,
    r_level: np.ndarray,
    camera_height_m: float,
    resolution_m: float,
    grid_forward_m: float,
    grid_width_m: float,
    blind_fill_forward_m: float = BLIND_FILL_FORWARD_M_PLACEHOLDER,
) -> np.ndarray:
    """유효한 카메라-optical 포인트들을 수평화된 고도 그리드에 투영한다.

    그리드 규약은 gradient_map.py와 일치: col은 +x(전방) 방향으로 증가,
    row는 -y 방향으로 증가(row 0 = 최대 y = 왼쪽). col=0 / row=center가
    로봇/카메라 자신의 위치이며; 전방 범위는 [0, grid_forward_m), 측면
    범위는 [-grid_width_m/2, grid_width_m/2)이다.

    같은 셀에 떨어지는 여러 포인트는 중앙값으로 집계한다 (반사면 뎁스
    노이즈에 강건함). 빈 셀은 NaN이다.

    Blind-spot 채움: 그리드 원점(col=0, 로봇/카메라 바로 아래)은 depth
    카메라의 최소 인식거리와 FOV 하단 한계 때문에 항상 실측이 불가능한
    사각지대라 NaN이 된다. x_forward < blind_fill_forward_m 범위의 셀 중
    실측 포인트가 없어 NaN인 셀은 0.0으로 채운다 — level 프레임의 높이
    기준이 (카메라 위치 - camera_height_m)이므로 로봇 발밑 지면은 정의상
    대략 0이고, 로봇이 현재 서 있는 자리는 통행 가능함이 자명하기 때문이다.
    이 범위 밖의 NaN(사각지대가 아니라 단순히 실측이 안 된 먼 거리 등)은
    채우지 않고 그대로 NaN으로 남긴다.
    """
    h_grid = max(1, round(grid_width_m / resolution_m))
    w_grid = max(1, round(grid_forward_m / resolution_m))

    pts = points_optical[valid]
    if pts.size == 0:
        grid = np.full((h_grid, w_grid), np.nan, dtype=np.float32)
        return _blind_fill(grid, resolution_m, blind_fill_forward_m)

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
        return _blind_fill(grid, resolution_m, blind_fill_forward_m)

    flat_idx = row * w_grid + col
    order = np.argsort(flat_idx)
    sorted_idx, sorted_elev = flat_idx[order], elevation[order]
    unique_idx, start_pos = np.unique(sorted_idx, return_index=True)
    for idx, group in zip(unique_idx, np.split(sorted_elev, start_pos[1:])):
        grid.flat[idx] = np.median(group)

    return _blind_fill(grid, resolution_m, blind_fill_forward_m)


def _blind_fill(grid: np.ndarray, resolution_m: float, blind_fill_forward_m: float) -> np.ndarray:
    """x_forward < blind_fill_forward_m 범위의 NaN 셀을 0.0(발밑 지면)으로 채운다.

    셀 col의 대표 x_forward는 그 셀의 하한(col * resolution_m)으로 판정한다
    (points_to_elevation_grid에서 col = floor(x_forward / resolution_m)로
    투영하는 것과 동일한 규약).
    """
    n_blind_cols = int(np.ceil(blind_fill_forward_m / resolution_m))
    n_blind_cols = min(n_blind_cols, grid.shape[1])
    if n_blind_cols <= 0:
        return grid
    blind = grid[:, :n_blind_cols]
    blind[np.isnan(blind)] = 0.0
    return grid


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class ElevationMapNode(Node):
    """ROS2 wrapper: depth+camera_info+imu를 구독해 /terrain/elevation_map을 게시한다."""

    def __init__(self):
        super().__init__('elevation_map_node')

        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/depth/camera_info')
        self.declare_parameter('imu_topic', '/camera/camera/imu')

        # 캘리브레이션 피클(config/calibration/)이 있으면 그 값을, 없으면
        # dolbotz.utils.attitude의 MOUNT_*_PLACEHOLDER 상수를 마운트 파라미터
        # 기본값으로 쓴다 (resolve_mount_defaults 참고). --ros-args로 명시적으로
        # 넘긴 값은 이 기본값과 무관하게 항상 최우선 적용된다.
        self.declare_parameter('camera_serial_no', '')
        serial_no = str(self.get_parameter('camera_serial_no').value)
        mount_defaults = resolve_mount_defaults(serial_no)

        # TODO: 아래 5개는 실측 전 임시값. 모듈 docstring의 파라미터 섹션 및
        # PLACEHOLDER 상수(BLIND_FILL_FORWARD_M_PLACEHOLDER) 참고.
        self.declare_parameter('camera_height_m', mount_defaults['camera_height_m'])
        self.declare_parameter('camera_pitch_offset_deg', mount_defaults['camera_pitch_offset_deg'])
        self.declare_parameter('camera_roll_offset_deg', mount_defaults['camera_roll_offset_deg'])
        self.declare_parameter('resolution_m', 0.15)
        self.declare_parameter('grid_forward_m', 4.0)
        self.declare_parameter('grid_width_m', 4.0)
        self.declare_parameter('min_depth_m', 0.5)
        self.declare_parameter('max_depth_m', 4.0)
        self.declare_parameter('blind_fill_forward_m', BLIND_FILL_FORWARD_M_PLACEHOLDER)
        self.declare_parameter('complementary_filter_alpha', mount_defaults['complementary_filter_alpha'])
        self.declare_parameter('bench_log_hz', 1.0)

        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        imu_topic = self.get_parameter('imu_topic').value

        self._camera_height_m = float(self.get_parameter('camera_height_m').value)
        self._pitch_offset = math.radians(float(self.get_parameter('camera_pitch_offset_deg').value))
        self._roll_offset = math.radians(float(self.get_parameter('camera_roll_offset_deg').value))
        self._res = float(self.get_parameter('resolution_m').value)
        self._grid_forward_m = float(self.get_parameter('grid_forward_m').value)
        self._grid_width_m = float(self.get_parameter('grid_width_m').value)
        self._min_depth_m = float(self.get_parameter('min_depth_m').value)
        self._max_depth_m = float(self.get_parameter('max_depth_m').value)
        self._blind_fill_forward_m = float(self.get_parameter('blind_fill_forward_m').value)
        self._alpha = float(self.get_parameter('complementary_filter_alpha').value)
        bench_period = 1.0 / max(0.1, self.get_parameter('bench_log_hz').value)

        self._bridge = CvBridge()
        self._timings: list[tuple] = []

        self._fx = self._fy = self._cx = self._cy = None
        self._roll_est: float | None = None
        self._pitch_est: float | None = None
        self._last_imu_stamp: float | None = None

        qos = qos_profile_sensor_data

        self._info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._on_camera_info, qos)
        self._imu_sub = self.create_subscription(
            Imu, imu_topic, self._on_imu, qos)
        self._depth_sub = self.create_subscription(
            Image, depth_topic, self._on_depth, qos)

        self._elevation_pub = self.create_publisher(Image, '/terrain/elevation_map', 10)

        self._bench_timer = self.create_timer(bench_period, self._log_timing)
        self.get_logger().info(
            f'ElevationMapNode ready — resolution={self._res} m, '
            f'listening on {depth_topic}, {camera_info_topic}, {imu_topic}'
        )

    # ------------------------------------------------------------------

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._fx, self._fy = msg.k[0], msg.k[4]
        self._cx, self._cy = msg.k[2], msg.k[5]

    def _on_imu(self, msg: Imu) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        accel_body = optical_vector_to_body(msg.linear_acceleration)
        gyro_body = optical_vector_to_body(msg.angular_velocity)

        dt = 0.0 if self._last_imu_stamp is None else max(0.0, stamp - self._last_imu_stamp)
        self._roll_est, self._pitch_est = update_complementary_filter(
            self._roll_est, self._pitch_est, gyro_body, accel_body, dt, alpha=self._alpha)
        self._last_imu_stamp = stamp

    def _on_depth(self, msg: Image) -> None:
        if self._fx is None:
            self.get_logger().warn(
                'CameraInfo not received yet; skipping frame.', throttle_duration_sec=2.0)
            return
        if self._roll_est is None or self._pitch_est is None:
            self.get_logger().warn(
                'No IMU message received yet; skipping frame.', throttle_duration_sec=2.0)
            return
            

        t0 = time.perf_counter()

        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().error(f'CvBridge decode failed: {exc}')
            return
        depth_m = depth_to_meters(np.asarray(cv_img))

        t1 = time.perf_counter()

        points_optical, valid = backproject_depth_to_points(
            depth_m, self._fx, self._fy, self._cx, self._cy,
            self._min_depth_m, self._max_depth_m)

        r_level = camera_body_to_level_matrix(
            self._roll_est, self._pitch_est, self._roll_offset, self._pitch_offset)

        grid = points_to_elevation_grid(
            points_optical, valid, r_level, self._camera_height_m, self._res,
            self._grid_forward_m, self._grid_width_m, self._blind_fill_forward_m)

        t2 = time.perf_counter()

        out = self._bridge.cv2_to_imgmsg(grid.astype(np.float32), encoding='32FC1')
        out.header = msg.header
        self._elevation_pub.publish(out)

        t3 = time.perf_counter()

        self._timings.append((
            (t1 - t0) * 1e3,   # decode
            (t2 - t1) * 1e3,   # compute
            (t3 - t2) * 1e3,   # publish
            (t3 - t0) * 1e3,   # total
        ))

    def _log_timing(self) -> None:
        if not self._timings:
            return
        arr = np.array(self._timings, dtype=np.float64)
        mean = arr.mean(axis=0)
        p95 = np.percentile(arr, 95, axis=0)
        self.get_logger().info(
            f'[bench {len(self._timings)} frames] '
            f'mean  decode={mean[0]:.2f} compute={mean[1]:.2f} '
            f'pub={mean[2]:.2f} total={mean[3]:.2f} ms | '
            f'p95_total={p95[3]:.2f} ms'
        )
        self._timings.clear()


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ElevationMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
