"""
평지 주행가능영역 추종 노드 — 색상 카메라로 세그멘테이션한 "주행가능영역"의
중심선을 코스 경계를 벗어나지 않는 목표경로로 게시한다.

gradient_map.py(경사 구간)와 역할 경계가 동일하다: 이 노드는 좌표(경로)만
계산해서 발행하고, 실제 로봇 제어(속도/조향 명령)나 평지/경사 전환 판단은
하지 않는다 — 항상 "지금 보이는 화면 기준 주행가능영역 경로"만 계산한다.

순수 계산 부분(호모그래피 유도, 세그멘테이션 마스크 -> BEV -> centerline)은
ROS에 의존하지 않으므로 단독으로 단위 테스트 가능하다 (test/test_flat_drive.py).

좌표 규약 (elevation_map.py/gradient_map.py와 일치): world-level(=body-level)
프레임은 카메라 위치를 원점으로 하며 x=전방, y=왼쪽, z=위이다.

IMU 자세보정: 이 노드의 IMU는 elevation_map.py와 동일한 카메라 내장 IMU이다
(별도 섀시 장착 IMU 아님, frame_id 'camera_*_optical_frame'). orientation
필드는 항상 무효값(0,0,0,0)이므로 절대 사용하지 않는다 — linear_acceleration/
angular_velocity만으로 dolbotz.utils.attitude의 상보필터를 돌려 roll/pitch를
추정한다. 이 추정치는 이미 마운트+섀시 결합 총 기울기이므로, 한 번만 되돌리면
world-level에 도달한다 (elevation_map.py의 camera_body_to_level_matrix()에서
발견/수정된 마운트 오프셋 이중 제거 버그와 동일한 원칙 — 별도로 마운트
오프셋을 다시 빼지 않는다).

구독 토픽:
  /camera/camera/color/image_raw    sensor_msgs/Image        (bgr8)
  /camera/camera/color/camera_info  sensor_msgs/CameraInfo   (fx,fy,cx,cy,왜곡계수)
  /camera/camera/imu                sensor_msgs/Imu          (원시 자이로 + 가속도만)

게시 토픽:
  /flatdrive/planned_path   nav_msgs/Path          목표경로 (x=전방,y=좌측, m)
  /planning/target_point    geometry_msgs/PointStamped  디버그용 첫 waypoint
  /bev/image                sensor_msgs/Image      BEV로 투영한 컬러 이미지 (디버그)
  /bev/mask                 sensor_msgs/Image      BEV로 투영한 세그멘테이션 마스크 (디버그)
  /bev/debug_overlay        sensor_msgs/Image      왜곡보정 원본 + 세그멘테이션 컨투어 (디버그)
  /bev/centerline_overlay   sensor_msgs/Image      BEV 마스크 + 추출된 중심선 (디버그)
  /bev/H                    std_msgs/Float64MultiArray  image->ground 호모그래피 (디버그)

파라미터 (마운트 오프셋/BEV 해상도는 임시값 — 아래 PLACEHOLDER 주석 참고):
  camera_height_m           float  기본값 0.5    — elevation_map.py와 동일 파라미터명, 미실측
  camera_pitch_offset_deg   float  기본값 10.0   — elevation_map.py와 동일 파라미터명, 미실측
  camera_roll_offset_deg    float  기본값 0.0    — elevation_map.py와 동일 파라미터명, 미실측
  complementary_filter_alpha float 기본값 0.97   — attitude.py 상수 재사용, 실주행 튜닝 대상
  bev_meters_per_pixel      float  기본값 0.03   — PLACEHOLDER, 6m x 6m 커버리지 기준 재계산값
  bev_img_width             int    기본값 200    — PLACEHOLDER (0.03 * 200 = 6.0 m)
  bev_img_height             int    기본값 200    — PLACEHOLDER (0.03 * 200 = 6.0 m)
  model_path                str    기본값 'runs/segment/dolbotz_seg_v1/weights/best_openvino_model'
  conf_threshold             float  기본값 0.5
  min_row_pixels             int    기본값 5      — PLACEHOLDER, 실측 튜닝 대상
  bench_log_hz               float  기본값 1.0
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation

from dolbotz.utils.attitude import (
    COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER,
    R_BODY_TO_OPTICAL,
    optical_vector_to_body,
    update_complementary_filter,
)

try:
    from ultralytics import YOLO
    _YOLO_OK = True
except ImportError:
    _YOLO_OK = False

# PLACEHOLDER — 실측/튜닝 전 임시값. 세그멘테이션 마스크의 row별 유효 픽셀 수가
# 이보다 적으면 그 row는 경로에서 건너뛴다 (bev_mask_to_centerline_path 참고).
MIN_ROW_PIXELS_PLACEHOLDER = 5

# 세그멘테이션 모델이 인식하는 유일한 클래스 (runs/segment/dolbotz_seg_v1/
# weights/best_openvino_model/metadata.yaml 참고: names: {0: area}).
DRIVABLE_AREA_CLASS_NAME = 'area'


# ---------------------------------------------------------------------------
# 순수 계산 — ROS 의존성 없음 (지면 호모그래피)
# ---------------------------------------------------------------------------
#
# 좌표 규약 (elevation_map.py/gradient_map.py와 일치): world-level(=body-level)
# 프레임은 카메라 위치를 원점으로 하며 x=전방, y=왼쪽, z=위이다. 지면은
# z=-camera_height_m 평면으로 취급한다 (elevation_map.py의
# `elevation = pts_level[:,2] + camera_height_m` 규약과 동일 — 지면
# elevation=0은 z=-camera_height_m에 대응).
#
# 이 노드의 IMU는 elevation_map.py와 동일한 카메라 내장 IMU이므로 동일한
# 원칙이 적용된다: roll_meas/pitch_meas(상보필터 추정치)는 이미 마운트+섀시
# 결합 총 기울기이며, 한 번만 되돌리면 world-level에 도달한다.
# camera_body_to_level_matrix()의 관계를 그대로 재사용한다:
#   R_meas := Rotation.from_euler('xyz', [roll_meas, pitch_meas, 0])
#   accel_camera_frame = R_meas.inv().apply(accel_world_level)
# 즉 R_meas.inv()가 world-level → camera-현재-body 프레임 회전이다.


def ground_to_image_homography(
    camera_matrix: np.ndarray,
    roll_meas: float,
    pitch_meas: float,
    roll_offset: float,
    pitch_offset: float,
    camera_height_m: float,
) -> np.ndarray:
    """지면(z=-camera_height_m 평면) 위의 world-level 좌표 (x_forward, y_left)를
    원본(왜곡보정된) 카메라 이미지의 동차 픽셀 좌표로 매핑하는 3x3 호모그래피
    H_g2i를 핀홀 투영식으로부터 처음부터 유도한다.

    유도:
      P_level = [x_forward, y_left, -camera_height_m]  (지면 위 점, 카메라 위치 원점)
      P_body_current = R_meas.inv().apply(P_level)      (world-level → 카메라 현재 body 프레임)
      P_optical = R_BODY_TO_OPTICAL @ P_body_current    (body → optical, 검증된 상수)
      [u,v,w] ~ camera_matrix @ P_optical                (표준 핀홀 투영)

    A := R_BODY_TO_OPTICAL @ R_meas.inv().as_matrix() 로 두면
    P_optical = x_forward*A[:,0] + y_left*A[:,1] - camera_height_m*A[:,2] 이므로
      H_g2i = camera_matrix @ [A[:,0], A[:,1], -camera_height_m*A[:,2]]
    (열벡터 3개를 나열한 3x3 행렬).

    roll_offset/pitch_offset은 camera_body_to_level_matrix()와 동일한 이유로
    이 계산에 쓰이지 않는다 — roll_meas/pitch_meas가 이미 마운트+섀시 결합
    총 기울기이므로 마운트 오프셋을 별도로 다시 반영하면 존재하지 않는
    성분을 중복 제거/추가하는 오차가 생긴다. 호출 시그니처는 마운트 오프셋
    값을 호출부에 명시적으로 남겨두기 위해 유지한다.
    """
    del roll_offset, pitch_offset  # 의도적으로 미사용 — 위 docstring 참고
    r_meas_inv = Rotation.from_euler('xyz', [roll_meas, pitch_meas, 0]).inv()
    a = R_BODY_TO_OPTICAL @ r_meas_inv.as_matrix()
    ground_cols = np.column_stack([a[:, 0], a[:, 1], -camera_height_m * a[:, 2]])
    return camera_matrix @ ground_cols


def bev_ground_projection_matrix(
    bev_width_px: int,
    bev_height_px: int,
    bev_meters_per_pixel: float,
) -> np.ndarray:
    """world-level 지면 좌표 (x_forward, y_left)를 BEV 픽셀 (col, row)로 매핑하는
    3x3 아핀 행렬 M.

    BEV 이미지 레이아웃(일반적인 주행 BEV 관례 — 로봇이 이미지 하단 중앙에
    위치, 전방이 이미지 위쪽으로 멀어짐):
      col(가로) = bev_width_px/2  - y_left / mpp   (y_left>0=왼쪽 → col 감소, 즉 이미지 왼쪽)
      row(세로) = bev_height_px   - x_forward / mpp (x_forward↑ → row 감소, 즉 이미지 위쪽)

    row=bev_height_px-1(하단)이 로봇 바로 앞(x_forward≈0), row=0(상단)이
    가장 먼 전방이다. 이 레이아웃 덕분에 "row별로 훑으며 좌우(col) 중심을
    구해 전방 경로점으로 삼는다"는 이후 단계(bev_mask_to_centerline_path)가
    자연스럽게 성립한다.
    """
    mpp = bev_meters_per_pixel
    return np.array([
        [0.0, -1.0 / mpp, bev_width_px / 2.0],
        [-1.0 / mpp, 0.0, float(bev_height_px)],
        [0.0, 0.0, 1.0],
    ])


def bev_pixel_to_meters(
    row: float,
    col: float,
    bev_width_px: int,
    bev_height_px: int,
    bev_meters_per_pixel: float,
) -> tuple[float, float]:
    """bev_ground_projection_matrix()의 역변환: BEV 픽셀 (row, col) -> world-level
    미터 좌표 (x_forward, y_left)."""
    mpp = bev_meters_per_pixel
    x_forward = (bev_height_px - row) * mpp
    y_left = (bev_width_px / 2.0 - col) * mpp
    return float(x_forward), float(y_left)


def image_to_bev_homography(
    camera_matrix: np.ndarray,
    roll_meas: float,
    pitch_meas: float,
    roll_offset: float,
    pitch_offset: float,
    camera_height_m: float,
    bev_width_px: int,
    bev_height_px: int,
    bev_meters_per_pixel: float,
) -> np.ndarray:
    """원본 이미지 픽셀 -> BEV 픽셀 순방향 호모그래피 (cv2.warpPerspective(src, H,
    (bev_width_px, bev_height_px))에 바로 사용). ground_to_image_homography()의
    역행렬(image->ground)에 bev_ground_projection_matrix()(ground->bev)를
    합성한다."""
    h_g2i = ground_to_image_homography(
        camera_matrix, roll_meas, pitch_meas, roll_offset, pitch_offset, camera_height_m)
    h_i2g = np.linalg.inv(h_g2i)
    m = bev_ground_projection_matrix(bev_width_px, bev_height_px, bev_meters_per_pixel)
    return m @ h_i2g


# ---------------------------------------------------------------------------
# 순수 계산 — ROS 의존성 없음 (세그멘테이션 마스크 -> BEV -> centerline 경로)
# ---------------------------------------------------------------------------

def mask_to_bev(
    mask: np.ndarray,
    homography: np.ndarray,
    bev_width_px: int,
    bev_height_px: int,
) -> np.ndarray:
    """단일 채널 바이너리(0/255) 마스크를 순방향 호모그래피로 BEV 평면에 투영한다.

    cv2.warpPerspective의 얇은 래퍼. 반드시 원본(왜곡보정된) 원근 이미지
    좌표계의 마스크를 입력으로 받아야 한다 — 세그멘테이션 모델은 일반 원근
    시점 이미지로 학습되었으므로, BEV로 먼저 왜곡한 이미지에 추론을 돌리면
    학습 분포를 벗어나 결과가 나빠진다. 따라서 추론은 항상 원본 이미지에서
    먼저 수행하고, 그 결과 마스크만 이 함수로 BEV에 투영한다.
    마스크는 이진값이므로 보간으로 인한 경계 흐림을 피하기 위해
    INTER_NEAREST를 사용한다.
    """
    return cv2.warpPerspective(
        mask, homography, (bev_width_px, bev_height_px), flags=cv2.INTER_NEAREST)


def bev_mask_to_centerline_path(
    bev_mask: np.ndarray,
    min_row_pixels: int,
    bev_width_px: int,
    bev_height_px: int,
    bev_meters_per_pixel: float,
) -> list[tuple[float, float]]:
    """BEV 마스크에서 row(전방 거리)별 좌우 중심(centroid) 경로를 추출한다.

    bev_ground_projection_matrix()의 레이아웃(row가 작을수록 더 먼 전방,
    row=bev_height_px-1이 로봇 바로 앞)을 그대로 따라서, 로봇에 가까운
    row부터 먼 row 순서로 훑는다. 각 row에서 마스크가 켜진(>0) 픽셀 개수가
    min_row_pixels 미만이면 그 row는 건너뛴다(구멍이 있는 경로로 남음 —
    보간/채움을 하지 않는다). 나머지 row는 켜진 픽셀들의 열(column) 평균을
    그 row의 좌우 중심으로 삼아 bev_pixel_to_meters()로 미터 좌표로 변환한다.

    Returns:
        (x_forward, y_left) 미터 좌표 튜플의 리스트, 로봇에 가까운 순서부터.
        min_row_pixels를 만족하는 row가 하나도 없으면 빈 리스트.
    """
    path: list[tuple[float, float]] = []
    for row in range(bev_height_px - 1, -1, -1):
        row_on = bev_mask[row] > 0
        if int(np.count_nonzero(row_on)) < min_row_pixels:
            continue
        cols = np.flatnonzero(row_on)
        centroid_col = float(cols.mean())
        path.append(bev_pixel_to_meters(
            row, centroid_col, bev_width_px, bev_height_px, bev_meters_per_pixel))
    return path


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class FlatDriveNode(Node):
    """ROS2 wrapper: color+camera_info+imu를 구독해 /flatdrive/planned_path를 게시한다."""

    def __init__(self):
        super().__init__('flat_drive_node')

        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('imu_topic', '/camera/camera/imu')

        # TODO: 아래는 실측 전 임시값. elevation_map.py와 동일 파라미터명 유지
        # (같은 카메라이므로 launch에서 값 공유 가능).
        self.declare_parameter('camera_height_m', 0.5)
        self.declare_parameter('camera_pitch_offset_deg', 10.0)
        self.declare_parameter('camera_roll_offset_deg', 0.0)
        self.declare_parameter('complementary_filter_alpha', COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER)

        # PLACEHOLDER — gradient_map.py의 grid_forward_m=4.0과 커버리지를 맞추는
        # 방향으로 재계산된 임시값 (0.03 m/px * 200 px = 6.0 m 정사각 커버리지).
        # 실측/튜닝 대상.
        self.declare_parameter('bev_meters_per_pixel', 0.03)
        self.declare_parameter('bev_img_width', 200)
        self.declare_parameter('bev_img_height', 200)

        self.declare_parameter('model_path', 'runs/segment/dolbotz_seg_v1/weights/best_openvino_model')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('min_row_pixels', MIN_ROW_PIXELS_PLACEHOLDER)

        self.declare_parameter('bench_log_hz', 1.0)

        color_topic = str(self.get_parameter('color_topic').value)
        camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        imu_topic = str(self.get_parameter('imu_topic').value)

        self._camera_height_m = float(self.get_parameter('camera_height_m').value)
        self._pitch_offset_rad = np.deg2rad(float(self.get_parameter('camera_pitch_offset_deg').value))
        self._roll_offset_rad = np.deg2rad(float(self.get_parameter('camera_roll_offset_deg').value))
        self._alpha = float(self.get_parameter('complementary_filter_alpha').value)

        self._mpp = float(self.get_parameter('bev_meters_per_pixel').value)
        self._bev_w = int(self.get_parameter('bev_img_width').value)
        self._bev_h = int(self.get_parameter('bev_img_height').value)

        model_path = str(self.get_parameter('model_path').value)
        self._conf_th = float(self.get_parameter('conf_threshold').value)
        self._min_row_pixels = int(self.get_parameter('min_row_pixels').value)

        bench_period = 1.0 / max(0.1, self.get_parameter('bench_log_hz').value)

        self._model = self._load_model(model_path)

        self._bridge = CvBridge()
        self._timings: list[tuple] = []

        self._camera_matrix = None
        self._dist_coeffs = None
        self._roll_est: float | None = None
        self._pitch_est: float | None = None
        self._last_imu_stamp: float | None = None

        qos = qos_profile_sensor_data

        self._info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._on_camera_info, qos)
        self._imu_sub = self.create_subscription(
            Imu, imu_topic, self._on_imu, qos)
        self._image_sub = self.create_subscription(
            Image, color_topic, self._on_image, qos)

        self._path_pub = self.create_publisher(Path, '/flatdrive/planned_path', 10)
        self._target_pub = self.create_publisher(PointStamped, '/planning/target_point', 10)
        self._bev_image_pub = self.create_publisher(Image, '/bev/image', 10)
        self._mask_pub = self.create_publisher(Image, '/bev/mask', 10)
        self._debug_image_pub = self.create_publisher(Image, '/bev/debug_overlay', 10)
        self._centerline_overlay_pub = self.create_publisher(Image, '/bev/centerline_overlay', 10)
        self._homography_pub = self.create_publisher(Float64MultiArray, '/bev/H', 10)

        self._bench_timer = self.create_timer(bench_period, self._log_timing)
        self.get_logger().info(
            f'FlatDriveNode ready — bev={self._bev_w}x{self._bev_h}@{self._mpp}m/px, '
            f'listening on {color_topic}, {camera_info_topic}, {imu_topic}'
        )

    # ------------------------------------------------------------------

    def _load_model(self, path: str):
        if not path:
            self.get_logger().warn('model_path 미설정 — 세그멘테이션 모델 경로를 파라미터로 전달하세요')
            return None
        if not _YOLO_OK:
            self.get_logger().error('ultralytics 미설치 — pip install ultralytics openvino')
            return None
        model = YOLO(path)
        self.get_logger().info(f'세그멘테이션 모델 로드 완료: {path}')
        return model

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._camera_matrix is None:
            self._camera_matrix = np.array(msg.k).reshape((3, 3))
            self._dist_coeffs = np.array(msg.d)
            self.destroy_subscription(self._info_sub)
            self.get_logger().info('CameraInfo 수신 완료, 구독 해제.')

    def _on_imu(self, msg: Imu) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        accel_body = optical_vector_to_body(msg.linear_acceleration)
        gyro_body = optical_vector_to_body(msg.angular_velocity)

        dt = 0.0 if self._last_imu_stamp is None else max(0.0, stamp - self._last_imu_stamp)
        self._roll_est, self._pitch_est = update_complementary_filter(
            self._roll_est, self._pitch_est, gyro_body, accel_body, dt, alpha=self._alpha)
        self._last_imu_stamp = stamp

    # ------------------------------------------------------------------

    def _segmentation_mask(self, image_bgr: np.ndarray) -> np.ndarray:
        """원본(왜곡보정된) 이미지에 세그멘테이션 추론을 돌려 'area' 클래스의
        바이너리(0/255) 마스크를 만든다. 모델은 정적 640x640 입력으로 export되어
        있으므로 imgsz=640을 명시한다 (metadata.yaml: dynamic=false)."""
        h, w = image_bgr.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)

        results = self._model(image_bgr, imgsz=640, conf=self._conf_th, verbose=False)
        for r in results:
            if r.masks is None or r.boxes is None:
                continue
            for poly, cls_idx in zip(r.masks.xy, r.boxes.cls):
                cls_name = self._model.names.get(int(cls_idx), '')
                if cls_name != DRIVABLE_AREA_CLASS_NAME:
                    continue
                pts = poly.astype(np.int32)
                if pts.shape[0] >= 3:
                    cv2.fillPoly(mask, [pts], 255)
        return mask

    def _path_to_msg(self, path_points: list[tuple[float, float]], header) -> Path:
        msg = Path()
        msg.header = header
        for x_forward, y_left in path_points:
            pose = PoseStamped()
            pose.header = header
            pose.pose.position.x = float(x_forward)
            pose.pose.position.y = float(y_left)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def _draw_centerline_overlay(self, bev_mask: np.ndarray, path_points: list[tuple[float, float]]) -> np.ndarray:
        overlay = cv2.cvtColor(bev_mask, cv2.COLOR_GRAY2BGR)
        pts_px = []
        for x_forward, y_left in path_points:
            col = self._bev_w / 2.0 - y_left / self._mpp
            row = self._bev_h - x_forward / self._mpp
            pts_px.append((int(round(col)), int(round(row))))
        for pt in pts_px:
            cv2.circle(overlay, pt, 2, (0, 0, 255), -1)
        for p1, p2 in zip(pts_px, pts_px[1:]):
            cv2.line(overlay, p1, p2, (0, 255, 0), 1)
        return overlay

    # ------------------------------------------------------------------

    def _on_image(self, msg: Image) -> None:
        if self._camera_matrix is None:
            self.get_logger().warn('CameraInfo 대기 중.', throttle_duration_sec=2.0)
            return
        if self._roll_est is None or self._pitch_est is None:
            self.get_logger().warn('IMU 메시지 대기 중.', throttle_duration_sec=2.0)
            return
        if self._model is None:
            self.get_logger().warn('세그멘테이션 모델 미로드.', throttle_duration_sec=2.0)
            return

        t0 = time.perf_counter()

        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'CvBridge decode failed: {exc}')
            return
        undistorted = cv2.undistort(cv_image, self._camera_matrix, self._dist_coeffs)

        t1 = time.perf_counter()

        raw_mask = self._segmentation_mask(undistorted)

        final_homography = image_to_bev_homography(
            self._camera_matrix, self._roll_est, self._pitch_est,
            self._roll_offset_rad, self._pitch_offset_rad, self._camera_height_m,
            self._bev_w, self._bev_h, self._mpp)
        bev_mask = mask_to_bev(raw_mask, final_homography, self._bev_w, self._bev_h)
        path_points = bev_mask_to_centerline_path(
            bev_mask, self._min_row_pixels, self._bev_w, self._bev_h, self._mpp)

        t2 = time.perf_counter()

        self._path_pub.publish(self._path_to_msg(path_points, msg.header))

        if path_points:
            x_forward, y_left = path_points[0]
            target = PointStamped()
            target.header = msg.header
            target.point.x = float(x_forward)
            target.point.y = float(y_left)
            self._target_pub.publish(target)
        else:
            self.get_logger().warn('주행가능영역 경로를 찾지 못함.', throttle_duration_sec=2.0)

        bev_color = cv2.warpPerspective(undistorted, final_homography, (self._bev_w, self._bev_h))
        bev_image_msg = self._bridge.cv2_to_imgmsg(bev_color, encoding='bgr8')
        bev_image_msg.header = msg.header
        self._bev_image_pub.publish(bev_image_msg)

        mask_msg = self._bridge.cv2_to_imgmsg(bev_mask, encoding='mono8')
        mask_msg.header = msg.header
        self._mask_pub.publish(mask_msg)

        debug_overlay = undistorted.copy()
        contours, _ = cv2.findContours(raw_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug_overlay, contours, -1, (0, 255, 0), 2)
        debug_msg = self._bridge.cv2_to_imgmsg(debug_overlay, encoding='bgr8')
        debug_msg.header = msg.header
        self._debug_image_pub.publish(debug_msg)

        centerline_overlay = self._draw_centerline_overlay(bev_mask, path_points)
        centerline_msg = self._bridge.cv2_to_imgmsg(centerline_overlay, encoding='bgr8')
        centerline_msg.header = msg.header
        self._centerline_overlay_pub.publish(centerline_msg)

        h_g2i = ground_to_image_homography(
            self._camera_matrix, self._roll_est, self._pitch_est,
            self._roll_offset_rad, self._pitch_offset_rad, self._camera_height_m)
        h_msg = Float64MultiArray()
        h_msg.data = np.linalg.inv(h_g2i).flatten().tolist()
        self._homography_pub.publish(h_msg)

        t3 = time.perf_counter()

        self._timings.append((
            (t1 - t0) * 1e3,   # decode+undistort
            (t2 - t1) * 1e3,   # segment+bev+centerline
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
    node = FlatDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
