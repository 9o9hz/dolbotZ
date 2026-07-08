import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Float64MultiArray, Float32
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation
from dolbotz.utils.pruning import prune_branches
from dolbotz.utils.attitude import R_BODY_TO_OPTICAL


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


class UnifiedDriveNode(Node):
    def __init__(self):
        super().__init__('unified_drive_node')
        self.get_logger().info('Unified Drive Node has been started.')

        self.bridge = CvBridge()

        # --- Parameters ---
        self.declare_parameters(
            namespace='',
            parameters=[
                # BEV Parameters
                ('camera_height_m', 0.5),
                ('camera_pitch_offset_deg', 10.0),
                ('camera_roll_offset_deg', 0.0),
                ('bev_img_width', 400),
                ('bev_img_height', 400),
                ('bev_meters_per_pixel', 0.05),
                ('lane_threshold', 200),
                ('morph_kernel_size', 5),

                # Pruning Parameters
                ('length_threshold', 10),

                # Path Planner Parameters
                ('lookahead_distance_pixels', 50),
                ('steering_gain', 0.005),
                ('max_speed', 0.2)
            ])
        self.load_parameters()
        self.add_on_set_parameters_callback(self.parameters_callback)

        # --- Subscribers ---
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, qos_profile_sensor_data)
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.cam_info_callback, qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(
            Imu, '/camera/camera/imu', self.imu_callback, qos_profile_sensor_data)

        # --- Publishers ---
        # Control Outputs
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_pub = self.create_publisher(PointStamped, '/planning/target_point', 10)
        self.steering_debug_pub = self.create_publisher(Float32, '/planning/steering_debug', 10)

        # Intermediate Visual/Data Outputs (For visualizer_node and debugging)
        self.bev_image_pub = self.create_publisher(Image, '/bev/image', 10)
        self.mask_pub = self.create_publisher(Image, '/bev/mask', 10)
        self.skeleton_pub = self.create_publisher(Image, '/skeleton_image', 10)
        self.pruned_skeleton_pub = self.create_publisher(Image, '/pruned_skeleton', 10)
        self.debug_image_pub = self.create_publisher(Image, '/bev/debug_overlay', 10)
        self.homography_pub = self.create_publisher(Float64MultiArray, '/bev/H', 10)

        # Relay Outputs
        self.orientation_publisher = self.create_publisher(Imu, '/imu/orientation', 10)

        # --- State Variables ---
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_imu_msg = None
        self.image_to_ground_homography = None

    def load_parameters(self):
        self.camera_height = self.get_parameter('camera_height_m').value
        self.pitch_offset_rad = np.deg2rad(self.get_parameter('camera_pitch_offset_deg').value)
        self.roll_offset_rad = np.deg2rad(self.get_parameter('camera_roll_offset_deg').value)
        self.bev_width = self.get_parameter('bev_img_width').value
        self.bev_height = self.get_parameter('bev_img_height').value
        self.mpp = self.get_parameter('bev_meters_per_pixel').value
        self.lane_threshold = self.get_parameter('lane_threshold').value
        self.morph_kernel_size = self.get_parameter('morph_kernel_size').value

        self.length_threshold = self.get_parameter('length_threshold').value
        self.lookahead_dist = self.get_parameter('lookahead_distance_pixels').value
        self.k_p = self.get_parameter('steering_gain').value
        self.max_speed = self.get_parameter('max_speed').value

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'camera_height_m':
                self.camera_height = param.value
            elif param.name == 'camera_pitch_offset_deg':
                self.pitch_offset_rad = np.deg2rad(param.value)
            elif param.name == 'camera_roll_offset_deg':
                self.roll_offset_rad = np.deg2rad(param.value)
            elif param.name == 'bev_img_width':
                self.bev_width = param.value
            elif param.name == 'bev_img_height':
                self.bev_height = param.value
            elif param.name == 'bev_meters_per_pixel':
                self.mpp = param.value
            elif param.name == 'lane_threshold':
                self.lane_threshold = param.value
            elif param.name == 'morph_kernel_size':
                self.morph_kernel_size = param.value
            elif param.name == 'length_threshold':
                self.length_threshold = param.value
            elif param.name == 'lookahead_distance_pixels':
                self.lookahead_dist = param.value
            elif param.name == 'steering_gain':
                self.k_p = param.value
            elif param.name == 'max_speed':
                self.max_speed = param.value

        if self.latest_imu_msg is not None:
            self.update_homography(self.latest_imu_msg)
        return SetParametersResult(successful=True)

    def cam_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.destroy_subscription(self.cam_info_sub)
            self.get_logger().info('Camera info received and subscription destroyed.')

    def imu_callback(self, msg):
        # IMU Orientation Node Relay
        self.orientation_publisher.publish(msg)
        self.latest_imu_msg = msg
        if self.camera_matrix is not None:
            self.update_homography(msg)

    def update_homography(self, imu_msg):
        q = imu_msg.orientation
        R_world_to_imu = Rotation.from_quat([q.x, q.y, q.z, q.w])
        R_imu_to_cam_body = Rotation.from_euler('xyz', [self.roll_offset_rad, self.pitch_offset_rad, 0])
        R_world_to_cam_body = R_imu_to_cam_body * R_world_to_imu
        R_body_to_optical = Rotation.from_matrix([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])
        R_world_to_cam_optical = R_body_to_optical * R_world_to_cam_body

        R = R_world_to_cam_optical.as_matrix().T
        t_cam_in_world = np.array([0, 0, self.camera_height])
        t_vec = -R @ t_cam_in_world

        H_g2i = self.camera_matrix @ np.hstack((R[:, 0:1], R[:, 1:2], t_vec.reshape(3,1)))
        try:
            self.image_to_ground_homography = np.linalg.inv(H_g2i)
            h_msg = Float64MultiArray()
            h_msg.data = self.image_to_ground_homography.flatten().tolist()
            self.homography_pub.publish(h_msg)
        except np.linalg.LinAlgError:
            self.image_to_ground_homography = None

    def image_callback(self, msg):
        if self.camera_matrix is None or self.image_to_ground_homography is None:
            self.get_logger().warn(
                f'Waiting for data... CamInfo: {self.camera_matrix is not None}, '
                f'Homography: {self.image_to_ground_homography is not None}',
                throttle_duration_sec=2.0
            )
            return

        try:
            # 1. BEV Projection Node
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            undistorted_image = cv2.undistort(cv_image, self.camera_matrix, self.dist_coeffs)

            M = np.array([
                [1/self.mpp, 0, self.bev_width / 2],
                [0, -1/self.mpp, self.bev_height],
                [0, 0, 1]
            ])
            final_homography = M @ self.image_to_ground_homography
            bev_image = cv2.warpPerspective(undistorted_image, final_homography, (self.bev_width, self.bev_height))

            # 2. Edge Lane Node
            gray_image = cv2.cvtColor(bev_image, cv2.COLOR_BGR2GRAY)
            _, mask = cv2.threshold(gray_image, self.lane_threshold, 255, cv2.THRESH_BINARY)
            kernel_size = max(1, int(self.morph_kernel_size))
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
            mask_morphed = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            # 3. Skeletonization Node
            skeleton = cv2.ximgproc.thinning(mask_morphed, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)

            # 4. Branch Pruning Node
            pruned_image = prune_branches(skeleton, self.length_threshold)

            # 5. Path Planner Node
            h, w = pruned_image.shape
            robot_x = w // 2
            path_pixels = np.argwhere(pruned_image > 0)

            cmd_msg = Twist()
            if len(path_pixels) >= 10:
                target_y_idx = h - self.lookahead_dist
                differences = np.abs(path_pixels[:, 0] - target_y_idx)
                min_idx = np.argmin(differences)
                target_y, target_x = path_pixels[min_idx]

                error_x = robot_x - target_x
                steering_angle = self.k_p * error_x
                linear_speed = self.max_speed * max(0.1, 1.0 - abs(steering_angle))

                cmd_msg.linear.x = float(linear_speed)
                cmd_msg.angular.z = float(steering_angle)

                # Debug Target Publish
                p_msg = PointStamped()
                p_msg.header = msg.header
                p_msg.point.x = float(target_x)
                p_msg.point.y = float(target_y)
                self.target_pub.publish(p_msg)
                self.steering_debug_pub.publish(Float32(data=steering_angle))
            else:
                self.get_logger().warn('No path detected, stopping.', throttle_duration_sec=2.0)

            self.cmd_vel_pub.publish(cmd_msg)

            # Publish Intermediate Representations (For Visualizer Node)
            bev_msg = self.bridge.cv2_to_imgmsg(bev_image, "bgr8")
            bev_msg.header = msg.header
            self.bev_image_pub.publish(bev_msg)

            debug_msg = self.bridge.cv2_to_imgmsg(undistorted_image, "bgr8")
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)

            mask_msg = self.bridge.cv2_to_imgmsg(mask_morphed, "mono8")
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)

            skeleton_msg = self.bridge.cv2_to_imgmsg(skeleton, "mono8")
            skeleton_msg.header = msg.header
            self.skeleton_pub.publish(skeleton_msg)

            pruned_msg = self.bridge.cv2_to_imgmsg(pruned_image, 'mono8')
            pruned_msg.header = msg.header
            self.pruned_skeleton_pub.publish(pruned_msg)

        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
        except Exception as e:
            self.get_logger().error(f'Processing error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = UnifiedDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
