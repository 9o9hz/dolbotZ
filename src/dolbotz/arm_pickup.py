import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import message_filters

try:
    from ultralytics import YOLO
    _YOLO_OK = True
except ImportError:
    _YOLO_OK = False


class ArmPickupNode(Node):
    """
    RealSense 뎁스 카메라 + YOLO 학습모델로 서플라이 박스를 탐지하고
    카메라 좌표계 3D 좌표(m)를 로봇팔 제어용으로 퍼블리시한다.

    퍼블리시:
      /arm/target_point  (geometry_msgs/PointStamped)  — 카메라 프레임 XYZ
      /arm/debug_image   (sensor_msgs/Image)            — 바운딩박스 시각화

    파라미터:
      model_path         : YOLO .pt 경로 (필수)
      target_class       : 탐지할 클래스 이름 (기본 'supply_box')
      conf_threshold     : 최소 confidence (기본 0.5)
      depth_roi_radius   : 깊이 샘플링 반경 픽셀 (기본 5)
      max_depth_m        : 유효 거리 상한 m (기본 2.0)
      color_topic        : 컬러 이미지 토픽
      depth_topic        : 컬러 정렬 뎁스 토픽 (aligned_depth_to_color)
      camera_info_topic  : 컬러 카메라 info 토픽
    """

    def __init__(self):
        super().__init__('arm_pickup_node')

        self.declare_parameter('model_path', '/home/jecs/dolbotZ/supplybest.pt')
        self.declare_parameter('target_class', 'supplybox')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('depth_roi_radius', 5)
        self.declare_parameter('max_depth_m', 2.0)
        self.declare_parameter(
            'color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter(
            'depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter(
            'camera_info_topic', '/camera/camera/color/camera_info')

        model_path   = str(self.get_parameter('model_path').value)
        self.target  = str(self.get_parameter('target_class').value)
        self.conf_th = float(self.get_parameter('conf_threshold').value)
        self.depth_r = int(self.get_parameter('depth_roi_radius').value)
        self.max_d   = float(self.get_parameter('max_depth_m').value)
        color_topic  = str(self.get_parameter('color_topic').value)
        depth_topic  = str(self.get_parameter('depth_topic').value)
        info_topic   = str(self.get_parameter('camera_info_topic').value)

        self.model = self._load_model(model_path)

        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None

        self.create_subscription(
            CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)

        sub_color = message_filters.Subscriber(
            self, Image, color_topic, qos_profile=qos_profile_sensor_data)
        sub_depth = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=qos_profile_sensor_data)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [sub_color, sub_depth], queue_size=5, slop=0.05)
        self._sync.registerCallback(self._on_frames)

        self.pub_point = self.create_publisher(
            PointStamped, '/arm/target_point', 10)
        self.pub_debug = self.create_publisher(
            Image, '/arm/debug_image', 10)

        self.get_logger().info(
            f'ArmPickupNode ready  |  target={self.target}  '
            f'conf>={self.conf_th}  max_depth={self.max_d}m')

    # ------------------------------------------------------------------
    def _load_model(self, path: str):
        if not path:
            self.get_logger().warn(
                'model_path 미설정 — 학습파일 경로를 파라미터로 전달하세요')
            return None
        if not _YOLO_OK:
            self.get_logger().error(
                'ultralytics 미설치 — pip install ultralytics')
            return None
        model = YOLO(path)
        self.get_logger().info(f'모델 로드 완료: {path}')
        return model

    def _on_info(self, msg: CameraInfo):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]

    @staticmethod
    def _to_meters(cv_img: np.ndarray) -> np.ndarray:
        if cv_img.dtype == np.uint16:
            return cv_img.astype(np.float32) * 0.001
        return cv_img.astype(np.float32)

    def _sample_depth(self, depth: np.ndarray, u: int, v: int) -> float:
        """bbox 중심 주변 패치의 유효 깊이 중앙값 (m). 실패시 0.0."""
        h, w = depth.shape
        r = self.depth_r
        patch = depth[max(0, v - r):min(h, v + r + 1),
                      max(0, u - r):min(w, u + r + 1)]
        valid = patch[(patch > 0.05) & (patch < self.max_d)]
        return float(np.median(valid)) if valid.size >= 3 else 0.0

    def _on_frames(self, color_msg: Image, depth_msg: Image):
        if self.fx is None or self.model is None:
            return

        import cv2

        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth = self._to_meters(
            self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough'))

        results = self.model(color, verbose=False)

        best = None  # (conf, X, Y, Z, bbox, cls_name)
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf < self.conf_th:
                    continue
                cls_name = self.model.names.get(int(box.cls[0]), '')
                if cls_name != self.target:
                    continue
                if best and conf <= best[0]:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)
                z = self._sample_depth(depth, u, v)
                if z <= 0.0:
                    continue

                X = (u - self.cx) * z / self.fx
                Y = (v - self.cy) * z / self.fy
                best = (conf, X, Y, z, (x1, y1, x2, y2), cls_name, u, v)

        if best is None:
            return

        conf, X, Y, Z, bbox, cls_name, u, v = best

        pt = PointStamped()
        pt.header = color_msg.header
        pt.point.x = X
        pt.point.y = Y
        pt.point.z = Z
        self.pub_point.publish(pt)

        self.get_logger().info(
            f'[{cls_name} conf={conf:.2f}]  '
            f'X={X:.3f} Y={Y:.3f} Z={Z:.3f} m')

        # 디버그 이미지
        bx1, by1, bx2, by2 = (int(c) for c in bbox)
        cv2.rectangle(color, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
        cv2.circle(color, (u, v), 6, (0, 0, 255), -1)
        cv2.putText(
            color,
            f'{cls_name} {conf:.2f} | Z={Z:.2f}m',
            (bx1, by1 - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        dbg = self.bridge.cv2_to_imgmsg(color, encoding='bgr8')
        dbg.header = color_msg.header
        self.pub_debug.publish(dbg)


def main():
    rclpy.init()
    node = ArmPickupNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
