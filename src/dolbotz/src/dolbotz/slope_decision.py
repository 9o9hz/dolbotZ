import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class _DepthRoiWindow:
    """Owns one OpenCV window showing the depth ROI and the computed slope."""

    def __init__(self, window_name: str):
        self.window_name = window_name
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def show(self, depth_m: np.ndarray, left_box, right_box, slope_deg=None, max_depth_m=3.0):
        """left_box / right_box: (x0, y0, x1, y1) rectangles in image coordinates."""
        vis = np.clip(depth_m, 0, max_depth_m) / max_depth_m
        vis = (vis * 255).astype(np.uint8)
        vis = cv2.applyColorMap(vis, cv2.COLORMAP_JET)

        for (x0, y0, x1, y1), color in ((left_box, (0, 255, 0)), (right_box, (0, 0, 255))):
            cv2.rectangle(vis, (x0, y0), (x1, y1), color, 2)

        if slope_deg is not None:
            cv2.putText(vis, f"slope: {slope_deg:+.1f} deg", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        cv2.imshow(self.window_name, vis)
        cv2.waitKey(1)

    def close(self):
        cv2.destroyWindow(self.window_name)


class SideSlopeTriggerNode(Node):
    def __init__(self):
        super().__init__('side_slope_trigger_node')

        # ROI 설정 (바닥 영역)
        self.declare_parameter('roi_y_start_ratio', 0.60)
        self.declare_parameter('roi_y_end_ratio',   0.90)
        self.declare_parameter('sample_step', 8)
        self.declare_parameter('max_depth_m', 3.0)
        self.declare_parameter('depth_info_topic', '/camera/camera/depth/camera_info')
        self.declare_parameter('depth_image_topic', '/camera/camera/depth/image_rect_raw')

        # 로봇의 좌우 기준 거리(트레드 폭 등) — 중요!
        self.declare_parameter('track_width_m', 0.45)  # 예시: 45cm

        self.roi_y0 = float(self.get_parameter('roi_y_start_ratio').value)
        self.roi_y1 = float(self.get_parameter('roi_y_end_ratio').value)
        self.sample_step = int(self.get_parameter('sample_step').value)
        self.max_depth = float(self.get_parameter('max_depth_m').value)
        self.depth_info_topic = str(self.get_parameter('depth_info_topic').value)
        self.depth_image_topic = str(self.get_parameter('depth_image_topic').value)
        self.track_w = float(self.get_parameter('track_width_m').value)

        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None
        self.vis = _DepthRoiWindow('slope_decision')

        self.sub_info = self.create_subscription(
            CameraInfo, self.depth_info_topic, self.on_info, qos_profile_sensor_data
        )
        self.sub_depth = self.create_subscription(
            Image, self.depth_image_topic, self.on_depth, qos_profile_sensor_data
        )
        self.pub_slope = self.create_publisher(Float32, '/terrain/side_slope_angle_deg', 10)

        self.get_logger().info(f'Subscribing camera info: {self.depth_info_topic}')
        self.get_logger().info(f'Subscribing depth image: {self.depth_image_topic}')

    def on_info(self, msg: CameraInfo):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]

    def _depth_to_meters(self, cv_img: np.ndarray) -> np.ndarray:
        if cv_img.dtype == np.uint16:
            return cv_img.astype(np.float32) * 0.001
        return cv_img.astype(np.float32)

    def on_depth(self, msg: Image):
        if self.fx is None:
            return

        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth = self._depth_to_meters(cv_img)

        h, w = depth.shape[:2]
        y_start = int(h * self.roi_y0)
        y_end   = int(h * self.roi_y1)
        y_start = max(0, min(h-1, y_start))
        y_end   = max(0, min(h, y_end))

        # 중앙 일부(예: 10%)는 제외하고 좌/우를 더 “확실히” 나누는 게 안정적
        mid_margin = int(w * 0.05)
        left_x0, left_x1   = 0, w//2 - mid_margin
        right_x0, right_x1 = w//2 + mid_margin, w

        # stride 샘플링
        left_roi  = depth[y_start:y_end:self.sample_step, left_x0:left_x1:self.sample_step]
        right_roi = depth[y_start:y_end:self.sample_step, right_x0:right_x1:self.sample_step]

        def median_height(roi: np.ndarray, u0: int, v0: int):
            # 유효 depth만
            mask = np.isfinite(roi) & (roi > 0.1) & (roi < self.max_depth)
            if np.count_nonzero(mask) < 80:
                return None

            z = roi[mask]

            # roi 내 픽셀 좌표 복원(대략)
            # (정밀하게 하려면 mask의 (i,j) 인덱스를 써서 u,v를 만들면 됨)
            # 여기서는 “대표 높이”만 추정이 목적이라 간단화.
            v_center = (y_start + y_end) * 0.5
            u_center = u0 + (u0 + (roi.shape[1]*self.sample_step)) * 0.0  # 안 씀

            # 카메라 좌표에서 Y는 "아래" 방향일 수 있음. height proxy로 -Y를 사용
            # Y ≈ (v - cy) * Z / fy, 여기서 v를 ROI 중앙으로 둔 근사
            Y = (v_center - self.cy) * z / self.fy
            height = -Y  # 위쪽이 +가 되도록

            return float(np.nanmedian(height))

        hL = median_height(left_roi, left_x0, y_start)
        hR = median_height(right_roi, right_x0, y_start)
        if hL is None or hR is None:
            return

        dh = (hR - hL)  # +면 오른쪽이 더 높다(부호는 실측으로 확인 필요)
        roll_rad = math.atan2(dh, self.track_w)
        roll_deg = math.degrees(roll_rad)

        self.vis.show(
            depth,
            (left_x0, y_start, left_x1, y_end),
            (right_x0, y_start, right_x1, y_end),
            slope_deg=roll_deg,
            max_depth_m=self.max_depth,
        )

        out = Float32()
        out.data = float(roll_deg)
        self.pub_slope.publish(out)

    def destroy_node(self):
        self.vis.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = SideSlopeTriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
