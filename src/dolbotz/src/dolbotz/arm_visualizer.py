"""카메라 원본 영상 위에 arm_pickup의 탐지 결과(bbox)와 target_point(XYZ)를 합성해 보여주는 라이브 뷰."""
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

from dolbotz.utils.detection import decode_detection


class ArmVisualizerNode(Node):
    """카메라 원본 영상을 구독해 arm_pickup의 detection/target_point를 오버레이하고 창에 띄운다."""

    def __init__(self):
        super().__init__('arm_visualizer_node')

        self.declare_parameter(
            'image_topic', '/camera/camera/color/image_raw/compressed')
        self.declare_parameter('detection_topic', '/arm/detection')
        self.declare_parameter('point_topic', '/arm/target_point')
        self.declare_parameter('target_class', 'supplybox')
        self.declare_parameter('window_name', 'arm_debug')
        self.declare_parameter('point_stale_sec', 0.5)

        image_topic = str(self.get_parameter('image_topic').value)
        detection_topic = str(self.get_parameter('detection_topic').value)
        point_topic = str(self.get_parameter('point_topic').value)
        self.target_class = str(self.get_parameter('target_class').value)
        self.window_name = str(self.get_parameter('window_name').value)
        self.point_stale_sec = float(self.get_parameter('point_stale_sec').value)

        self.bridge = CvBridge()
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self._last_point = None
        self._last_point_stamp = None
        self._last_detection = None
        self._last_detection_stamp = None

        self.create_subscription(CompressedImage, image_topic, self._on_image, 10)
        self.create_subscription(
            Float32MultiArray, detection_topic, self._on_detection, 10)
        self.create_subscription(PointStamped, point_topic, self._on_point, 10)

        self.get_logger().info(
            f'ArmVisualizerNode ready  |  image={image_topic}  '
            f'detection={detection_topic}  point={point_topic}')

    def _is_fresh(self, stamp) -> bool:
        return (stamp is not None
                and (self.get_clock().now() - stamp).nanoseconds
                <= self.point_stale_sec * 1e9)

    def _on_point(self, msg: PointStamped):
        self._last_point = msg.point
        self._last_point_stamp = self.get_clock().now()

    def _on_detection(self, msg: Float32MultiArray):
        self._last_detection = decode_detection(msg)
        self._last_detection_stamp = self.get_clock().now()

    def _on_image(self, msg: CompressedImage):
        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self._is_fresh(self._last_detection_stamp):
            d = self._last_detection
            bx1, by1, bx2, by2 = (int(round(c)) for c in (d.x1, d.y1, d.x2, d.y2))
            u, v = int(round(d.u)), int(round(d.v))
            cv2.rectangle(img, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
            cv2.circle(img, (u, v), 6, (0, 0, 255), -1)
            cv2.putText(
                img,
                f'{self.target_class} {d.confidence:.2f}',
                (bx1, by1 - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if self._is_fresh(self._last_point_stamp):
            p = self._last_point
            text = f'X={p.x:.3f} Y={p.y:.3f} Z={p.z:.3f} m'
        else:
            text = 'no target'

        cv2.putText(img, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2)

        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyWindow(self.window_name)
        super().destroy_node()


def main():
    rclpy.init()
    node = ArmVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
