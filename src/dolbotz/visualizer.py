"""Shared OpenCV window helpers for dolbotz nodes."""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class SlopeVisualizer:
    """Owns one named OpenCV window for a node and renders into it."""

    def __init__(self, window_name: str):
        self.window_name = window_name
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def show_depth_roi(self, depth_m: np.ndarray, left_box, right_box, slope_deg=None, max_depth_m=3.0):
        """Render a depth image with the left/right sampling ROIs and the slope angle.

        left_box / right_box: (x0, y0, x1, y1) rectangles in image coordinates.
        """
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

    def show_image(self, bgr_img: np.ndarray):
        """Render an already-annotated BGR image as-is (e.g. detection overlays)."""
        cv2.imshow(self.window_name, bgr_img)
        cv2.waitKey(1)

    def show_value_panel(self, label: str, value, size=(400, 200)):
        """Render a blank panel with a label/value pair, for nodes with no image to show."""
        w, h = size
        panel = np.zeros((h, w, 3), dtype=np.uint8)
        cv2.putText(panel, label, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(panel, f"{value}", (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 2)
        cv2.imshow(self.window_name, panel)
        cv2.waitKey(1)

    def close(self):
        cv2.destroyWindow(self.window_name)


class DebugImageViewerNode(Node):
    """Subscribes to an already-annotated debug Image topic and shows it in a window."""

    def __init__(self):
        super().__init__('debug_image_viewer_node')

        self.declare_parameter('image_topic', '/arm/debug_image')
        self.declare_parameter('window_name', 'arm_debug')

        image_topic = str(self.get_parameter('image_topic').value)
        window_name = str(self.get_parameter('window_name').value)

        self.bridge = CvBridge()
        self.vis = SlopeVisualizer(window_name)

        self.create_subscription(Image, image_topic, self._on_image, 10)

        self.get_logger().info(f'DebugImageViewerNode ready  |  subscribing={image_topic}')

    def _on_image(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.vis.show_image(img)

    def destroy_node(self):
        self.vis.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = DebugImageViewerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
