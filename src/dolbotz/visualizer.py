"""Shared OpenCV window helpers for dolbotz nodes."""
import cv2
import numpy as np


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
