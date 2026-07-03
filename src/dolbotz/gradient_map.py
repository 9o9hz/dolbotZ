"""
Gradient Field Node — step (a) of the terrain path-planning pipeline.

Pure computation (compute_gradient_field) is ROS-free so it can be unit-tested
and benchmarked standalone.

Coordinate convention used throughout this package:
  elevation[r, c]  — height in metres at grid cell (r, c)
  col  increases in  +x direction  (robot forward)
  row  increases in  -y direction  (row 0 = max y = robot left side)

  gx = dH/dx  — positive means uphill going forward
  gy = dH/dy  — positive means uphill going left

Subscribed topics:
  /terrain/elevation_map    sensor_msgs/Image  32FC1  (metres; NaN = unknown)

Published topics:
  /terrain/gradient_x          sensor_msgs/Image  32FC1
  /terrain/gradient_y          sensor_msgs/Image  32FC1
  /terrain/gradient_magnitude  sensor_msgs/Image  32FC1  (unitless m/m)
  /terrain/gradient_direction  sensor_msgs/Image  32FC1  (radians; 0 = +x/forward)
  /terrain/slope_deg           sensor_msgs/Image  32FC1  (degrees)

Parameters:
  resolution_m   float  cell size [m]          default 0.15
  bench_log_hz   float  timing log interval [Hz]  default 1.0
"""

import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


# ---------------------------------------------------------------------------
# Pure computation — no ROS dependency
# ---------------------------------------------------------------------------

def compute_gradient_field(
    elevation: np.ndarray,
    resolution: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Compute per-cell gradient vectors from a float32 elevation map.

    Uses numpy central-difference (2nd-order accurate) via np.gradient.
    NaN cells propagate into adjacent gradient cells — caller should mask or
    inpaint unknowns before calling if NaN coverage is high.

    Args:
        elevation:  (H, W) float32 array, heights in metres.
        resolution: isotropic cell size in metres.

    Returns:
        gx        (H, W): dH/dx — positive = uphill going forward (+x)
        gy        (H, W): dH/dy — positive = uphill going left (+y)
        magnitude (H, W): sqrt(gx²+gy²) — steepest-descent slope magnitude
        direction (H, W): atan2(gy, gx) [rad] pointing uphill; 0 = +x
        slope_deg (H, W): arctan(magnitude) in degrees — true slope angle
    """
    grad_row, grad_col = np.gradient(elevation, resolution)
    gx = grad_col          # d(elev)/d(col_coord) == dH/dx
    gy = -grad_row         # row↑ ≡ y↓  →  dH/dy = -dH/d(row_coord)
    magnitude = np.hypot(gx, gy)
    direction = np.arctan2(gy, gx)
    slope_deg = np.degrees(np.arctan(magnitude))
    return gx, gy, magnitude, direction, slope_deg


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class GradientMapNode(Node):
    """ROS2 wrapper: subscribes to elevation map, publishes gradient field."""

    def __init__(self):
        super().__init__('gradient_map_node')

        self.declare_parameter('resolution_m', 0.15)
        self.declare_parameter('bench_log_hz', 1.0)

        self._res: float = self.get_parameter('resolution_m').value
        bench_period = 1.0 / max(0.1, self.get_parameter('bench_log_hz').value)

        self._bridge = CvBridge()
        self._timings: list[tuple] = []

        qos = qos_profile_sensor_data

        self._sub = self.create_subscription(
            Image, '/terrain/elevation_map', self._on_elevation, qos)

        self._pubs = {
            'gx':        self.create_publisher(Image, '/terrain/gradient_x', 10),
            'gy':        self.create_publisher(Image, '/terrain/gradient_y', 10),
            'magnitude': self.create_publisher(Image, '/terrain/gradient_magnitude', 10),
            'direction': self.create_publisher(Image, '/terrain/gradient_direction', 10),
            'slope_deg': self.create_publisher(Image, '/terrain/slope_deg', 10),
        }

        self._bench_timer = self.create_timer(bench_period, self._log_timing)
        self.get_logger().info(
            f'GradientMapNode ready — resolution={self._res} m, '
            f'listening on /terrain/elevation_map'
        )

    # ------------------------------------------------------------------

    def _on_elevation(self, msg: Image) -> None:
        t0 = time.perf_counter()

        try:
            elevation = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding='32FC1').astype(np.float32)
        except Exception as exc:
            self.get_logger().error(f'CvBridge decode failed: {exc}')
            return

        t1 = time.perf_counter()

        gx, gy, mag, direction, slope_deg = compute_gradient_field(
            elevation, self._res)

        t2 = time.perf_counter()

        arrays = {
            'gx': gx,
            'gy': gy,
            'magnitude': mag,
            'direction': direction,
            'slope_deg': slope_deg,
        }
        for key, arr in arrays.items():
            out = self._bridge.cv2_to_imgmsg(
                arr.astype(np.float32), encoding='32FC1')
            out.header = msg.header
            self._pubs[key].publish(out)

        t3 = time.perf_counter()

        self._timings.append((
            (t1 - t0) * 1e3,   # decode
            (t2 - t1) * 1e3,   # compute
            (t3 - t2) * 1e3,   # encode+publish
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
    node = GradientMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
