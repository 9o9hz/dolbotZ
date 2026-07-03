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
  /terrain/planned_path        nav_msgs/Path            (cells with slope <= max_slope_deg)

Parameters:
  resolution_m   float  cell size [m]            default 0.15
  bench_log_hz   float  timing log interval [Hz]  default 1.0
  max_slope_deg  float  max traversable slope [deg]  default 30.0  (placeholder — see MAX_SLOPE_DEG_PLACEHOLDER)
"""

import heapq
import math
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
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


# 최대 허용 경사각 [deg] — 로봇이 등반 가능한 실제 한계는 아직 실측하지 않았으므로
# 30도를 임시 상수로 사용한다. 추후 하드웨어 트랙션 테스트 후
# max_slope_deg ROS 파라미터로 노출해 런타임에 조정할 수 있게 한다.
MAX_SLOPE_DEG_PLACEHOLDER = 30.0

# 8-연결 그리드 이동: (행 변화량, 열 변화량, 이동 비용)
_SQRT2 = math.sqrt(2.0)
_NEIGHBOR_STEPS = [
    (-1, -1, _SQRT2), (-1, 0, 1.0), (-1, 1, _SQRT2),
    (0, -1, 1.0),                   (0, 1, 1.0),
    (1, -1, _SQRT2),  (1, 0, 1.0),  (1, 1, _SQRT2),
]


def plan_path_on_slope_field(
    slope_deg: np.ndarray,
    start: tuple[int, int],
    target_col: int,
    max_slope_deg: float = MAX_SLOPE_DEG_PLACEHOLDER,
) -> list[tuple[int, int]] | None:
    """Plan the cheapest slope-limited path from `start` to column `target_col`.

    Cells with slope_deg > max_slope_deg (or NaN) are treated as impassable.
    Dijkstra search over an 8-connected grid finds the lowest-cost route to
    the nearest traversable cell in `target_col`, routing sideways around
    steep patches when the direct path forward is blocked.

    Args:
        slope_deg:  (H, W) slope angle in degrees, as returned by
                    compute_gradient_field.
        start:      (row, col) starting cell — must be traversable.
        target_col: column index to reach (forward distance); clamped to grid.
        max_slope_deg: cells steeper than this are impassable.

    Returns:
        List of (row, col) cells from start to the reached goal (inclusive),
        or None if start is impassable or no traversable route exists.
    """
    h, w = slope_deg.shape
    start_r, start_c = start
    if not (0 <= start_r < h and 0 <= start_c < w):
        raise ValueError(f'start {start} outside grid bounds {(h, w)}')
    target_col = max(0, min(w - 1, target_col))

    traversable = np.isfinite(slope_deg) & (slope_deg <= max_slope_deg)
    if not traversable[start_r, start_c]:
        return None

    dist = np.full((h, w), np.inf)
    dist[start_r, start_c] = 0.0
    visited = np.zeros((h, w), dtype=bool)
    prev: dict[tuple[int, int], tuple[int, int]] = {}
    pq: list[tuple[float, tuple[int, int]]] = [(0.0, (start_r, start_c))]

    goal = None
    while pq:
        d, node = heapq.heappop(pq)
        if visited[node]:
            continue
        visited[node] = True
        if node[1] == target_col:
            goal = node
            break

        r, c = node
        for dr, dc, step_cost in _NEIGHBOR_STEPS:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < h and 0 <= nc < w):
                continue
            if visited[nr, nc] or not traversable[nr, nc]:
                continue
            nd = d + step_cost
            if nd < dist[nr, nc]:
                dist[nr, nc] = nd
                prev[(nr, nc)] = node
                heapq.heappush(pq, (nd, (nr, nc)))

    if goal is None:
        return None

    path = [goal]
    while path[-1] != (start_r, start_c):
        path.append(prev[path[-1]])
    path.reverse()
    return path


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class GradientMapNode(Node):
    """ROS2 wrapper: subscribes to elevation map, publishes gradient field."""

    def __init__(self):
        super().__init__('gradient_map_node')

        self.declare_parameter('resolution_m', 0.15)
        self.declare_parameter('bench_log_hz', 1.0)
        # TODO: 임시 기본값. 하드웨어 등반 한계 실측 후 조정할 것 (MAX_SLOPE_DEG_PLACEHOLDER 참고).
        self.declare_parameter('max_slope_deg', MAX_SLOPE_DEG_PLACEHOLDER)

        self._res: float = self.get_parameter('resolution_m').value
        self._max_slope_deg: float = self.get_parameter('max_slope_deg').value
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
        self._path_pub = self.create_publisher(Path, '/terrain/planned_path', 10)

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

        h, w = slope_deg.shape
        start = (h // 2, 0)   # 로봇 현재 위치: 좌우 중앙, 그리드 최근접 열
        target_col = w - 1    # 최대한 전방(+x)으로 이동
        path_cells = plan_path_on_slope_field(
            slope_deg, start, target_col, self._max_slope_deg)

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

        if path_cells is not None:
            self._path_pub.publish(
                self._cells_to_path_msg(path_cells, h, msg.header))
        else:
            self.get_logger().warn(
                f'No path within max_slope_deg={self._max_slope_deg}° found.',
                throttle_duration_sec=2.0)

        t3 = time.perf_counter()

        self._timings.append((
            (t1 - t0) * 1e3,   # decode
            (t2 - t1) * 1e3,   # compute
            (t3 - t2) * 1e3,   # encode+publish
            (t3 - t0) * 1e3,   # total
        ))

    def _cells_to_path_msg(self, cells, grid_h: int, header) -> Path:
        """Convert (row, col) grid cells to a metric nav_msgs/Path.

        x = col * resolution (forward distance from robot)
        y = (center_row - row) * resolution (left of centerline is +y,
            matching the gy = dH/dy convention documented at module top)
        """
        center_row = grid_h // 2
        path_msg = Path()
        path_msg.header = header
        for row, col in cells:
            pose = PoseStamped()
            pose.header = header
            pose.pose.position.x = float(col * self._res)
            pose.pose.position.y = float((center_row - row) * self._res)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        return path_msg

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
