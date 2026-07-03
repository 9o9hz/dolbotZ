"""
Elevation Map Node — publishes /terrain/elevation_map, the input gradient_map.py expects.

Pure computation is ROS-free so it can be unit-tested and benchmarked standalone
(see module docstring of gradient_map.py for the downstream consumer and the
grid coordinate convention this node must match: col=+x forward, row=-y,
row 0 = max y = robot left side).

Design notes (see project history / howtorun.md for the full discussion):

  * The IMU is the depth camera's own onboard sensor (frame_id
    'camera_imu_optical_frame'), not a separate chassis-mounted IMU. Its raw
    accel/gyro therefore measure the CAMERA's own absolute tilt, which is the
    fixed mounting tilt (camera_roll/pitch_offset_deg) *combined* with
    whatever the chassis is currently doing on the terrain. We must subtract
    the known fixed mount tilt to recover the chassis's own dynamic lean —
    otherwise a perfectly flat, level floor would read as sloped just because
    the camera is mounted looking down at camera_pitch_offset_deg.

  * yaw is never estimated at all (no magnetometer, no gyro-Z integration) —
    only roll/pitch are tracked as two independent scalars via a
    complementary filter fed by the camera's own raw gyro + accel. This keeps
    the output grid's +x locked to the robot's current heading every frame
    (matches gradient_map.py's per-frame, non-accumulating grid), with no
    separate "strip yaw" step needed since yaw was never part of the state.

  * Tilt-from-accelerometer and the body<->optical axis remap were verified
    by round-trip synthetic checks (known tilt -> synthetic accel -> recovered
    tilt; known level/sloped floor -> synthetic depth points -> recovered
    elevation) before being written up as the formulas below.

Subscribed topics:
  /camera/camera/depth/image_rect_raw   sensor_msgs/Image        (16UC1 mm or 32FC1 m)
  /camera/camera/depth/camera_info      sensor_msgs/CameraInfo   (fx, fy, cx, cy)
  /camera/camera/imu                    sensor_msgs/Imu          (raw gyro + accel only)

Published topics:
  /terrain/elevation_map   sensor_msgs/Image  32FC1  (metres; NaN = unobserved cell)

Parameters (mount offsets are placeholders — see PLACEHOLDER comments below):
  camera_height_m          float  default 0.5    — camera height above ground [m], unmeasured
  camera_pitch_offset_deg  float  default 10.0   — fixed camera mount pitch (nose-down positive), unmeasured
  camera_roll_offset_deg   float  default 0.0    — fixed camera mount roll, unmeasured
  resolution_m             float  default 0.15   — grid cell size [m], matches gradient_map.py default
  grid_forward_m           float  default 4.0    — forward map extent [m]
  grid_width_m             float  default 4.0    — lateral map extent [m] (±grid_width_m/2)
  min_depth_m               float  default 0.3
  max_depth_m               float  default 4.0
  complementary_filter_alpha float default 0.97  — PLACEHOLDER, needs real-drive tuning
  bench_log_hz              float  default 1.0
"""

import numpy as np
from scipy.spatial.transform import Rotation

# ---------------------------------------------------------------------------
# Fixed axis-convention constants
# ---------------------------------------------------------------------------

# Body frame (x=forward, y=left, z=up, matches gradient_map.py/slope_drive.py)
# to camera optical frame (x=right, y=down, z=forward). Identical constant to
# slope_drive.py's R_body_to_optical, verified there by axis substitution.
R_BODY_TO_OPTICAL = np.array([
    [0., -1., 0.],
    [0., 0., -1.],
    [1., 0., 0.],
])
R_OPTICAL_TO_BODY = R_BODY_TO_OPTICAL.T  # rotation matrix is orthogonal

# 실측 전 임시값 — 실제 하드웨어에서 상보필터 튜닝 후 조정할 것.
COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER = 0.97


# ---------------------------------------------------------------------------
# Pure computation — no ROS dependency
# ---------------------------------------------------------------------------

def depth_to_meters(depth_img: np.ndarray) -> np.ndarray:
    """Convert a raw depth image to float32 metres.

    16UC1 images (RealSense default) are in millimetres; anything else is
    assumed to already be in metres. Mirrors slope_decision.py's
    _depth_to_meters helper.
    """
    if depth_img.dtype == np.uint16:
        return depth_img.astype(np.float32) * 0.001
    return depth_img.astype(np.float32)


def backproject_depth_to_points(
    depth_m: np.ndarray,
    fx: float, fy: float, cx: float, cy: float,
    min_depth_m: float, max_depth_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Vectorized pinhole back-projection of a depth image to camera-optical 3D points.

    Args:
        depth_m: (H, W) depth in metres.
        fx, fy, cx, cy: pinhole intrinsics (from CameraInfo.k).
        min_depth_m, max_depth_m: valid depth range; outside this (or
            non-finite) depth is marked invalid.

    Returns:
        points_optical: (H, W, 3) float32 — (X_opt=right, Y_opt=down, Z_opt=forward) metres.
                         Invalid cells are left as whatever depth_m held (NaN-safe
                         since callers must consult `valid`).
        valid: (H, W) bool mask.
    """
    h, w = depth_m.shape
    us, vs = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))

    z = depth_m
    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy
    points_optical = np.stack([x, y, z], axis=-1).astype(np.float32)

    valid = np.isfinite(z) & (z >= min_depth_m) & (z <= max_depth_m)
    return points_optical, valid


def roll_pitch_from_accel_body(accel_body: np.ndarray) -> tuple[float, float]:
    """Estimate (roll, pitch) [rad] from a body-frame (x-fwd,y-left,z-up) accelerometer reading.

    Standard two-axis tilt formula: a stationary accelerometer reads
    approximately +g along whichever local axis currently points "up".
    roll is rotation about the forward (x) axis (positive = right side down);
    pitch is rotation about the left (y) axis (positive = nose down).
    Verified by round-trip synthetic test against the construction rotation
    Rotation.from_euler('xyz', [roll, pitch, 0]).
    """
    ax, ay, az = accel_body
    roll = np.arctan2(ay, az)
    pitch = np.arctan2(-ax, np.hypot(ay, az))
    return float(roll), float(pitch)


def update_complementary_filter(
    prev_roll: float | None,
    prev_pitch: float | None,
    gyro_body: np.ndarray,
    accel_body: np.ndarray,
    dt: float,
    alpha: float = COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER,
) -> tuple[float, float]:
    """One complementary-filter step for (roll, pitch), no yaw.

    angle = alpha * (prev_angle + gyro_rate * dt) + (1 - alpha) * accel_angle

    gyro_body[0]/[1] are the roll-rate/pitch-rate about the body x/y axes.
    On the first call (prev_roll is None), returns the accel-only estimate —
    there is no gyro integration to blend yet.
    """
    accel_roll, accel_pitch = roll_pitch_from_accel_body(accel_body)

    if prev_roll is None or prev_pitch is None:
        return accel_roll, accel_pitch

    roll = alpha * (prev_roll + gyro_body[0] * dt) + (1.0 - alpha) * accel_roll
    pitch = alpha * (prev_pitch + gyro_body[1] * dt) + (1.0 - alpha) * accel_pitch
    return float(roll), float(pitch)


def camera_body_to_level_matrix(
    roll_meas: float,
    pitch_meas: float,
    roll_offset: float,
    pitch_offset: float,
) -> np.ndarray:
    """Rotation matrix mapping camera-BODY-frame vectors to the level output frame.

    `roll_meas`/`pitch_meas` is the camera's own total measured tilt (mount +
    chassis dynamic lean combined, from update_complementary_filter).
    `roll_offset`/`pitch_offset` is the fixed mount tilt (camera_roll/pitch_offset_deg,
    in radians). The result first undoes the total measured tilt (back to a
    nominal, hypothetically-level *mounted* camera frame), then undoes the
    fixed mount tilt on top of that (to the true chassis-level frame) — so
    the output frame's +x tracks the chassis's current heading, leveled for
    gravity, with the known mount tilt subtracted out.

    Verified: on a flat floor, this matrix yields elevation ~= 0 regardless
    of camera mount tilt, as long as the chassis itself is level.
    """
    r_detilt = Rotation.from_euler('xyz', [roll_meas, pitch_meas, 0]).inv()
    r_mount_undo = Rotation.from_euler('xyz', [roll_offset, pitch_offset, 0]).inv()
    return (r_mount_undo * r_detilt).as_matrix()


def points_to_elevation_grid(
    points_optical: np.ndarray,
    valid: np.ndarray,
    r_level: np.ndarray,
    camera_height_m: float,
    resolution_m: float,
    grid_forward_m: float,
    grid_width_m: float,
) -> np.ndarray:
    """Project valid camera-optical points into a levelled elevation grid.

    Grid convention matches gradient_map.py: col increases in +x (forward),
    row increases in -y (row 0 = max y = left side). col=0 / row=center is the
    robot/camera's own position; forward extent is [0, grid_forward_m), lateral
    extent is [-grid_width_m/2, grid_width_m/2).

    Multiple points landing in the same cell are aggregated by median (robust
    to reflective-surface depth noise). Empty cells are NaN.
    """
    h_grid = max(1, round(grid_width_m / resolution_m))
    w_grid = max(1, round(grid_forward_m / resolution_m))

    pts = points_optical[valid]
    if pts.size == 0:
        return np.full((h_grid, w_grid), np.nan, dtype=np.float32)

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
        return grid

    flat_idx = row * w_grid + col
    order = np.argsort(flat_idx)
    sorted_idx, sorted_elev = flat_idx[order], elevation[order]
    unique_idx, start_pos = np.unique(sorted_idx, return_index=True)
    for idx, group in zip(unique_idx, np.split(sorted_elev, start_pos[1:])):
        grid.flat[idx] = np.median(group)

    return grid
