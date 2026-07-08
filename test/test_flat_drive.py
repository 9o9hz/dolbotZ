"""
Unit tests for the ROS-free pure functions in dolbotz/slope_drive.py.

Run (requires ROS env sourced, since slope_drive.py imports rclpy at module level
— same requirement as test/test_gradient_field.py, see howtorun.md):
    source /opt/ros/humble/setup.bash
    python3 -m pytest test/test_flat_drive.py -v

Independent-physics style, mirroring TestCameraBodyToLevelMatrix in
test_elevation_map.py: synthetic ground truth is built from a from-scratch
R_total rotation (mount composed with chassis dynamic tilt), never by feeding
the function-under-test's own output back into itself. This is deliberate —
that kind of circularity is exactly what let the mount-offset-double-removal
bug in elevation_map.py's camera_body_to_level_matrix() slip through undetected.
"""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dolbotz.slope_drive import (
    bev_ground_projection_matrix,
    bev_pixel_to_meters,
    ground_to_image_homography,
    image_to_bev_homography,
)
from dolbotz.utils.attitude import R_BODY_TO_OPTICAL, roll_pitch_from_accel_body

G = 9.81
CAMERA_HEIGHT = 0.5
FX = FY = 500.0
CX, CY = 320.0, 240.0
CAMERA_MATRIX = np.array([
    [FX, 0.0, CX],
    [0.0, FY, CY],
    [0.0, 0.0, 1.0],
])


def _r_total(mount_roll_deg, mount_pitch_deg, chassis_roll_deg, chassis_pitch_deg) -> Rotation:
    """The one true physical rotation (world-level -> camera's current frame):
    fixed mount tilt composed with the chassis's current dynamic lean. Built
    completely independently of anything in slope_drive.py or elevation_map.py."""
    mount = Rotation.from_euler(
        'xyz', [np.radians(mount_roll_deg), np.radians(mount_pitch_deg), 0])
    chassis = Rotation.from_euler(
        'xyz', [np.radians(chassis_roll_deg), np.radians(chassis_pitch_deg), 0])
    return mount * chassis


def _independent_raw_pixel(r_total: Rotation, x_forward: float, y_left: float) -> tuple[float, float]:
    """Ground truth: project a known world-level ground point straight through
    pinhole optics, using only R_total (independently built) + the already-
    validated R_BODY_TO_OPTICAL constant + the definitional camera_matrix
    projection. Never calls ground_to_image_homography()."""
    p_level = np.array([x_forward, y_left, -CAMERA_HEIGHT])
    p_body_current = r_total.inv().apply(p_level)
    p_optical = R_BODY_TO_OPTICAL @ p_body_current
    pixel_h = CAMERA_MATRIX @ p_optical
    return float(pixel_h[0] / pixel_h[2]), float(pixel_h[1] / pixel_h[2])


MOUNT_PITCH_DEGS = [0.0, 10.0, 20.0]
CHASSIS_PITCH_DEGS = [0.0, 7.0, -7.0]
GROUND_POINTS = [(1.0, 0.0), (2.0, 0.8), (2.5, -1.2), (0.8, 0.3)]


class TestGroundToImageHomography:
    @pytest.mark.parametrize("mount_pitch_deg", MOUNT_PITCH_DEGS)
    @pytest.mark.parametrize("chassis_pitch_deg", CHASSIS_PITCH_DEGS)
    @pytest.mark.parametrize("x_forward,y_left", GROUND_POINTS)
    def test_known_ground_point_projects_to_correct_raw_pixel(
        self, mount_pitch_deg, chassis_pitch_deg, x_forward, y_left,
    ):
        r_total = _r_total(0.0, mount_pitch_deg, 0.0, chassis_pitch_deg)
        accel_body = r_total.inv().apply([0, 0, G])
        roll_meas, pitch_meas = roll_pitch_from_accel_body(accel_body)

        u_expected, v_expected = _independent_raw_pixel(r_total, x_forward, y_left)

        h_g2i = ground_to_image_homography(
            CAMERA_MATRIX, roll_meas, pitch_meas, 0.0, np.radians(mount_pitch_deg), CAMERA_HEIGHT)
        pixel_h = h_g2i @ np.array([x_forward, y_left, 1.0])
        u_actual, v_actual = pixel_h[0] / pixel_h[2], pixel_h[1] / pixel_h[2]

        assert u_actual == pytest.approx(u_expected, abs=1e-6)
        assert v_actual == pytest.approx(v_expected, abs=1e-6)

    def test_mount_offset_parameters_do_not_affect_result(self):
        """Passing different roll_offset/pitch_offset values must not change the
        output at all — they are structurally unused (see docstring), exactly
        mirroring camera_body_to_level_matrix()'s fixed behaviour."""
        roll_meas, pitch_meas = np.radians(3.0), np.radians(12.0)
        h1 = ground_to_image_homography(CAMERA_MATRIX, roll_meas, pitch_meas, 0.0, 0.0, CAMERA_HEIGHT)
        h2 = ground_to_image_homography(
            CAMERA_MATRIX, roll_meas, pitch_meas, np.radians(99.0), np.radians(-42.0), CAMERA_HEIGHT)
        np.testing.assert_allclose(h1, h2)


class TestBevGroundProjectionMatrix:
    @pytest.mark.parametrize("x_forward,y_left", GROUND_POINTS)
    def test_meters_to_pixel_and_back_round_trips(self, x_forward, y_left):
        m = bev_ground_projection_matrix(bev_width_px=400, bev_height_px=400, bev_meters_per_pixel=0.02)
        pixel_h = m @ np.array([x_forward, y_left, 1.0])
        col, row = pixel_h[0] / pixel_h[2], pixel_h[1] / pixel_h[2]

        x_rec, y_rec = bev_pixel_to_meters(row, col, 400, 400, 0.02)
        assert x_rec == pytest.approx(x_forward, abs=1e-9)
        assert y_rec == pytest.approx(y_left, abs=1e-9)

    def test_forward_distance_increases_toward_top_of_image(self):
        """Larger x_forward (farther ahead) must land at a smaller row (higher up
        the BEV image) — the conventional 'road recedes upward' BEV layout."""
        m = bev_ground_projection_matrix(bev_width_px=400, bev_height_px=400, bev_meters_per_pixel=0.02)
        near = m @ np.array([0.5, 0.0, 1.0])
        far = m @ np.array([3.0, 0.0, 1.0])
        assert far[1] / far[2] < near[1] / near[2]

    def test_leftward_offset_moves_toward_smaller_column(self):
        """Positive y_left (robot's left) must land at a smaller column (left
        side of the BEV image), matching an intuitive, non-mirrored top-down view."""
        m = bev_ground_projection_matrix(bev_width_px=400, bev_height_px=400, bev_meters_per_pixel=0.02)
        center = m @ np.array([1.0, 0.0, 1.0])
        left = m @ np.array([1.0, 0.8, 1.0])
        assert left[0] / left[2] < center[0] / center[2]


class TestImageToBevHomographyEndToEnd:
    @pytest.mark.parametrize("mount_pitch_deg", MOUNT_PITCH_DEGS)
    @pytest.mark.parametrize("chassis_pitch_deg", CHASSIS_PITCH_DEGS)
    @pytest.mark.parametrize("x_forward,y_left", GROUND_POINTS)
    def test_known_ground_point_raw_pixel_warps_to_correct_bev_pixel(
        self, mount_pitch_deg, chassis_pitch_deg, x_forward, y_left,
    ):
        """Full chain: an independently-computed raw camera pixel for a known
        ground point, run through image_to_bev_homography(), must land at the
        BEV pixel that bev_ground_projection_matrix() independently predicts for
        that same ground point. This is the check that actually matters for the
        real pipeline (cv2.warpPerspective consumes exactly this homography)."""
        bev_w, bev_h, mpp = 400, 400, 0.02

        r_total = _r_total(0.0, mount_pitch_deg, 0.0, chassis_pitch_deg)
        accel_body = r_total.inv().apply([0, 0, G])
        roll_meas, pitch_meas = roll_pitch_from_accel_body(accel_body)

        u_raw, v_raw = _independent_raw_pixel(r_total, x_forward, y_left)

        h_full = image_to_bev_homography(
            CAMERA_MATRIX, roll_meas, pitch_meas, 0.0, np.radians(mount_pitch_deg),
            CAMERA_HEIGHT, bev_w, bev_h, mpp)
        bev_h_pixel = h_full @ np.array([u_raw, v_raw, 1.0])
        col_actual, row_actual = bev_h_pixel[0] / bev_h_pixel[2], bev_h_pixel[1] / bev_h_pixel[2]

        m = bev_ground_projection_matrix(bev_w, bev_h, mpp)
        expected_h = m @ np.array([x_forward, y_left, 1.0])
        col_expected, row_expected = expected_h[0] / expected_h[2], expected_h[1] / expected_h[2]

        assert col_actual == pytest.approx(col_expected, abs=1e-4)
        assert row_actual == pytest.approx(row_expected, abs=1e-4)
