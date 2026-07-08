"""
Unit tests for the ROS-free pure functions in dolbotz/flat_drive.py.

Run (requires ROS env sourced, since flat_drive.py imports rclpy at module level
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

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dolbotz.flat_drive import (
    bev_ground_projection_matrix,
    bev_mask_to_centerline_path,
    bev_pixel_to_meters,
    ground_to_image_homography,
    image_to_bev_homography,
    mask_to_bev,
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
    completely independently of anything in flat_drive.py or elevation_map.py."""
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


class TestMaskToBev:
    def test_identity_homography_is_a_passthrough(self):
        mask = np.zeros((50, 50), dtype=np.uint8)
        mask[10:20, 15:25] = 255
        bev = mask_to_bev(mask, np.eye(3), bev_width_px=50, bev_height_px=50)
        np.testing.assert_array_equal(bev, mask)

    def test_uses_nearest_neighbor_no_gray_values(self):
        """A binary mask must warp to another binary mask (0/255 only) — no
        antialiased/gray edge pixels from linear interpolation."""
        mask = np.zeros((60, 60), dtype=np.uint8)
        mask[20:40, 20:40] = 255
        # A homography that isn't axis-aligned forces resampling at non-integer
        # source coordinates, which is where interpolation artifacts would show.
        rot = np.array([
            [np.cos(0.3), -np.sin(0.3), 5.0],
            [np.sin(0.3), np.cos(0.3), 3.0],
            [0.0, 0.0, 1.0],
        ])
        bev = mask_to_bev(mask, rot, bev_width_px=60, bev_height_px=60)
        assert set(np.unique(bev).tolist()) <= {0, 255}


class TestBevMaskToCenterlinePath:
    BEV_W, BEV_H, MPP = 100, 100, 0.05

    def _corridor_mask(self, col_center_fn, half_width_px=10):
        """Build a synthetic BEV mask of a corridor whose column center varies
        per row according to col_center_fn(row)."""
        mask = np.zeros((self.BEV_H, self.BEV_W), dtype=np.uint8)
        for row in range(self.BEV_H):
            c = col_center_fn(row)
            lo, hi = int(round(c - half_width_px)), int(round(c + half_width_px))
            mask[row, max(0, lo):min(self.BEV_W, hi + 1)] = 255
        return mask

    def test_straight_centered_corridor_reads_zero_lateral_offset(self):
        mask = self._corridor_mask(lambda row: self.BEV_W / 2.0)
        path = bev_mask_to_centerline_path(mask, min_row_pixels=5,
                                            bev_width_px=self.BEV_W, bev_height_px=self.BEV_H,
                                            bev_meters_per_pixel=self.MPP)
        assert len(path) == self.BEV_H
        y_lefts = np.array([y for _, y in path])
        np.testing.assert_allclose(y_lefts, 0.0, atol=self.MPP)  # within half a pixel

    def test_straight_corridor_forward_distance_increases_from_robot_outward(self):
        mask = self._corridor_mask(lambda row: self.BEV_W / 2.0)
        path = bev_mask_to_centerline_path(mask, min_row_pixels=5,
                                            bev_width_px=self.BEV_W, bev_height_px=self.BEV_H,
                                            bev_meters_per_pixel=self.MPP)
        x_forwards = np.array([x for x, _ in path])
        assert np.all(np.diff(x_forwards) > 0)  # near (robot) -> far, strictly increasing
        assert x_forwards[0] == pytest.approx(self.MPP, abs=1e-9)  # row=H-1, nearest to robot
        assert x_forwards[-1] == pytest.approx(self.BEV_H * self.MPP, abs=1e-9)  # row=0, farthest

    def test_curved_corridor_lateral_offset_tracks_known_curve(self):
        """Corridor that drifts linearly leftward as it recedes (row decreases)
        — the extracted centerline's y_left must track the known drift."""
        slope_px_per_row = 0.3

        def col_center(row):
            return self.BEV_W / 2.0 + slope_px_per_row * (self.BEV_H - 1 - row)

        mask = self._corridor_mask(col_center)
        path = bev_mask_to_centerline_path(mask, min_row_pixels=5,
                                            bev_width_px=self.BEV_W, bev_height_px=self.BEV_H,
                                            bev_meters_per_pixel=self.MPP)
        for x_forward, y_left in path:
            row = self.BEV_H - x_forward / self.MPP
            expected_col = col_center(row)
            expected_y_left = (self.BEV_W / 2.0 - expected_col) * self.MPP
            assert y_left == pytest.approx(expected_y_left, abs=self.MPP)

    def test_rows_below_min_pixel_threshold_are_skipped_as_holes(self):
        mask = self._corridor_mask(lambda row: self.BEV_W / 2.0)  # full-width rows elsewhere
        gap_rows = [40, 41, 42]
        for row in gap_rows:
            mask[row] = 0
            mask[row, 48:50] = 255  # only 2 lit pixels — below min_row_pixels=5

        path = bev_mask_to_centerline_path(mask, min_row_pixels=5,
                                            bev_width_px=self.BEV_W, bev_height_px=self.BEV_H,
                                            bev_meters_per_pixel=self.MPP)
        assert len(path) == self.BEV_H - len(gap_rows)

        gap_x_forwards = {(self.BEV_H - r) * self.MPP for r in gap_rows}
        present_x_forwards = {x for x, _ in path}
        assert gap_x_forwards.isdisjoint(present_x_forwards)

    def test_empty_mask_returns_empty_path(self):
        mask = np.zeros((self.BEV_H, self.BEV_W), dtype=np.uint8)
        path = bev_mask_to_centerline_path(mask, min_row_pixels=5,
                                            bev_width_px=self.BEV_W, bev_height_px=self.BEV_H,
                                            bev_meters_per_pixel=self.MPP)
        assert path == []


class TestSegmentationToPathEndToEnd:
    """Ties the re-derived homography together with mask_to_bev +
    bev_mask_to_centerline_path: a known straight corridor in world-level
    ground coordinates is projected into a synthetic *raw camera image* mask
    (independent of the homography under test — via the same from-scratch
    R_total + pinhole projection helper used in TestGroundToImageHomography),
    then run through the real mask_to_bev -> bev_mask_to_centerline_path
    pipeline, and the recovered centerline must match the known corridor
    centerline."""

    def test_known_world_corridor_recovered_through_full_pipeline(self):
        bev_w, bev_h, mpp = 200, 200, 0.02
        mount_pitch_deg = 10.0
        chassis_pitch_deg = 3.0
        y_center = 0.3  # corridor centered 0.3 m left of the robot's forward axis
        half_width = 0.4  # 0.8 m wide corridor

        r_total = _r_total(0.0, mount_pitch_deg, 0.0, chassis_pitch_deg)
        accel_body = r_total.inv().apply([0, 0, G])
        roll_meas, pitch_meas = roll_pitch_from_accel_body(accel_body)

        img_w, img_h = 640, 480
        raw_mask = np.zeros((img_h, img_w), dtype=np.uint8)
        corners_world = []
        for x_forward in np.linspace(0.8, 3.5, 60):
            for y_left in (y_center - half_width, y_center + half_width):
                corners_world.append((x_forward, y_left))
        pixels = np.array([
            _independent_raw_pixel(r_total, xf, yl) for xf, yl in corners_world
        ], dtype=np.float32)
        hull = cv2.convexHull(pixels)
        cv2.fillConvexPoly(raw_mask, hull.astype(np.int32), 255)

        h_full = image_to_bev_homography(
            CAMERA_MATRIX, roll_meas, pitch_meas, 0.0, np.radians(mount_pitch_deg),
            CAMERA_HEIGHT, bev_w, bev_h, mpp)
        bev_mask = mask_to_bev(raw_mask, h_full, bev_w, bev_h)

        path = bev_mask_to_centerline_path(bev_mask, min_row_pixels=5,
                                            bev_width_px=bev_w, bev_height_px=bev_h,
                                            bev_meters_per_pixel=mpp)

        recovered = [(x, y) for x, y in path if 1.2 <= x <= 3.0]
        assert len(recovered) > 20
        y_lefts = np.array([y for _, y in recovered])
        np.testing.assert_allclose(y_lefts, y_center, atol=0.05)
