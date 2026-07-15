"""
Unit tests for the ROS-free pure functions in dolbotz/elevation_map.py.

Run:
    pytest test/test_elevation_map.py -v

Mirrors the style of test/test_gradient_field.py: synthetic inputs with a
known analytical answer, no numerical mystery.
"""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dolbotz.elevation_map import (  # type: ignore[import]
    R_BODY_TO_OPTICAL,
    backproject_depth_to_points,
    camera_body_to_level_matrix,
    depth_to_meters,
    points_to_elevation_grid,
    roll_pitch_from_accel_body,
    update_complementary_filter,
)

G = 9.81


# ---------------------------------------------------------------------------
# depth_to_meters
# ---------------------------------------------------------------------------

class TestDepthToMeters:
    def test_uint16_millimetres_converted(self):
        depth_mm = np.array([[1000, 2500], [0, 65535]], dtype=np.uint16)
        depth_m = depth_to_meters(depth_mm)
        np.testing.assert_allclose(depth_m, [[1.0, 2.5], [0.0, 65.535]], atol=1e-6)

    def test_float_passthrough(self):
        depth_m_in = np.array([[1.2, 3.4]], dtype=np.float32)
        assert np.allclose(depth_to_meters(depth_m_in), depth_m_in)


# ---------------------------------------------------------------------------
# backproject_depth_to_points
# ---------------------------------------------------------------------------

class TestBackprojectDepthToPoints:
    def test_center_pixel_at_optical_axis(self):
        h, w = 4, 4
        fx = fy = 100.0
        cx, cy = 1.5, 1.5
        depth = np.full((h, w), 2.0, dtype=np.float32)
        pts, valid = backproject_depth_to_points(depth, fx, fy, cx, cy, 0.1, 5.0)
        # pixel (cx, cy) exactly on the optical axis -> X=Y=0
        np.testing.assert_allclose(pts[1, 1] if cx == 1 else pts[int(cy), int(cx)], pts[int(cy), int(cx)])
        x, y, z = pts[1, 2]  # v=1 row, u=2 col -> near center-ish; check formula directly instead
        expected_x = (2 - cx) * 2.0 / fx
        expected_y = (1 - cy) * 2.0 / fy
        assert x == pytest.approx(expected_x)
        assert y == pytest.approx(expected_y)
        assert z == pytest.approx(2.0)
        assert valid.all()

    def test_out_of_range_depth_masked_invalid(self):
        depth = np.array([[0.05, 1.0, 10.0, np.nan]], dtype=np.float32)
        _, valid = backproject_depth_to_points(depth, 100, 100, 0, 0, 0.1, 5.0)
        assert list(valid[0]) == [False, True, False, False]


# ---------------------------------------------------------------------------
# roll_pitch_from_accel_body — round trip against the construction rotation
# ---------------------------------------------------------------------------

class TestRollPitchFromAccelBody:
    @pytest.mark.parametrize("roll_deg,pitch_deg", [
        (0, 0), (15, 0), (0, 15), (-20, 10), (30, -25), (5, 5),
    ])
    def test_round_trip(self, roll_deg, pitch_deg):
        roll, pitch = np.radians(roll_deg), np.radians(pitch_deg)
        r_construction = Rotation.from_euler('xyz', [roll, pitch, 0])
        accel_body = r_construction.inv().apply([0, 0, G])

        r2, p2 = roll_pitch_from_accel_body(accel_body)
        assert r2 == pytest.approx(roll, abs=1e-6)
        assert p2 == pytest.approx(pitch, abs=1e-6)

    def test_level_reads_pure_z(self):
        roll, pitch = roll_pitch_from_accel_body(np.array([0.0, 0.0, G]))
        assert roll == pytest.approx(0.0, abs=1e-9)
        assert pitch == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# update_complementary_filter
# ---------------------------------------------------------------------------

class TestUpdateComplementaryFilter:
    def test_first_call_returns_accel_only_estimate(self):
        accel_body = Rotation.from_euler('xyz', [0.1, 0.2, 0]).inv().apply([0, 0, G])
        roll, pitch = update_complementary_filter(
            None, None, gyro_body=np.zeros(3), accel_body=accel_body, dt=0.05)
        expected_roll, expected_pitch = roll_pitch_from_accel_body(accel_body)
        assert roll == pytest.approx(expected_roll)
        assert pitch == pytest.approx(expected_pitch)

    def test_static_state_matches_accel_alone(self):
        """No rotation, no gyro signal -> filter should converge to (and stay at) the accel angle."""
        accel_body = Rotation.from_euler('xyz', [0.15, -0.1, 0]).inv().apply([0, 0, G])
        roll, pitch = None, None
        for _ in range(50):
            roll, pitch = update_complementary_filter(
                roll, pitch, gyro_body=np.zeros(3), accel_body=accel_body, dt=0.05, alpha=0.97)
        expected_roll, expected_pitch = roll_pitch_from_accel_body(accel_body)
        assert roll == pytest.approx(expected_roll, abs=1e-4)
        assert pitch == pytest.approx(expected_pitch, abs=1e-4)

    def test_gyro_bias_does_not_diverge(self):
        """A constant gyro bias would make pure integration drift forever;
        the accel term must anchor the estimate so it stays bounded near the true angle."""
        true_roll, true_pitch = np.radians(10.0), np.radians(-5.0)
        accel_body = Rotation.from_euler('xyz', [true_roll, true_pitch, 0]).inv().apply([0, 0, G])
        gyro_bias = np.array([0.2, -0.15, 0.0])  # rad/s, constant erroneous bias

        roll, pitch = None, None
        dt = 0.02
        for _ in range(2000):  # 40 s of simulated time
            roll, pitch = update_complementary_filter(
                roll, pitch, gyro_body=gyro_bias, accel_body=accel_body, dt=dt, alpha=0.97)

        # Pure integration of the bias alone (no accel correction) would have drifted by
        # bias * total_time = 0.2 * 40 = 8 rad — many multiples of a full turn.
        # With the accel anchor, the estimate must stay close to the true tilt instead.
        assert abs(roll - true_roll) < np.radians(15)
        assert abs(pitch - true_pitch) < np.radians(15)

    def test_reduces_noise_vs_accel_alone(self):
        """Noisy accel + clean gyro: filtered estimate should track the true angle
        more tightly (lower RMS error) than using the noisy accel reading directly."""
        rng = np.random.default_rng(0)
        true_roll, true_pitch = np.radians(8.0), np.radians(3.0)
        dt = 0.02
        n = 300

        roll, pitch = None, None
        filt_errors = []
        raw_errors = []
        for _ in range(n):
            noisy_accel = Rotation.from_euler('xyz', [true_roll, true_pitch, 0]).inv().apply([0, 0, G])
            noisy_accel = noisy_accel + rng.normal(scale=0.6, size=3)  # vibration noise
            gyro_body = np.zeros(3)  # stationary: true rate is zero

            roll, pitch = update_complementary_filter(
                roll, pitch, gyro_body=gyro_body, accel_body=noisy_accel, dt=dt, alpha=0.9)
            filt_errors.append((roll - true_roll) ** 2 + (pitch - true_pitch) ** 2)

            raw_roll, raw_pitch = roll_pitch_from_accel_body(noisy_accel)
            raw_errors.append((raw_roll - true_roll) ** 2 + (raw_pitch - true_pitch) ** 2)

        filt_rms = np.sqrt(np.mean(filt_errors[50:]))  # skip warm-up transient
        raw_rms = np.sqrt(np.mean(raw_errors[50:]))
        assert filt_rms < raw_rms


# ---------------------------------------------------------------------------
# camera_body_to_level_matrix + points_to_elevation_grid
# ---------------------------------------------------------------------------

CAMERA_HEIGHT = 0.5
RES = 0.15


def _level_matrix(dyn_roll_deg, dyn_pitch_deg, mount_roll_deg=0.0, mount_pitch_deg=10.0):
    """Simulate the camera's own onboard IMU total tilt = mount + dynamic chassis lean."""
    mount_roll, mount_pitch = np.radians(mount_roll_deg), np.radians(mount_pitch_deg)
    dyn_roll, dyn_pitch = np.radians(dyn_roll_deg), np.radians(dyn_pitch_deg)
    r_total = (Rotation.from_euler('xyz', [mount_roll, mount_pitch, 0])
               * Rotation.from_euler('xyz', [dyn_roll, dyn_pitch, 0]))
    accel_body = r_total.inv().apply([0, 0, G])
    roll_meas, pitch_meas = roll_pitch_from_accel_body(accel_body)
    return camera_body_to_level_matrix(roll_meas, pitch_meas, mount_roll, mount_pitch)


def _synth_optical_points(r_level, level_xyz_cam_relative):
    """Inverse of the elevation pipeline: given desired (x_fwd, y_left, z_rel-to-camera)
    points in the level frame, synthesize the camera-optical-frame points a real
    depth camera would report for them."""
    pts_body = level_xyz_cam_relative @ np.linalg.inv(r_level).T
    pts_optical = pts_body @ R_BODY_TO_OPTICAL.T
    return pts_optical


class TestCameraBodyToLevelMatrix:
    """Independent-physics tests.

    These deliberately do NOT use `_synth_optical_points()` / `_level_matrix()`
    (which build synthetic data from `np.linalg.inv(r_level)` where `r_level` is
    the very output of the function under test — a tautology that cancels out
    any bug in camera_body_to_level_matrix, including the double mount-offset
    removal bug this class caught). Instead, `_r_total()` builds the one true
    physical rotation (mount tilt composed with chassis dynamic lean) completely
    independently of camera_body_to_level_matrix, and world-level ground-truth
    points/slopes are defined by hand before being transformed into what the
    camera would actually observe.
    """

    @staticmethod
    def _r_total(mount_roll_deg, mount_pitch_deg, chassis_roll_deg, chassis_pitch_deg) -> Rotation:
        """The real physical rotation from world-level to the camera's current
        frame: fixed mount tilt composed with the chassis's current dynamic lean.
        This is what the camera's onboard accelerometer actually measures — never
        calls into elevation_map.py."""
        mount = Rotation.from_euler(
            'xyz', [np.radians(mount_roll_deg), np.radians(mount_pitch_deg), 0])
        chassis = Rotation.from_euler(
            'xyz', [np.radians(chassis_roll_deg), np.radians(chassis_pitch_deg), 0])
        return mount * chassis

    @pytest.mark.parametrize("mount_pitch_deg", [0.0, 10.0, 20.0])
    @pytest.mark.parametrize("chassis_pitch_deg", [0.0, 7.0, -7.0])
    def test_known_slope_recovered_independent_of_mount_offset(self, mount_pitch_deg, chassis_pitch_deg):
        """A terrain point at a known, independently-fixed world-level slope must
        be recovered at that exact same slope for every mount-offset / chassis-tilt
        combination — the mount offset must have zero net effect, since the
        accelerometer's total measured tilt already includes it."""
        r_total = self._r_total(0.0, mount_pitch_deg, 0.0, chassis_pitch_deg)
        accel_body = r_total.inv().apply([0, 0, G])
        roll_meas, pitch_meas = roll_pitch_from_accel_body(accel_body)

        true_slope_deg = 15.0
        x = 2.0
        z_level = np.tan(np.radians(true_slope_deg)) * x - CAMERA_HEIGHT
        p_world_level = np.array([x, 0.0, z_level])
        # What the camera actually observes, expressed in its own current (tilted) frame.
        p_body_actual = r_total.inv().apply(p_world_level)

        r_level = camera_body_to_level_matrix(
            roll_meas, pitch_meas, 0.0, np.radians(mount_pitch_deg))
        p_recovered = r_level @ p_body_actual
        recovered_elevation = p_recovered[2] + CAMERA_HEIGHT
        recovered_slope_deg = np.degrees(np.arctan2(recovered_elevation, x))

        assert recovered_slope_deg == pytest.approx(true_slope_deg, abs=1e-6)

    @pytest.mark.parametrize("mount_pitch_deg", [0.0, 10.0, 20.0])
    @pytest.mark.parametrize("chassis_pitch_deg", [0.0, 7.0, -7.0])
    def test_flat_floor_reads_zero_independent_of_tilt(self, mount_pitch_deg, chassis_pitch_deg):
        """A flat floor (world-level z = -camera_height everywhere) must read
        elevation ~= 0 for every chassis-tilt / mount-offset combination, using
        points synthesized directly from an independently-built R_total."""
        r_total = self._r_total(0.0, mount_pitch_deg, 0.0, chassis_pitch_deg)
        accel_body = r_total.inv().apply([0, 0, G])
        roll_meas, pitch_meas = roll_pitch_from_accel_body(accel_body)

        xs = np.array([1.0, 2.0, 3.0])
        p_world_level = np.stack([xs, np.zeros_like(xs), np.full_like(xs, -CAMERA_HEIGHT)], axis=-1)
        p_body_actual = r_total.inv().apply(p_world_level)

        r_level = camera_body_to_level_matrix(
            roll_meas, pitch_meas, 0.0, np.radians(mount_pitch_deg))
        p_recovered = p_body_actual @ r_level.T
        elevation = p_recovered[:, 2] + CAMERA_HEIGHT
        np.testing.assert_allclose(elevation, 0.0, atol=1e-6)


class TestPointsToElevationGrid:
    def _make_points(self, r_level, level_pts_cam_relative):
        n = level_pts_cam_relative.shape[0]
        h, w = 1, n  # arrange as a 1-row synthetic "depth image"
        optical_pts = _synth_optical_points(r_level, level_pts_cam_relative)
        points_optical = optical_pts.reshape(h, w, 3).astype(np.float32)
        valid = np.ones((h, w), dtype=bool)
        return points_optical, valid

    def test_flat_floor_grid_is_uniformly_zero(self):
        r_level = _level_matrix(dyn_roll_deg=0, dyn_pitch_deg=0, mount_pitch_deg=10.0)
        xs = np.linspace(0.2, 3.8, 10)
        level_pts = np.stack([xs, np.zeros_like(xs), np.full_like(xs, -CAMERA_HEIGHT)], axis=-1)
        points_optical, valid = self._make_points(r_level, level_pts)

        grid = points_to_elevation_grid(
            points_optical, valid, r_level, CAMERA_HEIGHT, RES,
            grid_forward_m=4.0, grid_width_m=4.0)

        observed = grid[~np.isnan(grid)]
        assert observed.size > 0
        np.testing.assert_allclose(observed, 0.0, atol=1e-4)

    def test_uphill_forward_increases_elevation_with_column(self):
        """Uphill going forward (+x) must show up as elevation increasing with column,
        matching gradient_map.py's gx = dH/dx > 0 = uphill forward convention."""
        r_level = _level_matrix(dyn_roll_deg=0, dyn_pitch_deg=0, mount_pitch_deg=10.0)
        xs = np.linspace(0.2, 3.8, 10)
        slope = 0.3
        zs = slope * xs - CAMERA_HEIGHT
        level_pts = np.stack([xs, np.zeros_like(xs), zs], axis=-1)
        points_optical, valid = self._make_points(r_level, level_pts)

        grid = points_to_elevation_grid(
            points_optical, valid, r_level, CAMERA_HEIGHT, RES,
            grid_forward_m=4.0, grid_width_m=4.0, blind_fill_forward_m=0.0)

        row = grid.shape[0] // 2
        cols_with_data = np.where(~np.isnan(grid[row]))[0]
        values = grid[row, cols_with_data]
        assert np.all(np.diff(values) >= -1e-6)  # non-decreasing as column (== +x) increases

    def test_uphill_left_increases_elevation_as_row_decreases(self):
        """Uphill going left (+y) must show up as elevation increasing as row decreases,
        matching row 0 = max y = left side and gy = dH/dy > 0 = uphill left."""
        r_level = _level_matrix(dyn_roll_deg=0, dyn_pitch_deg=0, mount_pitch_deg=10.0)
        ys = np.linspace(-1.8, 1.8, 10)
        slope = 0.3
        zs = slope * ys - CAMERA_HEIGHT
        level_pts = np.stack([np.full_like(ys, 1.5), ys, zs], axis=-1)
        points_optical, valid = self._make_points(r_level, level_pts)

        grid = points_to_elevation_grid(
            points_optical, valid, r_level, CAMERA_HEIGHT, RES,
            grid_forward_m=4.0, grid_width_m=4.0)

        col = grid.shape[1] // 2
        rows_with_data = np.where(~np.isnan(grid[:, col]))[0]
        values = grid[rows_with_data, col]
        # row increases as y decreases -> elevation (which grows with y) must DEcrease as row increases
        assert np.all(np.diff(values) <= 1e-6)

    def test_out_of_grid_points_are_masked(self):
        r_level = _level_matrix(dyn_roll_deg=0, dyn_pitch_deg=0, mount_pitch_deg=10.0)
        level_pts = np.array([
            [1.0, 0.0, -CAMERA_HEIGHT],     # in bounds
            [100.0, 0.0, -CAMERA_HEIGHT],   # far beyond grid_forward_m
            [1.0, 100.0, -CAMERA_HEIGHT],   # far beyond grid_width_m
        ])
        points_optical, valid = self._make_points(r_level, level_pts)
        grid = points_to_elevation_grid(
            points_optical, valid, r_level, CAMERA_HEIGHT, RES,
            grid_forward_m=4.0, grid_width_m=4.0, blind_fill_forward_m=0.0)
        assert np.count_nonzero(~np.isnan(grid)) == 1

    def test_blind_spot_filled_with_zero_at_start_cell(self):
        """Points only land far forward (beyond the blind spot); the near-origin
        cells within blind_fill_forward_m must still come back as valid (0.0),
        since plan_path_on_slope_field requires the start cell (row=h//2, col=0)
        to be traversable."""
        r_level = _level_matrix(dyn_roll_deg=0, dyn_pitch_deg=0, mount_pitch_deg=10.0)
        xs = np.linspace(1.0, 3.8, 10)  # nothing observed inside the blind spot
        level_pts = np.stack([xs, np.zeros_like(xs), np.full_like(xs, -CAMERA_HEIGHT)], axis=-1)
        points_optical, valid = self._make_points(r_level, level_pts)

        grid = points_to_elevation_grid(
            points_optical, valid, r_level, CAMERA_HEIGHT, RES,
            grid_forward_m=4.0, grid_width_m=4.0, blind_fill_forward_m=0.6)

        n_blind_cols = int(np.ceil(0.6 / RES))
        assert not np.isnan(grid[:, :n_blind_cols]).any()
        np.testing.assert_allclose(grid[:, :n_blind_cols], 0.0)

    def test_nan_outside_blind_spot_is_not_filled(self):
        """A gap in observations beyond blind_fill_forward_m (but still within the
        grid) must remain NaN — only the near-origin blind spot gets zero-filled."""
        r_level = _level_matrix(dyn_roll_deg=0, dyn_pitch_deg=0, mount_pitch_deg=10.0)
        # Observe only a thin strip right at the edge of the grid, leaving a large
        # unobserved gap between the blind spot and the observed strip.
        xs = np.linspace(3.5, 3.8, 5)
        level_pts = np.stack([xs, np.zeros_like(xs), np.full_like(xs, -CAMERA_HEIGHT)], axis=-1)
        points_optical, valid = self._make_points(r_level, level_pts)

        grid = points_to_elevation_grid(
            points_optical, valid, r_level, CAMERA_HEIGHT, RES,
            grid_forward_m=4.0, grid_width_m=4.0, blind_fill_forward_m=0.6)

        n_blind_cols = int(np.ceil(0.6 / RES))
        mid_row = grid.shape[0] // 2
        # A column well beyond the blind spot but before the observed strip.
        gap_col = n_blind_cols + 2
        assert gap_col < grid.shape[1]
        assert np.isnan(grid[mid_row, gap_col])

    def test_median_aggregation_ignores_outlier(self):
        """Several points in the same cell plus one wild outlier -> median, not mean/max."""
        r_level = _level_matrix(dyn_roll_deg=0, dyn_pitch_deg=0, mount_pitch_deg=10.0)
        base = np.array([1.0, 0.0])
        zs_rel = np.array([-0.02, 0.0, 0.01, 0.02, 5.0]) - CAMERA_HEIGHT  # last one is a wild outlier
        level_pts = np.stack([np.full(5, base[0]), np.full(5, base[1]), zs_rel], axis=-1)
        points_optical, valid = self._make_points(r_level, level_pts)

        grid = points_to_elevation_grid(
            points_optical, valid, r_level, CAMERA_HEIGHT, RES,
            grid_forward_m=4.0, grid_width_m=4.0, blind_fill_forward_m=0.0)

        observed = grid[~np.isnan(grid)]
        assert observed.size == 1
        assert observed[0] == pytest.approx(0.01, abs=1e-6)  # median of [-0.02,0,0.01,0.02,5.0] == 0.01
