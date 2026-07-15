"""
Unit + benchmark tests for compute_gradient_field().

Run:
    pytest test/test_gradient_field.py -v
or via colcon:
    colcon test --packages-select dolbotz

All synthetic elevation maps are constructed so the expected analytical
gradient is known — no numerical mystery.
"""

import time

import numpy as np
import pytest

from dolbotz.gradient_map import (  # type: ignore[import]
    compute_gradient_field,
    plan_path_on_slope_field,
)

RES = 0.15  # metres — default resolution used throughout


# ---------------------------------------------------------------------------
# Synthetic map builders
# ---------------------------------------------------------------------------

def flat_map(h: int = 40, w: int = 40, val: float = 0.0) -> np.ndarray:
    return np.full((h, w), val, dtype=np.float32)


def ramp_x(h: int = 40, w: int = 40, slope: float = 0.2, res: float = RES) -> np.ndarray:
    """Slope in +x direction: elevation[r, c] = slope * c * res  →  gx = slope, gy = 0."""
    cols = (np.arange(w, dtype=np.float32) * res * slope)
    return np.tile(cols, (h, 1))


def ramp_y(h: int = 40, w: int = 40, slope: float = 0.2, res: float = RES) -> np.ndarray:
    """Slope in +y direction.

    Convention: row↑ ≡ y↓, so elevation[r, c] = -slope * r * res  →  gy = slope.
    Derivation:
      grad_row[r] ≈ d(elev)/d(row_coord) = -slope * res / res = -slope
      gy = -grad_row = slope  ✓
    """
    rows = (-slope * np.arange(h, dtype=np.float32) * res)
    return np.tile(rows.reshape(-1, 1), (1, w))


def diagonal_ramp(slope_x: float = 0.1, slope_y: float = 0.1,
                  h: int = 40, w: int = 40, res: float = RES) -> np.ndarray:
    """Combined ramp: elevation[r, c] = slope_x*c*res - slope_y*r*res."""
    cols = slope_x * np.arange(w, dtype=np.float32) * res
    rows = -slope_y * np.arange(h, dtype=np.float32) * res
    return rows.reshape(-1, 1) + cols.reshape(1, -1)


# ---------------------------------------------------------------------------
# Correctness tests
# ---------------------------------------------------------------------------

class TestFlatMap:
    def test_gradient_is_zero(self):
        gx, gy, mag, direction, slope_deg = compute_gradient_field(flat_map(), RES)
        np.testing.assert_allclose(gx, 0.0, atol=1e-6)
        np.testing.assert_allclose(gy, 0.0, atol=1e-6)
        np.testing.assert_allclose(mag, 0.0, atol=1e-6)
        np.testing.assert_allclose(slope_deg, 0.0, atol=1e-4)

    def test_output_shape_preserved(self):
        h, w = 13, 27
        elev = flat_map(h, w)
        for arr in compute_gradient_field(elev, RES):
            assert arr.shape == (h, w), f'Shape mismatch: {arr.shape}'


class TestRampX:
    slope = 0.25

    def test_gx_equals_slope_interior(self):
        """Interior cells should have gx ≈ slope (central diff is exact for linear)."""
        elev = ramp_x(slope=self.slope)
        gx, gy, mag, direction, slope_deg = compute_gradient_field(elev, RES)
        np.testing.assert_allclose(gx[1:-1, 1:-1], self.slope, atol=1e-5)

    def test_gy_near_zero_interior(self):
        elev = ramp_x(slope=self.slope)
        _, gy, _, _, _ = compute_gradient_field(elev, RES)
        np.testing.assert_allclose(gy[1:-1, 1:-1], 0.0, atol=1e-5)

    def test_magnitude_equals_slope(self):
        elev = ramp_x(slope=self.slope)
        _, _, mag, _, _ = compute_gradient_field(elev, RES)
        np.testing.assert_allclose(mag[1:-1, 1:-1], self.slope, atol=1e-5)

    def test_direction_points_forward(self):
        """Uphill in +x → direction ≈ 0 radians."""
        elev = ramp_x(slope=self.slope)
        _, _, _, direction, _ = compute_gradient_field(elev, RES)
        np.testing.assert_allclose(direction[1:-1, 1:-1], 0.0, atol=1e-5)

    def test_slope_deg_arctan(self):
        elev = ramp_x(slope=self.slope)
        _, _, _, _, slope_deg = compute_gradient_field(elev, RES)
        expected = np.degrees(np.arctan(self.slope))
        np.testing.assert_allclose(slope_deg[1:-1, 1:-1], expected, atol=1e-3)


class TestRampY:
    slope = 0.18

    def test_gy_equals_slope_interior(self):
        elev = ramp_y(slope=self.slope)
        _, gy, _, _, _ = compute_gradient_field(elev, RES)
        np.testing.assert_allclose(gy[1:-1, 1:-1], self.slope, atol=1e-5)

    def test_gx_near_zero_interior(self):
        elev = ramp_y(slope=self.slope)
        gx, _, _, _, _ = compute_gradient_field(elev, RES)
        np.testing.assert_allclose(gx[1:-1, 1:-1], 0.0, atol=1e-5)

    def test_direction_points_left(self):
        """Uphill in +y → direction ≈ pi/2 radians."""
        elev = ramp_y(slope=self.slope)
        _, _, _, direction, _ = compute_gradient_field(elev, RES)
        np.testing.assert_allclose(direction[1:-1, 1:-1], np.pi / 2, atol=1e-4)


class TestDiagonalRamp:
    sx, sy = 0.1, 0.15

    def test_combined_gx_gy(self):
        elev = diagonal_ramp(slope_x=self.sx, slope_y=self.sy)
        gx, gy, _, _, _ = compute_gradient_field(elev, RES)
        np.testing.assert_allclose(gx[1:-1, 1:-1], self.sx, atol=1e-5)
        np.testing.assert_allclose(gy[1:-1, 1:-1], self.sy, atol=1e-5)

    def test_magnitude(self):
        elev = diagonal_ramp(slope_x=self.sx, slope_y=self.sy)
        _, _, mag, _, _ = compute_gradient_field(elev, RES)
        expected = np.hypot(self.sx, self.sy)
        np.testing.assert_allclose(mag[1:-1, 1:-1], expected, atol=1e-5)


class TestSlopeDeg45:
    """45° slope: tan(45°) = 1.0 → slope_deg = 45."""

    def test_45deg(self):
        # elevation[r, c] = 1.0 * c * RES  →  gx = 1.0  →  slope = arctan(1) = 45°
        elev = ramp_x(slope=1.0)
        _, _, _, _, slope_deg = compute_gradient_field(elev, RES)
        np.testing.assert_allclose(slope_deg[1:-1, 1:-1], 45.0, atol=0.01)


# ---------------------------------------------------------------------------
# plan_path_on_slope_field
# ---------------------------------------------------------------------------

class TestPlanPathOnSlopeField:
    def test_flat_field_goes_straight(self):
        """No obstacles → shortest path is the straight row toward target_col."""
        slope = np.zeros((10, 10), dtype=np.float32)
        path = plan_path_on_slope_field(slope, start=(5, 0), target_col=9)
        assert path is not None
        assert path[0] == (5, 0)
        assert path[-1][1] == 9
        assert all(r == 5 for r, _ in path)

    def test_routes_around_localized_steep_patch(self):
        """A steep patch blocking only the middle rows should be detoured around."""
        h, w = 10, 10
        elev = np.zeros((h, w), dtype=np.float32)
        elev[3:7, 4:7] = 50.0  # steep wall, but only in rows 3-6
        _, _, _, _, slope_deg = compute_gradient_field(elev, RES)

        path = plan_path_on_slope_field(slope_deg, start=(5, 0), target_col=9,
                                         max_slope_deg=30.0)
        assert path is not None
        assert path[0] == (5, 0)
        assert path[-1][1] == 9
        # every visited cell must respect the slope limit
        for r, c in path:
            assert slope_deg[r, c] <= 30.0

    def test_returns_none_when_fully_blocked(self):
        """A steep wall spanning every row is impassable → no path exists."""
        h, w = 10, 10
        elev = np.zeros((h, w), dtype=np.float32)
        elev[:, 6:] = 100.0  # every row blocked from column 6 onward
        _, _, _, _, slope_deg = compute_gradient_field(elev, RES)

        path = plan_path_on_slope_field(slope_deg, start=(5, 0), target_col=9,
                                         max_slope_deg=30.0)
        assert path is None

    def test_returns_none_when_start_impassable(self):
        slope = np.full((10, 10), 45.0, dtype=np.float32)
        path = plan_path_on_slope_field(slope, start=(5, 0), target_col=9,
                                         max_slope_deg=30.0)
        assert path is None

    def test_raises_on_start_out_of_bounds(self):
        slope = np.zeros((10, 10), dtype=np.float32)
        with pytest.raises(ValueError):
            plan_path_on_slope_field(slope, start=(20, 0), target_col=9)

    def test_target_col_beyond_grid_is_clamped(self):
        slope = np.zeros((10, 10), dtype=np.float32)
        path = plan_path_on_slope_field(slope, start=(5, 0), target_col=999)
        assert path is not None
        assert path[-1][1] == 9  # clamped to last column


# ---------------------------------------------------------------------------
# Benchmark tests
# ---------------------------------------------------------------------------

class TestBenchmark:
    """Timing gates — fail if gradient computation is unreasonably slow.

    Target pipeline: 5–10 Hz on onboard i7 (no GPU).
    Budget per gradient call: well under 10 ms even for large maps.
    """

    @staticmethod
    def _time_ms(h: int, w: int, n: int = 200) -> float:
        elev = np.random.rand(h, w).astype(np.float32)
        compute_gradient_field(elev, RES)          # warm-up
        t0 = time.perf_counter()
        for _ in range(n):
            compute_gradient_field(elev, RES)
        return (time.perf_counter() - t0) / n * 1e3

    def test_bench_nominal_27x27(self):
        """4 m × 4 m @ 0.15 m → 27×27 cells.  Must finish < 2 ms."""
        elapsed = self._time_ms(27, 27)
        print(f'\n  gradient_field 27×27: {elapsed:.3f} ms/call')
        assert elapsed < 2.0, f'{elapsed:.2f} ms > 2 ms threshold'

    def test_bench_medium_50x50(self):
        """7.5 m × 7.5 m @ 0.15 m → 50×50 cells.  Must finish < 5 ms."""
        elapsed = self._time_ms(50, 50)
        print(f'\n  gradient_field 50×50: {elapsed:.3f} ms/call')
        assert elapsed < 5.0, f'{elapsed:.2f} ms > 5 ms threshold'

    def test_bench_large_100x100(self):
        """15 m × 15 m @ 0.15 m → 100×100 cells.  Must finish < 15 ms."""
        elapsed = self._time_ms(100, 100)
        print(f'\n  gradient_field 100×100: {elapsed:.3f} ms/call')
        assert elapsed < 15.0, f'{elapsed:.2f} ms > 15 ms threshold'
