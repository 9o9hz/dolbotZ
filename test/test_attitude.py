"""
Unit tests for dolbotz/utils/attitude.py's mount-parameter constants and the
calibration-pickle override logic (resolve_mount_defaults), plus a regression
test that elevation_map.py/flat_drive.py actually wire their declare_parameter
defaults to these constants instead of silently drifting back to duplicated
literals.

Run (requires ROS env sourced):
    source /opt/ros/humble/setup.bash
    python3 -m pytest test/test_attitude.py -v
"""

import rclpy

from dolbotz.utils import attitude as attitude_module
from dolbotz.utils.attitude import (
    COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER,
    MOUNT_CAMERA_HEIGHT_M_PLACEHOLDER,
    MOUNT_PITCH_OFFSET_DEG_PLACEHOLDER,
    MOUNT_ROLL_OFFSET_DEG_PLACEHOLDER,
    resolve_mount_defaults,
)


# ---------------------------------------------------------------------------
# Constants sanity check
# ---------------------------------------------------------------------------

class TestMountConstants:
    def test_constants_exist_with_reasonable_float_types(self):
        assert isinstance(MOUNT_CAMERA_HEIGHT_M_PLACEHOLDER, float)
        assert isinstance(MOUNT_PITCH_OFFSET_DEG_PLACEHOLDER, float)
        assert isinstance(MOUNT_ROLL_OFFSET_DEG_PLACEHOLDER, float)
        assert isinstance(COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER, float)

        assert MOUNT_CAMERA_HEIGHT_M_PLACEHOLDER > 0.0
        assert 0.0 < COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER < 1.0


# ---------------------------------------------------------------------------
# resolve_mount_defaults — pure function, no ROS needed
# ---------------------------------------------------------------------------

class TestResolveMountDefaults:
    def test_no_calibration_falls_back_to_constants(self, monkeypatch):
        monkeypatch.setattr(attitude_module, 'load_calibration', lambda serial_no: None)

        defaults = resolve_mount_defaults('')
        assert defaults == {
            'camera_height_m': MOUNT_CAMERA_HEIGHT_M_PLACEHOLDER,
            'camera_pitch_offset_deg': MOUNT_PITCH_OFFSET_DEG_PLACEHOLDER,
            'camera_roll_offset_deg': MOUNT_ROLL_OFFSET_DEG_PLACEHOLDER,
            'complementary_filter_alpha': COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER,
        }

    def test_full_calibration_overrides_all_constants(self, monkeypatch):
        pickle_values = {
            'serial_no': '339222071362',
            'camera_height_m': 0.512,
            'camera_pitch_offset_deg': 9.3,
            'camera_roll_offset_deg': -0.4,
            'complementary_filter_alpha': 0.95,
        }
        monkeypatch.setattr(attitude_module, 'load_calibration', lambda serial_no: pickle_values)

        defaults = resolve_mount_defaults('339222071362')
        assert defaults == {
            'camera_height_m': 0.512,
            'camera_pitch_offset_deg': 9.3,
            'camera_roll_offset_deg': -0.4,
            'complementary_filter_alpha': 0.95,
        }

    def test_partial_calibration_overrides_only_present_keys(self, monkeypatch):
        """A pickle missing complementary_filter_alpha (e.g. an older schema
        version) must fall back to the constant for that one key only."""
        pickle_values = {
            'serial_no': '339222071362',
            'camera_height_m': 0.48,
            'camera_pitch_offset_deg': 11.0,
            'camera_roll_offset_deg': 0.2,
        }
        monkeypatch.setattr(attitude_module, 'load_calibration', lambda serial_no: pickle_values)

        defaults = resolve_mount_defaults('339222071362')
        assert defaults['camera_height_m'] == 0.48
        assert defaults['camera_pitch_offset_deg'] == 11.0
        assert defaults['camera_roll_offset_deg'] == 0.2
        assert defaults['complementary_filter_alpha'] == COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER

    def test_calls_load_calibration_with_given_serial_no(self, monkeypatch):
        received = {}

        def _fake_load(serial_no):
            received['serial_no'] = serial_no
            return None
        monkeypatch.setattr(attitude_module, 'load_calibration', _fake_load)

        resolve_mount_defaults('D435I_339222071362')
        assert received['serial_no'] == 'D435I_339222071362'


# ---------------------------------------------------------------------------
# Regression guard: the ROS nodes must actually use these constants as their
# declare_parameter defaults, not duplicated literals.
# ---------------------------------------------------------------------------

class TestNodeParameterDefaultsMatchAttitudeConstants:
    """With camera_serial_no left at its default ('' -> no calibration pickle
    found), both nodes' mount-parameter defaults must come out exactly equal
    to attitude.py's constants. This is the regression this whole task exists
    to prevent: elevation_map.py and flat_drive.py silently drifting back to
    independently-duplicated literal defaults."""

    def test_elevation_map_node_defaults(self):
        from dolbotz.elevation_map import ElevationMapNode

        rclpy.init()
        try:
            node = ElevationMapNode()
            try:
                assert node.get_parameter('camera_height_m').value == MOUNT_CAMERA_HEIGHT_M_PLACEHOLDER
                assert node.get_parameter('camera_pitch_offset_deg').value == MOUNT_PITCH_OFFSET_DEG_PLACEHOLDER
                assert node.get_parameter('camera_roll_offset_deg').value == MOUNT_ROLL_OFFSET_DEG_PLACEHOLDER
                assert node.get_parameter('complementary_filter_alpha').value == COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER
            finally:
                node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_flat_drive_node_defaults(self):
        from dolbotz.flat_drive import FlatDriveNode

        rclpy.init()
        try:
            node = FlatDriveNode()
            try:
                assert node.get_parameter('camera_height_m').value == MOUNT_CAMERA_HEIGHT_M_PLACEHOLDER
                assert node.get_parameter('camera_pitch_offset_deg').value == MOUNT_PITCH_OFFSET_DEG_PLACEHOLDER
                assert node.get_parameter('camera_roll_offset_deg').value == MOUNT_ROLL_OFFSET_DEG_PLACEHOLDER
                assert node.get_parameter('complementary_filter_alpha').value == COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER
            finally:
                node.destroy_node()
        finally:
            rclpy.shutdown()
