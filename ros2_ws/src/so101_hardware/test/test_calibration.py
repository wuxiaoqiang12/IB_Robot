"""Tests for so101_hardware calibration module."""

import pytest


def test_calibration_constants():
    """Test that calibration constants are defined."""
    from so101_hardware.calibration.constants import (
        MOTOR_COUNT,
        MOTOR_IDS,
        JOINT_NAMES,
        DEFAULT_SERIAL_PORT,
        DEFAULT_LEADER_PUBLISH_RATE,
        DEFAULT_CONTROL_RATE,
    )

    assert MOTOR_COUNT == 6
    assert len(MOTOR_IDS) == 6
    assert len(JOINT_NAMES) == 6
    assert DEFAULT_SERIAL_PORT == "/dev/ttyACM0"
    assert DEFAULT_LEADER_PUBLISH_RATE == 50.0
    assert DEFAULT_CONTROL_RATE == 100.0
