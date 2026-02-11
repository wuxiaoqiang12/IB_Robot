"""Tests for robot_config package."""

import pytest
from pathlib import Path
from robot_config.config import (
    RobotConfig,
    Ros2ControlConfig,
    CameraConfig,
    ContractExtensionConfig,
)
from robot_config.loader import load_robot_config, validate_config


def test_load_single_arm_config():
    """Test loading SO-101 single arm configuration."""
    # This test assumes the example config exists
    config_path = Path(__file__).parent.parent / "config" / "robots" / "so101_single_arm.yaml"

    if not config_path.exists():
        pytest.skip(f"Config file not found: {config_path}")

    config = load_robot_config(config_path)

    assert config.name == "so101_single_arm"
    assert config.robot_type == "so_101"
    assert config.ros2_control.hardware_plugin == "so101_hardware/SO101SystemHardware"
    assert len(config.peripherals) == 2

    # Check cameras
    top_cam = config.get_camera("top")
    assert top_cam is not None
    assert top_cam.driver == "opencv"
    assert top_cam.width == 640
    assert top_cam.height == 480
    assert top_cam.fps == 30

    wrist_cam = config.get_camera("wrist")
    assert wrist_cam is not None
    assert wrist_cam.fps == 60  # Higher FPS for wrist camera


def test_validate_valid_config():
    """Test validation of valid configuration."""
    config = RobotConfig(
        name="test_robot",
        type="so101",
        robot_type="so_101",
        ros2_control=Ros2ControlConfig(
            hardware_plugin="so101_hardware/SO101SystemHardware",
            params={"port": "/dev/ttyACM0"},
        ),
        peripherals=[
            CameraConfig(
                name="test_cam",
                driver="opencv",
                index_or_port=0,
                width=640,
                height=480,
                fps=30,
                frame_id="camera_test_frame",
            )
        ],
        contract=ContractExtensionConfig(
            observations=[],
            actions=[],
        ),
    )

    errors = validate_config(config)
    assert len(errors) == 0


def test_validate_duplicate_camera_names():
    """Test validation catches duplicate camera names."""
    config = RobotConfig(
        name="test_robot",
        type="so101",
        robot_type="so_101",
        ros2_control=Ros2ControlConfig(
            hardware_plugin="so101_hardware/SO101SystemHardware",
            params={},
        ),
        peripherals=[
            CameraConfig(
                name="test_cam",
                driver="opencv",
                index_or_port=0,
                width=640,
                height=480,
                fps=30,
                frame_id="camera_test_frame",
            ),
            CameraConfig(
                name="test_cam",  # Duplicate name
                driver="opencv",
                index_or_port=1,
                width=640,
                height=480,
                fps=30,
                frame_id="camera_test_frame2",
            ),
        ],
        contract=ContractExtensionConfig(
            observations=[],
            actions=[],
        ),
    )

    errors = validate_config(config)
    assert len(errors) > 0
    assert any("Duplicate camera name" in error for error in errors)


def test_validate_invalid_camera_dimensions():
    """Test validation catches invalid camera dimensions."""
    config = RobotConfig(
        name="test_robot",
        type="so101",
        robot_type="so_101",
        ros2_control=Ros2ControlConfig(
            hardware_plugin="so101_hardware/SO101SystemHardware",
            params={},
        ),
        peripherals=[
            CameraConfig(
                name="test_cam",
                driver="opencv",
                index_or_port=0,
                width=0,  # Invalid
                height=480,
                fps=30,
                frame_id="camera_test_frame",
            )
        ],
        contract=ContractExtensionConfig(
            observations=[],
            actions=[],
        ),
    )

    errors = validate_config(config)
    assert len(errors) > 0
    assert any("Invalid camera dimensions" in error for error in errors)


def test_get_all_cameras():
    """Test getting all cameras from configuration."""
    config = RobotConfig(
        name="test_robot",
        type="so101",
        robot_type="so_101",
        ros2_control=Ros2ControlConfig(
            hardware_plugin="so101_hardware/SO101SystemHardware",
            params={},
        ),
        peripherals=[
            CameraConfig(
                name="cam1",
                driver="opencv",
                index_or_port=0,
                width=640,
                height=480,
                fps=30,
                frame_id="camera_cam1_frame",
            ),
            CameraConfig(
                name="cam2",
                driver="realsense",
                index_or_port="12345678",
                width=640,
                height=480,
                fps=30,
                frame_id="camera_cam2_frame",
            ),
        ],
        contract=ContractExtensionConfig(
            observations=[],
            actions=[],
        ),
    )

    cameras = config.get_all_cameras()
    assert len(cameras) == 2
    assert cameras[0].name == "cam1"
    assert cameras[1].name == "cam2"
