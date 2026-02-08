"""Main robot launch file for robot_config.

This launch file loads robot configuration from YAML and dynamically generates:
- ros2_control hardware interface and controllers
- Robot state publisher
- Camera drivers (usb_cam, realsense2_camera)
- Static TF publishers for camera frames

Usage:
    ros2 launch robot_config robot.launch.py robot_config:=test_cam use_sim:=false
    ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm use_sim:=false
"""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration


def load_robot_config(robot_config_name, config_path_override=None):
    """Load robot configuration from YAML file.

    Args:
        robot_config_name: Robot configuration name
        config_path_override: Optional full path to config file

    Returns:
        Robot configuration dict
    """
    import yaml

    # Get package share directory
    try:
        robot_config_share = get_package_share_directory("robot_config")
    except:
        robot_config_share = str(Path(__file__).parent.parent)

    # Determine config file path
    if config_path_override:
        config_path = Path(config_path_override)
    else:
        config_path = Path(robot_config_share) / "config" / "robots" / f"{robot_config_name}.yaml"

    print(f"[robot_config] Loading config from: {config_path}")
    print(f"[robot_config] Config exists: {config_path.exists()}")

    # Load YAML
    with open(config_path, "r") as f:
        data = yaml.safe_load(f)

    robot_config = data.get("robot", {})
    print(f"[robot_config] Loaded robot: {robot_config.get('name', 'UNKNOWN')}")
    print(f"[robot_config] Peripherals: {len(robot_config.get('peripherals', []))}")

    return robot_config


def launch_setup(context, *args, **kwargs):
    """Launch setup function that generates all nodes.

    Args:
        context: Launch context

    Returns:
        List of launch actions
    """
    actions = []

    # Get launch parameters
    robot_config_name = context.launch_configurations.get('robot_config', 'test_cam')
    config_path_override = context.launch_configurations.get('config_path', '')
    use_sim = context.launch_configurations.get('use_sim', 'false')

    print(f"[robot_config] Launch setup with:")
    print(f"[robot_config]   robot_config: {robot_config_name}")
    print(f"[robot_config]   config_path: {config_path_override if config_path_override else '(none)'}")
    print(f"[robot_config]   use_sim: {use_sim}")

    # Load robot configuration
    try:
        robot_config = load_robot_config(robot_config_name, config_path_override if config_path_override else None)
    except Exception as e:
        print(f"[robot_config] ERROR loading config: {e}")
        raise

    # TODO: Add node generation functions in subsequent commits
    # - generate_ros2_control_nodes()
    # - generate_camera_nodes()
    # - generate_tf_nodes()
    # - generate_gazebo_nodes()

    print(f"[robot_config] Total nodes to launch: {len(actions)}")

    return actions


def generate_launch_description():
    """Generate launch description for robot system."""
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_config",
            default_value="test_cam",
            description="Robot configuration name (without .yaml extension)",
        ),
        DeclareLaunchArgument(
            "config_path",
            default_value="",
            description="Optional: Full path to robot config file (overrides robot_config)",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Use simulation mode (skip camera nodes)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
