"""Teleoperation node generation for robot_config.

This module provides utilities to generate teleoperation nodes
for integration with the robot_config launch system.
"""

import os
import re
import json
from pathlib import Path
from launch_ros.actions import Node
from typing import Dict, List, Any

from robot_config.utils import resolve_ros_path, prepare_lerobot_env


def generate_teleop_nodes(robot_config: dict) -> List[Node]:
    """
    Generate teleoperation nodes based on robot configuration.

    This function creates ROS 2 nodes for teleoperation based on the
    robot configuration YAML. It integrates with the robot_config launch system.

    Args:
        robot_config: Robot configuration dictionary loaded from YAML

    Returns:
        List of Node actions for teleoperation

    Example:
        >>> from robot_config.launch_builders.teleop import generate_teleop_nodes
        >>> config = load_robot_config('so101_single_arm')
        >>> nodes = generate_teleop_nodes(config)
        >>> ld.add_action(nodes[0])

    Configuration Format:
        ```yaml
        robot:
          teleoperation:
            enabled: true
            active_device: "so101_leader"
            devices:
              - name: "so101_leader"
                type: "leader_arm"
                port: "/dev/ttyUSB0"
                calib_file: "~/.calibrate/so101_leader_calibrate.json"
            safety:
              joint_limits:
                "1": {"min": -3.14, "max": 3.14}
        ```
    """
    nodes = []

    # Get teleoperation config
    teleop_config = robot_config.get('teleoperation', {})
    if not teleop_config.get('enabled', False):
        print("[teleop_builder] Teleoperation not enabled, skipping")
        return nodes

    # Get active device
    active_device_name = teleop_config.get('active_device', '')
    if not active_device_name:
        print("[teleop_builder] No active_device specified")
        return nodes

    # Find device config
    device_config = None
    for device in teleop_config.get('devices', []):
        if device.get('name') == active_device_name:
            device_config = device
            break

    if not device_config:
        print(f"[teleop_builder] Active device '{active_device_name}' not found")
        return nodes

    # Get joint limits from safety config
    safety_config = teleop_config.get('safety', {})
    joint_limits = safety_config.get('joint_limits', {})

    # Get control frequency (default: 50 Hz)
    control_frequency = 50.0

    # Get joint names from robot config
    joints_config = robot_config.get('joints', {})
    arm_joint_names = joints_config.get('arm', [])
    gripper_joint_names = joints_config.get('gripper', [])

    # Build device config for node parameter
    device_param = {
        'type': device_config.get('type', ''),
        'name': device_config.get('name', ''),
    }

    # Add optional device parameters
    if 'port' in device_config:
        device_param['port'] = device_config['port']
    if 'calib_file' in device_config:
        # Expand environment variables in calib_file path
        calib_file_raw = device_config['calib_file']
        calib_file_expanded = resolve_ros_path(device_config['calib_file'])
        print(f"[DEBUG] teleop.py: calib_file_raw: {calib_file_raw}")
        print(f"[DEBUG] teleop.py: calib_file_expanded: {calib_file_expanded}")
        device_param['calib_file'] = calib_file_expanded
    if 'joint_mapping' in device_config:
        device_param['joint_mapping'] = device_config['joint_mapping']

    # Add joint limits for proper scaling
    if joint_limits:
        device_param['joint_limits'] = joint_limits

    # Add any extra device-specific parameters
    known_keys = {'name', 'type', 'port', 'calib_file', 'joint_mapping'}
    for key, value in device_config.items():
        if key not in known_keys:
            device_param[key] = value

    # Prepare lerobot environment
    env = prepare_lerobot_env()

    # Convert dicts to JSON strings for ROS 2 parameter passing
    device_param_json = json.dumps(device_param)
    joint_limits_json = json.dumps(joint_limits)
    print(f"[DEBUG] teleop.py: device_param_json: {device_param_json}")
    print(f"[DEBUG] teleop.py: device_param dict: {device_param}")

    # Create teleop node
    teleop_node = Node(
        package='robot_teleop',
        executable='teleop_node',
        name='robot_teleop_node',
        output='screen',
        env=env,
        parameters=[{
            'control_frequency': control_frequency,
            'device_config': device_param_json,
            'joint_limits': joint_limits_json,
            'arm_joint_names': arm_joint_names,
            'gripper_joint_names': gripper_joint_names,
        }],
    )

    nodes.append(teleop_node)
    print(f"[teleop_builder] Generated teleop node for device: {active_device_name} (type: {device_config.get('type')})")

    return nodes


def validate_teleop_config(teleop_config: Dict[str, Any]) -> List[str]:
    """
    Validate teleoperation configuration.

    Args:
        teleop_config: Teleoperation configuration dictionary

    Returns:
        List of validation error messages (empty if valid)

    Example:
        >>> errors = validate_teleop_config(config['teleoperation'])
        >>> if errors:
        ...     for error in errors:
        ...         print(f"Error: {error}")
    """
    errors = []

    if not teleop_config.get('enabled', False):
        return errors  # Not enabled, skip validation

    # Check active device
    active_device = teleop_config.get('active_device')
    if not active_device:
        errors.append("active_device must be specified when teleoperation is enabled")
        return errors

    # Check devices list
    devices = teleop_config.get('devices', [])
    if not devices:
        errors.append("devices list is empty")
        return errors

    # Find active device in list
    device_found = False
    for device in devices:
        if device.get('name') == active_device:
            device_found = True

            # Validate device type
            if not device.get('type'):
                errors.append(f"Device '{active_device}': missing 'type' field")

            # Type-specific validation
            if device.get('type') == 'leader_arm':
                if not device.get('port'):
                    errors.append(f"Device '{active_device}': leader_arm requires 'port' field")

            break

    if not device_found:
        errors.append(f"active_device '{active_device}' not found in devices list")

    # Validate safety config
    safety = teleop_config.get('safety', {})
    joint_limits = safety.get('joint_limits', {})

    if not joint_limits:
        errors.append("safety.joint_limits not specified (recommended for safe operation)")
    else:
        # Validate joint limit format
        for joint_name, limits in joint_limits.items():
            if 'min' not in limits or 'max' not in limits:
                errors.append(f"Joint '{joint_name}': limits must have 'min' and 'max' fields")
            elif limits['min'] >= limits['max']:
                errors.append(f"Joint '{joint_name}': min must be less than max")

    return errors


def get_recording_topics(robot_config: dict) -> List[str]:
    """
    Get list of topics to record for teleoperation sessions.

    Args:
        robot_config: Robot configuration dictionary

    Returns:
        List of topic names for rosbag recording

    Example:
        >>> topics = get_recording_topics(config)
        >>> cmd = ['ros2', 'bag', 'record'] + topics
    """
    topics = []

    # Always record joint states
    topics.append('/joint_states')

    # Add controller command topics
    topics.append('/arm_position_controller/commands')
    topics.append('/gripper_position_controller/commands')

    # Add teleop diagnostics
    topics.append('/diagnostics')

    # Add camera topics from peripherals
    peripherals = robot_config.get('peripherals', [])
    for peripheral in peripherals:
        if peripheral.get('type') == 'camera':
            name = peripheral.get('name', 'camera')
            # Add common camera topics
            topics.append(f'/camera/{name}/image_raw')
            topics.append(f'/camera/{name}/camera_info')

    return topics
