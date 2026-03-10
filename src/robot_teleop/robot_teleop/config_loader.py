"""
Teleoperation configuration loader and validator

Provides utilities to load and validate teleoperation device configurations
from YAML files.
"""

import os
from pathlib import Path
from typing import Dict, Any, List, Optional
from dataclasses import dataclass, field

import yaml
from rclpy.logging import get_logger

logger = get_logger("robot_teleop.config_loader")


@dataclass
class TeleopDeviceConfig:
    """Configuration for a teleoperation device."""
    name: str
    type: str
    port: Optional[str] = None
    calib_file: Optional[str] = None
    joint_mapping: Dict[str, str] = field(default_factory=dict)
    # Device-specific parameters
    extra_params: Dict[str, Any] = field(default_factory=dict)


@dataclass
class TeleopSafetyConfig:
    """Safety configuration for teleoperation."""
    joint_limits: Dict[str, Dict[str, float]] = field(default_factory=dict)
    estop_topic: str = "/emergency_stop"


@dataclass
class TeleoperationConfig:
    """Complete teleoperation configuration."""
    enabled: bool = True
    active_device: str = ""
    devices: List[TeleopDeviceConfig] = field(default_factory=list)
    safety: TeleopSafetyConfig = field(default_factory=TeleopSafetyConfig)


def load_teleoperation_config(config_path: Optional[Path] = None,
                               config_dict: Optional[Dict] = None) -> TeleoperationConfig:
    """
    Load teleoperation configuration from YAML file or dict.

    Args:
        config_path: Path to YAML configuration file
        config_dict: Configuration dictionary (alternative to file)

    Returns:
        TeleoperationConfig object

    Raises:
        ValueError: If configuration is invalid

    Example:
        >>> config = load_teleoperation_config(
        ...     config_path=Path("config/so101_teleop.yaml")
        ... )
        >>> print(config.active_device)
        'so101_leader'
    """
    if config_dict is None:
        if config_path is None:
            raise ValueError("Either config_path or config_dict must be provided")

        with open(config_path, 'r') as f:
            config_dict = yaml.safe_load(f)

    # Extract teleoperation section
    robot_config = config_dict.get('robot', config_dict)
    teleop_config = robot_config.get('teleoperation', {})

    if not teleop_config:
        logger.warning("No teleoperation section found in config")
        return TeleoperationConfig(enabled=False)

    # Parse configuration
    enabled = teleop_config.get('enabled', True)
    active_device = teleop_config.get('active_device', '')

    # Parse devices
    devices = []
    for device_data in teleop_config.get('devices', []):
        device = _parse_device_config(device_data)
        devices.append(device)

    # Parse safety
    safety_data = teleop_config.get('safety', {})
    safety = TeleopSafetyConfig(
        joint_limits=safety_data.get('joint_limits', {}),
        estop_topic=safety_data.get('estop_topic', '/emergency_stop')
    )

    # Validate
    if enabled and devices:
        if not active_device:
            # Use first device if not specified
            active_device = devices[0].name
            logger.warning(f"No active_device specified, using first device: {active_device}")

        # Verify active_device exists
        device_names = [d.name for d in devices]
        if active_device not in device_names:
            raise ValueError(
                f"active_device '{active_device}' not found in devices list: {device_names}"
            )

    config = TeleoperationConfig(
        enabled=enabled,
        active_device=active_device,
        devices=devices,
        safety=safety
    )

    logger.info(f"Loaded teleoperation config: enabled={enabled}, "
                f"active_device={active_device}, num_devices={len(devices)}")

    return config


def _parse_device_config(data: Dict[str, Any]) -> TeleopDeviceConfig:
    """Parse device configuration from dict."""
    name = data.get('name')
    device_type = data.get('type')

    if not name:
        raise ValueError("Device configuration must have 'name' field")
    if not device_type:
        raise ValueError(f"Device '{name}' must have 'type' field")

    # Extract known fields
    known_fields = {'name', 'type', 'port', 'calib_file', 'joint_mapping'}
    extra_params = {k: v for k, v in data.items() if k not in known_fields}

    # Resolve paths
    calib_file = data.get('calib_file')
    if calib_file:
        calib_file = _resolve_path(calib_file)

    port = data.get('port')
    if port:
        port = _resolve_path(port)

    return TeleopDeviceConfig(
        name=name,
        type=device_type,
        port=port,
        calib_file=calib_file,
        joint_mapping=data.get('joint_mapping', {}),
        extra_params=extra_params
    )


def _resolve_path(path: str) -> str:
    """Resolve environment variables in path."""
    if not path:
        return path

    # Resolve $(env VAR)
    if "$(env " in path:
        import re
        pattern = r'\$\(env\s+(\w+)\)'
        matches = re.findall(pattern, path)
        for var_name in matches:
            env_value = os.environ.get(var_name, "")
            path = path.replace(f"$(env {var_name})", env_value)

    # Resolve $(find package)
    if "$(find " in path:
        # Keep as-is, will be resolved by ROS2 launch system
        pass

    return path


def get_active_device_config(config: TeleoperationConfig) -> Optional[TeleopDeviceConfig]:
    """
    Get the active device configuration.

    Args:
        config: Teleoperation configuration

    Returns:
        Active device config or None if not found
    """
    if not config.enabled or not config.devices:
        return None

    for device in config.devices:
        if device.name == config.active_device:
            return device

    logger.error(f"Active device '{config.active_device}' not found")
    return None


def validate_device_config(device: TeleopDeviceConfig) -> List[str]:
    """
    Validate device configuration.

    Args:
        device: Device configuration to validate

    Returns:
        List of validation errors (empty if valid)
    """
    errors = []

    # Type-specific validation
    if device.type == "leader_arm":
        if not device.port:
            errors.append(f"Device '{device.name}': leader_arm requires 'port' field")

        if device.calib_file and not Path(device.calib_file).exists():
            errors.append(
                f"Device '{device.name}': calibration file not found: {device.calib_file}"
            )

    # Add validation for other device types here
    # elif device.type == "xbox_controller":
    #     ...

    return errors


# Convenience function for ROS 2 node parameter conversion
def device_config_to_ros_param(device: TeleopDeviceConfig) -> Dict[str, Any]:
    """
    Convert device config to ROS 2 node parameter dict.

    Args:
        device: Device configuration

    Returns:
        Dictionary suitable for ROS 2 node parameters
    """
    param = {
        'type': device.type,
        'name': device.name,
    }

    if device.port:
        param['port'] = device.port
    if device.calib_file:
        param['calib_file'] = device.calib_file
    if device.joint_mapping:
        param['joint_mapping'] = device.joint_mapping

    # Add extra parameters
    param.update(device.extra_params)

    return param
