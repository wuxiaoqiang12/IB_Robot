"""
Device factory for dynamic teleoperation device instantiation

Provides a factory pattern for creating teleoperation devices based on
configuration, enabling easy extension without modifying core code.
"""

from typing import Dict, Type
from .base_teleop import BaseTeleopDevice
from .devices.leader_arm import LeaderArmDevice


# Device registry - add new device types here
DEVICE_MAP: Dict[str, Type[BaseTeleopDevice]] = {
    "leader_arm": LeaderArmDevice,  # SO-101 leader arm
    # "xbox_controller": XboxDevice,   # Future implementation
    # "vr_device": VRDevice,            # Future implementation
}


def device_factory(config: dict) -> BaseTeleopDevice:
    """
    Create a teleoperation device instance based on configuration.

    Args:
        config: Device configuration dictionary with at minimum a 'type' key
                Example: {
                    "type": "leader_arm",
                    "name": "so101_leader",
                    "port": "/dev/ttyUSB0",
                    ...
                }

    Returns:
        BaseTeleopDevice: Instantiated device object

    Raises:
        ValueError: If device type is unknown or not registered

    Example:
        >>> config = {"type": "leader_arm", "port": "/dev/ttyUSB0"}
        >>> device = device_factory(config)
        >>> device.connect()
    """
    if not config:
        raise ValueError("Device configuration cannot be empty")

    dev_type = config.get("type")
    if not dev_type:
        raise ValueError("Device configuration must include 'type' field")

    if dev_type not in DEVICE_MAP:
        available_types = ", ".join(DEVICE_MAP.keys())
        raise ValueError(
            f"Unknown teleop device type: '{dev_type}'. "
            f"Available types: [{available_types}]"
        )

    device_class = DEVICE_MAP[dev_type]
    return device_class(config)


def register_device(device_type: str, device_class: Type[BaseTeleopDevice]) -> None:
    """
    Register a new device type in the global device map.

    This allows external packages to extend the teleop system with custom devices.

    Args:
        device_type: String identifier for the device (e.g., "leader_arm")
        device_class: Class that inherits from BaseTeleopDevice

    Example:
        >>> class CustomDevice(BaseTeleopDevice):
        ...     # ... implementation
        >>> register_device("custom", CustomDevice)
    """
    if device_type in DEVICE_MAP:
        import warnings
        warnings.warn(
            f"Overwriting existing device type: {device_type}",
            UserWarning
        )

    DEVICE_MAP[device_type] = device_class
