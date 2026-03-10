"""
robot_teleop - Minimal serial-to-controller bridge for zero-latency teleoperation

This package provides a unified teleoperation interface for IB-Robot,
supporting multiple teleoperation devices (leader arms, gamepads, VR controllers)
through a device abstraction layer.
"""

from .base_teleop import BaseTeleopDevice
from .device_factory import device_factory, DEVICE_MAP
from .safety_filter import SafetyFilter
from .devices.leader_arm import LeaderArmDevice
from .config_loader import (
    TeleoperationConfig,
    TeleopDeviceConfig,
    TeleopSafetyConfig,
    load_teleoperation_config,
    get_active_device_config,
)

__all__ = [
    'BaseTeleopDevice',
    'device_factory',
    'DEVICE_MAP',
    'SafetyFilter',
    'LeaderArmDevice',
    'TeleoperationConfig',
    'TeleopDeviceConfig',
    'TeleopSafetyConfig',
    'load_teleoperation_config',
    'get_active_device_config',
]

__version__ = '0.1.0'
