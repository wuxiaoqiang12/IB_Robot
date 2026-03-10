"""
Base teleoperation device interface

Defines the abstract interface that all teleoperation devices must implement.
"""

import logging
from abc import ABC, abstractmethod
from typing import Dict


class BaseTeleopDevice(ABC):
    """
    Abstract base class for all teleoperation devices.

    All teleoperation devices (leader arms, gamepads, VR controllers) must
    inherit from this class and implement the required abstract methods.

    The class provides a standardized interface for:
    - Device connection management
    - Joint target acquisition
    - Resource cleanup

    Attributes:
        _is_connected (bool): Internal connection status flag
        logger: Python logger for device messages
    """

    def __init__(self, config: dict):
        """
        Initialize the teleoperation device.

        Args:
            config (dict): Device configuration from robot_config YAML
        """
        self._is_connected = False
        self._config = config
        self.logger = logging.getLogger(self.__class__.__name__)

    @property
    def is_connected(self) -> bool:
        """
        Check if the device is currently connected.

        Returns:
            bool: True if device is connected and operational, False otherwise
        """
        return self._is_connected

    @abstractmethod
    def connect(self) -> bool:
        """
        Establish connection to the teleoperation device.

        This method should:
        - Open serial ports, Bluetooth connections, or other communication channels
        - Initialize device hardware
        - Verify device is responsive

        Returns:
            bool: True if connection successful, False otherwise

        Raises:
            ConnectionError: If device cannot be reached
        """
        pass

    @abstractmethod
    def get_joint_targets(self) -> Dict[str, float]:
        """
        Read current joint targets from the teleoperation device.

        This is the core method called at each control cycle. It should:
        - Read device state (joint positions, button states, poses, etc.)
        - Apply device-specific transformations (calibration, mapping, IK)
        - Return joint-angle mapping in a standardized format

        Device-specific behavior:
        - Leader Arm: Direct joint mapping from serial readings
        - Xbox Controller: Maintain internal position state, apply delta increments
        - VR Controller: Call IK solver to convert pose → joint angles

        Returns:
            Dict[str, float]: Mapping from joint names to target angles (radians)
                             Example: {"1": 0.5, "2": 1.2, "3": -0.3, ...}

        Note:
            If device is disconnected or read fails, return empty dict {}
            The teleop node will handle the failure gracefully
        """
        pass

    @abstractmethod
    def disconnect(self):
        """
        Disconnect from the teleoperation device and release resources.

        This method should:
        - Close communication channels (serial ports, Bluetooth)
        - Release file handles and hardware resources
        - Reset internal state

        Safe to call multiple times.
        """
        pass

    def __enter__(self):
        """Context manager entry - connect to device."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - disconnect from device."""
        self.disconnect()
        return False
