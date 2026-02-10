"""Configuration dataclasses for unified robot configuration."""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Union


@dataclass
class Ros2ControlConfig:
    """ros2_control hardware configuration."""

    hardware_plugin: str
    params: Dict[str, Any] = field(default_factory=dict)
    urdf_path: Optional[str] = None


@dataclass
class CameraConfig:
    """Camera peripheral configuration.

    This configuration is used to generate launch parameters for
    existing ROS2 camera drivers:
    - usb_cam for USB cameras (driver: opencv)
    - realsense2_camera for RealSense D400 series (driver: realsense)
    """

    name: str
    driver: str  # opencv, realsense, etc.
    index_or_port: Union[str, int]  # USB index for opencv, serial/port for realsense
    width: int
    height: int
    fps: int
    frame_id: str
    optical_frame_id: Optional[str] = None
    camera_info_url: Optional[str] = None  # Path to calibration file
    pixel_format: str = "bgr8"  # bgr8, rgb8, etc.

    # Realsense-specific parameters
    depth_width: Optional[int] = None
    depth_height: Optional[int] = None
    depth_fps: Optional[int] = None
    enable_pointcloud: bool = False
    enable_sync: bool = True
    align_depth: bool = False

    # USB camera specific parameters
    brightness: Optional[int] = None
    contrast: Optional[int] = None
    saturation: Optional[int] = None
    sharpness: Optional[int] = None

    # Transform (parent to camera frame)
    transform: Optional[Dict[str, float]] = None  # {x, y, z, roll, pitch, yaw}


@dataclass
class PeripheralConfig:
    """Generic peripheral device configuration.

    Use CameraConfig for camera-specific configuration.
    """

    type: str  # camera, microphone, etc.
    name: str
    driver: str
    params: Dict[str, Any] = field(default_factory=dict)
    frame_id: Optional[str] = None


@dataclass
class ContractObservation:
    """Contract observation reference."""

    key: str
    topic: str
    peripheral: Optional[str] = None  # References peripheral by name
    selector: Optional[Dict[str, Any]] = None
    image: Optional[Dict[str, Any]] = None
    align: Optional[Dict[str, Any]] = None
    qos: Optional[Dict[str, Any]] = None


@dataclass
class ContractAction:
    """Contract action definition."""

    key: str
    publish: Dict[str, Any]
    selector: Optional[Dict[str, Any]] = None
    from_tensor: Optional[Dict[str, Any]] = None
    safety_behavior: str = "zeros"


@dataclass
class ContractExtensionConfig:
    """Rosetta contract extension configuration."""

    base_contract: Optional[str] = None
    observations: List[ContractObservation] = field(default_factory=list)
    actions: List[ContractAction] = field(default_factory=list)
    rate_hz: float = 20.0
    max_duration_s: float = 30.0


@dataclass
class RobotConfig:
    """Unified robot configuration.

    This is the main configuration class that defines:
    - Robot metadata (name, type, robot_type for LeRobot)
    - ros2_control configuration (joints/motors)
    - Peripheral devices (cameras, etc.)
    - Contract extensions (ML I/O mapping)
    """

    name: str
    type: str
    robot_type: str  # For LeRobot dataset metadata (e.g., so_101)
    ros2_control: Ros2ControlConfig
    peripherals: List[CameraConfig] = field(default_factory=list)
    contract: ContractExtensionConfig = field(default_factory=ContractExtensionConfig)

    def get_camera(self, name: str) -> Optional[CameraConfig]:
        """Get camera configuration by name."""
        for cam in self.peripherals:
            if cam.name == name:
                return cam
        return None

    def get_all_cameras(self) -> List[CameraConfig]:
        """Get all camera configurations."""
        return [p for p in self.peripherals if isinstance(p, CameraConfig)]
