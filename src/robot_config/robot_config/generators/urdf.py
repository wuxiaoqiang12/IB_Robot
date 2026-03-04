"""URDF generators from robot configuration."""

from io import StringIO
from typing import Optional
from xml.etree import ElementTree as ET

from robot_config.config import RobotConfig


def generate_ros2_control_urdf(config: RobotConfig) -> str:
    """Generate ros2_control URDF snippet from robot configuration.

    This generates the <ros2_control> section that can be included in a URDF.

    Args:
        config: Robot configuration

    Returns:
        URDF XML string
    """
    ros2_control = ET.Element("ros2_control")
    ros2_control.set("name", "RobotSystem")
    ros2_control.set("type", "system")

    # Hardware plugin
    hardware = ET.SubElement(ros2_control, "hardware")
    plugin = ET.SubElement(hardware, "plugin")
    plugin.text = config.ros2_control.hardware_plugin

    # Add hardware parameters
    for key, value in config.ros2_control.params.items():
        param = ET.SubElement(hardware, "param")
        param.set("name", key)
        if isinstance(value, (dict, list)):
            # Convert to JSON string for complex types
            import json

            param.text = json.dumps(value)
        else:
            param.text = str(value)

    # Note: Joints should be defined in the base URDF file
    # This generator only creates the ros2_control section

    # Pretty print
    return _prettify_xml(ros2_control)


def generate_sensor_plugins_urdf(config: RobotConfig) -> str:
    """Generate sensor plugin URDF snippets for cameras.

    This generates the <gazebo> sensor plugins for simulation.

    Args:
        config: Robot configuration

    Returns:
        URDF XML string
    """
    root = ET.Element("root")

    for cam in config.peripherals:
        gazebo = ET.SubElement(root, "gazebo", reference=cam.frame_id)

        if cam.driver == "realsense":
            # Realsense camera plugin for Gazebo
            sensor = ET.SubElement(gazebo, "sensor")
            sensor.set("type", "depth")
            sensor.set("name", f"{cam.name}_camera")

            update_rate = ET.SubElement(sensor, "update_rate")
            update_rate.text = str(cam.fps)

            camera = ET.SubElement(sensor, "camera")
            camera.set("name", f"{cam.name}_camera")

            # Add image and depth camera sections
            image = ET.SubElement(camera, "image")
            image.set("width", str(cam.width))
            image.set("height", str(cam.height))
            image.set("format", cam.pixel_format.upper())

            if cam.depth_width:
                depth = ET.SubElement(camera, "depth")
                depth.set("width", str(cam.depth_width))
                depth.set("height", str(cam.depth_height))

        else:
            # Standard camera plugin for Gazebo
            sensor = ET.SubElement(gazebo, "sensor")
            sensor.set("type", "camera")
            sensor.set("name", f"{cam.name}_camera")

            update_rate = ET.SubElement(sensor, "update_rate")
            update_rate.text = str(cam.fps)

            camera = ET.SubElement(sensor, "camera")
            camera.set("name", f"{cam.name}_camera")

            image = ET.SubElement(camera, "image")
            image.set("width", str(cam.width))
            image.set("height", str(cam.height))
            image.set("format", cam.pixel_format.upper())

    return _prettify_xml(root)


def _prettify_xml(elem: ET.Element) -> str:
    """Pretty print XML element."""
    # Add newlines
    _indent(elem)

    # Convert to string
    xml_str = ET.tostring(elem, encoding="unicode")

    # Add XML declaration
    lines = xml_str.split("\n")
    result = ['<?xml version="1.0"?>']
    result.append(lines[0])  # root element

    # Add rest of lines
    for line in lines[1:]:
        result.append(f"  {line}")

    return "\n".join(result)


def _indent(elem: ET.Element, level: int = 0):
    """Add indentation to XML tree."""
    indent = "\n"
    for i, child in enumerate(elem):
        _indent(child, level + 1)
        # Add indentation before child
        child.tail = indent
        # Add indentation after last child
        if i == len(elem) - 1:
            child.tail = indent * (level + 1)
    if level == 0:
        elem.tail = "\n"
