#!/usr/bin/env python3
"""Validation script for robot configuration files.

This script validates robot_config YAML files and reports any errors.
Usage:
    python validate_config.py /path/to/robot_config.yaml
"""

import sys
from pathlib import Path

# Add robot_config to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from robot_config.loader import load_robot_config, validate_config
from rclpy.logging import get_logger

logger = get_logger("validate_config")


def main():
    if len(sys.argv) < 2:
        print("Usage: python validate_config.py <robot_config.yaml>")
        print("\nExample:")
        print("  python validate_config.py config/robots/so101_single_arm.yaml")
        sys.exit(1)

    config_path = sys.argv[1]

    try:
        print(f"Validating {config_path}...")
        config = load_robot_config(config_path)

        print(f"  Robot: {config.name} ({config.robot_type})")
        print(f"  Hardware plugin: {config.ros2_control.hardware_plugin}")
        print(f"  Cameras: {len(config.peripherals)}")

        for cam in config.peripherals:
            print(f"    - {cam.name}: {cam.driver} @ {cam.width}x{cam.height} {cam.fps}fps")

        errors = validate_config(config)

        if errors:
            print(f"\n❌ Validation failed with {len(errors)} error(s):")
            for error in errors:
                print(f"  - {error}")
            sys.exit(1)
        else:
            print("\n✅ Configuration is valid!")
            sys.exit(0)

    except FileNotFoundError as e:
        print(f"❌ Error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
