#!/usr/bin/env python3
"""Unified calibration tool for SO-101 Leader and Follower arms."""

import argparse
import pathlib
import rclpy
from rclpy.node import Node
from so101_hw_interface.motors.feetech import FeetechMotorsBus
from so101_hw_interface.motors import Motor, MotorNormMode
from so101_hw_interface.calibration import run_interactive_calibration, save_calibration

# Arm-specific configurations
ARM_CONFIGS = {
    "follower": {
        "calib_path": pathlib.Path.home() / ".calibrate" / "so101_follower_calibrate.json",
        "default_port": "/dev/ttyACM0",
        "joints": {
            "1": {"id": 1, "model": "sts3215", "mode": MotorNormMode.RANGE_M100_100},
            "2": {"id": 2, "model": "sts3215", "mode": MotorNormMode.RANGE_M100_100},
            "3": {"id": 3, "model": "sts3215", "mode": MotorNormMode.RANGE_M100_100},
            "4": {"id": 4, "model": "sts3215", "mode": MotorNormMode.RANGE_M100_100},
            "5": {"id": 5, "model": "sts3215", "mode": MotorNormMode.RANGE_M100_100},
            "6": {"id": 6, "model": "sts3215", "mode": MotorNormMode.RANGE_0_100},
        }
    },
    "leader": {
        "calib_path": pathlib.Path.home() / ".calibrate" / "so101_leader_calibrate.json",
        "default_port": "/dev/ttyACM1",
        "joints": {
            "1": {"id": 1, "model": "sts3215", "mode": MotorNormMode.RANGE_M100_100},
            "2": {"id": 2, "model": "sts3215", "mode": MotorNormMode.RANGE_M100_100},
            "3": {"id": 3, "model": "sts3215", "mode": MotorNormMode.RANGE_M100_100},
            "4": {"id": 4, "model": "sts3215", "mode": MotorNormMode.RANGE_M100_100},
            "5": {"id": 5, "model": "sts3215", "mode": MotorNormMode.RANGE_M100_100},
            "6": {"id": 6, "model": "sts3215", "mode": MotorNormMode.RANGE_0_100},
        }
    }
}


class UnifiedCalibrator(Node):
    def __init__(self, arm_type: str, port: str):
        super().__init__(f"so101_{arm_type}_calibrator")

        config = ARM_CONFIGS[arm_type]
        self.calib_path = config["calib_path"]

        self.get_logger().info(f"Starting calibration for {arm_type.upper()} arm on {port}")

        # Build motor objects
        motors = {
            name: Motor(cfg["id"], cfg["model"], cfg["mode"])
            for name, cfg in config["joints"].items()
        }
        joint_names = list(config["joints"].keys())

        # Connect to bus
        bus = FeetechMotorsBus(port, motors)
        self.get_logger().info(f"Connecting to motor bus on {port}...")
        bus.connect()

        # Run calibration
        calibration_data = run_interactive_calibration(bus, joint_names, self.get_logger())

        if calibration_data is None:
            self.get_logger().error("Calibration failed.")
            bus.disconnect()
            return

        # Write to motor firmware
        self.get_logger().info("Writing calibration to motor firmware...")
        bus.write_calibration(calibration_data)

        # Save to file
        save_calibration(calibration_data, self.calib_path, self.get_logger())

        self.get_logger().info(f"✅ {arm_type.upper()} arm calibration completed successfully!")
        bus.disconnect()


def main():
    parser = argparse.ArgumentParser(description="Calibrate SO-101 Leader or Follower arm")
    parser.add_argument("--arm", choices=["leader", "follower"], default="follower",
                        help="Which arm to calibrate (default: follower)")
    parser.add_argument("--port", type=str, default=None,
                        help="Serial port (default: /dev/ttyACM0 for follower, /dev/ttyACM1 for leader)")

    args = parser.parse_args()

    # Use default port if not specified
    port = args.port if args.port else ARM_CONFIGS[args.arm]["default_port"]

    rclpy.init()
    try:
        calibrator = UnifiedCalibrator(args.arm, port)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
