# !/usr/bin/env python3
# @Time    : 2025/08/01 15:00
# @Author  : Yida Hao
# @File    : main.py
"""
Main entry point for hardware interface.
Launches nodes for interacting with cameras and joints.
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from robot_interface.robot_interface import RobotInterface


def main(args=None):
    """
    Main entry point for robot interface node.
    """
    rclpy.init(args=args)

    robot_node = RobotInterface()

    executor = MultiThreadedExecutor()

    executor.add_node(robot_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        robot_node.cleanup()
        robot_node.destroy_node()
        rclpy.shutdown()
