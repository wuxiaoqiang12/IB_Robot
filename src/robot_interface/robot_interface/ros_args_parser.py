# !/usr/bin/env python3
# @Time    : 2025/11/20 10:26
# @Author  : Yida Hao
# @File    : robot_config.py
"""Robot configuration parser for loading ROS parameters."""

import rclpy
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.robots import Robot
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.robots.so101_follower.so101_follower import SO101Follower
from lerobot.robots.so101_follower.so101_follower_mock import SO101FollowerMock
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy import Parameter
from rclpy.node import Node

class RosArgsParser:
    """Parser for ROS parameters to configure robots and cameras."""

    def __init__(self, node: Node):
        """Initialize ROS args parser."""
        self.node = node
        self.is_mock = False
        self.robot_ids = []

    def load_ros_params(self) -> list[Robot]:
        """
        Load ROS parameters for robot configuration.
        """
        # Declare and get global parameters
        self.node.declare_parameter('is_mock', False, ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='Whether to use mock robot implementation'
        ))
        self.is_mock = self.node.get_parameter('is_mock').value

        # FYI: An array type parameter cannot defaults to an empty list, even if you declare
        # its type by using ParameterDescriptor as below
        # self.node.declare_parameter(
        #   'robot_ids', [], ParameterDescriptor(ParameterType.PARAMETER_STRING_ARRAY))
        # the default type is deduced as BYTES_ARRAY in all the cases (not STRINGS_ARRAY)
        # ROS2 issue: https://github.com/ros2/rclpy/issues/912
        # A weird workaround is to use get_parameter_or with user defined Parameter of empty list
        self.node.declare_parameter('robot_ids', descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING_ARRAY,
            description='List of robot IDs to initialize'
        ))
        self.robot_ids = self.node.get_parameter_or('robot_ids', Parameter(
            'robot_ids',
            type_=rclpy.Parameter.Type.STRING_ARRAY,
            value=[]
        )).value

        print(f"robot_ids: {self.robot_ids}")

        self.node.get_logger().info(f"is_mock: {self.is_mock}")

        robots: list[Robot] = []

        for robot_id in self.robot_ids:
            self._declare_required_param(f'{robot_id}.type', descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Robot type (e.g. so101_follower)',
            ))
            self._declare_required_param(f'{robot_id}.port', descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Robot port (e.g. /dev/ttyACM0)'
            ))
            self.node.declare_parameter(f'{robot_id}.use_degrees', False, ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description=f'Whether to use degrees for robot {robot_id} (default: radians)'
            ))

            param_name = f'{robot_id}.camera_names'
            self.node.declare_parameter(param_name, descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description=f'List of camera names for robot {robot_id}'
            ))
            camera_names = self.node.get_parameter_or(f'{robot_id}.camera_names', Parameter(
                f'{robot_id}.camera_names',
                type_=rclpy.Parameter.Type.STRING_ARRAY,
                value=[]
            )).value

            for camera_name in camera_names:
                self._declare_required_param(
                    f'{robot_id}.cameras.{camera_name}.index',
                    descriptor=ParameterDescriptor(
                        type=ParameterType.PARAMETER_INTEGER,
                        description='Camera index'
                    )
                )
                self._declare_required_param(
                    f'{robot_id}.cameras.{camera_name}.width',
                    descriptor=ParameterDescriptor(
                        type=ParameterType.PARAMETER_INTEGER,
                        description='Camera width'
                    )
                )
                self._declare_required_param(
                    f'{robot_id}.cameras.{camera_name}.height',
                    descriptor=ParameterDescriptor(
                        type=ParameterType.PARAMETER_INTEGER,
                        description='Camera height'
                    )
                )
                self.node.declare_parameter(
                    f'{robot_id}.cameras.{camera_name}.fps',
                    30,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_INTEGER,
                        description='Camera FPS'
                    )
                )

            robot_type = self.node.get_parameter(f'{robot_id}.type').value

            robots.append(self._make_robot(robot_type, robot_id))

        return robots

    def _declare_required_param(self, param_name: str, descriptor: ParameterDescriptor):
        """
        Validate required ROS parameters.
        """
        param = self.node.declare_parameter(param_name, descriptor=descriptor)
        if param.type_ == param.Type.NOT_SET:
            raise ValueError(f"Required parameter '{param_name}' is not set.")


    def _make_robot(self, robot_type: str, robot_id: str) -> Robot:
        """
        Factory method to create robot instances based on type.
        """
        if robot_type == 'so101_follower':
            return self._make_so101_follower(robot_id)
        raise ValueError(f"Unsupported robot type: {robot_type}")


    def _make_so101_follower(self, robot_id: str) -> Robot:
        """
        Create SO101Follower robot instance from ROS parameters.
        """
        port = self.node.get_parameter(f'{robot_id}.port').value
        camera_names = self.node.get_parameter_or(f'{robot_id}.camera_names', Parameter(
            f'{robot_id}.camera_names',
            type_=rclpy.Parameter.Type.STRING_ARRAY,
            value=[]
        )).value
        cameras_configs = {}

        for camera_name in camera_names:
            index = self.node.get_parameter(f'{robot_id}.cameras.{camera_name}.index').value
            width = self.node.get_parameter(f'{robot_id}.cameras.{camera_name}.width').value
            height = self.node.get_parameter(f'{robot_id}.cameras.{camera_name}.height').value
            fps = self.node.get_parameter(f'{robot_id}.cameras.{camera_name}.fps').value

            camera_config = OpenCVCameraConfig(
                index_or_path=index,
                width=width,
                height=height,
                fps=fps
            )
            cameras_configs[camera_name] = camera_config

        use_degrees = self.node.get_parameter(f'{robot_id}.use_degrees').value

        config = SO101FollowerConfig(
            port=port,
            id=robot_id,
            cameras=cameras_configs,
            use_degrees=use_degrees
        )

        if self.is_mock:
            return SO101FollowerMock(config)
        return SO101Follower(config)
