# !/usr/bin/env python3
# @Time    : 2025/12/10 16:04
# @Author  : Yida Hao
# @File    : robot_interface.py
# @Description : Robot hardware interface node aggregating all devices.

from concurrent.futures import ThreadPoolExecutor

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, JointState

from robot_interface.ros_args_parser import RosArgsParser

cvbridge = CvBridge()

# TODO: 将该前缀放到整个 workspace 的共享目录中
# 用来让其他节点识别到这是一个摄像头话题
CAMERA_PREFIX = "/camera/"

# Frequency to read motors
READING_FREQUENCY = 30  # Hz

class RobotInterface(Node):
    def __init__(self):
        super().__init__('robot_interface')
        ros_args_parser = RosArgsParser(self)

        self.robots = ros_args_parser.load_ros_params()

        for robot in self.robots:
            robot.connect()
            self.get_logger().info(f"Connected robot: {robot.id}")

        self.init_joint_state_publisher()
        self.init_joint_command_subscriber()
        self.init_camera_publishers()
        self.init_robot_reading_loop()


    def init_robot_reading_loop(self):
        self.timer = self.create_timer(1.0 / READING_FREQUENCY, self.read_robot)

    def init_joint_state_publisher(self):
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            qos
        )

    def init_joint_command_subscriber(self):
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.joint_command_subscriber = self.create_subscription(
            JointState,
            'joint_commands',
            self.joint_command_callback,
            qos
        )

    def joint_command_callback(self, msg: JointState):
        for robot in self.robots:
            action_dict = {}

            for name, position in zip(msg.name, msg.position):
                if not name.startswith(robot.id):
                    return

                # hardware motor name need to have '.pos' suffix
                motor_name = f'{name.removeprefix(f"{robot.id}:")}.pos'
                action_dict[motor_name] = position

            robot.send_action(action_dict)

    def init_camera_publishers(self):
        self.cam_publishers = {}

        for robot in self.robots:
            if not hasattr(robot, "cameras") or len(robot.cameras) == 0:
                self.get_logger().warning(f"Robot {robot.id} has no cameras")
                continue

            for camera_name in robot.cameras:
                if camera_name in self.cam_publishers:
                    self.get_logger().error(f"Camera {camera_name} already exists")
                    return
                topic_name = CAMERA_PREFIX + camera_name
                self.cam_publishers[camera_name] = self.create_publisher(Image, topic_name, 10)

    def read_robot(self):
        with ThreadPoolExecutor() as executor:
            futures = [executor.submit(robot.get_observation) for robot in self.robots]
            obs_list = [future.result() for future in futures]

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = []
        joint_state.position = []

        for i, obs in enumerate(obs_list):
            joint_names = [
                f'{self.robots[i].id}:{key.removesuffix(".pos")}'
                for key in obs
                if key.endswith(".pos")
            ]

            joint_state.name.extend(joint_names)
            joint_state.position.extend([obs[key] for key in obs if key.endswith(".pos")])
            self.joint_state_publisher.publish(joint_state)
            for camera_name, publisher in self.cam_publishers.items():
                if camera_name not in obs:
                    continue

                image = obs[camera_name]

                # FYI: 预训练的 ACTPolicy 需要的图片大小是 320x240，后续可能需要通过 config
                # 或者 param 来配置. 在硬件 config 中配置的图片分辨率与模型需要的不同时,
                # 需要在这里 resize, 否则在图片不必要地大的情况下会导致 ros 通信耗时很久
                # image = cv2.resize(image, (320, 240), interpolation=cv2.INTER_LINEAR)

                img_msg = cvbridge.cv2_to_imgmsg(image, encoding='rgb8')
                img_msg.header.stamp = self.get_clock().now().to_msg()
                publisher.publish(img_msg)

    def cleanup(self):
        for robot in self.robots:
            robot.disconnect()
            self.get_logger().info(f"Disconnected robot: {robot.id}")


