import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition  # <-- 1. 导入 IfCondition

from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # create a runtime lauch argument
    is_sim_arg = DeclareLaunchArgument(name="is_sim", default_value="True")

    # 2. 声明 "display" 启动参数，默认为 True
    display_arg = DeclareLaunchArgument(
        name="display",
        default_value="True",
        description="Launch RViz for visualization if True"
    )

    # get the argument value at runtime
    is_sim = LaunchConfiguration("is_sim")
    display = LaunchConfiguration("display")  # 3. 获取 "display" 的值

    # URDF
    robot_description_dir = get_package_share_directory("robot_description")
    so101_urdf_path = os.path.join(robot_description_dir, "urdf", "lerobot", "so101", "so101.urdf.xacro")

    

    moveit_config = (
            MoveItConfigsBuilder("so101", package_name="robot_moveit")
            .robot_description(file_path=so101_urdf_path)
            .robot_description_semantic(file_path="config/lerobot/so101/so101.srdf")
            .robot_description_kinematics(file_path="config/lerobot/so101/kinematics.yaml")
            .joint_limits(file_path="config/lerobot/so101/joint_limits.yaml")
            .trajectory_execution(file_path="config/lerobot/so101/moveit_controllers.yaml")
            .planning_pipelines(pipelines=["ompl"])
            .to_moveit_configs()
            )

    # Add pilz cartesian limits manually
    pilz_cartesian_limits_path = os.path.join(
        get_package_share_directory("robot_moveit"),
        "config", "lerobot", "so101", "pilz_cartesian_limits.yaml"
    )

    # moveit core
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": is_sim},
            {"publish_robot_description_semantic": True},
            pilz_cartesian_limits_path
        ],
        arguments=["--ros-args", "--log-level", "info"]
    )

    rviz_config_path = os.path.join(get_package_share_directory("robot_moveit"),"config", "lerobot", "so101", "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[moveit_config.robot_description, 
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits],
        condition=IfCondition(display)  # 4. 只有 "display" 为 True 时才启动
    )

    moveit_gateway_node = Node(
        package="robot_moveit",
        executable="moveit_gateway.py",
        name="moveit_gateway",
        output="screen",
        parameters=[
            {"arm_group_name": "arm"},
            {"base_link": "base"},
            {"ee_link": "gripper"},
            {"joint_names": ["1", "2", "3", "4", "5"]},
            {"use_sim_time": is_sim},
        ],
    )

    return LaunchDescription([
        is_sim_arg,
        display_arg,
        move_group_node,
        rviz_node,
        moveit_gateway_node
    ])