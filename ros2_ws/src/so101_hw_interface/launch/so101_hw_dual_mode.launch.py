from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('so101_hw_interface')

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation (Gazebo) or real hardware')

    use_cpp_plugin_arg = DeclareLaunchArgument(
        'use_cpp_plugin',
        default_value='false',
        description='Use C++ hardware plugin (true) or topic-based ros2_control (false)')

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Port for the Feetech motor bus')

    reset_positions_arg = DeclareLaunchArgument(
        'reset_positions',
        default_value='',
        description='Optional JSON string with reset positions for each joint (e.g., \'{"1": -1.17, "2": -1.73, ...}\'). If empty, preserves current positions.')

    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("lerobot_description"),
                "urdf",
                "so101.urdf.xacro",
            ]),
            " use_sim:=", LaunchConfiguration('use_sim'),
            " use_cpp_plugin:=", LaunchConfiguration('use_cpp_plugin'),
            " port:=", LaunchConfiguration('port'),
            " reset_positions:=", LaunchConfiguration('reset_positions'),
        ]),
        value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    # Motor bridge node - only for topic-based mode
    motor_bridge_node = Node(
        package="so101_hw_interface",
        executable="so101_motor_bridge",
        name="so101_motor_bridge",
        output="screen",
        parameters=[{'port': LaunchConfiguration('port')}],
        condition=UnlessCondition(LaunchConfiguration('use_cpp_plugin'))
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            os.path.join(pkg_share, "config", "so101_controllers.yaml"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        use_sim_arg,
        use_cpp_plugin_arg,
        port_arg,
        reset_positions_arg,
        robot_state_publisher,
        controller_manager,
        motor_bridge_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
    ])
