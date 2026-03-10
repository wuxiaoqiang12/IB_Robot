"""Control system launch builders.

This module handles:
- ros2_control node generation
- Controller spawner creation
- Joint configuration validation (delegates to utils.py)
"""

from pathlib import Path
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue

from robot_config.utils import resolve_ros_path, parse_bool, validate_joint_config


def generate_controller_spawners(controller_names, use_sim=True, controller_manager_name="controller_manager"):
    """Generate controller spawner nodes.

    Args:
        controller_names: List of controller names to spawn
        use_sim: Simulation mode (affects timeout and use_sim_time)
        controller_manager_name: Name of controller manager service

    Returns:
        List of Node actions for controller spawners
    """
    is_sim = parse_bool(use_sim, default=True)

    nodes = []
    timeout = 30 if is_sim else 10

    for controller_name in controller_names:
        nodes.append(Node(
            package="controller_manager",
            executable="spawner",
            name=f"spawner_{controller_name}",
            parameters=[
                {"use_sim_time": is_sim}
            ],
            arguments=[
                controller_name,
                "--controller-manager",
                f"/{controller_manager_name}",
                "--controller-manager-timeout",
                str(timeout),
            ],
            output="screen",
        ))

    return nodes


def generate_ros2_control_nodes(robot_config, use_sim, auto_start_controllers='true'):
    """Generate ros2_control nodes from configuration.

    Args:
        robot_config: Robot configuration dict
        use_sim: Simulation mode flag (string or bool)
        auto_start_controllers: Whether to automatically start controllers (string or bool)

    Returns:
        Tuple: (List of Node actions, Dictionary of spawner nodes)
    """
    is_sim = parse_bool(use_sim, default=False)
    is_auto_start = parse_bool(auto_start_controllers, default=True)

    nodes = []
    spawners_dict = {}
    ros2_control_config = robot_config.get("ros2_control")

    if not ros2_control_config:
        print("[robot_config] No ros2_control configuration found")
        return nodes, spawners_dict

    print("[robot_config] Creating ros2_control nodes")

    # Validate joint configuration
    validate_joint_config(robot_config)

    # Get URDF path
    urdf_path = ros2_control_config.get("urdf_path")
    if not urdf_path:
        print("[robot_config] WARNING: No urdf_path specified")
        return nodes, spawners_dict

    urdf_path = resolve_ros_path(urdf_path)
    print(f"[robot_config] URDF path: {urdf_path}")

    if not Path(urdf_path).exists():
        print(f"[robot_config] WARNING: URDF file not found at {urdf_path}")
        return nodes, spawners_dict

    # Get hardware parameters
    port = ros2_control_config.get("port", "/dev/ttyACM0")
    calib_file = resolve_ros_path(ros2_control_config.get("calib_file", ""))
    
    # Handle reset_positions
    import json
    reset_positions_dict = ros2_control_config.get("reset_positions", {})
    reset_positions_json = json.dumps(reset_positions_dict)

    # Generate robot_description using xacro
    xacro_args = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        urdf_path,
        " ",
        "use_sim:=",
        "true" if is_sim else "false",
        " ",
        "port:=",
        port,
        " ",
        "calib_file:=",
        calib_file,
        " ",
        "reset_positions:=",
        f"'{reset_positions_json}'",
    ]

    robot_description_content = ParameterValue(
        Command(xacro_args),
        value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    if is_sim:
        robot_description["use_sim_time"] = True

    # Robot State Publisher
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    ))

    # Get control mode configuration
    control_mode_name = robot_config.get("default_control_mode", "model_inference")
    control_modes = robot_config.get("control_modes", {})

    if control_modes:
        if control_mode_name not in control_modes:
            available_modes = list(control_modes.keys())
            print(f"[robot_config] ERROR: Control mode '{control_mode_name}' not found")
            print(f"[robot_config] Available modes: {available_modes}")
            if available_modes:
                control_mode_name = available_modes[0]

        if control_mode_name:
            mode_config = control_modes[control_mode_name]
            controller_names = mode_config.get("controllers", [])
            mode_description = mode_config.get("description", "No description")
            print(f"[robot_config] Using control mode: {control_mode_name}")
            print(f"[robot_config]   Description: {mode_description}")
            print(f"[robot_config]   Controllers: {controller_names}")
        else:
            controller_names = []
    else:
        controller_names = ros2_control_config.get("controllers", [])

    controllers_config = resolve_ros_path(ros2_control_config.get("controllers_config"))

    if not is_sim:
        # Real hardware mode
        print("[robot_config] Real hardware mode")

        if controllers_config and Path(controllers_config).exists():
            print(f"[robot_config] Controllers config: {controllers_config}")

            nodes.append(Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[controllers_config],
                remappings=[
                    ("~/robot_description", "/robot_description"),
                ],
                output="screen",
            ))

            if is_auto_start and controller_names:
                spawners = generate_controller_spawners(controller_names, use_sim=False)
                nodes.extend(spawners)
                # Store spawners in dict
                for i, name in enumerate(controller_names):
                    if i < len(spawners):
                        spawners_dict[name] = spawners[i]
    else:
        # Simulation mode
        print("[robot_config] Simulation mode: Gazebo provides controller_manager")
        print(f"[robot_config] Controllers to spawn: {controller_names}")

        if is_auto_start and controller_names:
            spawners = generate_controller_spawners(controller_names, use_sim=True)
            nodes.extend(spawners)
            print(f"[robot_config] Added {len(spawners)} controller spawners")
            # Store spawners in dict
            for i, name in enumerate(controller_names):
                if i < len(spawners):
                    spawners_dict[name] = spawners[i]

    return nodes, spawners_dict
