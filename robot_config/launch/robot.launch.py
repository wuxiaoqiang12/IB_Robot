"""Main robot launch file for robot_config.

This launch file loads robot configuration from YAML and dynamically generates:
- ros2_control hardware interface and controllers
- Robot state publisher
- Camera drivers (usb_cam, realsense2_camera)
- Static TF publishers for camera frames

Controllers are automatically spawned in both simulation and real hardware modes:
- Simulation mode: Uses Gazebo's gz_ros2_control plugin for controller_manager
- Hardware mode: Starts ros2_control_node for controller_manager

Usage:
    ros2 launch robot_config robot.launch.py robot_config:=test_cam use_sim:=false
    ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm use_sim:=false
    ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm use_sim:=true
    ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm use_sim:=true auto_start_controllers:=false

Launch Arguments:
    robot_config: Robot configuration name (default: test_cam)
    config_path: Optional full path to robot config file
    use_sim: Use simulation mode (default: false)
    auto_start_controllers: Automatically spawn controllers (default: true, set to false for debugging)
"""

import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)

# Import utility functions
from robot_config.utils import resolve_ros_path, parse_bool

# Import node generators from launch_builders modules
from robot_config.launch_builders.control import generate_ros2_control_nodes
from robot_config.launch_builders.perception import generate_camera_nodes, generate_tf_nodes
from robot_config.launch_builders.simulation import generate_gazebo_nodes
from robot_config.launch_builders.execution import generate_action_dispatcher_node


def load_robot_config(robot_config_name, config_path_override=None):
    """Load robot configuration from YAML file.

    Args:
        robot_config_name: Robot configuration name
        config_path_override: Optional full path to config file

    Returns:
        Robot configuration dict
    """
    # Get package share directory
    try:
        robot_config_share = get_package_share_directory("robot_config")
    except:
        robot_config_share = str(Path(__file__).parent.parent)

    # Determine config file path
    if config_path_override:
        config_path = Path(config_path_override)
    else:
        config_path = Path(robot_config_share) / "config" / "robots" / f"{robot_config_name}.yaml"

    print(f"[robot_config] Loading config from: {config_path}")
    print(f"[robot_config] Config exists: {config_path.exists()}")

    # Load YAML
    with open(config_path, "r") as f:
        data = yaml.safe_load(f)

    robot_config = data.get("robot", {})
    print(f"[robot_config] Loaded robot: {robot_config.get('name', 'UNKNOWN')}")
    print(f"[robot_config] Peripherals: {len(robot_config.get('peripherals', []))}")

    return robot_config


def launch_setup(context, *args, **kwargs):
    """Launch setup function that generates all nodes.

    This is the "orchestrator" that:
    1. Loads and normalizes all parameters
    2. Calls each builder module to generate nodes
    3. Returns the combined actions list

    Args:
        context: Launch context

    Returns:
        List of launch actions
    """
    actions = []

    # ========== 1. Get and normalize launch parameters ==========
    robot_config_name = context.launch_configurations.get('robot_config', 'test_cam')
    config_path_override = context.launch_configurations.get('config_path', '')
    use_sim_str = context.launch_configurations.get('use_sim', 'false')
    auto_start_controllers = context.launch_configurations.get('auto_start_controllers', 'true')
    control_mode_override = context.launch_configurations.get('control_mode', '')

    # Normalize use_sim to boolean
    use_sim = parse_bool(use_sim_str, default=False)

    print(f"[robot_config] ========== Launch Parameters ==========")
    print(f"[robot_config] robot_config: {robot_config_name}")
    print(f"[robot_config] config_path: {config_path_override if config_path_override else '(none)'}")
    print(f"[robot_config] use_sim: {use_sim} (from '{use_sim_str}')")
    print(f"[robot_config] auto_start_controllers: {auto_start_controllers}")
    print(f"[robot_config] control_mode: {control_mode_override if control_mode_override else '(from config)'}")

    # ========== 2. Load robot configuration ==========
    try:
        robot_config = load_robot_config(
            robot_config_name,
            config_path_override if config_path_override else None
        )
    except Exception as e:
        print(f"[robot_config] ERROR loading config: {e}")
        raise

    # ========== 3. Apply control mode override ==========
    if control_mode_override:
        original_mode = robot_config.get('default_control_mode', 'unknown')
        robot_config['default_control_mode'] = control_mode_override
        print(f"[robot_config] Control mode override: {original_mode} -> {control_mode_override}")
    else:
        print(f"[robot_config] Using default control mode: {robot_config.get('default_control_mode', 'teleop_act')}")

    # ========== 4. Generate Control System Nodes ==========
    print(f"[robot_config] ========== Generating Control Nodes ==========")
    try:
        control_nodes = generate_ros2_control_nodes(robot_config, use_sim, auto_start_controllers)
        actions.extend(control_nodes)
        print(f"[robot_config] Added {len(control_nodes)} control nodes")
    except Exception as e:
        print(f"[robot_config] ERROR generating control nodes: {e}")
        raise

    # ========== 5. Generate Simulation Nodes (only in simulation mode) ==========
    if use_sim:
        print(f"[robot_config] ========== Generating Simulation Nodes ==========")
        try:
            gazebo_nodes = generate_gazebo_nodes(robot_config)
            actions.extend(gazebo_nodes)
            print(f"[robot_config] Added {len(gazebo_nodes)} simulation nodes")
        except Exception as e:
            print(f"[robot_config] ERROR generating simulation nodes: {e}")
            raise

    # ========== 6. Generate Perception Nodes ==========
    print(f"[robot_config] ========== Generating Perception Nodes ==========")
    try:
        # Camera nodes
        camera_nodes = generate_camera_nodes(robot_config, use_sim)
        actions.extend(camera_nodes)
        print(f"[robot_config] Added {len(camera_nodes)} camera nodes")

        # Static TF publishers
        tf_nodes = generate_tf_nodes(robot_config)
        actions.extend(tf_nodes)
        print(f"[robot_config] Added {len(tf_nodes)} TF nodes")
    except Exception as e:
        print(f"[robot_config] ERROR generating perception nodes: {e}")
        raise

    # ========== 7. Generate Execution Nodes ==========
    print(f"[robot_config] ========== Generating Execution Nodes ==========")
    try:
        action_dispatcher_node = generate_action_dispatcher_node(robot_config, use_sim)
        actions.append(action_dispatcher_node)
        print(f"[robot_config] Added action dispatcher node")
    except Exception as e:
        print(f"[robot_config] ERROR generating execution nodes: {e}")
        raise

    print(f"[robot_config] ========== Total nodes to launch: {len(actions)} ==========")

    return actions


def generate_launch_description():
    """Generate launch description for robot system."""
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_config",
            default_value="test_cam",
            description="Robot configuration name (without .yaml extension)",
        ),
        DeclareLaunchArgument(
            "config_path",
            default_value="",
            description="Optional: Full path to robot config file (overrides robot_config)",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Use simulation mode (skip camera nodes)",
        ),
        DeclareLaunchArgument(
            "auto_start_controllers",
            default_value="true",
            description="Automatically spawn controllers (set to false for debugging)",
        ),
        DeclareLaunchArgument(
            "control_mode",
            default_value="",
            description="Override control mode from YAML (teleop_act or moveit_planning). If empty, uses default_control_mode from config file",
        ),
        OpaqueFunction(function=launch_setup),
    ])
