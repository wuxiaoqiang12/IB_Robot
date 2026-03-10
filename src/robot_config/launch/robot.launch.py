"""Main robot launch file for robot_config.

This launch file loads robot configuration from YAML and dynamically generates:
- ros2_control hardware interface and controllers
- Robot state publisher
- Camera drivers (usb_cam, realsense2_camera)
- Static TF publishers for camera frames
- Inference service and action dispatcher (optional, auto-detected)
- MoveIt motion planning (optional, auto-detected)

Controllers are automatically spawned in both simulation and real hardware modes:
- Simulation mode: Uses Gazebo's gz_ros2_control plugin for controller_manager
- Hardware mode: Starts ros2_control_node for controller_manager

**CRITICAL**: This workspace uses ROS_DOMAIN_ID=<ID> to avoid conflicts with other ROS 2 systems.
Always set this before launching:
```bash
export ROS_DOMAIN_ID=<ID>
```

Usage:
    # Basic simulation
    ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm use_sim:=true

    # Model inference mode (auto-detected)
    ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm use_sim:=true control_mode:=model_inference

    # Teleop mode (human teleoperation)
    ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm control_mode:=teleop record:=true

    # MoveIt planning mode (auto-detected, with RViz)
    ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm control_mode:=moveit_planning use_sim:=true

    # MoveIt mode without RViz (headless)
    ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm control_mode:=moveit_planning use_sim:=true moveit_display:=false

    # Real hardware
    ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm use_sim:=false

    # Override auto-detection
    ros2 launch robot_config robot.launch.py control_mode:=model_inference with_inference:=true use_sim:=true

**Cleanup**: If you encounter "Controller already loaded" errors, run:
```bash
./scripts/cleanup_ros.sh
```

Launch Arguments:
    robot_config: Robot configuration name (default: test_cam)
    config_path: Optional full path to robot config file
    use_sim: Use simulation mode (default: false)
    auto_start_controllers: Automatically spawn controllers (default: true, set to false for debugging)
    control_mode: Override control mode from YAML (teleop, model_inference, or moveit_planning). If empty, uses default_control_mode from config file
    with_inference: Enable inference pipeline. If empty, auto-detects from control mode config
    with_moveit: Enable MoveIt motion planning. If empty, auto-detects from control mode name
    moveit_display: Launch RViz for MoveIt visualization (default: true, only used if MoveIt is enabled)
    record: Enable automatic rosbag recording (default: false, auto-discovers topics from config)
"""

import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit

# Import utility functions
from robot_config.utils import resolve_ros_path, parse_bool

# Import node generators from launch_builders modules
from robot_config.launch_builders.control import generate_ros2_control_nodes
from robot_config.launch_builders.perception import generate_camera_nodes, generate_tf_nodes
from robot_config.launch_builders.simulation import generate_gazebo_nodes
from robot_config.launch_builders.execution import generate_execution_nodes
from robot_config.launch_builders.teleop import generate_teleop_nodes


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
        robot_config['default_control_mode'] = control_mode_override
    
    active_control_mode = robot_config.get('default_control_mode', 'model_inference')
    print(f"[robot_config] Active control mode: {active_control_mode}")

    # Determine with_inference flag globally
    with_inference_str = context.launch_configurations.get('with_inference', '')
    if with_inference_str != '':
        with_inference = parse_bool(with_inference_str, default=False)
    else:
        control_mode_config = robot_config.get('control_modes', {}).get(active_control_mode, {})
        with_inference = control_mode_config.get('inference', {}).get('enabled', False)

    # Force disable inference in teleop mode if not explicitly overridden
    if active_control_mode == 'teleop' and with_inference_str == '':
        with_inference = False
        print("[robot_config] Teleop mode: forcing with_inference=False")

    print(f"[robot_config] Final with_inference={with_inference}")

    # ========== 4. Generate Control System Nodes ==========
    print(f"[robot_config] ========== Generating Control Nodes ==========")
    try:
        control_nodes, spawners_dict = generate_ros2_control_nodes(robot_config, use_sim, auto_start_controllers)
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
        # Camera nodes (Physical drivers)
        camera_nodes = generate_camera_nodes(robot_config, use_sim)
        actions.extend(camera_nodes)
        print(f"[robot_config] Added {len(camera_nodes)} camera nodes")

        # Virtual camera relay nodes (Topic tools)
        from robot_config.launch_builders.perception import generate_virtual_camera_relays
        virtual_nodes = generate_virtual_camera_relays(robot_config)
        actions.extend(virtual_nodes)
        if virtual_nodes:
            print(f"[robot_config] Added {len(virtual_nodes)} virtual camera relays")

        # Static TF publishers
        tf_nodes = generate_tf_nodes(robot_config)
        actions.extend(tf_nodes)
        print(f"[robot_config] Added {len(tf_nodes)} TF nodes")
    except Exception as e:
        print(f"[robot_config] ERROR generating perception nodes: {e}")
        raise

    # ========== 7. Generate Teleop Nodes (if in teleop mode) ==========
    print(f"[robot_config] ========== Checking Teleop Mode ==========")
    try:
        # Check if teleop mode is enabled
        if active_control_mode == 'teleop':
            print(f"[robot_config] TELEOP MODE DETECTED")

            # Check if teleoperation is configured
            teleop_config = robot_config.get('teleoperation', {})
            if not teleop_config.get('enabled', False):
                print(f"[robot_config] WARNING: Teleop mode requested but teleoperation config not found")
            else:
                # Generate teleop nodes
                teleop_nodes = generate_teleop_nodes(robot_config)
                actions.extend(teleop_nodes)
                print(f"[robot_config] Added {len(teleop_nodes)} teleop nodes")
        else:
            print(f"[robot_config] Skipping teleop nodes (mode is {active_control_mode})")
    except Exception as e:
        print(f"[robot_config] ERROR checking teleop mode: {e}")
        raise

    # ========== 8. Generate Execution Nodes ==========
    print(f"[robot_config] ========== Generating Execution Nodes ==========")
    try:
        if with_inference:
            execution_nodes = generate_execution_nodes(robot_config, active_control_mode, use_sim)
            actions.extend(execution_nodes)
            print(f"[robot_config] Added {len(execution_nodes)} execution nodes")
        else:
            print(f"[robot_config] Skipping execution nodes")
    except Exception as e:
        print(f"[robot_config] ERROR generating execution nodes: {e}")
        raise

    # ========== 9. Generate MoveIt Nodes ==========
    try:
        # Determine with_moveit flag
        with_moveit_str = context.launch_configurations.get('with_moveit', '')
        moveit_display = parse_bool(context.launch_configurations.get('moveit_display', 'true'), default=True)

        if with_moveit_str != '':
            with_moveit = parse_bool(with_moveit_str, default=False)
        else:
            # Auto-detect: true if mode is 'moveit_planning' or contains 'moveit'
            with_moveit = 'moveit' in active_control_mode.lower()
        
        print(f"[robot_config] with_moveit={with_moveit}")

        if with_moveit:
            from robot_config.launch_builders.moveit import generate_moveit_nodes
            moveit_nodes = generate_moveit_nodes(robot_config, active_control_mode, use_sim, moveit_display)
            
            # Find the joint_state_broadcaster spawner to use as a trigger
            jsb_spawner = spawners_dict.get('joint_state_broadcaster')
            
            if jsb_spawner:
                print(f"[robot_config] Delaying MoveIt nodes until joint_state_broadcaster_spawner exits...")
                actions.append(RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=jsb_spawner,
                        on_exit=moveit_nodes,
                    )
                ))
            else:
                print(f"[robot_config] Warning: joint_state_broadcaster spawner not found in spawners_dict, launching MoveIt immediately")
                actions.extend(moveit_nodes)
        else:
            print(f"[robot_config] Skipping MoveIt nodes")
    except Exception as e:
        print(f"[robot_config] ERROR generating MoveIt nodes: {e}")
        print(f"[robot_config] Continuing without MoveIt...")

    # ========== 9. Automatic Recording (if record:=true) ==========
    try:
        record_str = context.launch_configurations.get('record', 'false')
        record_enabled = parse_bool(record_str, default=False)

        if record_enabled:
            from launch.actions import ExecuteProcess
            from datetime import datetime

            print(f"[robot_config] ========== Setting up Automatic Recording ==========")

            # Auto-discover topics to record
            topics = ['/joint_states']

            # Add controller command topics
            topics.append('/arm_position_controller/commands')
            topics.append('/gripper_position_controller/commands')

            # Add diagnostics
            topics.append('/diagnostics')

            # Add camera topics from peripherals
            peripherals = robot_config.get('peripherals', [])
            for peripheral in peripherals:
                if peripheral.get('type') == 'camera':
                    name = peripheral.get('name', 'camera')
                    # Add common camera topics
                    topics.append(f'/camera/{name}/image_raw')
                    topics.append(f'/camera/{name}/camera_info')

            # Generate filename with timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            robot_name = robot_config.get('name', 'robot')
            output_file = f"~/rosbag/{robot_name}_{timestamp}.mcap"

            # Expand ~ to actual home directory
            output_file = str(Path(output_file).expanduser())

            print(f"[robot_config] Recording {len(topics)} topics to: {output_file}")
            print(f"[robot_config] Topics: {topics}")

            # Create recording action
            recording_action = ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-o', output_file] + topics,
                output='screen'
            )

            actions.append(recording_action)
            print(f"[robot_config] ✓ Recording action added")
        else:
            print(f"[robot_config] Recording disabled (record:={record_str})")
    except Exception as e:
        print(f"[robot_config] ERROR setting up recording: {e}")
        print(f"[robot_config] Continuing without recording...")

    print(f"[robot_config] ========== Total nodes to launch: {len(actions)} ==========")

    return actions


def generate_launch_description():
    """Generate launch description for robot system."""
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_config",
            default_value="so101_single_arm",
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
            description="Override control mode from YAML (teleop, model_inference, or moveit_planning). If empty, uses default_control_mode from config file",
        ),
        DeclareLaunchArgument(
            "with_inference",
            default_value="",
            description="Enable full execution pipeline (inference + dispatcher). If empty, auto-detects from control mode config",
        ),
        DeclareLaunchArgument(
            "with_moveit",
            default_value="",
            description="Enable MoveIt motion planning. If empty, auto-detects from control mode config",
        ),
        DeclareLaunchArgument(
            "moveit_display",
            default_value="true",
            description="Launch RViz for MoveIt visualization (only used if MoveIt is enabled)",
        ),
        DeclareLaunchArgument(
            "record",
            default_value="false",
            description="Enable automatic rosbag recording (auto-discovers topics from config)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
