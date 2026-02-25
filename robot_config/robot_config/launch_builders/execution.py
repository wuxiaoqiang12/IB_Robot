"""Execution system launch builders.

This module handles:
- Action dispatcher node
- Inference service nodes (ACT, pi0, etc.)
- Control mode integration
"""

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from robot_config.utils import parse_bool


def generate_action_dispatcher_node(robot_config, use_sim=False):
    """Generate action dispatcher node based on robot configuration.

    The action_dispatcher node provides unified action execution for both
    teleop_act (TopicExecutor) and moveit_planning (ActionExecutor) modes.

    Args:
        robot_config: Robot configuration dictionary
        use_sim: Simulation mode flag

    Returns:
        Node action for action_dispatcher
    """
    is_sim = parse_bool(use_sim, default=False)

    # Get control mode
    control_mode_name = robot_config.get("default_control_mode", "teleop_act")

    # Map control mode to executor mode
    # teleop_act -> TopicExecutor (position control)
    # moveit_planning -> ActionExecutor (trajectory control)
    executor_mode = control_mode_name  # Use the same name for simplicity

    # Get robot name
    robot_name = robot_config.get("name", "so101")

    # Get joint names from robot config
    robot_joints_config = robot_config.get("joints", {})
    all_joints = robot_joints_config.get("all", ["1", "2", "3", "4", "5", "6"])

    print(f"[robot_config] Creating action_dispatcher node")
    print(f"[robot_config]   executor_mode: {executor_mode}")
    print(f"[robot_config]   enable_dual_mode: True")
    print(f"[robot_config]   robot_name: {robot_name}")
    print(f"[robot_config]   use_sim_time: {is_sim}")

    # Create action_dispatcher node
    action_dispatcher_node = Node(
        package="action_dispatch",
        executable="action_dispatcher_node",
        name="action_dispatcher",
        parameters=[{
            # Dual-mode executor settings
            "enable_dual_mode": True,
            "executor_mode": executor_mode,

            # Robot configuration
            "robot_name": robot_name,
            "joint_names": all_joints,

            # Queue settings
            "queue_size": 100,
            "watermark_threshold": 20,
            "min_queue_size": 10,

            # Control settings
            "control_frequency": 100.0,
            "control_mode": control_mode_name,

            # Interpolation settings
            "interpolation_enabled": True,
            "interpolation_step": 0.1,
            "max_interpolation_time": 2.0,

            # Safety settings
            "on_inference_failure": "hold",
            "on_queue_exhausted": "hold",
            "max_inference_timeout": 1.0,
            "max_retry_attempts": 3,
            "retry_backoff_base": 0.5,
            "stale_obs_threshold_ms": 500,
            "exhaustion_timeout": 2.0,

            # Topics
            "joint_state_topic": "/joint_states",
            "dispatch_action_topic": "/action_dispatch/dispatch_action",

            # Inference settings
            "inference_action_server": "/inference/dispatch",
            "inference_prompt": "",

            # Simulation time
            "use_sim_time": is_sim,
        }],
        output="screen",
    )

    return action_dispatcher_node


def generate_execution_nodes(robot_config, control_mode='teleop_act', with_inference=False):
    """Generate inference and action dispatch nodes.

    Args:
        robot_config: Robot configuration dict
        control_mode: Control mode (teleop_act, moveit_planning, etc.)
        with_inference: Whether to launch inference service

    Returns:
        List of Node actions for execution system
    """
    from robot_config.utils import parse_bool

    nodes = []
    should_infer = parse_bool(with_inference, default=False)

    if not should_infer:
        return nodes

    # Get control mode configuration
    control_modes = robot_config.get("control_modes", {})

    if control_mode not in control_modes:
        print(f"[robot_config] WARNING: Control mode '{control_mode}' not found")
        return nodes

    mode_config = control_modes[control_mode]
    inference_config = mode_config.get("inference", {})

    if not inference_config.get("enabled", False):
        print(f"[robot_config] Inference not enabled for mode '{control_mode}'")
        return nodes

    print(f"[robot_config] Creating inference nodes for mode '{control_mode}'")

    # Get model configuration
    model_name = inference_config.get("model")
    models_config = robot_config.get("models", {})

    if model_name not in models_config:
        print(f"[robot_config] ERROR: Model '{model_name}' not found in robot_config")
        return nodes

    model_cfg = models_config[model_name]
    model_path = model_cfg.get("path", "")

    if not model_path:
        print(f"[robot_config] ERROR: Model path not specified for '{model_name}'")
        return nodes

    print(f"[robot_config] Model: {model_name}")
    print(f"[robot_config]   Path: {model_path}")

    # Generate contract with normalization metadata
    # TODO: Call contract generator here
    # For now, use existing contract path
    contract_name = f"{robot_config.get('name', 'robot')}_act"
    contract_path = PathJoinSubstitution([
        FindPackageShare('inference_service'),
        'config/contracts',
        f'{contract_name}.yaml'
    ])

    # Launch inference service
    action_server = inference_config.get("action_server", "DispatchInfer")

    nodes.append(Node(
        package='inference_service',
        executable='lerobot_policy_node',
        name='act_inference_node',
        parameters=[{
            'model_path': model_path,
            'contract_path': contract_path,
            'passive_mode': True,
            'device': 'auto',
        }],
        output='screen',
    ))

    print(f"[robot_config]   Action server: {action_server}")

    # Launch action dispatcher
    executor_mode = mode_config.get("executor", "topic")
    enable_dual_mode = executor_mode == "topic"

    nodes.append(Node(
        package='action_dispatch',
        executable='action_dispatcher_node',
        name='action_dispatcher',
        parameters=[{
            'robot_name': robot_config.get('name', 'robot'),
            'robot_config': robot_config.get('name', 'robot'),
            'executor_mode': executor_mode,
            'enable_dual_mode': enable_dual_mode,
            'inference_action_server': action_server,
            'queue_size': 100,
            'watermark_threshold': 30,
            'control_frequency': 100.0,
        }],
        output='screen',
    ))

    print(f"[robot_config]   Executor: {executor_mode}")
    print(f"[robot_config]   Dual mode: {enable_dual_mode}")

    return nodes
