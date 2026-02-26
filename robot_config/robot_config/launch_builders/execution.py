"""Execution system launch builders (refactored).

This module handles:
- Inference service node generation (with contract synthesis)
- Action dispatcher node generation
- Automatic parameter binding from robot_config
"""

import os
import sys
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from pathlib import Path

from robot_config.utils import parse_bool
from robot_config.contract_builder import synthesize_contract, get_contract_output_path, save_contract


def generate_inference_node(robot_config, control_mode, use_sim=False):
    """Generate inference service node with auto-synthesized contract.

    This function:
    1. Synthesizes contract from robot_config
    2. Saves contract for debugging and inference_service
    3. Creates inference node with proper parameters

    Args:
        robot_config: Robot configuration dict
        control_mode: Active control mode
        use_sim: Simulation mode flag

    Returns:
        Node action for inference service, or None if not enabled

    Raises:
        ContractSynthesisError: If configuration is architecturally invalid
    """
    is_sim = parse_bool(use_sim, default=False)

    # Get control mode configuration
    control_modes = robot_config.get('control_modes', {})
    if control_mode not in control_modes:
        print(f"[robot_config] WARNING: Control mode '{control_mode}' not found")
        return None

    mode_config = control_modes[control_mode]
    inference_config = mode_config.get('inference', {})

    if not inference_config.get('enabled', False):
        print(f"[robot_config] Inference not enabled for mode '{control_mode}'")
        return None

    print(f"[robot_config] ========== Generating Inference Node ==========")
    print(f"[robot_config] Control mode: {control_mode}")

    # Step 1: Synthesize contract
    contract = synthesize_contract(robot_config, control_mode)
    if not contract:
        print(f"[robot_config] ERROR: Failed to synthesize contract")
        return None

    # Step 2: Save contract (for debugging and inference_service)
    contract_path = get_contract_output_path(robot_config, control_mode)

    try:
        save_contract(contract, contract_path)
    except Exception as e:
        print(f"[robot_config] WARNING: Could not save contract: {e}")

    # Step 3: Get model configuration
    model_name = inference_config['model']
    models = robot_config.get('models', {})
    if model_name not in models:
        print(f"[robot_config] ERROR: Model '{model_name}' not found in config")
        return None

    model_config = models[model_name]

    print(f"[robot_config] Model: {model_name}")
    print(f"[robot_config]   Path: {model_config['path']}")
    print(f"[robot_config]   Policy type: {model_config.get('policy_type', 'unknown')}")
    print(f"[robot_config]   Contract: {contract_path}")

    # Step 4: Prepare environment for inference node (CRITICAL for venv and library loading)
    env = os.environ.copy()
    
    # 1. Inject PYTHONPATH to find robot_config, lerobot and other packages
    workspace_path = os.environ.get('WORKSPACE', os.getcwd())
    # FIX: lerobot package is inside 'src' directory of the library
    lerobot_src = os.path.join(workspace_path, 'libs/lerobot/src')
    
    # Debug information for environment
    print(f"[robot_config] ========== Environment Diagnostics ==========")
    print(f"[robot_config] Workspace: {workspace_path}")
    print(f"[robot_config] Lerobot SRC: {lerobot_src} (Exists: {os.path.exists(lerobot_src)})")
    
    # Get AMENT_PREFIX_PATH to find other built packages
    ament_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
    site_packages_paths = []
    if ament_prefix:
        for path in ament_prefix.split(':'):
            if 'install' in path:
                sp = os.path.join(path, 'lib', 'python3.10', 'site-packages')
                if os.path.exists(sp): site_packages_paths.append(sp)
    
    # Construct new PYTHONPATH
    new_python_paths = []
    if os.path.exists(lerobot_src): 
        new_python_paths.append(lerobot_src)
    
    # Also include the venv site-packages if we are in one
    venv_path = os.environ.get('VIRTUAL_ENV')
    if venv_path:
        venv_site_packages = os.path.join(venv_path, 'lib', f'python{sys.version_info.major}.{sys.version_info.minor}', 'site-packages')
        if os.path.exists(venv_site_packages):
            new_python_paths.append(venv_site_packages)
            print(f"[robot_config] Venv site-packages added: {venv_site_packages}")

    new_python_paths.extend(site_packages_paths)
    
    if 'PYTHONPATH' in env:
        env['PYTHONPATH'] = f"{':'.join(new_python_paths)}:{env['PYTHONPATH']}"
    else:
        env['PYTHONPATH'] = ':'.join(new_python_paths)

    # 2. Inject LD_LIBRARY_PATH to find librcl_action.so and other C libraries
    ros_lib_path = "/opt/ros/humble/lib"
    if 'LD_LIBRARY_PATH' in env:
        if ros_lib_path not in env['LD_LIBRARY_PATH']:
            env['LD_LIBRARY_PATH'] = f"{ros_lib_path}:{env['LD_LIBRARY_PATH']}"
    else:
        env['LD_LIBRARY_PATH'] = ros_lib_path

    print(f"[robot_config] Final PYTHONPATH: {env.get('PYTHONPATH', 'NOT_SET')[:200]}...")
    print(f"[robot_config] Final LD_LIBRARY_PATH: {env.get('LD_LIBRARY_PATH', 'NOT_SET')}")
    print(f"[robot_config] ===============================================")

    # Step 5: Create inference node with enriched environment
    node_params = {
        'checkpoint': model_config['path'], # Changed from model_path to checkpoint
        'contract_path': str(contract_path),
        'passive_mode': True,
        'device': 'auto',
        'use_sim_time': is_sim,
        'node_name': 'act_inference_node',
    }

    inference_node = Node(
        package='inference_service',
        executable='lerobot_policy_node',
        name='act_inference_node',
        env=env, # <--- Pass the enriched environment
        parameters=[node_params],
        output='screen',
    )

    print(f"[robot_config] ✓ Inference node configured")
    return inference_node


def generate_action_dispatcher_node(robot_config, control_mode, use_sim=False):
    """Generate action dispatcher node with configuration binding.

    Args:
        robot_config: Robot configuration dict
        control_mode: Active control mode
        use_sim: Simulation mode flag

    Returns:
        Node action for action_dispatcher
    """
    is_sim = parse_bool(use_sim, default=False)

    # Get control mode configuration
    control_modes = robot_config.get('control_modes', {})
    mode_config = control_modes.get(control_mode, {})
    executor_config = mode_config.get('executor', {})

    # Get robot configuration
    # Note: robot_config is already the robot object (from data.get("robot", {}))
    robot_name = robot_config.get('name', 'so101')
    robot_joints = robot_config.get('joints', {})
    all_joints = robot_joints.get('all', ["1", "2", "3", "4", "5", "6"])

    # Executor settings
    executor_type = executor_config.get('type', 'topic')
    executor_mode = executor_config.get('mode', control_mode)

    print(f"[robot_config] ========== Generating Action Dispatcher ==========")
    print(f"[robot_config] Robot: {robot_name}")
    print(f"[robot_config] Control mode: {control_mode}")
    print(f"[robot_config] Executor type: {executor_type}")
    print(f"[robot_config] Use sim time: {is_sim}")

    # Get inference configuration
    inference_config = mode_config.get('inference', {})

    # CRITICAL: Align with inference_service's actual action server name
    # The LeRobotPolicyNode (PassiveInferenceNode) creates an action server named 'DispatchInfer'
    # under its own node name ('act_inference_node').
    action_server = '/act_inference_node/DispatchInfer'

    # Create action dispatcher node
    action_dispatcher_node = Node(
        package="action_dispatch",
        executable="action_dispatcher_node",
        name="action_dispatcher",
        parameters=[{
            # Executor settings
            "enable_dual_mode": executor_type == 'topic',
            "executor_mode": executor_mode,

            # Robot configuration
            "robot_name": robot_name,
            "joint_names": all_joints,

            # Queue settings (from config or defaults)
            "queue_size": executor_config.get('queue_size', 100),
            "watermark_threshold": executor_config.get('watermark_threshold', 20),
            "min_queue_size": executor_config.get('min_queue_size', 10),

            # Control settings
            "control_frequency": executor_config.get('control_frequency', 100.0),
            "control_mode": control_mode,

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
            "inference_action_server": action_server,
            "inference_prompt": "",

            # Simulation time
            "use_sim_time": is_sim,
        }],
        output="screen",
    )

    print(f"[robot_config] ✓ Action dispatcher configured")
    return action_dispatcher_node


def generate_execution_nodes(robot_config, control_mode='teleop_act', use_sim=False):
    """Generate all execution nodes (inference + dispatcher).

    This is the main entry point for execution system generation.
    It automatically determines whether to launch inference based on
    the control_mode configuration.

    Args:
        robot_config: Robot configuration dict
        control_mode: Active control mode (defaults to robot's default_control_mode)
        use_sim: Simulation mode flag

    Returns:
        List of Node actions for execution system
    """
    nodes = []

    # Use robot's default_control_mode if not specified
    if not control_mode or control_mode == 'default':
        robot_cfg = robot_config.get('robot', {})
        control_mode = robot_cfg.get('default_control_mode', 'teleop_act')

    # Step 1: Generate inference node (if enabled)
    try:
        inference_node = generate_inference_node(robot_config, control_mode, use_sim)
        if inference_node:
            nodes.append(inference_node)
    except Exception as e:
        print(f"[robot_config] ERROR generating inference node: {e}")
        # Don't raise - allow system to continue without inference

    # Step 2: Generate action dispatcher node (always needed)
    try:
        dispatcher_node = generate_action_dispatcher_node(
            robot_config, control_mode, use_sim
        )
        nodes.append(dispatcher_node)
    except Exception as e:
        print(f"[robot_config] ERROR generating action dispatcher: {e}")
        raise

    return nodes
