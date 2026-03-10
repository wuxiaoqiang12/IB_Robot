"""Execution system launch builders (refactored).

This module handles:
- Inference service node generation (with contract synthesis)
- Action dispatcher node generation
- Automatic parameter binding from robot_config
- Support for distributed (component) mode
"""

import os
from launch_ros.actions import Node

from robot_config.utils import parse_bool, prepare_lerobot_env
from robot_config.contract_builder import (
    synthesize_contract,
    get_contract_output_path,
    save_contract,
)

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

    control_modes = robot_config.get("control_modes", {})
    if control_mode not in control_modes:
        print(f"[robot_config] WARNING: Control mode '{control_mode}' not found")
        return None

    mode_config = control_modes[control_mode]
    inference_config = mode_config.get("inference", {})

    if not inference_config.get("enabled", False):
        print(f"[robot_config] Inference not enabled for mode '{control_mode}'")
        return None

    # Check if distributed mode is enabled
    distributed = inference_config.get("distributed", False)

    if distributed:
        return generate_distributed_inference_nodes(robot_config, control_mode, use_sim)

    return generate_monolithic_inference_node(robot_config, control_mode, use_sim)


def generate_monolithic_inference_node(robot_config, control_mode, use_sim=False):
    """Generate monolithic (self-contained) inference node."""
    is_sim = parse_bool(use_sim, default=False)

    control_modes = robot_config.get("control_modes", {})
    mode_config = control_modes[control_mode]
    inference_config = mode_config.get("inference", {})

    print(
        f"[robot_config] ========== Generating Inference Node (Monolithic) =========="
    )
    print(f"[robot_config] Control mode: {control_mode}")

    # Step 1: Synthesize contract
    contract = synthesize_contract(robot_config, control_mode)
    if not contract:
        print(f"[robot_config] ERROR: Failed to synthesize contract")
        return None

    # Step 2: Save contract
    contract_path = get_contract_output_path(robot_config, control_mode)
    try:
        save_contract(contract, contract_path)
    except Exception as e:
        print(f"[robot_config] WARNING: Could not save contract: {e}")

    # Step 3: Get model configuration
    model_name = inference_config["model"]
    models = robot_config.get("models", {})
    if model_name not in models:
        print(f"[robot_config] ERROR: Model '{model_name}' not found in config")
        return None

    model_config = models[model_name]

    print(f"[robot_config] Model: {model_name}")
    print(f"[robot_config]   Path: {model_config['path']}")
    print(f"[robot_config]   Policy type: {model_config.get('policy_type', 'unknown')}")
    print(f"[robot_config]   Contract: {contract_path}")

    # Step 4: Prepare environment
    env = prepare_lerobot_env()
    if env.get("PYTHONPATH"):
        print(f"[robot_config] Injected PYTHONPATH: {env['PYTHONPATH']}")

    # Step 5: Create inference node
    node_params = {
        "checkpoint": model_config["path"],
        "contract_path": str(contract_path),
        "passive_mode": True,
        "device": "auto",
        "use_sim_time": is_sim,
        "node_name": "act_inference_node",
    }

    inference_node = Node(
        package="inference_service",
        executable="lerobot_policy_node",
        name="act_inference_node",
        env=env,
        parameters=[node_params],
        output="screen",
    )

    print(f"[robot_config] ✓ Monolithic inference node configured")
    return inference_node


def generate_distributed_inference_nodes(robot_config, control_mode, use_sim=False):
    """Generate distributed inference nodes (preprocessor + inference + postprocessor)."""
    is_sim = parse_bool(use_sim, default=False)

    control_modes = robot_config.get("control_modes", {})
    mode_config = control_modes[control_mode]
    inference_config = mode_config.get("inference", {})

    print(
        f"[robot_config] ========== Generating Inference Nodes (Distributed) =========="
    )
    print(f"[robot_config] Control mode: {control_mode}")

    # Step 1: Synthesize contract
    contract = synthesize_contract(robot_config, control_mode)
    if not contract:
        print(f"[robot_config] ERROR: Failed to synthesize contract")
        return None

    # Step 2: Save contract
    contract_path = get_contract_output_path(robot_config, control_mode)
    try:
        save_contract(contract, contract_path)
    except Exception as e:
        print(f"[robot_config] WARNING: Could not save contract: {e}")

    # Step 3: Get model configuration
    model_name = inference_config["model"]
    models = robot_config.get("models", {})
    if model_name not in models:
        print(f"[robot_config] ERROR: Model '{model_name}' not found in config")
        return None

    model_config = models[model_name]
    policy_path = model_config["path"]

    print(f"[robot_config] Model: {model_name}")
    print(f"[robot_config]   Path: {policy_path}")
    print(f"[robot_config]   Contract: {contract_path}")

    # Step 4: Prepare environment
    env = prepare_lerobot_env()

    # Topic names for distributed pipeline
    preprocessed_topic = "/preprocessed/batch"
    action_topic = "/inference/action"

    nodes = []

    # Node 1: Preprocessor
    preprocessor_node = Node(
        package="inference_service",
        executable="preprocessor_node",
        name="preprocessor",
        env=env,
        parameters=[
            {
                "contract_path": str(contract_path),
                "policy_path": policy_path,
                "output_topic": preprocessed_topic,
                "device": "auto",
                "use_sim_time": is_sim,
            }
        ],
        output="screen",
    )
    nodes.append(preprocessor_node)
    print(f"[robot_config]   Preprocessor -> {preprocessed_topic}")

    # Node 2: Pure Inference
    inference_node = Node(
        package="inference_service",
        executable="pure_inference_node",
        name="pure_inference",
        env=env,
        parameters=[
            {
                "policy_path": policy_path,
                "input_topic": preprocessed_topic,
                "output_topic": action_topic,
                "device": "auto",
                "use_sim_time": is_sim,
            }
        ],
        output="screen",
    )
    nodes.append(inference_node)
    print(f"[robot_config]   PureInference: {preprocessed_topic} -> {action_topic}")

    # Node 3: Postprocessor
    postprocessor_node = Node(
        package="inference_service",
        executable="postprocessor_node",
        name="postprocessor",
        env=env,
        parameters=[
            {
                "contract_path": str(contract_path),
                "policy_path": policy_path,
                "input_topic": action_topic,
                "device": "auto",
                "use_sim_time": is_sim,
            }
        ],
        output="screen",
    )
    nodes.append(postprocessor_node)
    print(f"[robot_config]   Postprocessor: {action_topic} -> controllers")

    print(
        f"[robot_config] ✓ Distributed inference nodes configured ({len(nodes)} nodes)"
    )
    return nodes


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

    # Get contract path (deterministic)
    contract_path = get_contract_output_path(robot_config, control_mode)

    # Get control mode configuration
    control_modes = robot_config.get("control_modes", {})
    mode_config = control_modes.get(control_mode, {})
    executor_config = mode_config.get("executor", {})

    # Get robot configuration
    # Note: robot_config is already the robot object (from data.get("robot", {}))
    robot_name = robot_config.get("name", "so101")
    robot_joints = robot_config.get("joints", {})
    all_joints = robot_joints.get("all", ["1", "2", "3", "4", "5", "6"])

    # Executor settings
    executor_type = executor_config.get("type", "topic")
    executor_mode = executor_config.get("mode", control_mode)

    print(f"[robot_config] ========== Generating Action Dispatcher ==========")
    print(f"[robot_config] Robot: {robot_name}")
    print(f"[robot_config] Control mode: {control_mode}")
    print(f"[robot_config] Executor type: {executor_type}")
    print(f"[robot_config] Use sim time: {is_sim}")

    # Get inference configuration
    inference_config = mode_config.get("inference", {})

    # CRITICAL: Align with inference_service's actual action server name
    # The LeRobotPolicyNode (PassiveInferenceNode) creates an action server named 'DispatchInfer'
    # under its own node name ('act_inference_node').
    action_server = "/act_inference_node/DispatchInfer"

    # Create action dispatcher node
    action_dispatcher_node = Node(
        package="action_dispatch",
        executable="action_dispatcher_node",
        name="action_dispatcher",
        parameters=[
            {
                # Executor settings
                "enable_dual_mode": executor_type == "topic",
                "executor_mode": executor_mode,
                # Robot configuration
                "robot_name": robot_name,
                "joint_names": all_joints,
                # Queue settings (from config or defaults)
                "queue_size": executor_config.get("queue_size", 100),
                "watermark_threshold": executor_config.get("watermark_threshold", 20),
                "min_queue_size": executor_config.get("min_queue_size", 10),
                # Control settings
                "control_frequency": executor_config.get("control_frequency", 100.0),
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
                "contract_path": str(contract_path),
                # Inference settings
                "inference_action_server": action_server,
                "inference_prompt": "",
                # Simulation time
                "use_sim_time": is_sim,
            }
        ],
        output="screen",
    )

    print(f"[robot_config] ✓ Action dispatcher configured")
    return action_dispatcher_node


def generate_execution_nodes(robot_config, control_mode="model_inference", use_sim=False):
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

    if not control_mode or control_mode == "default":
        control_mode = robot_config.get("default_control_mode", "model_inference")

    # Step 1: Generate inference node(s) (if enabled)
    try:
        inference_result = generate_inference_node(robot_config, control_mode, use_sim)
        if inference_result:
            # Handle both single node and list of nodes (distributed mode)
            if isinstance(inference_result, list):
                nodes.extend(inference_result)
            else:
                nodes.append(inference_result)
    except Exception as e:
        print(f"[robot_config] ERROR generating inference node: {e}")

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
