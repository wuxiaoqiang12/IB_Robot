"""Contract synthesis from robot configuration.

This module automatically generates inference service contracts
from robot_config.yaml, eliminating the need to manually maintain
separate contract YAML files.

The synthesized contract includes:
- Observation mappings from peripherals
- Action mappings from controller joints
- Model configuration
- Normalization parameters
"""

import os
from typing import Dict, Any, List
from pathlib import Path


class ContractSynthesisError(Exception):
    """Raised when contract synthesis fails due to architectural errors."""
    pass


def get_contract_output_path(robot_config: Dict, control_mode: str) -> str:
    """Get proper ROS2-compliant path for generated contract.

    Priority order:
    1. Environment variable ROBOT_CONFIG_CACHE_DIR (for containerized deployment)
    2. /tmp/robot_config/contracts/ (standard tmp location)
    3. Fallback to package share if needed

    Args:
        robot_config: Robot configuration dict
        control_mode: Active control mode name

    Returns:
        Absolute path for contract YAML file
    """
    # Option 1: Environment variable (for Docker/K8s)
    cache_dir = os.environ.get('ROBOT_CONFIG_CACHE_DIR')
    if cache_dir:
        return os.path.join(
            cache_dir,
            'contracts',
            f"{robot_config['name']}_{control_mode}.yaml"
        )

    # Option 2: Standard temp directory (recommended for single-machine)
    # Benefits: Auto-cleanup on reboot, not git-tracked, simple permissions
    contract_dir = '/tmp/robot_config/contracts'
    os.makedirs(contract_dir, exist_ok=True)
    return os.path.join(
        contract_dir,
        f"{robot_config['name']}_{control_mode}.yaml"
    )


def validate_control_mode_config(robot_config: Dict, control_mode: str) -> None:
    """Validate control mode configuration BEFORE synthesis.

    This performs architectural validation to catch configuration errors
    at system startup time, not during inference.

    Args:
        robot_config: Full robot configuration dict
        control_mode: Control mode name to validate

    Raises:
        ContractSynthesisError: If configuration is architecturally invalid
    """
    errors = []
    warnings = []

    # Check 1: Control mode exists
    control_modes = robot_config.get('control_modes', {})
    if control_mode not in control_modes:
        errors.append(f"Control mode '{control_mode}' not defined in robot_config")
        raise ContractSynthesisError("\n".join(errors))

    mode_config = control_modes[control_mode]

    # Check 2: Inference configuration
    inference_config = mode_config.get('inference', {})
    if inference_config.get('enabled', False):

        # Check 2.1: Model reference exists
        model_name = inference_config.get('model')
        models = robot_config.get('models', {})
        if model_name not in models:
            errors.append(
                f"Control mode '{control_mode}' requires model '{model_name}' "
                f"but it's not defined in robot_config.models"
            )

        # Check 2.2: Observation sources exist
        for obs_spec in inference_config.get('observations', []):
            source = obs_spec.get('source')
            key = obs_spec.get('key')

            if not source or not key:
                errors.append(
                    f"Invalid observation spec in mode '{control_mode}': "
                    f"missing 'source' or 'key'"
                )
                continue

            if source == 'joint_states':
                # Joint states always available (from ros2_control)
                continue

            # Check if peripheral exists
            peripheral = next(
                (p for p in robot_config.get('peripherals', []) if p['name'] == source),
                None
            )
            if not peripheral:
                errors.append(
                    f"Control mode '{control_mode}' requires observation '{source}' "
                    f"but peripheral '{source}' is not defined in robot_config.peripherals"
                )
            else:
                # Warning: Check peripheral type
                if peripheral.get('type') != 'camera':
                    warnings.append(
                        f"Observation source '{source}' is not a camera "
                        f"(type={peripheral.get('type')})"
                    )

    # Check 3: Executor configuration
    executor_config = mode_config.get('executor', {})
    executor_type = executor_config.get('type')
    if executor_type and executor_type not in ['topic', 'action']:
        errors.append(
            f"Invalid executor type '{executor_type}' in mode '{control_mode}'. "
            f"Must be 'topic' or 'action'"
        )

    # Check 4: Controllers exist
    controllers = mode_config.get('controllers', [])
    ros2_control_config = robot_config.get('ros2_control', {})
    defined_controllers = ros2_control_config.get('controllers', [])

    for ctrl in controllers:
        if ctrl not in defined_controllers:
            warnings.append(
                f"Controller '{ctrl}' used in mode '{control_mode}' "
                f"but not listed in ros2_control.controllers"
            )

    # Report results
    if warnings:
        print(f"[robot_config] ⚠ Configuration warnings for mode '{control_mode}':")
        for warning in warnings:
            print(f"  - {warning}")

    if errors:
        error_msg = (
            f"❌ Architectural errors in control mode '{control_mode}':\n" +
            "\n".join(f"  - {e}" for e in errors)
        )
        print(f"[robot_config] {error_msg}")
        raise ContractSynthesisError(error_msg)


def synthesize_contract(robot_config: Dict, control_mode: str) -> Dict[str, Any]:
    """Synthesize inference contract from robot configuration.

    This function automatically builds the contract by:
    1. Collecting observation topics from peripherals
    2. Mapping joint names from robot joints
    3. Configuring action space from controller joints

    Args:
        robot_config: Full robot configuration dict
        control_mode: Active control mode name

    Returns:
        Contract dictionary compatible with inference_service

    Raises:
        ContractSynthesisError: If configuration is architecturally invalid
    """
    print(f"[robot_config] ========== Contract Synthesis ==========")
    print(f"[robot_config] Control mode: {control_mode}")

    # Step 1: Validate configuration FIRST
    try:
        validate_control_mode_config(robot_config, control_mode)
    except ContractSynthesisError as e:
        print(f"[robot_config] ✗ Contract synthesis FAILED")
        raise

    # Step 2: Get mode configuration
    mode_config = robot_config['control_modes'][control_mode]
    inference_config = mode_config.get('inference', {})

    if not inference_config.get('enabled', False):
        print(f"[robot_config] Inference not enabled for mode '{control_mode}'")
        return None

    # Step 3: Get model configuration
    model_name = inference_config['model']
    models = robot_config.get('models', {})
    if model_name not in models:
        print(f"[robot_config] ERROR: Model '{model_name}' not found")
        return None

    model_config = models[model_name]

    # Step 4: Build observation mapping from peripherals
    # Rosetta expects observations as a LIST, not a dict
    observations = []
    for obs_spec in inference_config.get('observations', []):
        source = obs_spec['source']
        key = obs_spec['key']
        
        # Ensure key follows LeRobot convention
        lerobot_key = f"observation.{key}" if not key.startswith('observation.') else key

        if source == 'joint_states':
            # Joint positions from /joint_states
            joint_names = robot_config['joints']['all']
            # Convert to selector format (position.1, position.2, etc.)
            joint_selector_names = [f"position.{name}" for name in joint_names]
            obs_entry = {
                'key': lerobot_key,
                'topic': '/joint_states',
                'type': 'sensor_msgs/msg/JointState',
                'selector': {
                    'names': joint_selector_names
                },
                'align': {
                    'strategy': 'hold',
                    'stamp': 'header',
                    'tol_ms': 1500
                },
                'qos': {
                    'reliability': 'best_effort',
                    'history': 'keep_last',
                    'depth': 50
                }
            }
            observations.append(obs_entry)
            print(f"[robot_config] Observation: {lerobot_key} ← /joint_states ({len(joint_names)} joints)")
        else:
            # Camera observations from peripherals
            peripheral = next(
                (p for p in robot_config['peripherals'] if p['name'] == source),
                None
            )
            if peripheral:
                # Extract dimensions from peripheral config, fallback to LeRobot defaults
                width = peripheral.get('width', 640)
                height = peripheral.get('height', 480)
                
                obs_entry = {
                    'key': lerobot_key,
                    'topic': f'/camera/{source}/image_raw',
                    'type': 'sensor_msgs/msg/Image',
                    'image': {
                        'resize': [height, width]  # [H, W] format
                    },
                    'align': {
                        'strategy': 'hold',
                        'stamp': 'header',
                        'tol_ms': 1500
                    },
                    'qos': {
                        'reliability': 'best_effort',
                        'history': 'keep_last',
                        'depth': 10
                    }
                }
                observations.append(obs_entry)
                print(f"[robot_config] Observation: {lerobot_key} ← {obs_entry['topic']}")

    # Step 5: Build action mapping from controller joints
    actions = []
    
    for ctrl_name in mode_config['controllers']:
        if 'position_controller' not in ctrl_name:
            continue
            
        # Determine joints for this specific controller
        current_ctrl_joints = []
        if 'arm' in ctrl_name:
            current_ctrl_joints = robot_config['joints']['arm']
        elif 'gripper' in ctrl_name:
            current_ctrl_joints = robot_config['joints']['gripper']
        elif 'all' in ctrl_name:
            current_ctrl_joints = robot_config['joints']['all']
        
        if not current_ctrl_joints:
            continue

        # Convert to selector format (position.1, position.2, etc.)
        joint_selector_names = [f"position.{name}" for name in current_ctrl_joints]
        
        action_entry = {
            'key': f"action_{ctrl_name}",
            'selector': {
                'names': joint_selector_names
            },
            'publish': {
                'topic': f"/{ctrl_name}/commands",
                'type': 'std_msgs/msg/Float64MultiArray',
                'layout': 'flat',
                'qos': {
                    'reliability': 'best_effort',
                    'history': 'keep_last',
                    'depth': 10
                },
                'strategy': {
                    'mode': 'nearest',
                    'tolerance_ms': 500
                }
            },
            'safety_behavior': 'hold'
        }
        actions.append(action_entry)
        print(f"[robot_config] Action mapping: {ctrl_name} ({len(current_ctrl_joints)} joints) → {action_entry['publish']['topic']}")

    # Fallback if no controllers found
    if not actions:
        all_joints = robot_config['joints']['all']
        joint_selector_names = [f"position.{name}" for name in all_joints]
        actions = [{
            'key': 'action',
            'selector': {'names': joint_selector_names},
            'publish': {
                'topic': '/arm_position_controller/commands',
                'type': 'std_msgs/msg/Float64MultiArray',
                'layout': 'flat',
                'qos': {'reliability': 'best_effort', 'history': 'keep_last', 'depth': 10},
                'strategy': {'mode': 'nearest', 'tolerance_ms': 500}
            },
            'safety_behavior': 'hold'
        }]

    # Step 6: Assemble contract with rosetta-compliant structure
    contract = {
        'name': f"{robot_config['name']}_{control_mode}",
        'version': 1,
        'robot_type': robot_config.get('robot_type', 'so_101'),
        'rate_hz': 20,
        'max_duration_s': 90.0,
        'model': {
            'policy_type': model_config.get('policy_type', 'act'),
            'path': model_config['path']
        },
        'observations': observations,
        'actions': actions,
        'metadata': {
            'robot_name': robot_config['name'],
            'control_mode': control_mode,
            'generated_by': 'robot_config.contract_builder',
            'version': '1.0'
        }
    }

    # Optional: Add normalization if specified
    if 'normalization' in model_config:
        contract['normalization'] = model_config['normalization']
        print(f"[robot_config] Normalization: {model_config['normalization']}")

    print(f"[robot_config] ✓ Contract synthesis SUCCESS")
    print(f"[robot_config]   Observations: {len(observations)}")
    
    total_joints = sum(len(a['selector']['names']) for a in actions)
    print(f"[robot_config]   Actions: {total_joints} joints")
    return contract


def save_contract(contract: Dict, output_path: str) -> None:
    """Save synthesized contract to YAML file.

    Args:
        contract: Contract dictionary
        output_path: Path to save contract YAML
    """
    import yaml

    output_file = Path(output_path)
    output_file.parent.mkdir(parents=True, exist_ok=True)

    with open(output_file, 'w') as f:
        yaml.dump(contract, f, default_flow_style=False, sort_keys=False)

    print(f"[robot_config] Contract saved to: {output_path}")
