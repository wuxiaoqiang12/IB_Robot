"""Recording node generation for robot_config.

This module provides utilities to generate recording nodes
for integration with the robot_config launch system.

Supports two recording modes:
1. Continuous: Traditional ros2 bag record (all-in-one file)
2. Episodic: Triggered episode-by-episode recording via episode_recorder Action Server
"""

import os
from pathlib import Path
from datetime import datetime
from typing import List, Union
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

from robot_config.utils import resolve_ros_path


def generate_recording_nodes(robot_config: dict, active_control_mode: str, record_mode: str = 'continuous') -> List[Union[Node, ExecuteProcess]]:
    """
    Generate recording nodes based on robot configuration and recording mode.

    This function creates ROS 2 nodes/actions for data recording based on the
    recording mode. It integrates with the robot_config launch system.

    Args:
        robot_config: Robot configuration dictionary loaded from YAML
        active_control_mode: The control mode currently active
        record_mode: Recording mode - 'continuous' or 'episodic'
                     - continuous: Uses ros2 bag record for all-in-one recording
                     - episodic: Uses episode_recorder Action Server for triggered recording

    Returns:
        List of Node or ExecuteProcess actions for recording

    Example:
        >>> from robot_config.launch_builders.recording import generate_recording_nodes
        >>> config = load_robot_config('so101_single_arm')
        >>> nodes = generate_recording_nodes(config, record_mode='episodic')
        >>> ld.add_action(nodes[0])

    Usage in launch file:
        # Continuous recording (default)
        ros2 launch robot_config robot.launch.py record:=true

        # Episodic recording (requires manual record_cli in separate terminal)
        ros2 launch robot_config robot.launch.py record:=true record_mode:=episodic
        # Then in another terminal: ros2 run dataset_tools record_cli
    """
    if record_mode == 'episodic':
        return generate_episodic_recording_node(robot_config, active_control_mode)
    else:
        return generate_continuous_recording_action(robot_config)


def generate_continuous_recording_action(robot_config: dict) -> List[ExecuteProcess]:
    """
    Generate continuous recording action using ros2 bag record.

    This creates a single rosbag file that records everything continuously
    from launch until shutdown.

    Args:
        robot_config: Robot configuration dictionary

    Returns:
        List containing ExecuteProcess action for ros2 bag record

    Behavior:
        - Auto-discovers topics from robot config (joints, cameras, controllers)
        - Generates filename: ~/rosbag/<robot_name>_<timestamp>.mcap
        - Records continuously until node shutdown
    """
    print(f"[recording_builder] Using CONTINUOUS recording (ros2 bag record)")

    # Auto-discover topics to record
    topics = get_recording_topics(robot_config)

    # Generate filename with timestamp
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    robot_name = robot_config.get('name', 'robot')
    output_file = f"~/rosbag/{robot_name}_{timestamp}.mcap"

    # Expand ~ to actual home directory
    output_file = str(Path(output_file).expanduser())

    print(f"[recording_builder] Recording {len(topics)} topics to: {output_file}")
    print(f"[recording_builder] Topics: {topics}")

    # Create recording action
    recording_action = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', output_file] + topics,
        output='screen'
    )

    print(f"[recording_builder] ✓ Continuous recording action created")
    return [recording_action]


def generate_episodic_recording_node(robot_config: dict, active_control_mode: str) -> List[Node]:
    """
    Generate episodic recording node using episode_recorder Action Server.

    This creates an Action Server that waits for trigger commands to start
    recording individual episodes. Each episode is saved as a separate bag file
    with semantic metadata (operator prompt).

    **IMPORTANT**: The episode_recorder Action Server runs in the background.
    You MUST manually run `record_cli` in a separate terminal to trigger recordings.

    Args:
        robot_config: Robot configuration dictionary
        active_control_mode: The active control mode string

    Returns:
        List containing Node action for episode_recorder

    Behavior:
        - Synthesizes contract dynamically from robot_config based on active_control_mode
        - Starts episode_recorder Action Server (background service)
        - Each episode saved as: ~/rosbag_demos/episodes/<timestamp>
        - Operator prompt embedded in bag metadata
    """
    from robot_config.contract_builder import (
        synthesize_contract,
        get_contract_output_path,
        save_contract,
    )
    import os

    print(f"[recording_builder] Using EPISODIC recording (episode_recorder Action Server)")

    # 1. Synthesize contract dynamically
    contract = synthesize_contract(robot_config, active_control_mode)
    
    if not contract:
        print(f"[recording_builder] INFO: Inference disabled for '{active_control_mode}', falling back to static contract section in robot config.")
        contract = robot_config.get('contract')
        if not contract:
            print(f"[recording_builder] ERROR: No 'contract' section found in robot configuration.")
            return []

    # 2. Save contract to /tmp/...
    contract_path = get_contract_output_path(robot_config, active_control_mode)
    try:
        save_contract(contract, contract_path)
        print(f"[recording_builder] Contract synthesized and saved to: {contract_path}")
    except Exception as e:
        print(f"[recording_builder] ERROR: Could not save contract: {e}")
        return []

    # Determine bag output directory
    import os
    
    # Check if a custom directory is specified in the robot_config YAML
    recording_config = robot_config.get('recording', {})
    custom_dir = recording_config.get('bag_base_dir', '~/rosbag_demos/episodes')
    bag_base_dir = os.path.expanduser(custom_dir)

    # Create episode_recorder node (Action Server)
    episode_recorder_node = Node(
        package='dataset_tools',
        executable='episode_recorder',
        name='episode_recorder',
        output='screen',
        parameters=[
            {'contract_path': contract_path},
            {'bag_base_dir': bag_base_dir},
        ],
    )

    print(f"[recording_builder] ✓ Episode recorder node created")
    print(f"[recording_builder]")
    print(f"[recording_builder] " + "="*70)
    print(f"[recording_builder] ⚠️  IMPORTANT: Use SEPARATE TERMINAL to trigger recordings:")
    print(f"[recording_builder]     ros2 run dataset_tools record_cli")
    print(f"[recording_builder] " + "="*70)

    return [episode_recorder_node]


def find_contract_path(robot_name: str) -> str:
    """
    Find contract file path for a given robot.

    Args:
        robot_name: Robot name (e.g., 'so101', 'turtlebot')

    Returns:
        Contract file path (absolute path)

    Search order:
        1. robot_config package share directory: config/contracts/<robot_name>.yaml
        2. Workspace relative path: src/robot_config/config/contracts/<robot_name>.yaml
        3. Fallback: act_grab_pan.yaml
    """
    contract_filename = f"{robot_name}.yaml"

    # Try package share directory
    try:
        from ament_index_python.packages import get_package_share_directory
        config_share = get_package_share_directory('robot_config')
        contract_path = os.path.join(config_share, 'config', 'contracts', contract_filename)
        if os.path.exists(contract_path):
            return contract_path
    except:
        pass

    # Try workspace relative path
    workspace_root = find_workspace_root()
    if workspace_root:
        contract_path = os.path.join(workspace_root, 'src', 'robot_config', 'config', 'contracts', contract_filename)
        if os.path.exists(contract_path):
            return contract_path

    # Fallback to act_grab_pan.yaml
    if workspace_root:
        fallback_path = os.path.join(workspace_root, 'src', 'robot_config', 'config', 'contracts', 'act_grab_pan.yaml')
        if os.path.exists(fallback_path):
            return fallback_path

    # Last resort: return expected path even if it doesn't exist
    return f"src/robot_config/config/contracts/{contract_filename}"


def find_workspace_root() -> str:
    """
    Find IB_Robot workspace root directory.

    Returns:
        Workspace root path or None if not found
    """
    # Try to find from current file location
    current_path = Path(__file__).resolve()

    # Walk up the directory tree looking for install/setup.bash
    for parent in current_path.parents:
        if (parent / 'install' / 'setup.bash').exists():
            return str(parent)

    return None


def get_recording_topics(robot_config: dict) -> List[str]:
    """
    Get list of topics to record based on robot configuration.

    Args:
        robot_config: Robot configuration dictionary

    Returns:
        List of topic names for rosbag recording

    Example:
        >>> topics = get_recording_topics(config)
        >>> print(topics)
        ['/joint_states', '/arm_position_controller/commands', '/camera/cam0/image_raw', ...]
    """
    topics = []

    # Always record joint states
    topics.append('/joint_states')

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

    return topics
