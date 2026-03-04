"""MoveIt 2 launch builders.

This module handles:
- MoveIt 2 core node generation (move_group)
- RViz visualization for MoveIt
- Inclusion of external MoveIt launch files
"""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_moveit_nodes(robot_config, control_mode, use_sim=False, display=True):
    """Generate MoveIt 2 nodes.

    Args:
        robot_config: Robot configuration dict
        control_mode: Active control mode
        use_sim: Simulation mode flag
        display: Whether to launch RViz visualization

    Returns:
        List of launch actions for MoveIt 2
    """
    actions = []
    
    # Check if MoveIt is needed for this control mode
    # Usually enabled for 'moveit_planning' or any mode with 'moveit' in name
    with_moveit = 'moveit' in control_mode.lower()
    
    if not with_moveit:
        return actions

    print(f"[robot_config] ========== Generating MoveIt Nodes ==========")
    print(f"[robot_config] Control mode: {control_mode}")
    print(f"[robot_config] MoveIt Display: {display}")

    # Find MoveIt launch file
    try:
        moveit_package_dir = get_package_share_directory('robot_moveit')
        moveit_launch_file = Path(moveit_package_dir) / 'launch' / 'so101_moveit.launch.py'

        if moveit_launch_file.exists():
            # Include MoveIt launch file
            moveit_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(moveit_launch_file)),
                launch_arguments={
                    'is_sim': 'True' if use_sim else 'False',
                    'display': 'True' if display else 'False',
                }.items()
            )
            actions.append(moveit_launch)
            print(f"[robot_config] Added MoveIt launch (is_sim={use_sim}, display={display})")
        else:
            print(f"[robot_config] WARNING: MoveIt launch file not found at {moveit_launch_file}")
    except Exception as e:
        print(f"[robot_config] WARNING: Could not find robot_moveit package: {e}")
        print(f"[robot_config] Continuing without MoveIt...")

    return actions
