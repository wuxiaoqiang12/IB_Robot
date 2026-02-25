"""Launch builder modules for robot_config.

This package contains modules for building ROS2 launch components:
- control.py: ros2_control nodes, controller spawners
- perception.py: Camera drivers, TF publishers
- simulation.py: Gazebo and simulation nodes
- execution.py: Action dispatcher and inference nodes
"""

from robot_config.launch_builders.control import (
    generate_ros2_control_nodes,
    generate_controller_spawners,
    validate_joint_config
)

from robot_config.launch_builders.perception import (
    generate_camera_nodes,
    generate_tf_nodes,
    generate_virtual_camera_relays
)

from robot_config.launch_builders.simulation import (
    generate_gazebo_nodes
)

__all__ = [
    # Control
    'generate_ros2_control_nodes',
    'generate_controller_spawners',
    'validate_joint_config',
    # Perception
    'generate_camera_nodes',
    'generate_tf_nodes',
    'generate_virtual_camera_relays',
    # Simulation
    'generate_gazebo_nodes',
]
