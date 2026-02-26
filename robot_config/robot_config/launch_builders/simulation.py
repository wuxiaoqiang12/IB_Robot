"""Simulation launch builders.

This module handles:
- Gazebo Ignition launch
- Gazebo-ROS bridge nodes
- World file configuration
"""

from pathlib import Path
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from robot_config.utils import resolve_ros_path


def generate_gazebo_nodes(robot_config):
    """Generate Gazebo simulation nodes.

    Args:
        robot_config: Robot configuration dict

    Returns:
        List of launch actions for Gazebo
    """
    import os

    actions = []
    ros2_control_config = robot_config.get("ros2_control")

    # Collect all resource paths to set them together
    gazebo_resource_paths = []
    gazebo_model_paths = []

    # Set Gazebo resource path for so101_hardware
    try:
        gazebo_models_path = os.path.join(
            FindPackageShare('so101_hardware').find('so101_hardware'),
            'models'
        )
        gazebo_resource_paths.append(gazebo_models_path)
        gazebo_model_paths.append(gazebo_models_path)
        print(f"[robot_config] Added so101_hardware models path: {gazebo_models_path}")
    except Exception as e:
        print(f"[robot_config] WARNING: Could not find so101_hardware package: {e}")

    # Set robot_description package path for mesh files (CRITICAL for robot visualization)
    # IMPORTANT: Gazebo looks for model://robot_description/... which resolves to
    # <resource_path>/robot_description/..., so we need to point to the install/share
    # directory, not directly to robot_description
    try:
        install_share = os.path.dirname(FindPackageShare('robot_description').find('robot_description'))
        gazebo_resource_paths.append(install_share)
        gazebo_model_paths.append(install_share)
        print(f"[robot_config] Added install share path for Gazebo model resolution: {install_share}")

        # Verify mesh files exist
        robot_desc_share = FindPackageShare('robot_description').find('robot_description')
        mesh_path = os.path.join(robot_desc_share, 'meshes', 'lerobot', 'so101')
        if Path(mesh_path).exists():
            mesh_files = list(Path(mesh_path).glob('*.stl'))
            print(f"[robot_config] Verified {len(mesh_files)} mesh files at {mesh_path}")
        else:
            print(f"[robot_config] WARNING: Mesh directory not found at {mesh_path}")
    except Exception as e:
        print(f"[robot_config] WARNING: Could not find robot_description package: {e}")

    # Set Gazebo resource paths
    # Gazebo Garden (sim7) uses GZ_SIM_RESOURCE_PATH
    # Older versions use IGN_GAZEBO_RESOURCE_PATH
    if gazebo_resource_paths:
        combined_resource_path = ':'.join(gazebo_resource_paths)
        actions.append(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', combined_resource_path))
        actions.append(SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', combined_resource_path))
        print(f"[robot_config] Set Gazebo resource paths with {len(gazebo_resource_paths)} path(s)")

    if gazebo_model_paths:
        combined_model_path = ':'.join(gazebo_model_paths)
        actions.append(SetEnvironmentVariable('GAZEBO_MODEL_PATH', combined_model_path))
        actions.append(SetEnvironmentVariable('GZ_SIM_MODEL_PATH', combined_model_path))
        print(f"[robot_config] Set Gazebo model paths with {len(gazebo_model_paths)} path(s)")

    # Get world file path
    # Default to robot_config package's worlds directory
    world_file = robot_config.get("world_file", "")
    if not world_file:
        world_file = "$(find robot_config)/config/worlds/simulation.world"
    elif not world_file.startswith("$("):
        # If it's a relative path without $(find), assume it's in robot_config/worlds
        world_file = f"$(find robot_config)/config/worlds/{world_file}"

    world_path = resolve_ros_path(world_file)

    if not Path(world_path).exists():
        print(f"[robot_config] WARNING: World file not found at {world_path}")

    # Gazebo bridge configurations for cameras
    camera_bridges = []
    for periph in robot_config.get("peripherals", []):
        if periph.get("type") == "camera":
            name = periph["name"]
            camera_bridges.append({
                "name": name,
                "sensor_name": f"{name}_camera",
                "image_topic": f"/camera/{name}/image_raw",
                "camera_info_topic": f"/camera/{name}/camera_info",
            })

    if camera_bridges:
        print(f"[robot_config] Gazebo: Creating {len(camera_bridges)} camera bridge(s)")
        for bridge in camera_bridges:
            print(f"[robot_config]   Camera bridge: {bridge['name']} -> Gazebo sensor: {bridge['sensor_name']}")

    # Create entity node
    urdf_path = ros2_control_config.get("urdf_path")
    urdf_path = resolve_ros_path(urdf_path)

    create_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_config.get("name", "so101"),
            '-allow-renaming',
            '-topic', '/robot_description',
            '-x', str(robot_config.get("initial_pose_x", 0.0)),
            '-y', str(robot_config.get("initial_pose_y", 0.0)),
            '-z', str(robot_config.get("initial_pose_z", 0.0)),
        ],
        output='screen',
    )

    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Joint state bridge
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[f'/world/demo/model/{robot_config.get("name", "so101")}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'],
        output='screen',
    )

    actions.extend([create_entity, clock_bridge, joint_state_bridge])

    # Camera bridges
    for bridge in camera_bridges:
        sensor_name = bridge['sensor_name']
        model_name = robot_config.get("name", "so101")

        # Image bridge
        actions.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'/world/demo/model/{model_name}/link/{sensor_name}_link/sensor/{sensor_name}/{sensor_name}/image@sensor_msgs/msg/Image[gz.msgs.Image'
            ],
            output='screen',
        ))

        # Camera info bridge
        actions.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'/world/demo/model/{model_name}/link/{sensor_name}_link/sensor/{sensor_name}/{sensor_name}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
            ],
            output='screen',
        ))

    # Gazebo server and GUI
    # Use empty world if world file doesn't exist
    gz_args = ""
    if Path(world_path).exists():
        gz_args = f"-r {world_path}"
    else:
        print(f"[robot_config] Using empty Gazebo world (world file not found at {world_path})")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': gz_args,
        }.items()
    )

    actions.append(gazebo_launch)

    return actions
