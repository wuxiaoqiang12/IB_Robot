"""Main robot launch file for robot_config.

This launch file loads robot configuration from YAML and dynamically generates:
- ros2_control hardware interface and controllers
- Robot state publisher
- Camera drivers (usb_cam, realsense2_camera)
- Static TF publishers for camera frames

Usage:
    ros2 launch robot_config robot.launch.py robot_config:=test_cam use_sim:=false
    ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm use_sim:=false
"""

import os
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    LogInfo,
    IncludeLaunchDescription,
)
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


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


def generate_camera_nodes(robot_config, use_sim):
    """Generate camera nodes from configuration.

    Args:
        robot_config: Robot configuration dict
        use_sim: Simulation mode flag

    Returns:
        List of Node actions for cameras
    """
    nodes = []

    # Skip cameras in simulation mode
    if use_sim == 'true':
        print("[robot_config] Simulation mode: skipping camera nodes")
        return nodes

    peripherals = robot_config.get("peripherals", [])
    print(f"[robot_config] Generating nodes for {len(peripherals)} peripherals")

    for periph in peripherals:
        if periph.get("type") != "camera":
            continue

        name = periph["name"]
        driver = periph.get("driver", "opencv")
        print(f"[robot_config] Creating camera node: {name} (driver={driver})")

        if driver == "opencv":
            # Use usb_cam package
            index = periph.get("index", 0)
            video_device = f"/dev/video{index}" if isinstance(index, int) else index

            params = {
                "camera_name": name,
                "framerate": float(periph.get("fps", 30)),
                "image_width": periph.get("width", 640),
                "image_height": periph.get("height", 480),
                "pixel_format": periph.get("pixel_format", "mjpeg"),
                "camera_frame_id": periph.get("frame_id", f"camera_{name}_frame"),
                "video_device": video_device,
            }

            # Add camera_info_url if specified
            if "camera_info_url" in periph:
                params["camera_info_url"] = periph["camera_info_url"]

            # Add optional parameters
            if "brightness" in periph:
                params["brightness"] = periph["brightness"]
            if "contrast" in periph:
                params["contrast"] = periph["contrast"]
            if "saturation" in periph:
                params["saturation"] = periph["saturation"]
            if "sharpness" in periph:
                params["sharpness"] = periph["sharpness"]

            print(f"[robot_config]   Camera params: {params}")

            nodes.append(Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name=f"{name}_camera",
                parameters=[params],
                remappings=[
                    ("image_raw", f"/camera/{name}/image_raw"),
                    ("camera_info", f"/camera/{name}/camera_info"),
                ],
                output="screen",
            ))

        elif driver == "realsense":
            # Use realsense2_camera package
            params = {
                "camera_name": name,
                "camera_fps": periph.get("fps", 30),
                "color_width": periph.get("width", 640),
                "color_height": periph.get("height", 480),
                "color_format": periph.get("pixel_format", "bgr8").upper(),
                "camera_frame_id": periph.get("frame_id", f"camera_{name}_frame"),
                "enable_pointcloud": periph.get("enable_pointcloud", False),
                "enable_sync": periph.get("enable_sync", True),
                "align_depth": periph.get("align_depth", False),
            }

            # Add depth parameters if specified
            if "depth_width" in periph:
                params["depth_width"] = periph["depth_width"]
                params["depth_height"] = periph["depth_height"]
            if "depth_fps" in periph:
                params["depth_fps"] = periph["depth_fps"]

            # Add serial number if specified
            if "serial_number" in periph:
                params["serial_no"] = str(periph["serial_number"])

            print(f"[robot_config]   RealSense params: {params}")

            nodes.append(Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                name=f"{name}_camera",
                parameters=[params],
                remappings=[
                    (f"/camera/{name}/color/image_raw", f"/camera/{name}/image_raw"),
                    (f"/camera/{name}/color/camera_info", f"/camera/{name}/camera_info"),
                ],
                output="screen",
            ))

    return nodes


def generate_gazebo_nodes(robot_config, urdf_path):
    """Generate Gazebo simulation nodes.

    Args:
        robot_config: Robot configuration dict
        urdf_path: Resolved URDF path

    Returns:
        List of launch actions for Gazebo
    """
    from launch.actions import SetEnvironmentVariable
    import os

    actions = []
    ros2_control_config = robot_config.get("ros2_control")

    # Set Gazebo resource path
    try:
        lerobot_desc_share = get_package_share_directory("robot_description")
        import os
        gazebo_resource_path = SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=str(Path(lerobot_desc_share).parent.resolve())
        )
        actions.append(gazebo_resource_path)
    except:
        print("[robot_config] WARNING: Could not find robot_description package")

    # Get world file path (use custom world with Sensors plugin)
    try:
        robot_config_share = get_package_share_directory("robot_config")
        world_path = os.path.join(robot_config_share, "config", "worlds", "simulation.world")
        if not Path(world_path).exists():
            print(f"[robot_config] WARNING: World file not found at {world_path}, using empty.sdf")
            world_path = "empty.sdf"
        else:
            print(f"[robot_config] Using world file: {world_path}")
    except:
        world_path = "empty.sdf"

    # Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch"
            ),
            "/gz_sim.launch.py"
        ]),
        launch_arguments=[
            ("gz_args", [f"-v 4 -r {world_path}"])
        ]
    )
    actions.append(gazebo_launch)

    # Spawn entity in Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "so101"],
    )
    actions.append(gz_spawn_entity)

    # Clock bridge
    gz_ros2_clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ]
    )
    actions.append(gz_ros2_clock_bridge)

    # Get world name from the world file (default to "demo" for simulation.world)
    world_name = "demo"  # simulation.world uses "demo" as world name

    # Joint state bridge for ros2_control
    gz_ros2_joint_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            f"/world/{world_name}/model/so101/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
        ]
    )
    actions.append(gz_ros2_joint_bridge)

    # Camera bridges (dynamically generated from config)
    peripherals = robot_config.get("peripherals", [])
    cameras = [p for p in peripherals if p.get("type") == "camera"]

    if cameras:
        print(f"[robot_config] Gazebo: Creating {len(cameras)} camera bridge(s) from config")

        # Mapping from camera name to Gazebo sensor name
        # Can be extended or made configurable
        gazebo_sensor_map = {
            "wrist": "wrist_camera",
            "top": "top_camera",
            "test": "top_camera",  # Default: map 'test' to top_camera
        }

        for periph in cameras:
            name = periph["name"]
            gazebo_sensor_name = periph.get("gazebo_sensor", gazebo_sensor_map.get(name, f"{name}_camera"))
            topic_prefix = periph.get("topic_prefix", f"/camera/{name}")

            print(f"[robot_config]   Camera bridge: {name} -> Gazebo sensor: {gazebo_sensor_name}")

            # Gazebo camera topic format: /world/{world_name}/model/{model_name}/link/{link_name}/sensor/{sensor_name}/{topic}
            # Note: The camera sensors use topic names like "wrist_camera/image" and "top_camera/image"
            gz_image_topic = f"/world/{world_name}/model/so101/link/{gazebo_sensor_name}_link/sensor/{gazebo_sensor_name}/{gazebo_sensor_name}/image"
            gz_info_topic = f"/world/{world_name}/model/so101/link/{gazebo_sensor_name}_link/sensor/{gazebo_sensor_name}/{gazebo_sensor_name}/camera_info"

            camera_bridge = Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    f"{gz_image_topic}@sensor_msgs/msg/Image[gz.msgs.Image",
                    f"{gz_info_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                ],
                remappings=[
                    (gz_image_topic, f"{topic_prefix}/image_raw"),
                    (gz_info_topic, f"{topic_prefix}/camera_info"),
                ],
            )
            actions.append(camera_bridge)

    print(f"[robot_config] Created {len(actions)} Gazebo nodes")
    return actions


def generate_ros2_control_nodes(robot_config, use_sim):
    """Generate ros2_control nodes from configuration.

    Args:
        robot_config: Robot configuration dict
        use_sim: Simulation mode flag

    Returns:
        List of Node actions for ros2_control
    """
    nodes = []
    ros2_control_config = robot_config.get("ros2_control")

    if not ros2_control_config:
        print("[robot_config] No ros2_control configuration found, skipping ros2_control nodes")
        return nodes

    print("[robot_config] Creating ros2_control nodes")

    # Get URDF path
    urdf_path = ros2_control_config.get("urdf_path")
    if not urdf_path:
        print("[robot_config] WARNING: No urdf_path specified in ros2_control config")
        return nodes

    # Resolve $(find package) and $(env VAR) substitutions
    if "$(find " in urdf_path:
        # Extract package name from $(find package_name)/path
        import re
        match = re.search(r'\$\(find\s+(\w+)\)', urdf_path)
        if match:
            package_name = match.group(1)
            try:
                pkg_share = get_package_share_directory(package_name)
                urdf_path = urdf_path.replace(f"$(find {package_name})", pkg_share)
            except:
                print(f"[robot_config] ERROR: Could not find package {package_name}")
                return nodes

    if "$(env " in urdf_path:
        # Replace $(env VAR) with environment variable
        import re
        match = re.search(r'\$\(env\s+(\w+)\)', urdf_path)
        if match:
            var_name = match.group(1)
            var_value = os.environ.get(var_name, "")
            urdf_path = urdf_path.replace(f"$(env {var_name})", var_value)

    print(f"[robot_config] URDF path: {urdf_path}")

    # Check if URDF file exists
    if not Path(urdf_path).exists():
        print(f"[robot_config] WARNING: URDF file not found at {urdf_path}")
        return nodes

    # Check if cameras should be enabled
    enable_cameras = False
    for periph in robot_config.get("peripherals", []):
        if periph.get("type") == "camera":
            enable_cameras = True
            break

    # Get hardware parameters
    port = ros2_control_config.get("port", "/dev/ttyACM0")
    calib_file = ros2_control_config.get("calib_file", "")

    # Resolve $(env HOME) in calib_file if present
    if "$(env HOME)" in calib_file:
        calib_file = calib_file.replace("$(env HOME)", os.environ.get("HOME", ""))

    # Generate robot_description using xacro
    xacro_args = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        urdf_path,
        " ",
        "use_sim:=",
        "true" if use_sim == 'true' else "false",
        " ",
        "port:=",
        port,
        " ",
        "calib_file:=",
        calib_file,
    ]

    # Add use_cameras argument if cameras are configured
    if enable_cameras:
        xacro_args.extend([
            " ",
            "use_cameras:=true",
        ])

    robot_description_content = ParameterValue(
        Command(xacro_args),
        value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    # Add use_sim_time for simulation
    if use_sim == 'true':
        robot_description["use_sim_time"] = True

    # Robot State Publisher
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    ))

    # In simulation mode: Gazebo's gz_ros2_control plugin handles hardware and controllers
    # In real hardware mode: Start ros2_control_node and spawners
    if use_sim != 'true':
        print("[robot_config] Real hardware mode: starting ros2_control_node and controller spawners")

        hardware_plugin = ros2_control_config.get("hardware_plugin", "")
        controllers_config = ros2_control_config.get("controllers_config")

        if not controllers_config:
            # Try to find default controllers config based on robot type
            if "so101" in hardware_plugin.lower():
                try:
                    so101_hw_share = get_package_share_directory("so101_hardware")
                    controllers_config = os.path.join(so101_hw_share, "config", "so101_controllers.yaml")
                except:
                    print("[robot_config] WARNING: Could not find so101_hardware package for controllers config")

        if controllers_config and Path(controllers_config).exists():
            print(f"[robot_config] Controllers config: {controllers_config}")

            control_node_params = [controllers_config]
            if use_sim == 'true':
                control_node_params.append({"use_sim_time": True})

            nodes.append(Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=control_node_params,
                remappings=[
                    ("~/robot_description", "/robot_description"),
                ],
                output="screen",
            ))

            # Spawn joint_state_broadcaster
            nodes.append(Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
            ))

            # Spawn arm_controller
            nodes.append(Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller", "--controller-manager", "/controller_manager"],
            ))

            # Spawn gripper_controller
            nodes.append(Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
            ))
        else:
            print(f"[robot_config] WARNING: Controllers config not found at {controllers_config}")
    else:
        print("[robot_config] Simulation mode: Gazebo's gz_ros2_control will handle controllers")

    print(f"[robot_config] Created {len(nodes)} ros2_control nodes")
    return nodes


def generate_tf_nodes(robot_config):
    """Generate static transform publishers for camera frames.

    Args:
        robot_config: Robot configuration dict

    Returns:
        List of Node actions for TF publishers
    """
    nodes = []
    peripherals = robot_config.get("peripherals", [])

    for periph in peripherals:
        if periph.get("type") != "camera":
            continue

        name = periph["name"]
        frame_id = periph.get("frame_id", f"camera_{name}_frame")
        optical_frame_id = periph.get("optical_frame_id")
        transform = periph.get("transform")

        print(f"[robot_config] Creating TF for camera: {name}")

        # Parent frame transform (if specified in config)
        if transform:
            parent_frame = transform.get("parent_frame", "base_link")
            x = transform.get("x", 0.0)
            y = transform.get("y", 0.0)
            z = transform.get("z", 0.0)
            roll = transform.get("roll", 0.0)
            pitch = transform.get("pitch", 0.0)
            yaw = transform.get("yaw", 0.0)

            print(f"[robot_config]   TF: {parent_frame} -> {frame_id} pos=({x},{y},{z})")

            nodes.append(Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    '--x', str(x), '--y', str(y), '--z', str(z),
                    '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
                    '--frame-id', parent_frame, '--child-frame-id', frame_id
                ],
                output="screen",
            ))

        # Optical frame transform (standard ROS2 convention)
        if optical_frame_id:
            print(f"[robot_config]   Optical TF: {frame_id} -> {optical_frame_id}")
            # Standard optical frame rotation: -90° around X, -90° around Y
            nodes.append(Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    '--x', '0', '--y', '0', '--z', '0',
                    '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
                    '--frame-id', frame_id, '--child-frame-id', optical_frame_id
                ],
                output="screen",
            ))

    return nodes


def launch_setup(context, *args, **kwargs):
    """Launch setup function that generates all nodes.

    Args:
        context: Launch context

    Returns:
        List of launch actions
    """
    actions = []

    # Get launch parameters
    robot_config_name = context.launch_configurations.get('robot_config', 'test_cam')
    config_path_override = context.launch_configurations.get('config_path', '')
    use_sim = context.launch_configurations.get('use_sim', 'false')

    print(f"[robot_config] Launch setup with:")
    print(f"[robot_config]   robot_config: {robot_config_name}")
    print(f"[robot_config]   config_path: {config_path_override if config_path_override else '(none)'}")
    print(f"[robot_config]   use_sim: {use_sim}")

    # Load robot configuration
    try:
        robot_config = load_robot_config(robot_config_name, config_path_override if config_path_override else None)
    except Exception as e:
        print(f"[robot_config] ERROR loading config: {e}")
        raise

    # ========== ros2_control Nodes ==========
    try:
        ros2_control_nodes = generate_ros2_control_nodes(robot_config, use_sim)
        actions.extend(ros2_control_nodes)
    except Exception as e:
        print(f"[robot_config] ERROR generating ros2_control nodes: {e}")
        raise

    # ========== Gazebo Nodes (only in simulation mode) ==========
    if use_sim == 'true':
        try:
            # Get URDF path for Gazebo
            ros2_control_config = robot_config.get("ros2_control", {})
            urdf_path = ros2_control_config.get("urdf_path", "")

            # Resolve path substitutions
            if urdf_path and "$(find " in urdf_path:
                import re
                match = re.search(r'\$\(find\s+(\w+)\)', urdf_path)
                if match:
                    package_name = match.group(1)
                    try:
                        pkg_share = get_package_share_directory(package_name)
                        urdf_path = urdf_path.replace(f"$(find {package_name})", pkg_share)
                    except:
                        urdf_path = ""

            if urdf_path:
                gazebo_nodes = generate_gazebo_nodes(robot_config, urdf_path)
                actions.extend(gazebo_nodes)
        except Exception as e:
            print(f"[robot_config] ERROR generating Gazebo nodes: {e}")
            raise

    # ========== Camera Nodes (dynamically generated from config) ==========
    try:
        camera_nodes = generate_camera_nodes(robot_config, use_sim)
        actions.extend(camera_nodes)
    except Exception as e:
        print(f"[robot_config] ERROR generating camera nodes: {e}")
        raise

    # ========== TF Nodes (dynamically generated from config) ==========
    try:
        tf_nodes = generate_tf_nodes(robot_config)
        actions.extend(tf_nodes)
    except Exception as e:
        print(f"[robot_config] ERROR generating TF nodes: {e}")
        raise

    print(f"[robot_config] Total nodes to launch: {len(actions)}")

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
        OpaqueFunction(function=launch_setup),
    ])
