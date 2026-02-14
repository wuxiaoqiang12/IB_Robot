"""Main robot launch file for robot_config.

This launch file loads robot configuration from YAML and dynamically generates:
- ros2_control hardware interface and controllers
- Robot state publisher
- Camera drivers (usb_cam, realsense2_camera)
- Static TF publishers for camera frames

Controllers are automatically spawned in both simulation and real hardware modes:
- Simulation mode: Uses Gazebo's gz_ros2_control plugin for controller_manager
- Hardware mode: Starts ros2_control_node for controller_manager

Usage:
    ros2 launch robot_config robot.launch.py robot_config:=test_cam use_sim:=false
    ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm use_sim:=false
    ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm use_sim:=true
    ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm use_sim:=true auto_start_controllers:=false

Launch Arguments:
    robot_config: Robot configuration name (default: test_cam)
    config_path: Optional full path to robot config file
    use_sim: Use simulation mode (default: false)
    auto_start_controllers: Automatically spawn controllers (default: true, set to false for debugging)
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


def resolve_ros_path(path):
    """Resolve ROS-style path substitutions like $(find pkg) and $(env VAR).

    Handles ROS path substitution syntax:
    - $(find package_name): Resolves to package share directory
    - $(env VAR_NAME): Resolves to environment variable value

    Args:
        path: Path string that may contain $(find package) or $(env VAR)

    Returns:
        Resolved path string. Returns original path if it's None or empty.

    Example:
        >>> resolve_ros_path("$(find so101_hardware)/config/controllers.yaml")
        "/home/user/workspace/install/share/so101_hardware/config/controllers.yaml"

        >>> resolve_ros_path("$(env HOME)/.config/robot.yaml")
        "/home/user/.config/robot.yaml"
    """
    if not path:
        return path

    import re

    # Resolve $(find package)
    find_pattern = re.compile(r'\$\(find\s+(\w+)\)')
    for match in find_pattern.finditer(path):
        pkg_name = match.group(1)
        try:
            pkg_path = get_package_share_directory(pkg_name)
            path = path.replace(f"$(find {pkg_name})", pkg_path)
        except Exception as e:
            print(f"[robot_config] WARNING: Could not find package '{pkg_name}': {e}")

    # Resolve $(env VAR)
    env_pattern = re.compile(r'\$\(env\s+(\w+)\)')
    for match in env_pattern.finditer(path):
        var_name = match.group(1)
        var_value = os.environ.get(var_name, "")
        path = path.replace(f"$(env {var_name})", var_value)
        if not var_value:
            print(f"[robot_config] WARNING: Environment variable '{var_name}' is not set or empty")

    return path


def parse_bool(value, default=False):
    """Parse various value types to boolean with robust handling.

    Handles multiple input formats:
    - Strings: "true", "TRUE", "True", "1", "yes", "on" -> True
    - Strings: "false", "FALSE", "False", "0", "no", "off" -> False
    - Booleans: True/False -> as-is
    - Numbers: 1/0 -> True/False
    - None: -> default value

    Args:
        value: Input value to parse (string, bool, int, or None)
        default: Default value if input is None or unparseable

    Returns:
        Boolean value

    Example:
        >>> parse_bool("true")
        True
        >>> parse_bool("FALSE")
        False
        >>> parse_bool(True)
        True
        >>> parse_bool(None, default=False)
        False
    """
    if value is None:
        return default

    # Handle boolean types directly
    if isinstance(value, bool):
        return value

    # Convert to string and normalize
    str_value = str(value).strip().lower()

    # Check for true-like values
    if str_value in ('true', '1', 'yes', 'on'):
        return True

    # Check for false-like values
    if str_value in ('false', '0', 'no', 'off', ''):
        return False

    # Unknown value, return default
    return default


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
        use_sim: Simulation mode flag (string or bool)

    Returns:
        List of Node actions for cameras
    """
    # Convert to boolean for robust comparison
    is_sim = parse_bool(use_sim, default=False)

    nodes = []

    # Skip cameras in simulation mode
    if is_sim:
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


def generate_ros2_control_nodes(robot_config, use_sim, auto_start_controllers='true'):
    """Generate ros2_control nodes from configuration.

    Args:
        robot_config: Robot configuration dict
        use_sim: Simulation mode flag (string or bool)
        auto_start_controllers: Whether to automatically start controllers (string or bool)

    Returns:
        List of Node actions for ros2_control
    """
    # Convert string parameters to boolean for robust comparison
    is_sim = parse_bool(use_sim, default=False)
    is_auto_start = parse_bool(auto_start_controllers, default=True)

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

    # Resolve ROS path substitutions $(find package) and $(env VAR)
    urdf_path = resolve_ros_path(urdf_path)

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

    # Resolve ROS path substitutions in calib_file
    calib_file = resolve_ros_path(calib_file)

    # Generate robot_description using xacro
    xacro_args = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        urdf_path,
        " ",
        "use_sim:=",
        "true" if is_sim else "false",
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
    if is_sim:
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
    # Read controller configuration from YAML
    controller_names = ros2_control_config.get("controllers", [])

    # Resolve ROS path substitutions in controllers_config
    controllers_config = resolve_ros_path(ros2_control_config.get("controllers_config"))

    if not is_sim:
        # Real hardware mode
        print("[robot_config] Real hardware mode: starting ros2_control_node and controller spawners")

        # Check for required configuration
        if not controllers_config:
            print("[robot_config] WARNING: No controllers_config specified in robot configuration YAML")
            print("[robot_config] Please add 'controllers_config' under 'ros2_control' section")
            print("[robot_config] Example: controllers_config: $(find package_name)/config/controllers.yaml")

        if not controller_names:
            print("[robot_config] WARNING: No controllers list specified in robot configuration YAML")
            print("[robot_config] Please add 'controllers' list under 'ros2_control' section")
            print("[robot_config] Example:")
            print("[robot_config]   controllers:")
            print("[robot_config]     - joint_state_broadcaster")
            print("[robot_config]     - arm_controller")

        if controllers_config and Path(controllers_config).exists():
            print(f"[robot_config] Controllers config: {controllers_config}")
            print(f"[robot_config] Controllers to spawn: {controller_names}")

            # Start ros2_control_node (provides controller_manager)
            nodes.append(Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[controllers_config],
                remappings=[
                    ("~/robot_description", "/robot_description"),
                ],
                output="screen",
            ))

            # Spawn controllers if auto_start_controllers is enabled
            if is_auto_start and controller_names:
                spawners = generate_controller_spawners(controller_names, use_sim=False, controller_manager_name="controller_manager")
                nodes.extend(spawners)
                print(f"[robot_config] Added {len(spawners)} controller spawners (hardware mode, 10s timeout)")
            else:
                print("[robot_config] Skipping controller auto-start (auto_start_controllers=false or no controllers specified)")
        else:
            print(f"[robot_config] ERROR: Controllers config not found at {controllers_config}")
            print(f"[robot_config] Cannot start ros2_control_node without valid config")
    else:
        # Simulation mode
        print("[robot_config] Simulation mode: Gazebo's gz_ros2_control provides controller_manager")
        print(f"[robot_config] Controllers to spawn: {controller_names}")

        # Spawn controllers if auto_start_controllers is enabled
        if is_auto_start and controller_names:
            spawners = generate_controller_spawners(controller_names, use_sim=True, controller_manager_name="controller_manager")
            nodes.extend(spawners)
            print(f"[robot_config] Added {len(spawners)} controller spawners (simulation mode, 30s timeout, use_sim_time=True)")
        else:
            if not controller_names:
                print("[robot_config] WARNING: No controllers list specified in robot configuration YAML")
            print("[robot_config] Skipping controller auto-start (auto_start_controllers=false or no controllers specified)")

    print(f"[robot_config] Created {len(nodes)} ros2_control nodes")
    return nodes


def generate_controller_spawners(controller_names, use_sim=False, controller_manager_name="controller_manager"):
    """Generate controller spawner nodes.

    Dynamically creates spawners for the specified controllers.

    Args:
        controller_names: List of controller names to spawn (e.g., ["joint_state_broadcaster", "arm_controller"])
        use_sim: Whether in simulation mode
                 - True: Adds use_sim_time parameter, 30s timeout (for Gazebo initialization)
                 - False: No use_sim_time, 10s timeout (for hardware initialization)
        controller_manager_name: Name of controller_manager
                 - Uses relative name "controller_manager" by default (supports ROS 2 namespaces)
                 - In GroupAction with namespace, auto-resolves to /namespace/controller_manager
                 - Supports future multi-arm configurations (e.g., "left_arm/controller_manager")

    Returns:
        List of Node actions for controller spawners

    Note:
        - Simulation mode requires use_sim_time=True for proper TF synchronization with Gazebo clock
        - Timeout values are tuned for different scenarios:
          * 30s (sim): Accounts for Gazebo startup on low-spec machines
          * 10s (hardware): Prevents indefinite hanging while allowing slow hardware init
    """
    spawners = []

    # Simulation mode parameters
    sim_params = {"use_sim_time": True} if use_sim else {}

    # Timeout parameters: simulation needs longer wait time, hardware mode also needs appropriate timeout
    if use_sim:
        timeout_args = ["--controller-manager-timeout", "30"]
    else:
        timeout_args = ["--controller-manager-timeout", "10"]  # Shorter timeout for hardware mode

    # Spawn controllers dynamically based on configuration
    for controller_name in controller_names:
        spawners.append(Node(
            package="controller_manager",
            executable="spawner",
            name=f"spawner_{controller_name}",  # Explicit naming for easier debugging
            arguments=[
                controller_name,
                "--controller-manager", controller_manager_name,
                *timeout_args
            ],
            parameters=[sim_params] if sim_params else [],
        ))

    return spawners


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
    auto_start_controllers = context.launch_configurations.get('auto_start_controllers', 'true')

    print(f"[robot_config] Launch setup with:")
    print(f"[robot_config]   robot_config: {robot_config_name}")
    print(f"[robot_config]   config_path: {config_path_override if config_path_override else '(none)'}")
    print(f"[robot_config]   use_sim: {use_sim}")
    print(f"[robot_config]   auto_start_controllers: {auto_start_controllers}")

    # Load robot configuration
    try:
        robot_config = load_robot_config(robot_config_name, config_path_override if config_path_override else None)
    except Exception as e:
        print(f"[robot_config] ERROR loading config: {e}")
        raise

    # ========== ros2_control Nodes ==========
    try:
        ros2_control_nodes = generate_ros2_control_nodes(robot_config, use_sim, auto_start_controllers)
        actions.extend(ros2_control_nodes)
    except Exception as e:
        print(f"[robot_config] ERROR generating ros2_control nodes: {e}")
        raise

    # ========== Gazebo Nodes (only in simulation mode) ==========
    if parse_bool(use_sim, default=False):
        try:
            # Get URDF path for Gazebo
            ros2_control_config = robot_config.get("ros2_control", {})
            urdf_path = ros2_control_config.get("urdf_path", "")

            # Resolve ROS path substitutions
            urdf_path = resolve_ros_path(urdf_path)

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
        DeclareLaunchArgument(
            "auto_start_controllers",
            default_value="true",
            description="Automatically spawn controllers (set to false for debugging)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
