# robot_config

Unified robot configuration system for ros2_control and peripherals.

## Overview

This package provides a unified configuration system for robot hardware that bridges:

- **ros2_control**: For joint/motor control interfaces
- **Peripherals**: For cameras and other devices (via existing ROS2 drivers)
- **Rosetta**: For ML policy I/O contracts

The goal is to have a single source of truth for robot hardware configuration, eliminating duplication between different configuration systems.

## Features

- **Single YAML configuration**: Define ros2_control, cameras, and ML contracts in one file
- **Uses existing ROS2 camera drivers**:
  - `usb_cam` for USB cameras (OpenCV-based)
  - `realsense2_camera` for RealSense D400 series
- **TF publishing**: Automatic camera frame transform publishing
- **Calibration support**: Standard ROS2 camera_info_manager integration
- **Rosetta integration**: Contracts reference peripherals by name

## Architecture

```
robot_config YAML (single source of truth)
        │
        ├───► ros2_control (joints/motors)
        │       └───► so101_hardware plugin
        │
        ├───► Camera drivers (existing ROS2 packages)
        │       ├───► usb_cam (USB cameras)
        │       └───► realsense2_camera (RealSense D400)
        │
        └───► Rosetta contracts (ML I/O)
                └───► PolicyBridge / EpisodeRecorder
```

## Configuration Example

```yaml
robot:
  name: so101_single_arm
  type: so101
  robot_type: so_101

  ros2_control:
    hardware_plugin: so101_hardware/SO101SystemHardware
    port: /dev/ttyACM0
    calib_file: $(env HOME)/.calibrate/so101_follower_calibrate.json
    reset_positions:
      "1": 0.0813
      "2": 3.7905

  peripherals:
    - type: camera
      name: top
      driver: opencv  # Uses usb_cam package
      index: 0
      width: 640
      height: 480
      fps: 30
      frame_id: camera_top_frame
      optical_frame_id: camera_top_optical_frame

    - type: camera
      name: wrist
      driver: realsense  # Uses realsense2_camera package
      serial_number: "12345678"
      width: 640
      height: 480
      fps: 30
      depth_width: 640
      depth_height: 480
      frame_id: camera_wrist_frame

  contract:
    observations:
      - key: observation.images.top
        topic: /camera/top
        peripheral: top  # References camera above
        image:
          resize: [480, 640]
```

## Usage

### Launching the Robot

```bash
# Launch with real hardware
ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm

# Launch with simulation
ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm use_sim:=true
```

### Validating Configuration

```bash
# Validate a robot config file with Python directly
python3 src/ros2/ros2_ws/src/robot_config/robot_config/scripts/validate_config.py \
    src/ros2/ros2_ws/src/robot_config/config/robots/so101_single_arm.yaml
```

## Camera Drivers

### USB Cameras (via `usb_cam`)

```yaml
- type: camera
  name: usb_cam
  driver: opencv  # Uses usb_cam package
  index: 0  # USB device index (/dev/video0)
  width: 640
  height: 480
  fps: 30
  pixel_format: bgr8  # bgr8, rgb8, mono8, yuyv, etc.
```

**Install:**
```bash
sudo apt install ros-humble-usb-cam
```

### RealSense D400 Series (via `realsense2_camera`)

```yaml
- type: camera
  name: rs_cam
  driver: realsense  # Uses realsense2_camera package
  serial_number: "12345678"  # Device serial (optional)
  width: 640
  height: 480
  fps: 30
  depth_width: 640
  depth_height: 480
  depth_fps: 30
  enable_pointcloud: false
  align_depth: false
```

**Install:**
```bash
sudo apt install ros-humble-librealsense2*
```

## Camera Calibration

Camera intrinsics can be stored in the standard ROS2 location:

```yaml
- type: camera
  name: top
  driver: opencv
  index: 0
  width: 640
  height: 480
  camera_info_url: file://$(env HOME)/.ros/camera_info/top_camera.yaml
```

Calibration files can be created using the standard ROS2 camera calibration tools:

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.024 \
  image:=/camera/top/image_raw
```

## Rosetta Integration

The robot_config integrates with rosetta contracts by allowing observations to reference peripherals by name:

```yaml
# In robot_config
peripherals:
  - type: camera
    name: top
    width: 640
    height: 480

# In contract section
contract:
  observations:
    - key: observation.images.top
      topic: /camera/top
      peripheral: top  # Auto-fills width, height, fps from peripheral
```

When the contract is loaded, it will automatically include the camera metadata from the peripheral definition.

## Migration from robot_interface

The old `robot_interface` package used LeRobot's Robot class directly. This package replaces it with:

1. **No LeRobot dependency in ROS2 layer**: Uses ros2_control directly
2. **ros2_control native**: Standard ROS2 hardware interface
3. **Existing ROS2 camera drivers**: Uses `usb_cam` and `realsense2_camera` packages
4. **Single YAML configuration**: All hardware defined in one place

The two example configurations from `robot_interface` have been manually migrated to:
- `config/robots/so101_single_arm.yaml` (from `single_arm_banana.yaml`)
- `config/robots/so101_dual_arm.yaml` (from `dual_arms_pencil.yaml`)

## Troubleshooting

### Camera not opening

Check USB permissions:
```bash
ls -l /dev/video*
sudo chmod 666 /dev/video0
```

Or add user to `video` group:
```bash
sudo usermod -a -G video $USER
```

### RealSense camera not found

Install librealsense2:
```bash
sudo apt install librealsense2-utils librealsense2-dev
sudo apt install ros-humble-librealsense2*
```

Check camera is connected:
```bash
realsense-viewer
```

### Calibration file not found

Make sure the path is correct and starts with `file://`:
```yaml
camera_info_url: file:///home/user/.ros/camera_info/top.yaml
```

## References

- [usb_cam GitHub](https://github.com/ros-drivers/usb_cam) - USB camera driver for ROS2
- [realsense-ros GitHub](https://github.com/realsenseai/realsense-ros) - Intel RealSense ROS2 wrapper

## License

Apache-2.0
