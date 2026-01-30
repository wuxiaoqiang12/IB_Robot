# SO-101 Robotic Arm Hardware C++ Plugin

Native C++ ros2_control hardware interface plugin for SO-101 robotic arm with direct Feetech servo communication.

## Overview

This package provides a native C++ implementation of the ros2_control hardware interface for the SO-101 robotic arm. It directly communicates with Feetech servos using the FTServo SDK, offering better performance and lower latency compared to the topic-based approach.

## Features

- Direct servo communication via FTServo SDK
- Native ros2_control SystemInterface implementation
- JSON-based calibration file support
- Lifecycle management (configure, activate, deactivate)
- Real-time position read/write operations
- Configurable startup positions to prevent arm from jumping to zero on startup
- Automatic motor torque disable on shutdown for safety

## Architecture

```
ros2_control → SO101SystemHardware (C++) → FTServo SDK → Feetech Servos
```

## Dependencies

### Git Submodule

FTServo_Linux SDK is included as a git submodule:

```bash
# Initialize and update submodule
git submodule update --init --recursive
```

Source repository: `git@github.com:ftservo/FTServo_Linux.git`

### System Dependencies

- ROS 2 Humble
- nlohmann_json library
- hardware_interface
- pluginlib
- rclcpp_lifecycle

## Building

```bash
cd ~/Research/ledog_ros2/ros2_ws
colcon build --packages-select so101_hardware_cpp
source install/setup.zsh
```

The FTServo SDK is automatically compiled as a static library, no manual build required.

## Configuration

### URDF Parameters

The plugin requires the following parameters in the URDF:

```xml
<hardware>
  <plugin>so101_hardware_cpp/SO101SystemHardware</plugin>
  <param name="port">/dev/ttyACM0</param>
  <param name="calib_file">$(env HOME)/.calibrate/so101_follower_calibrate.json</param>
  <param name="reset_positions">$(arg reset_positions)</param>
</hardware>
```

Each joint must have an `id` parameter:

```xml
<joint name="1">
  <param name="id">1</param>
  ...
</joint>
```

### Startup Position Configuration (Optional)

The `reset_positions` parameter allows you to specify initial joint positions for the arm on startup. This is useful for:

- Dog-mounted arms with limited height clearance
- Ensuring the arm starts in a safe, known configuration
- Avoiding sudden movements to zero position

**Format**: JSON string with joint IDs as keys and radian values as values.

**Example** (safe folded position matching motor_bridge reset_normalized_goals):
```bash
ros2 launch so101_hw_interface so101_hw_dual_mode.launch.py \
  use_sim:=false \
  use_cpp_plugin:=true \
  port:=/dev/ttyACM0 \
  "reset_positions:='{\"1\": 0.0813, \"2\": 3.7905, \"3\": 7.0379, \"4\": -0.6228, \"5\": 2.3869}'"
```

These values are converted from the safe reset positions in `motor_bridge.py` to radians.

**Behavior**:
- If `reset_positions` is provided → arm moves to specified positions on startup
- If `reset_positions` is empty (default) → arm preserves current motor positions

**Note**: Positions are in radians, matching the joint state interface.

### Calibration File

The plugin reads calibration data from a JSON file (default: `~/.calibrate/so101_follower_calibrate.json`):

```json
{
  "1": {
    "homing_offset": 2048,
    "range_min": 0,
    "range_max": 4095
  },
  ...
}
```

Generate calibration file using:

```bash
ros2 run so101_hw_interface so101_calibrate_arm --arm follower --port /dev/ttyACM0
```

Or use the calibration service:

```bash
ros2 run so101_hw_interface so101_calibration_service --ros-args -p arm_type:=follower -p port:=/dev/ttyACM0
ros2 service call /calibrate_follower std_srvs/srv/Trigger
```

## Usage

### Launch with C++ Plugin

```bash
ros2 launch so101_hw_interface so101_hw_dual_mode.launch.py \
  use_sim:=false \
  use_cpp_plugin:=true \
  port:=/dev/ttyACM0
```

### Launch with Topic-Based Mode (Default)

```bash
ros2 launch so101_hw_interface so101_hw_dual_mode.launch.py use_sim:=false
```

### Launch with Custom Startup Positions

```bash
ros2 launch so101_hw_interface so101_hw_dual_mode.launch.py \
  use_sim:=false \
  use_cpp_plugin:=true \
  port:=/dev/ttyACM0 \
  "reset_positions:='{\"1\": 0.0813, \"2\": 3.7905, \"3\": 7.0379, \"4\": -0.6228, \"5\": 2.3869}'"
```

## Implementation Details

### Lifecycle States

1. **on_init**: Read URDF parameters (port, calib_file, joint IDs)
2. **on_configure**: Load calibration file (JSON format)
3. **on_activate**:
   - Connect to motors
   - If `reset_positions` is configured: move to specified positions
   - Otherwise: read and preserve current motor positions
4. **on_deactivate**:
   - Disable torque on all motors
   - Wait 100ms to ensure torque is fully disabled
   - Disconnect from serial port

### Read/Write Operations

- **read()**: Reads motor positions and converts to radians
- **write()**: Converts joint commands from radians to motor positions

### Coordinate Conversion

```cpp
// Motor position to radians
double radians = ((pos - range_min) / range - 0.5) * 2.0 * M_PI;

// Radians to motor position
s16 pos = (radians / (2.0 * M_PI) + 0.5) * range + range_min;
```

## Comparison: Topic-Based vs C++ Plugin

| Feature | Topic-Based | C++ Plugin |
|---------|-------------|------------|
| Latency | Higher (topic bridge) | Lower (direct) |
| Performance | Python overhead | Native C++ |
| Setup | Requires motor_bridge | Direct connection |
| Compatibility | Standard ros2_control | Standard ros2_control |
| Use Case | Development/Testing | Production |
| Startup Positions | Supported | Supported |
| Safe Shutdown | Supported | Supported |

## Troubleshooting

### Plugin Not Found

Ensure the package is built and sourced:

```bash
colcon build --packages-select so101_hardware_cpp
source install/setup.zsh
```

### Calibration File Not Found

Generate calibration file before launching:

```bash
ros2 run so101_hw_interface so101_calibrate_arm --arm follower --port /dev/ttyACM0
```

### Motor Connection Failed

Check serial port permissions:

```bash
sudo chmod 666 /dev/ttyACM0
```

Or add user to dialout group:

```bash
sudo usermod -a -G dialout $USER
```

### Submodule Not Initialized

If FTServo_Linux directory is empty:

```bash
cd ~/Research/ledog_ros2
git submodule update --init --recursive
```

## Files

- [so101_system_hardware.hpp](include/so101_hardware_cpp/so101_system_hardware.hpp) - Header file
- [so101_system_hardware.cpp](src/so101_system_hardware.cpp) - Implementation
- [CMakeLists.txt](CMakeLists.txt) - Build configuration
- [so101_hardware_cpp_plugin.xml](so101_hardware_cpp_plugin.xml) - Plugin description
- [FTServo_Linux](FTServo_Linux/) - Feetech SDK (git submodule)

## License

TODO: License declaration
