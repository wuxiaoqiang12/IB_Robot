# Action Dispatch Package

Pull-based action distribution layer for LeRobot-ROS2 integration.

## Overview

`action_dispatch` is a ROS2 package that implements a **pull-based action dispatcher** between inference models and `ros2_control`. It decouples inference latency from execution frequency through queue-based buffering.

### Key Features

- **Pull-based inference**: Dispatcher requests actions when queue水位 drops below threshold
- **Queue-based buffering**: FIFO queue with configurable capacity
- **High-frequency distribution**: 100Hz action publication to `ros2_control`
- **Linear interpolation**: Smooth motion continuation during queue starvation
- **Safety strategies**: Configurable handling of inference failures and queue exhaustion
- **Observation timestamping**: Explicit timestamps for freshness validation
- **Exponential backoff retry**: Automatic retry with increasing delays on failures

## Architecture

```
┌─────────────────────────────────────┐
│      Action Dispatcher Node         │
├─────────────────────────────────────┤
│  Queue Manager (FIFO deque)         │
│  Watermark Monitor (threshold)      │
│  Inference Client (Action client)    │
│  Linear Interpolator (smooth motion) │
│  Safety Monitor (retry logic)        │
│  Diagnostics Publisher              │
└─────────────────────────────────────┘
          │              │
          ▼              ▼
    Inference      ros2_control
       Service       (velocity/position)
```

## Usage

### Basic Launch

```bash
ros2 launch action_dispatch action_dispatch.launch.py
```

### Configuration

Configuration is loaded from `config/default_params.yaml`. Key parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `queue_size` | 100 | Maximum queue capacity |
| `watermark_threshold` | 30 | Trigger inference when queue < this |
| `control_frequency` | 100.0 | Action publication frequency (Hz) |
| `control_mode` | "velocity" | "velocity" or "position" |
| `interpolation_enabled` | true | Enable linear interpolation |
| `max_inference_timeout` | 1.0 | Inference timeout (seconds) |
| `max_retry_attempts` | 3 | Max consecutive retries before stop |
| `retry_backoff_base` | 0.5 | Exponential backoff base (seconds) |

### Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/action_dispatch/diagnostic` | `DiagnosticStatus` | Publish | Health status |
| `/action_dispatch/queue_size` | `Int32` | Publish | Current queue size |
| `/velocity_controller/command` | `Float64MultiArray` | Publish | Velocity commands |
| `/position_controller/command` | `JointTrajectory` | Publish | Position commands |
| `/joint_states` | `JointState` | Subscribe | Robot joint state |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `~/reset` | `std_srvs/Empty` | Reset queue and state |

## Dependencies

- `rclpy`: ROS2 Python client library
- `rclpy_action`: ROS2 Action support
- `rosetta_interfaces`: Shared message/action definitions
- `diagnostic_msgs`: Health monitoring
- `std_msgs`, `sensor_msgs`, `geometry_msgs`, `trajectory_msgs`: Standard ROS2 messages

## License

Apache-2.0
