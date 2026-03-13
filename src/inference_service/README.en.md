# Inference Service

`inference_service` is the core AI execution package for IB-Robot. It provides a standardized framework for running end-to-end Machine Learning policies (like ACT, pi0, etc.) on physical robots with strict temporal alignment and zero-copy latency optimizations.

## Architecture: Composition over Inheritance

The inference pipeline is decoupled into three pure-Python core components (`inference_service.core`):
1. **TensorPreprocessor**: Converts raw ROS 2 sensor data (images, joint states) into normalized PyTorch Tensors.
2. **PureInferenceEngine**: A completely stateless, ROS-agnostic GPU execution engine.
3. **TensorPostprocessor**: Denormalizes output action tensors back into physical control commands.

By separating the core math from the ROS 2 transport layer, this package supports two distinct deployment modes, toggleable via a single YAML parameter.

---

## 🚀 Execution Modes

### Mode A: Monolithic (Single-Machine Zero-Copy)
**Best for**: Robots equipped with high-performance onboard GPUs (e.g., RTX 4060).

In this mode, `lerobot_policy_node.py` instantiates an `InferenceCoordinator` that chains the Preprocessor, Engine, and Postprocessor together.
* **Data Flow**: Sensor data stays entirely within the RAM/VRAM of the single process. Tensors are passed by reference.
* **Performance**: Absolute lowest latency. Zero serialization/deserialization overhead.
* **Config**: `execution_mode: "monolithic"`

### Mode B: Device-Edge-Cloud Synergy (Distributed)
**Best for**: Lightweight robots (Device) running on low-power CPUs (e.g., Raspberry Pi) paired with a high-end computation node (Edge) or tower server (Cloud) over a LAN.

To preserve compatibility with the pull-based `action_dispatch` system without clogging the network with 30fps video streams, the Device node acts as an **Asynchronous Proxy**.
1. **Device Node (`lerobot_policy_node.py`)**: Receives the action goal, reads the cameras *on-demand*, runs the **Preprocessor** on CPU, and publishes the lightweight Tensor batch to `/preprocessed/batch`. The action callback is then suspended using an asynchronous `threading.Event`.
2. **Edge/Cloud Node (`pure_inference_node.py`)**: Subscribes to the batch, crunches the numbers on the GPU using `PureInferenceEngine`, and returns the raw action to `/inference/action`.
3. **Device Node**: Wakes up, runs the **Postprocessor**, and completes the Action sequence.

* **Performance**: Achieves "Compute Offloading" perfectly. The Device only sends the exact frames needed for inference (e.g., 20Hz), saving massive network bandwidth.
* **Config**: `execution_mode: "distributed"`

---

## ⚙️ Configuration & Usage

The execution mode is controlled seamlessly via your `robot_config` YAML files. You do not need to change launch files to switch modes on the device.

```yaml
# src/robot_config/config/robots/your_robot.yaml
control_modes:
  model_inference:
    inference:
      enabled: true
      execution_mode: "distributed"  # Or "monolithic"
      model: so101_act
```

### Launching

**Device (Robot)**:
Launch your robot normally. The launch builder automatically reads the YAML and configures the node behavior.
```bash
ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm control_mode:=model_inference
```

**Edge/Cloud (GPU Server - Only needed if `execution_mode: "distributed"`)**:
Ensure you are on the same ROS Domain ID. Launch the dedicated standalone cloud node.
```bash
ros2 launch inference_service cloud_inference.launch.py \
    policy_path:=/path/to/models/pretrained_model \
    device:=cuda
```

## 🧪 Testing
Because the core components are isolated from ROS, they can be validated entirely offline:
```bash
pytest src/inference_service/tests/
```
