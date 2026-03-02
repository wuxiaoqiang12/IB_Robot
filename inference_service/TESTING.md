# Pull-Based Action Dispatch 测试指南

## 概述

本指南用于测试完整的 Pull-Based Action Dispatch 系统，包括：
- **Mock 推理服务** - 模拟 ML 模型推理
- **Action Dispatcher** - 队列管理和动作分发
- **系统监控** - 实时监控队列状态

## 前置条件

1. **Gazebo 仿真环境** - 运行 SO-101 机械臂仿真
2. **ros2_control** - 确保控制器已启动
3. **编译完成** - 所有包已编译

## 快速开始

### 1. 启动 Gazebo 仿真

首先启动你的 SO-101 Gazebo 仿真环境（假设你已经有一个 launch 文件）：

```bash
# 示例：替换为你的实际 launch 命令
ros2 launch lerobot_description gazebo.launch.py robot_name:=so101
```

或者如果你有其他的仿真启动命令，使用那个。

### 2. 启动测试系统

在新的终端中，运行测试 launch 文件：

```bash
# Source ROS2 环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动测试系统（mock 推理 + dispatcher）
ros2 launch inference_service test_pull_based_dispatch.launch.py \
    robot_name:=so101 \
    control_mode:=velocity \
    watermark_threshold:=30 \
    chunk_size:=25
```

### 3. 启动系统监控（可选）

在另一个终端中：

```bash
source install/setup.bash
ros2 run inference_service test_system
```

## 预期行为

### 1. Mock 推理服务日志

你应该看到类似以下的日志输出：

```
Mock Inference Service Started
  - Joints: 6
  - Chunk size: 25 actions per request
  - Simulated delay: 0.1s

#1 | Inference request received: timestamp=1234567890.123456789, id=a1b2c3d4
✓ Inference complete | chunk_size=25 | latency=105.3ms | request_rate=0.85 Hz
```

### 2. Action Dispatcher 日志

```
Action Dispatcher Node ready (Refactored)
Loaded contract from robot_config: 1 actions, 6 joints: [joint1, joint2, ...]
Sent inference request for timestamp 1234567890.123456789
Appended 25 actions to queue
```

### 3. 队列大小监控

运行 `ros2 topic echo /action_dispatch/queue_size`，应该看到队列大小在 **30-80** 之间波动（锯齿状）。

### 4. Gazebo 中的机械臂

机械臂应该做**微小的正弦波摆动**（来自 mock 推理的假动作）。

## 手动验证步骤

### 步骤 1: 验证推理触发

在终端中运行：

```bash
# 查看 mock 推理服务的日志
# 应该看到周期性的推理请求

# 查看请求频率
ros2 topic hz /action_dispatch/diagnostic
```

**预期**: 推理请求频率应该约为 `chunk_size * (watermark_threshold / queue_size) Hz`，大约 0.5-1 Hz。

### 步骤 2: 验证动作分发频率

```bash
# 检查控制器指令频率（应该是 100Hz）
ros2 topic hz /velocity_controller/command
# 或
ros2 topic hz /position_controller/command
```

**预期**: 频率应该稳定在 **100 Hz**（±5 Hz），无论推理何时返回。

### 步骤 3: 验证队列状态

```bash
# 实时查看队列大小
ros2 topic echo /action_dispatch/queue_size
```

**预期**:
- 队列大小在 30-80 之间波动
- 当队列降到 30 以下时触发新的推理请求
- 推理返回后队列大小跳跃增加（+chunk_size）

### 步骤 4: 验证系统健康状态

```bash
# 查看诊断信息
ros2 topic echo /action_dispatch/diagnostic
```

**预期**:
- `level: 0` (OK)
- `message: "Dispatching normally"`

## 参数调整测试

### 测试 1: 调整 chunk_size

更大的 chunk_size 意味着每次推理返回更多动作，推理频率更低：

```bash
# 小 chunk (高频推理)
ros2 launch inference_service test_pull_based_dispatch.launch.py chunk_size:=10

# 大 chunk (低频推理)
ros2 launch inference_service test_pull_based_dispatch.launch.py chunk_size:=50
```

### 测试 2: 调整 watermark_threshold

更低的 threshold 意味着更早触发推理，但队列更容易耗尽：

```bash
# 保守策略（更早触发）
ros2 launch inference_service test_pull_based_dispatch.launch.py watermark_threshold:=40

# 激进策略（更晚触发）
ros2 launch inference_service test_pull_based_dispatch.launch.py watermark_threshold:=20
```

### 测试 3: 调整模拟推理延迟

编辑 `mock_inference_node.py`，修改 `self._inference_delay` 的值：

```python
self._inference_delay = 0.5  # 500ms 模拟慢速推理
```

然后重新编译并观察队列行为。

## 故障排查

### 问题 1: 没有推理请求

**可能原因**:
- Dispatcher 没有订阅到 `/joint_states` 话题
- 队列初始化未完成

**解决方案**:
```bash
# 检查 joint_states 是否发布
ros2 topic hz /joint_states

# 检查 dispatcher 是否启动
ros2 node list | grep action_dispatcher
```

### 问题 2: 找不到 Contract

**错误信息**:
```
Failed to find robot_config package
Robot config not found: .../so101.yaml
```

**解决方案**:
```bash
# 确保 robot_config 包已安装
ros2 pkg prefix robot_config

# 检查配置文件是否存在
ls $(ros2 pkg prefix robot_config)/share/robot_config/config/robots/
```

### 问题 3: 控制器不响应

**可能原因**:
- 控制器未启动
- 话题名称不匹配

**解决方案**:
```bash
# 检查控制器状态
ros2 control list_controllers

# 检查发布的话题
ros2 topic list | grep controller
```

### 问题 4: 队列持续增长或耗尽

**可能原因**:
- `chunk_size` 不合适
- 推理延迟太大
- `watermark_threshold` 设置不当

**解决方案**:
- 调整 `chunk_size` 使其与推理延迟匹配
- 降低 `watermark_threshold` 提前触发推理
- 检查 mock 推理的日志查看实际推理时间

## 性能指标

正常情况下的预期指标：

| 指标 | 预期值 | 说明 |
|------|--------|------|
| 推理请求频率 | 0.5-1 Hz | 取决于 chunk_size 和 watermark_threshold |
| 动作分发频率 | 100 Hz | 固定频率，由 dispatch timer 控制 |
| 队列大小 | 30-80 | 应该在 watermark_threshold 附近波动 |
| 系统健康状态 | OK (0) | 正常运行 |
| 推理延迟 | 100-150ms | Mock 节点的模拟延迟 |

## 下一步

测试通过后，你可以：

1. **替换为真实模型** - 将 `MockInferenceNode` 替换为 `PassiveInferenceNode` 并加载真实的 LeRobot 模型
2. **添加摄像头** - 确保 Contract 中包含图像观测，验证 StreamBuffer 的图像采样
3. **测试不同策略** - 尝试不同的 `on_inference_failure` 和 `on_queue_exhausted` 策略
4. **性能调优** - 调整队列大小、阈值等参数以优化系统性能
