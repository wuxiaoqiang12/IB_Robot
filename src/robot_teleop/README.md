# robot_teleop

极简的串口到控制器桥接，用于零延迟遥操作。

## 概述 (Overview)

`robot_teleop` 包为 IB-Robot 提供了一个统一的遥操作接口，通过设备抽象层支持多种遥操作设备（示教臂、游戏手柄、VR 控制器）。

**核心特性:**
- ✅ 零延迟控制 (端到端 < 5ms)
- ✅ 基于工厂模式的设备抽象
- ✅ 具备关节限位的安全过滤
- ✅ 通过 `robot_config` 驱动的配置
- ✅ 支持自动的 rosbag 录制
- ✅ 与 `robot_config` 启动系统深度集成

## 架构 (Architecture)

```text
Leader Arm (Serial) → LeaderArmDevice → SafetyFilter → TeleopNode → ROS 2 Controllers
```

**数据流:**
1. 设备读取关节位置 (50 Hz)
2. 应用校准和映射
3. 安全过滤器强制执行关节限位
4. 发布到 `/arm_position_controller/commands` 等
5. ROS 2 控制器执行运动

## 安装 (Installation)

```bash
# 编译
colcon build --packages-select robot_teleop --merge-install

# 刷新环境
source install/setup.bash
```

## 使用说明 (Usage)

### 1. 集成模式 (推荐)

通过 `robot_config` 启动并开启遥操作支持：

**配置** (在 `src/robot_config/config/robots/so101_single_arm.yaml` 中):

```yaml
robot:
  control_modes:
    teleop:
      description: "Human teleoperation mode (direct control)"
      controllers:
        - joint_state_broadcaster
        - arm_position_controller
        - gripper_position_controller
      inference:
        enabled: false
        force_disable: true

  teleoperation:
    enabled: true
    active_device: "so101_leader"
    devices:
      - name: "so101_leader"
        type: "leader_arm"
        port: "/dev/ttyACM1"
        calib_file: "$(env HOME)/.calibrate/so101_leader_calibrate.json"
    safety:
      joint_limits:
        "1": {"min": -3.14, "max": 3.14}
        "2": {"min": -1.57, "max": 1.57}
        # ... 更多关节
```

**启动:**

```bash
# 遥操作模式
ros2 launch robot_config robot.launch.py \
    robot_config:=so101_single_arm \
    control_mode:=teleop \
    use_sim:=false

# 附带自动录制
ros2 launch robot_config robot.launch.py \
    robot_config:=so101_single_arm \
    control_mode:=teleop \
    record:=true \
    use_sim:=false
```

### 2. 独立模式 (用于测试)

```bash
ros2 launch robot_teleop teleop_device.launch.py \
    port:=/dev/ttyACM1 \
    calib_file:=~/.calibrate/so101_leader_calibrate.json \
    control_frequency:=50.0
```

## 配置 Schema (Configuration Schema)

### 遥操作部分 (Teleoperation Section)

```yaml
robot:
  teleoperation:
    enabled: bool                    # 启用遥操作 (默认: true)
    active_device: string            # 激活的设备名称

    devices:
      - name: string                 # 唯一的设备名称
        type: string                 # 设备类型 (leader_arm, xbox_controller, vr_device)
        ...device-specific params... # 其他设备特定参数

    safety:
      joint_limits: dict             # 安全过滤器的关节限位
      estop_topic: string            # 紧急停止话题 (默认: /emergency_stop)
```

### 设备类型 (Device Types)

#### 1. leader_arm (SO-101 示教臂)

```yaml
- name: "so101_leader"
  type: "leader_arm"
  port: string                       # 串口 (例如: /dev/ttyACM1)
  calib_file: string                 # 校准 JSON 文件路径 (可选)
  joint_mapping: dict                # Leader → follower 关节映射 (可选)
```

**示例:**
```yaml
devices:
  - name: "so101_leader"
    type: "leader_arm"
    port: "/dev/ttyACM1"
    calib_file: "~/.calibrate/so101_leader_calibrate.json"
    joint_mapping:
      "1": "1"  # Leader joint 1 → Follower joint 1
      "2": "2"
      "3": "3"
      "4": "4"
      "5": "5"
      "6": "6"
```

#### 2. xbox_controller (规划中)

```yaml
- name: "xbox"
  type: "xbox_controller"
  max_velocity: float              # 最大关节速度 rad/s (默认: 0.5)
  control_mode: string             # "delta" 或 "absolute" (默认: "delta")
  deadman_button: string           # 死区按钮名称 (默认: "A")
```

#### 3. vr_controller (规划中)

```yaml
- name: "vr_controller"
  type: "vr_device"
  ... 待定 ...
```

### 验证规则 (Validation Rules)

1. **必填字段:**
   - `teleoperation.enabled` 必须为 true 才能启用遥操作
   - 启用时必须指定 `teleoperation.active_device`
   - 每个设备必须具有 `name` 和 `type` 字段

2. **设备特定要求:**
   - `leader_arm` 设备需要 `port` 字段
   - `xbox_controller` 需要订阅 `/joy` 话题
   - `vr_device` 需要集成 IK 求解器

3. **安全要求:**
   - `joint_limits` 应该覆盖 `robot.joints.all` 中的所有关节
   - 每个关节限位需要 `min` 和 `max` 字段
   - `min` 必须小于 `max`

## 话题 (Topics)

**由 TeleopNode 发布:**
- `/arm_position_controller/commands` (Float64MultiArray) - 50 Hz
- `/gripper_position_controller/commands` (Float64MultiArray) - 50 Hz
- `/diagnostics` (DiagnosticArray) - 1 Hz

**由 TeleopNode 订阅:**
- `/emergency_stop` (Bool) - 紧急停止信号

## 安全性 (Safety)

**关节限位强制执行:**
- 所有命令都要经过 `SafetyFilter`
- 超过限制的命令会被裁剪到最近的边界
- 对被裁剪的命令发出诊断警告

**紧急停止 (Emergency Stop):**
- 订阅 `/emergency_stop` 话题
- 当急停处于激活状态时停止发布命令
- 急停清除后恢复

## 性能目标 (Performance Targets)

- **控制循环频率:** 50 Hz
- **端到端延迟:** < 5ms (设备读取 → 话题发布)
- **串口通信:** < 2ms/周期
- **安全过滤:** < 0.5ms/周期

## 故障排除 (Troubleshooting)

### 问题: "Controller not responding" (控制器未响应)

**解决方案:** 验证控制器已生成 (spawned):
```bash
ros2 control list_controllers
# 应该显示: arm_position_controller[active]
```

### 问题: "Serial port permission denied" (串口权限被拒绝)

**解决方案:**
```bash
sudo chmod 666 /dev/ttyACM1
# 或者将用户添加到 dialout 用户组
sudo usermod -a -G dialout $USER
```

### 问题: "Teleop node not starting" (遥操作节点未启动)

**解决方案:** 检查配置:
1. 验证 YAML 中 `teleoperation.enabled: true`
2. 验证 `teleoperation.active_device` 是否与设备名称匹配
3. 验证设备 `type` 已在 `DEVICE_MAP` 中注册

## 文档 (Documentation)

- **集成指南:** [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md)
- **集成状态:** [INTEGRATION_COMPLETE.md](INTEGRATION_COMPLETE.md)
- **实施状态:** [IMPLEMENTATION_STATUS.md](IMPLEMENTATION_STATUS.md)

## 包结构 (Package Structure)

```text
src/robot_teleop/
├── robot_teleop/              # 核心 Python 模块
│   ├── __init__.py
│   ├── base_teleop.py       # 抽象设备接口
│   ├── config_loader.py     # 配置工具
│   ├── device_factory.py    # 工厂模式
│   ├── safety_filter.py     # 安全层
│   ├── teleop_node.py       # 主 ROS 2 节点
│   └── devices/
│       └── leader_arm.py    # SO-101 示教臂驱动
├── launch/
│   └── teleop_device.launch.py  # 独立启动文件
├── package.xml
└── setup.py
```

## 相关软件包 (Related Packages)

- **robot_config**: 配置管理与启动系统
- **inference_service**: 自动控制的模型推理
- **action_dispatch**: 动作执行和分发
- **so101_hardware**: SO-101 硬件接口

## 许可证 (License)

Apache-2.0

## 维护者 (Maintainer)

IB-Robot Team
