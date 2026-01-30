# SO-101 机械臂硬件 C++ 插件

SO-101 机械臂的 native C++ ros2_control 硬件接口插件，通过 Feetech 伺服驱动器直接通信。

## 概述

本包为 SO-101 机械臂提供了 ros2_control 硬件接口的原生 C++ 实现。它使用 FTServo SDK 与 Feetech 伺服电机直接通信，相比基于话题的实现方式，提供更好的性能和更低的延迟。

## 特性

- 通过 FTServo SDK 直接与伺服电机通信
- 原生 ros2_control SystemInterface 实现
- 支持 JSON 格式的标定文件
- 生命周期管理（配置、激活、停用）
- 实时位置读写操作
- 可配置的启动位置，避免机械臂启动时跳到零位
- 关闭时自动禁用电机扭矩，确保安全

## 架构

```
ros2_control → SO101SystemHardware (C++) → FTServo SDK → Feetech 伺服电机
```

## 依赖

### Git Submodule

FTServo_Linux SDK 作为 git submodule 引入：

```bash
# 初始化并更新 submodule
git submodule update --init --recursive
```

源仓库：`git@github.com:ftservo/FTServo_Linux.git`

### 系统依赖

- ROS 2 Humble
- nlohmann_json 库
- hardware_interface
- pluginlib
- rclcpp_lifecycle

## 编译

```bash
cd ~/Research/ledog_ros2/ros2_ws
colcon build --packages-select so101_hardware_cpp
source install/setup.zsh
```

FTServo SDK 会自动作为静态库编译，无需手动构建。

## 配置

### URDF 参数

插件在 URDF 中需要以下参数：

```xml
<hardware>
  <plugin>so101_hardware_cpp/SO101SystemHardware</plugin>
  <param name="port">/dev/ttyACM0</param>
  <param name="calib_file">$(env HOME)/.calibrate/so101_follower_calibrate.json</param>
  <param name="reset_positions">$(arg reset_positions)</param>
</hardware>
```

每个关节必须有 `id` 参数：

```xml
<joint name="1">
  <param name="id">1</param>
  ...
</joint>
```

### 启动位置配置（可选）

`reset_positions` 参数允许你指定机械臂启动时的初始关节位置。这在以下场景中非常有用：

- 安装在机器狗上的机械臂，高度空间有限
- 确保机械臂从安全、已知的配置启动
- 避免启动时突然移动到零位

**格式**：JSON 字符串，关节 ID 作为键，弧度值作为值。

**示例**（安全折叠位置，对应 motor_bridge 的 reset_normalized_goals）：
```bash
ros2 launch so101_hw_interface so101_hw_dual_mode.launch.py \
  use_sim:=false \
  use_cpp_plugin:=true \
  port:=/dev/ttyACM0 \
  "reset_positions:='{\"1\": 0.0813, \"2\": 3.7905, \"3\": 7.0379, \"4\": -0.6228, \"5\": 2.3869}'"
```

这些值是从 `motor_bridge.py` 的安全复位位置转换而来的弧度值。

**行为**：
- 如果提供了 `reset_positions` → 机械臂在启动时移动到指定位置
- 如果 `reset_positions` 为空（默认）→ 机械臂保持当前电机位置

**注意**：位置单位为弧度，与关节状态接口一致。

### 标定文件

插件从 JSON 文件读取标定数据（默认：`~/.calibrate/so101_follower_calibrate.json`）：

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

使用以下命令生成标定文件：

```bash
ros2 run so101_hw_interface so101_calibrate_arm --arm follower --port /dev/ttyACM0
```

或使用标定服务：

```bash
ros2 run so101_hw_interface so101_calibration_service --ros-args -p arm_type:=follower -p port:=/dev/ttyACM0
ros2 service call /calibrate_follower std_srvs/srv/Trigger
```

## 使用方法

### 使用 C++ 插件启动

```bash
ros2 launch so101_hw_interface so101_hw_dual_mode.launch.py \
  use_sim:=false \
  use_cpp_plugin:=true \
  port:=/dev/ttyACM0
```

### 使用话题模式启动（默认）

```bash
ros2 launch so101_hw_interface so101_hw_dual_mode.launch.py use_sim:=false
```

### 使用自定义启动位置启动

```bash
ros2 launch so101_hw_interface so101_hw_dual_mode.launch.py \
  use_sim:=false \
  use_cpp_plugin:=true \
  port:=/dev/ttyACM0 \
  "reset_positions:='{\"1\": 0.0813, \"2\": 3.7905, \"3\": 7.0379, \"4\": -0.6228, \"5\": 2.3869}'"
```

## 实现细节

### 生命周期状态

1. **on_init**：读取 URDF 参数（端口、标定文件、关节 ID）
2. **on_configure**：加载标定文件（JSON 格式）
3. **on_activate**：
   - 连接到电机
   - 如果配置了 `reset_positions`：移动到指定位置
   - 否则：读取并保持当前电机位置
4. **on_deactivate**：
   - 禁用所有电机扭矩
   - 等待 100ms 确保扭矩完全禁用
   - 断开与串口的连接

### 读写操作

- **read()**：读取电机位置并转换为弧度
- **write()**：将关节命令从弧度转换为电机位置

### 坐标转换

```cpp
// 电机位置转弧度
double radians = ((pos - range_min) / range - 0.5) * 2.0 * M_PI;

// 弧度转电机位置
s16 pos = (radians / (2.0 * M_PI) + 0.5) * range + range_min;
```

## 对比：话题模式 vs C++ 插件

| 特性 | 话题模式 | C++ 插件 |
|------|------------------|-------------------------|
| 延迟 | 较高（话题桥接） | 较低（直连） |
| 性能 | Python 开销 | 原生 C++ |
| 设置 | 需要 motor_bridge | 直接连接 |
| 兼容性 | 标准 ros2_control | 标准 ros2_control |
| 适用场景 | 开发/测试 | 生产环境 |
| 启动位置 | 支持 | 支持 |
| 安全关机 | 支持 | 支持 |

## 故障排除

### 插件未找到

确保包已编译并 source：

```bash
colcon build --packages-select so101_hardware_cpp
source install/setup.zsh
```

### 标定文件未找到

在启动前生成标定文件：

```bash
ros2 run so101_hw_interface so101_calibrate_arm --arm follower --port /dev/ttyACM0
```

### 电机连接失败

检查串口权限：

```bash
sudo chmod 666 /dev/ttyACM0
```

或将用户添加到 dialout 组：

```bash
sudo usermod -a -G dialout $USER
```

### Submodule 未初始化

如果 FTServo_Linux 目录为空：

```bash
cd ~/Research/ledog_ros2
git submodule update --init --recursive
```

## 文件说明

- [so101_system_hardware.hpp](include/so101_hardware_cpp/so101_system_hardware.hpp) - 头文件
- [so101_system_hardware.cpp](src/so101_system_hardware.cpp) - 实现文件
- [CMakeLists.txt](CMakeLists.txt) - 构建配置
- [so101_hardware_cpp_plugin.xml](so101_hardware_cpp_plugin.xml) - 插件描述
- [FTServo_Linux](FTServo_Linux/) - Feetech SDK（git submodule）

## 许可证

TODO: 许可证声明
