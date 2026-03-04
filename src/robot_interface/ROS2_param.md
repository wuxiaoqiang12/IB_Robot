# Hardware Interface 启动参数配置

本文档详细说明如何使用 ROS2 参数来配置 `hardware_interface` 包的启动参数，包括 YAML 配置文件的使用和参数覆写方法。
> 对于初次上手的同学, 可以看完本文后直接参考并按需修改 [single_arm_banana.yaml](../../src/hardware_interface_py/config/scenes/single_arm_banana.yaml)


## 概述

`hardware_interface` 包的主节点 `robot_interface` 通过 ROS2 参数系统来配置机器人硬件，支持：

- 多机器人同时控制
- 摄像头配置
- Mock 模式（用于测试）
- 灵活的参数覆写机制

---

## 参数结构

### 全局参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `is_mock` | bool | `false` | 是否使用 Mock 模式（不连接真实硬件） |
| `robot_ids` | string[] | `[]` | 机器人 ID 列表，用于初始化多个机器人 |

### 机器人基本参数

每个机器人使用 `robot_id` 作为命名空间前缀，例如 `so101_follower_1`：

| 参数名 | 类型 | 必需 | 默认值 | 说明 |
|--------|------|------|--------|------|
| `{robot_id}.type` | string | ✓ | - | 机器人类型（如 `so101_follower`） |
| `{robot_id}.port` | string | ✓ | - | 串口端口（如 `/dev/ttyACM0`） |
| `{robot_id}.use_degrees` | bool | ✗ | `false` | 是否使用角度制（默认弧度制） |
| `{robot_id}.camera_names` | string[] | ✗ | `[]` | 摄像头名称列表 |

### 摄像头参数

每个摄像头使用 `{robot_id}.cameras.{camera_name}` 作为命名空间：

| 参数名 | 类型 | 必需 | 默认值 | 说明 |
|--------|------|------|--------|------|
| `{robot_id}.cameras.{camera_name}.index` | int | ✓ | - | 摄像头设备索引 |
| `{robot_id}.cameras.{camera_name}.width` | int | ✓ | - | 图像宽度（像素） |
| `{robot_id}.cameras.{camera_name}.height` | int | ✓ | - | 图像高度（像素） |
| `{robot_id}.cameras.{camera_name}.fps` | int | ✗ | `30` | 帧率 |

---

## YAML 配置文件

用户可根据场景自己编写 yaml 配置文件. **在 hardware_interface_py 包的 config 目录下也提供了双臂三摄像头的配置文件。
(比如适用于双臂抓笔放到盒子中的场景)**


### 配置示例

```yaml
robot_interface:
  ros__parameters:
    is_mock: false
    robot_ids: [so101_follower_1, so101_follower_2]
    
    so101_follower_1:
      type: so101_follower
      port: /dev/ttyACM0
      camera_names: [top, leftwrist, rightwrist]
      cameras:
        top:
          index: 0
          width: 640
          height: 480
          fps: 30
        leftwrist:
          index: 2
          width: 640
          height: 480
          fps: 30
        rightwrist:
          index: 4
          width: 640
          height: 480
          fps: 30

    so101_follower_2:
      type: so101_follower
      port: /dev/ttyACM1
```

---


### 启动方法

```bash
# 使用 YAML 配置文件(推荐)
ros2 run hardware_interface_py launch --ros-args --params-file /path/to/config.yaml

# 也可以复写单个参数, 比如想完全使用某个 yaml 文件, 但是改成 mock 方式, 则:
ros2 run hardware_interface_py launch --ros-args --params-file /path/to/config.yaml -p is_mock:=true

# 也可以直接在命令行配置, 不过看起来比较麻烦
ros2 run hardware_interface_py launch --ros-args \
 -p is_mock:=false \
 -p robot_ids:=[so101_follower_1, so101_follower_2] \
 -p so101_follower_1.type:=so101_follower \
 -p so101_follower_1.port:=/dev/ttyACM0 \
 -p so101_follower_1.camera_names:=[top, leftwrist, rightwrist] \
 -p so101_follower_1.cameras.top.index:=0 \
 -p so101_follower_1.cameras.top.width:=640 \
```



