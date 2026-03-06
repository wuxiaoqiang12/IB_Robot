# MoveIt Gateway Node 文档

## 概述

`moveit_gateway` 是 IB-Robot 项目中的核心运动控制节点，充当高层控制接口与底层 MoveIt2 运动规划框架之间的网关。该节点专门针对 SO101 5自由度机械臂设计，解决了 5DOF 机械臂在逆运动学（IK）求解中的特殊约束问题。

## 主要功能

### 1. 位姿控制 (Pose Control)
- **订阅话题**: `/cmd_pose` (geometry_msgs/Pose)
- **功能**: 接收目标位姿，通过 IK 求解关节角度，控制机械臂运动
- **特点**: 支持多种约束策略和分层容差机制

### 2. 末端执行器状态发布
- **发布话题**: `/robot_status/ee_pose` (geometry_msgs/PoseStamped)
- **功能**: 实时发布末端执行器在 base 坐标系中的位姿

### 3. 关节状态同步
- **订阅话题**: `/joint_states` (sensor_msgs/JointState)
- **功能**: 同步当前关节状态，作为 IK 求解的起始状态

## 5DOF 机械臂的特殊处理

### 问题背景

5自由度机械臂只有 5 个关节，无法满足完整的 6DOF 位姿约束（3个位置 + 3个姿态）。直接使用标准 IK 求解器会导致：
- `NO_IK_SOLUTION` 错误
- IK 求解成功率低

### 解决方案

#### 1. 仅位置 IK (Position-Only IK)

在 `kinematics.yaml` 中配置：
```yaml
position_only_ik: True
```

这使得 IK 求解器只考虑位置约束，忽略姿态约束。

#### 2. 姿态约束优化

虽然启用了 `position_only_ik`，我们仍然提供姿态参考，通过以下方法：

**a) Z轴约束 (constrain_to_z_axis_only)**
- 保持末端执行器 Z 轴方向不变
- 放松绕 Z 轴的旋转（roll）
- 这符合 5DOF 机械臂的能力：2DOF 用于 Z 轴方向（pitch + yaw），剩余 3DOF 用于位置

**b) 姿态投影 (project_orientation_to_shoulder_xz_plane)**
- 将姿态投影到 shoulder 坐标系的 XZ 平面
- 适应 5DOF 机械臂的运动学约束

#### 3. 分层容差策略

从严格到宽松依次尝试：

| 策略 | 容差 (x, y, z) | 说明 |
|------|----------------|------|
| Strict | (0.1, 0.1, 0.05) | X/Y: ±5.7°, Z: ±2.8° |
| Medium | (0.3, 0.3, 0.1) | X/Y: ±17°, Z: ±5.7° |
| Relaxed | (0.5, 0.5, 0.15) | X/Y: ±28°, Z: ±8.6° |
| Z-axis only | (1.0, 1.0, 0.2) | X/Y: ±57°, 几乎不约束 |
| No constraints | None | 完全无姿态约束 |

#### 4. 多策略 Fallback

```
1. Gripper Z-axis constraint → 尝试所有容差
2. Current orientation (保持当前姿态) → 尝试所有容差
3. Default orientation (无旋转) → 尝试所有容差
```

## 坐标系变换

### Shoulder 坐标系计算

为了准确判断工作空间可达性，节点将目标位置从 base 坐标系转换到 shoulder 坐标系：

```
P_shoulder = R × (P_base - T)
```

其中：
- `P_base`: 目标点在 base 坐标系中的位置
- `T`: shoulder 原点在 base 坐标系中的平移偏移
- `R`: base 到 shoulder 的旋转矩阵
- `P_shoulder`: 目标点在 shoulder 坐标系中的位置

### 日志输出

```
[INFO] Target Pose: x=-0.046, y=-0.000, z=0.423
[INFO]   Target in shoulder frame: x=-0.034, y=0.012, z=-0.323
[INFO]   Distance from base origin: 0.426 m
[INFO]   Distance from shoulder origin: 0.325 m
```

## 控制流程图

```
┌─────────────────────────────────────────────────────────────────────┐
│                         moveit_gateway 节点                         │
└─────────────────────────────────────────────────────────────────────┘
                              │
        ┌─────────────────────┴─────────────────────┐
        │                                           │
        ▼                                           ▼
┌───────────────┐                           ┌───────────────┐
│ /cmd_pose     │                           │ /joint_states │
│ (Pose)        │                           │ (JointState)  │
└───────┬───────┘                           └───────┬───────┘
        │                                            │
        ▼                                            │
┌───────────────┐                                    │
│cmd_pose_      │                                    │
│callback       │                                    │
└───────┬───────┘                                    │
        │                                            │
        ▼                                            │
    ┌─────────────────────────────┐                   │
    │  坐标系变换与距离计算        │                   │
    │  ┌─────────────────────┐   │                   │
    │  │ 1. 获取 TF 变换     │   │                   │
    │  │ 2. 计算 p_relative  │   │                   │
    │  │    = P_base - T     │   │                   │
    │  │ 3. 应用旋转变换     │   │                   │
    │  │    P_shoulder = R ×  │   │                   │
    │  │              p_relative │   │                   │
    │  │ 4. 计算距离         │   │                   │
    │  └─────────────────────┘   │                   │
    └──────────┬──────────────────┘                   │
               ▼                                      │
    ┌─────────────────────────────┐                   │
    │  多策略 IK 求解              │                   │
    │  ┌─────────────────────┐   │                   │
    │  │ 策略循环:           │   │                   │
    │  │ 1. Z轴约束          │   │                   │
    │  │ 2. 当前姿态         │   │                   │
    │  │ 3. 默认姿态         │   │                   │
    │  └─────────┬───────────┘   │                   │
    │            ▼                │                   │
    │  ┌─────────────────────┐   │                   │
    │  │ 容差循环:           │   │                   │
    │  │ 1. Strict (0.1)     │   │                   │
    │  │ 2. Medium (0.3)     │   │                   │
    │  │ 3. Relaxed (0.5)    │   │                   │
    │  │ 4. Z-only (1.0)     │   │                   │
    │  │ 5. None             │   │                   │
    │  └─────────┬───────────┘   │                   │
    └────────────┼────────────────┘                   │
                 ▼                                    │
    ┌─────────────────────────────┐                   │
    │  solve_and_move()           │                   │
    │  ┌─────────────────────┐   │                   │
    │  │ 1. 创建 Constraints │   │                   │
    │  │    (如果有容差)     │   │                   │
    │  │ 2. 调用 compute_ik_ │   │                   │
    │  │    async()          │   │                   │
    │  │ 3. 等待结果         │   │                   │
    │  │ 4. move_to_joint()  │   │                   │
    │  └─────────────────────┘   │                   │
    └────────────┬────────────────┘                   │
                 ▼                                    │
        ┌───────────────────┐                         │
        │  IK 成功？        │                         │
        └───┬───────┬───────┘                         │
            │ YES   │ NO                              │
            ▼       ▼                                 │
    ┌───────────┐ ┌───────────────┐                   │
    │ 移动成功   │ │ 尝试下一个    │                   │
    │           │ │ 策略/容差     │                   │
    └───────────┘ └───────────────┘                   │
                 │                                    │
                 ▼                                    ▼
    ┌────────────────────────────────────────────────┐
    │              发布 /robot_status/ee_pose        │
    │              (10Hz 定时器)                     │
    └────────────────────────────────────────────────┘
```

## 约束创建流程

```
┌─────────────────────────────────────────────────────────────┐
│                    create_orientation_constraint             │
├─────────────────────────────────────────────────────────────┤
│  输入:                                                       │
│    - target_quat: 目标四元数 (x, y, z, w)                   │
│    - link_name: "gripper"                                   │
│    - frame_id: "base"                                       │
│    - tolerances: (x_tol, y_tol, z_tol)                      │
│                                                              │
│  输出:                                                       │
│    - OrientationConstraint 消息                             │
│                                                              │
│  字段设置:                                                   │
│    - orientation.x/y/z/w: 目标姿态                          │
│    - absolute_x_axis_tolerance: X轴容差 (放松 roll)         │
│    - absolute_y_axis_tolerance: Y轴容差 (放松 roll)         │
│    - absolute_z_axis_tolerance: Z轴容差 (保持方向)          │
│    - weight: 1.0 (约束权重)                                 │
└─────────────────────────────────────────────────────────────┘
```

## 四元数数学工具

### quaternion_multiply(q1, q2)
四元数乘法：q = q1 × q2

### quaternion_conjugate(q)
四元数共轭：q* = [-x, -y, -z, w]

### quaternion_to_rotation_matrix(q)
四元数转 3×3 旋转矩阵

### rotation_matrix_to_quaternion(R)
3×3 旋转矩阵转四元数（Shepperd 方法）

## 配置文件

### kinematics.yaml
```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.01
  kinematics_solver_timeout: 2.0
  kinematics_solver_attempts: 50
  position_only_ik: True  # 5DOF 机械臂关键配置
```

### moveit_controllers.yaml
```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_trajectory_controller
    - gripper_trajectory_controller

  arm_trajectory_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints: ["1", "2", "3", "4", "5"]

  gripper_trajectory_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints: ["6"]
```

## ROS2 话题接口

### 订阅话题

| 话题名 | 消息类型 | 频率 | 说明 |
|--------|----------|------|------|
| /cmd_pose | geometry_msgs/Pose | 按需 | 目标位姿命令 |
| /joint_states | sensor_msgs/JointState | 100Hz | 关节状态反馈 |

### 发布话题

| 话题名 | 消息类型 | 频率 | 说明 |
|--------|----------|------|------|
| /robot_status/ee_pose | geometry_msgs/PoseStamped | 10Hz | 末端执行器位姿 |

## 使用示例

### 发送位姿命令

```bash
ros2 topic pub /cmd_pose geometry_msgs/Pose "{
  position: {x: 0.15, y: 0.0, z: 0.25},
  orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}
}" --once
```

### 查看末端位姿

```bash
ros2 topic echo /robot_status/ee_pose
```

## 多线程模型

节点使用 `MultiThreadedExecutor` 和 `ReentrantCallbackGroup` 实现线程安全：

- **回调组**: 所有订阅者使用同一个可重入回调组
- **执行器**: 多线程执行器处理并发回调
- **线程安全**: IK 调用使用 async 模式，避免内部 spin_once 调用

## 日志级别

- **INFO**: 目标位姿、距离信息、IK 成功
- **WARNING**: IK 失败、TF 查询失败、无效关节状态
- **DEBUG**: 关节状态更新、策略尝试详情

## 故障排查

### IK 持续失败

1. 检查位置是否超出工作空间（shoulder 距离应 < 0.35m）
2. 确认 `position_only_ik: True` 已设置
3. 查看日志中的距离信息

### TF 查询失败

1. 确认 robot_state_publisher 正在运行
2. 检查 URDF 中是否定义了 shoulder link
3. 使用 `ros2 run tf2_tools view_frames` 查看 TF 树

### 关节状态无效

- 确认 joint_states 包含所需的 5 个关节
- 检查关节名称是否为 ['1', '2', '3', '4', '5']

## 版本历史

- **v7**: 多线程执行模型，修复 "wait set index too big" 错误
- **v8**: 添加 shoulder XZ 平面姿态投影，优化 5-DOF IK 约束
- **v9**: 修正坐标系变换，添加平移补偿，增加距离日志
- **v10**: 移除关节控制功能 (/cmd_joint)，简化接口
