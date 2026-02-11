# IB Robot

> 融合 LeRobot 与 ROS2 生态的智能机器人开发框架

## 项目定位

IB Robot 是一个**智能融合机器人开发框架**，旨在打通 Hugging Face LeRobot 机器学习生态与 ROS2 机器人中间件之间的壁垒，为具身智能研发提供完整的工具链。

### 核心融合能力

| 维度 | LeRobot 生态 | ROS2 生态 | IB Robot 融合方案 |
|------|-------------|-----------|------------------|
| **数据流** | Episode 回合 | Topic 话题 | 契约驱动的双向转换 |
| **时间观** | 离散时间步 | 连续时间流 | 时间对齐与插值平滑 |
| **控制方式** | 端到端神经网络 | 分层控制架构 | Action Chunking 调度 |
| **部署形态** | Python 脚本 | ROS2 节点 | 容器化微服务集群 |

## 系统架构

![IB Robot 架构图](docs/pictures/architecture.png)

### 架构深度解析

如上图所示，IB Robot 构建了一个从感知、决策到执行的端到端闭环体系，充分利用了 ROS 2 的实时通信能力与 LeRobot 的强大推理能力：

1. **多模态感知与采集**:
   - **底层感知**: 通过 ROS 2 Driver 统一接入相机、雷达及麦克风，屏蔽硬件差异。
   - **多样化采集**: 支持 **VR 手柄、Xbox 控制器及手机 IMU** 等多种遥操作设备，采集的数据经过预处理后，由 **TensorMsg** 转换为 LeRobot 标准格式。

2. **核心中间件 (TensorMsg)**:
   - 作为架构的枢纽，TensorMsg 负责 `ros_msg` 与 `tensor` 之间的双向实时转换，实现了机器学习世界与机器人控制世界的无缝对接。

3. **推理与调度**:
   - **Inference Service**: 支持各类 VLA（视觉-语言-动作）大模型（如 ACT, Pi0.5, SmolVLA）以及未来的 VLN（视觉语言导航）模型。
   - **Action Dispatch**: 引入了**时序集成 (Temporal Ensembling)** 和**跨帧平滑**算法，有效解决大模型输出的高频抖动问题，并支持 RTC (Real-Time Control) 增强以提升实时性。

4. **控制与执行**:
   - 利用 **ros2_control** 的 Hardware Interface 插件化架构，同一套控制逻辑可无缝分发至 **Gazebo/Isaac Sim/Mujoco** 仿真环境或 **So10x/LeKiwi/LeDog** 真实硬件。

5. **数据总线扩展**:
   - 架构底层不仅支持标准的 ROS 2 DDS，还为集成 **AGIROS、DORA、AimRT** 等高性能数据总线预留了扩展接口。

---

### 功能实现状态

#### ✅ 已核心实现 (Core Features)

| 模块 | 功能点 | 说明 |
|------|-------|------|
| **robot_config** | 系统总控 | 统一管理 URDF、参数及启动流程，支持一键切换 Sim/Real |
| **inference_service** | VLA 推理 | 支持 ACT, Diffusion, VQ-BeT, SmolVLA 等策略的高效推理 |
| **action_dispatch** | 动作调度 | 实现基础 Action Chunking 及线性插值平滑 |
| **tensormsg** | 协议桥接 | 实现 LeRobot 契约与 ROS 2 话题的严格对齐与转换 |
| **so101_hardware** | 实机驱动 | 基于 `FetchContent` 集成飞特舵机 SDK，稳定控制 SO-101 |
| **仿真环境** | Gazebo 支持 | 提供完整的物理仿真环境与传感器模拟 |

#### 🚧 规划与开发中 (Roadmap)

| 模块 | 规划功能 | 预期目标 |
|------|---------|----------|
| **数据采集** | 多源遥操作 | 增加对 VR 设备及手机 IMU 的遥操作支持 (目前仅支持基础数据流) |
| **动作分发** | 高级平滑 | 实现更复杂的**时序集成 (Temporal Ensembling)** 算法以提升动作细腻度 |
| **导航集成** | VLN & Nav2 | 融合视觉语言导航与 ROS 2 Nav2 导航栈，实现移动操作 |
| **中间件扩展** | 多总线支持 | 探索对 AGIROS / DORA / AimRT 等高性能数据总线的支持 |
| **仿真扩展** | Isaac Sim | 适配 NVIDIA Isaac Sim 以支持大规模强化学习训练 |

## 仓库结构

```
IB_Robot/                           # 主工作空间 (本仓库)
├── .gitmodules                     # Git 子模块配置
├── README.md                       # 本文件
├── LICENSE                         # Apache 2.0 许可证
│
├── libs/                           # 外部依赖库
│   └── lerobot/                    # [子模块] LeRobot 训练框架
│
├── src/                            # [子模块] ROS2 包集合 (核心源码)
│   ├── robot_config/               # 系统总控与启动入口
│   ├── inference_service/          # 多模型推理服务
│   ├── action_dispatch/            # 动作调度与平滑
│   ├── tensormsg/                  # LeRobot ↔ ROS2 协议转换 (原 rosetta)
│   ├── so101_hardware/             # SO-101 硬件驱动
│   ├── lerobot_description/        # URDF 与仿真配置
│   ├── tensormsg_interfaces/       # 消息/服务/动作定义 (原 rosetta_interfaces)
│   └── workflows/                  # CI/CD 配置
│
├── docs/                           # 文档
│   ├── architecture.md            # 架构详细说明
│   └── pictures/                  # 图片资源
│
└── build/                          # 编译输出 (自动创建)
```

## 快速开始

### 1. 编译项目

本项目使用 Git 子模块管理，请使用提供的脚本一键初始化：

```bash
# 初始化环境、拉取子模块、安装依赖
./scripts/setup.sh

# 编译工作空间
./scripts/build.sh
```

### 2. 运行项目

**启动仿真（无需硬件）：**

```bash
source install/setup.sh
ros2 launch robot_config robot.launch.py \
    robot_name:=so101_single_arm \
    use_sim:=true
```

**连接实机：**

```bash
# 确保 SO-101 机械臂已连接并上电
source install/setup.sh
ros2 launch robot_config robot.launch.py \
    robot_name:=so101_single_arm \
    use_sim:=false
```

## 许可证

本项目采用 [Apache License 2.0](LICENSE)。

---

**维护者**: IB Robot Team  
**项目地址**: https://gitcode.com/openeuler/IB_Robot  
**问题反馈**: https://gitcode.com/openeuler/IB_Robot/issues
