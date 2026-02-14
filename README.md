# IB-Robot

> IB-Robot (Intelligence Boom Robot): 融合 LeRobot 与 ROS 2 生态的智能具身机器人开发框架

## 项目定位

IB-Robot 是一个**智能融合机器人开发框架**，旨在打通 Hugging Face LeRobot 机器学习生态与 ROS 2 机器人中间件之间的壁垒，为具身智能研发提供从采集、训练到部署的完整工具链。

### 核心融合能力

| 维度 | LeRobot 生态 | ROS 2 生态 | IB-Robot 融合方案 |
|------|-------------|-----------|------------------|
| **数据流** | Episode 回合 | Topic 话题 | 契约驱动的双向实时转换 |
| **时间观** | 离散时间步 (Steps) | 连续时间流 (RT) | 自动对齐与高频插值平滑 |
| **控制方式** | 端到端神经网络模型 | 分层规划控制架构 | **双模控制 (ACT vs MoveIt)** |
| **部署形态** | Python 脚本 | ROS 2 节点 | 分布式端边协同部署 |

## 系统架构

![IB-Robot 架构图](docs/pictures/architecture.png)

### 架构深度解析

IB-Robot 构建了一个从感知、决策到执行的端到端闭环体系，实现了机器学习世界与机器人控制世界的无缝对接：

1. **多模态感知与采集**:
   - **底层感知**: 通过 ROS 2 Driver 统一接入多路相机 (USB/RealSense)、雷达及麦克风。
   - **多样化采集**: 支持 **VR 手柄、Xbox 控制器及手机 IMU** 等遥操作设备，为模仿学习提供专家示范数据。

2. **协议转换枢纽 (tensormsg)**:
   - 作为架构的枢纽，tensormsg 负责 `ros_msg` 与 `tensor` 之间的双向转换，通过合约（Contract）机制保证数据流的类型安全与一致性。

3. **推理与研发服务 (Inference Service)**:
   - 支持各类 VLA（视觉-语言-动作）大模型（如 SmolVLA, Pi0.5）以及端到端策略模型（如 ACT, Diffusion Policy）。

4. **统一动作执行器 (Action Dispatch)**:
   - 充当机器人的“小脑”。在 ACT 模式下负责 Action Chunking 调度与高频插值；在规划模式下对接 MoveIt 2 执行受限轨迹，并提供统一的 `RobotStatus` 汇报。

5. **配置驱动中心 (robot_config)**:
   - 实现“规格驱动本体行为”。通过单一 YAML 定义关节、控制器模式及传感器外参，支持一键切换仿真与实机环境。

---

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
├── src/                            # [子模块] 核心源码包集合
│   ├── robot_config/               # 系统总控、规格定义与启动入口
│   ├── action_dispatch/            # 统一动作执行器 (双模支持)
│   ├── tensormsg/                  # LeRobot ↔ ROS 2 协议转换枢纽
│   ├── robot_description/          # 统一机器人 URDF/SRDF 模型描述
│   ├── robot_moveit/               # MoveIt 2 运动规划集成
│   ├── inference_service/          # 多模型推理与部署服务
│   ├── so101_hardware/             # SO-101 电机驱动接口
│   ├── robot_interface/            # [已废弃] 机器人适配抽象层 (由 robot_config 替代)
│   ├── tensormsg_interfaces/       # [已废弃] 系统统一接口定义 (由各模块内部定义替代)
│   └── workflows/                  # CI/CD 配置
│
├── docs/                           # 深度架构文档与开发指南
├── scripts/                        # 环境配置与验证工具脚本
└── build/                          # 编译输出 (自动创建)
```

## 快速开始

### 0. 环境准备 (重要)
- **系统**: Ubuntu 22.04 + ROS 2 Humble
- **Python**: 系统原生 **Python 3.10** (严禁在活跃的 Conda 环境中运行)

### 1. 编译项目

使用一键脚本初始化环境并编译：

```bash
# 初始化子模块、安装依赖、配置 venv
./scripts/setup.sh

# 编译工作空间
./scripts/build.sh
```

### 2. 运行项目

**启动仿真控制模式 (MoveIt 规划模式)：**

```bash
source install/setup.sh
ros2 launch robot_config robot.launch.py \
    robot_config:=so101_single_arm \
    use_sim:=true \
    control_mode:=moveit_planning
```

**启动 ACT 实时推理模式 (实机)：**

```bash
# 确保硬件已连接
source install/setup.sh
ros2 launch robot_config robot.launch.py \
    robot_config:=so101_single_arm \
    control_mode:=teleop_act
```

---

## 路线图 (Roadmap)

- [x] **双模控制引擎**: 完美集成 ACT 流式控制与 MoveIt 轨迹控制。
- [x] **配置一致性校验**: 引入 `validate_config` 工具防止配置漂移。
- [ ] **多机器人协同**: 支持双臂及移动底座的规格化定义。
- [ ] **在线数据回流**: 实现从演示采集到自动导出 LeRobot 数据集的闭环。

---

**维护者**: IB-Robot Team  
**项目地址**: https://gitcode.com/BreezeWu/IB_Robot  
**反馈**: https://gitcode.com/BreezeWu/IB_Robot/issues
