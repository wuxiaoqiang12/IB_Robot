# IB-Robot

> IB-Robot (Intelligence Boom Robot): 融合 LeRobot 与 ROS 2 生态的智能具身机器人开发框架

## 项目定位

IB-Robot 是一个**智能融合机器人开发框架**，旨在打通 Hugging Face LeRobot 机器学习生态与 ROS 2 机器人中间件之间的壁垒，为具身智能研发提供从采集、训练到部署的完整工具链。

### 核心融合能力

| 维度 | LeRobot 生态 | ROS 2 生态 | IB-Robot 方案 |
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
   - 支持各类 VLA（视觉-语言-动作）大模型（如 SmolVLA, Pi0.5）以及端到端策略模型（如 ACT, Diffusion Policy）。系统支持 **自动检测后端** 并根据控制模式按需启动。

4. **统一动作执行器 (Action Dispatch)**:
   - 充当机器人的“小脑”。在 ACT 模式下负责 **Action Chunking** 调度与高频插值；在规划模式下对接 **MoveIt 2** 执行受限轨迹，并提供统一的 `RobotStatus` 汇报。

5. **配置驱动中心 (robot_config)**:
   - 实现“规格驱动本体行为”。通过单一 YAML 定义关节、控制器模式及传感器外参，支持一键切换仿真与实机环境。

---

## 仓库结构

```text
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
│   ├── tensormsg/                  # LeRobot ↔ ROS 2 协议转换枢纽 (张量与消息转换)
│   ├── ibrobot_msgs/               # 系统统一接口定义 (Message/Action)
│   ├── dataset_tools/              # 数据集采集与转换工具 (Episode Recorder)
│   ├── robot_teleop/               # 遥操作控制实现 (Leader-Follower/控制器)
│   ├── robot_description/          # 统一机器人 URDF/SRDF 模型描述
│   ├── robot_moveit/               # MoveIt 2 运动规划集成
│   ├── inference_service/          # 多模型推理与部署服务
│   ├── so101_hardware/             # SO-101 电机驱动接口
│   └── workflows/                  # CI/CD 配置
│
├── docs/                           # 深度架构文档与开发指南
├── scripts/                        # 环境配置与验证工具脚本
└── build/                          # 编译输出 (自动创建)
```

---

## 环境初始化 (First-time Setup)

**重要：本步骤仅需在初次克隆项目后运行一次。**

### 0. 系统要求
- **操作系统**: openEuler Embedded 24.03
- **ROS 版本**: ROS 2 Humble
- **Python**: 系统原生 Python 3.11。**严禁在 Conda 激活的环境中执行，否则会导致动态库冲突。**

### 1. 执行一键初始化
运行 `./scripts/setup.sh`。该脚本会自动完成以下重型操作：

1.  **子模块同步**: 执行 `git submodule update --init --recursive`，下载核心源码。
2.  **系统依赖安装**: 通过系统包管理器安装 C++ 编译工具、`nlohmann-json` 等硬件驱动依赖。
3.  **虚拟环境 (venv) 构建**: 在根目录创建 `venv` 文件夹。这能确保 ML 相关依赖（如 PyTorch）与系统 ROS 2 环境隔离，防止破坏系统工具。
4.  **ML 栈安装**: 自动在 `venv` 中安装 `torch`、`lerobot` 以及适配 ROS 2 Humble 的特定版本 `numpy (< 2.0)`。
5.  **环境脚本注入**: 自动生成或更新 `.shrc_local`，用于一键加载开发环境。

### 2. 开发者 Fork 设置 (可选)
脚本会询问是否设置个人 Fork 仓库。如果你是核心开发者，输入你的 GitCode 用户名，脚本会自动建立 `origin` (你的仓库) 和 `upstream` (主仓库) 的关联。

---

## 开发工作流

### 1. 加载环境
每次开启新终端，必须先加载项目环境变量。这会激活 `venv` 并注入必要的 `PYTHONPATH`。
```bash
source .shrc_local
```

### 2. 分配 Domain ID
为了避免与局域网内其他 ROS 2 用户冲突，建议设置唯一的 Domain ID：
```bash
export ROS_DOMAIN_ID=<0-232之间的唯一数字>
```

### 3. 编译项目
代码修改后，运行统一构建脚本：
```bash
./scripts/build.sh
```
*注：该脚本会自动处理 `lerobot` 的可编辑安装，并清理潜在的构建污染。*

---

## 运行指南

所有操作均通过 `robot_config` 包的统一入口 `robot.launch.py` 触发。

### 基础仿真（自动启动模型推理控制）
```bash
ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm use_sim:=true
```

### 基础仿真（无模型推理，仅控制器）
```bash
ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm use_sim:=true with_inference:=false
```

### MoveIt 规划模式（自动检测，带 RViz）
```bash
ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm control_mode:=moveit_planning use_sim:=true
```

### MoveIt 无 RViz（headless）
```bash
ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm control_mode:=moveit_planning use_sim:=true moveit_display:=false
```

### 真实硬件
```bash
ros2 launch robot_config robot.launch.py robot_config:=so101_single_arm use_sim:=false
```

### 手动覆盖自动检测
```bash
ros2 launch robot_config robot.launch.py control_mode:=model_inference with_inference:=true use_sim:=true
```

---

## 参数说明

| 参数名 | 说明 | 默认值 |
| :--- | :--- | :--- |
| `robot_config` | 机器人配置名称 (对应 `config/robots/` 下的 YAML) | `so101_single_arm` |
| `config_path` | 配置文件绝对路径 (可选，覆盖 `robot_config`) | 空 |
| `use_sim` | 是否使用 Gazebo 仿真模式 | `false` |
| `control_mode` | 覆盖默认控制模式 (`model_inference` / `moveit_planning` / `teleop`) | 从 YAML 读取 |
| `with_inference` | 强制启用/禁用推理服务 (空则自动检测) | 空 |
| `with_moveit` | 强制启用/禁用 MoveIt 核心 (空则自动检测) | 空 |
| `moveit_display` | 是否启动 MoveIt RViz 可视化界面 | `true` |
| `auto_start_controllers` | 是否在启动后自动激活控制器 | `true` |

---

## 故障排除

### 1. 控制器残留/清理
如果遇到控制器无法启动或端口占用的问题，请运行清理脚本重置 ROS 2 后台进程：
```bash
./scripts/cleanup_ros.sh
```

### 2. 共享内存 (SHM) 报错
若出现 `RTPS_TRANSPORT_SHM Error`，请尝试清理缓存：
```bash
sudo rm -rf /dev/shm/fastrtps_*
export ROS_LOCALHOST_ONLY=1
```

---

**维护者**: IB-Robot Team  
**项目地址**: https://gitcode.com/BreezeWu/IB_Robot  
**反馈**: https://gitcode.com/BreezeWu/IB_Robot/issues
