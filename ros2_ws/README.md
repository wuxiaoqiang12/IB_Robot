# rosetta — ROS 2 ⇄ LeRobot bridge

rosetta 是一个旨在将 ros2 与 lerobot 进行桥接的开源项目. 本文档为 rosetta 项目的快速上手指南, 详细信息请移步:
- [rosetta 官宣](https://discourse.openrobotics.org/t/announcing-rosetta-a-ros-2-lerobot-bridge/50657)
- [rosetta 官方仓库](https://github.com/iblnkn/rosetta)

> 注意: 环境配置的部分请参考本指南

# 1. Prerequisites
> 注意: 因为 ros 的限制, 在运行本项目的时候不能在 conda 环境中. 如果 lerobot 和依赖已经安装在 conda 里了, 参考后文进行适配
- 已经安装好 lerobot 和其依赖. 如果是安装在了 conda 环境中, 请记录下 lerobot 路径和该环境 python site-packages 路径到两个变量中, 后续需要用于环境配置, 如:
   
   `$ export LEROBOT_SRC_PATH=/home/username/lerobot_ros2/src`

   `$ export LEROBOT_DEPENDENCIES_PATH=/home/username/miniconda3/envs/lerobot_env/lib/python3.10/site-packages`

- 准备好 lerobot 训练好的模型
- 安装好 ROS2 humble (参考下文) 
- 系统为 openEuler embedded 24.03 或 ubuntu 22.04

# 2. Setup
> 注意: 在 oee 和 ubuntu 中的系统配置步骤可能有区别, 请注意分别

> 注意: 环境配置中遇到报错可先移步末尾的 Troubleshooting 部分查看解决方法

## 2.1. ROS2
### 2.1.1. 在 openEuler embedded 24.03 上安装 ros
> 注意: 严禁执行 `dnf update` 指令, 此操作可能导致嵌入式系统核心组件损坏或无法启动。

#### 2.1.1.1. 写入 ROS humble 源配置:
```shell
sudo bash -c 'cat << EOF > /etc/yum.repos.d/ROS.repo
[openEulerROS-humble]
name=openEulerROS-humble
baseurl=https://eulermaker.compass-ci.openeuler.openatom.cn/api/ems1/repositories/ROS-SIG-Multi-Version_ros-humble_openEuler-24.03-LTS-TEST4/openEuler%3A24.03-LTS/aarch64/
enabled=1
gpgcheck=0
EOF'
```

#### 2.1.1.2. 安装 ROS 核心组件
```shell
sudo dnf clean all && sudo dnf makecache
sudo dnf install -y ros-humble-ros-base \
ros-humble-controller-manager \
ros-humble-ros2-control \
ros-humble-ros2-controllers \
```

#### 2.1.1.3. 安装和配置 ros2 的编译和依赖工具:
```bash
pip install colcon-common-extensions
pip install rosdep
export ROS_OS_OVERRIDE=rhel:8
```

#### 2.1.1.4. 系统环境与依赖修复 (关键步骤)

嵌入式系统存在部分库文件缺失或软链接不完整的问题，必须执行以下修复才能成功编译。

修复库权限与开发包缺失, 强制重装开发包以修复头文件丢失（"幽灵包" 问题）：

```bash
# 修复库目录权限，防止 CMake 找不到依赖
sudo chmod -R 755 /usr/local/lib64

# 强制重装关键开发库
sudo dnf reinstall -y tinyxml2-devel yaml-cpp-devel boost-devel
```

修复 Boost 库软链接, 解决编译器找不到 `-lboost_filesystem` 等问题（以 Boost 1.83.0 为例）：

```bash
cd /usr/lib64
sudo ln -sf libboost_filesystem.so.1.83.0 libboost_filesystem.so
sudo ln -sf libboost_system.so.1.83.0 libboost_system.so
sudo ln -sf libboost_thread.so.1.83.0 libboost_thread.so
sudo ln -sf libboost_program_options.so.1.83.0 libboost_program_options.so
sudo ln -sf libboost_regex.so.1.83.0 libboost_regex.so
sudo ln -sf libboost_chrono.so.1.83.0 libboost_chrono.so
sudo ln -sf libboost_date_time.so.1.83.0 libboost_date_time.so
sudo ln -sf libboost_atomic.so.1.83.0 libboost_atomic.so
```

### 2.1.2. 在 ubuntu 22.04 上安装 ros
参照 https://docs.ros.org/en/humble/Installation.html 安装，推荐 deb packages 安装。


### 2.1.3. 安装验证
安装完成后可尝试官方示例测试， 可以看到 listener 在打印 talker 传输的数据：

```shell
# In one terminal, source the setup file and then run a C++ talker:
$ source /opt/ros/humble/setup.bash
$ ros2 run demo_nodes_cpp talker

# In another terminal source the setup file and then run a Python listener:
$ source /opt/ros/humble/setup.bash
$ ros2 run demo_nodes_py listener
```

> NOTE: ROS 的相关指令或者可执行文件都需要通过其内置脚本 source 到当前终端
> `source /opt/ros/humble/setup.bash`
> 所以每一个运行跟 ROS 相关指令的终端都要在启动时先这样 source 下，那么为了防止忘记，建议将该步骤放到终端默认启动项 ~/.bashrc 中， 参考 https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#add-sourcing-to-your-shell-startup-script


## 2.2. ROS2_WS
本目录(i.e. ros2_ws)是一个 “ROS2 workspace”. 其中包含了数个 ros2 packages, 如负责获取硬件数据的 `robot_interface`, 桥接模块 `rosetta` 等. 构建本项目时请在本目录进行: 

```shell
$ pwd
> /path/to/ledog_ros2/ros2_ws

$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build --symlink-install
$ source install/setup.bash

# 执行后, 可运行类似以下命令检测是否正常:
$ ros2 pkg executables robot_interface # 列出 robot_interface 这个 package 中所有定义的可执行文件
> robot_interface robot_interface # 表示 robot_interface 这个 package 中定义了一个 robot_interface 可执行文件
```
> NOTE: colcon build 的 --symlink-install 与 pip install 的 -e 类似，如果是 python 脚本，那么做了修改后没有必要再次编译了，直接重新运行就能使修改生效。


## 2.3. 配置 lerobot 路径 (按需)
> 注意: 如果 lerobot 跟 ros 一样安装在了系统环境而不是 conda 环境中, 可以跳过这一步

执行以下命令配置 lerobot 安装的 conda 路径到 python path 中, 以便 ros2_ws 中的包能找到 lerobot 相关模块:

```shell
$ export PYTHONPATH=$LEROBOT_SRC_PATH:$LEROBOT_DEPENDENCIES_PATH:$PYTHONPATH
```

## 2.4. startup 脚本
因为上述的 ros2 环境配置和 lerobot 路径配置经常需要在新终端中执行, 可以创建一个 startup.bash 脚本来简化这个过程, 并注意在 `ros2_ws` 目录下执行:

```shell
# startup.bash

source install/setup.bash

# lerobot conda env path
export LEROBOT_SRC_PATH=/home/username/lerobot_ros2/src
export LEROBOT_DEPENDENCIES_PATH=/home/username/miniconda3/envs/lerobot_env/lib/python3.10/site-packages
export PYTHONPATH=$LEROBOT_SRC_PATH:$LEROBOT_DEPENDENCIES_PATH:$PYTHONPATH
```

## 2.5. 运行前配置
环境配置完成后, 则需要进行运行前的配置, 主要目的包括:
1. 硬件配置, 配置机械臂和摄像头(如种类, 端口号等, 与 lerobot 的方法相似)
2. 模型配置, 主要配置模型路径和 device

### 2.5.1. 快速配置
快速运行 ACT 场景可简单地进行如下配置:
参考 [single_arm_banana.yaml](./src/robot_interface/config/scenes/single_arm_banana.yaml) 进行配置机器人参数, 如端口, 相机等. 如下举例:
```yaml
# single_arm_banana.yaml
robot_interface:
  ros__parameters:
    is_mock: false
    robot_ids: [follower_arm] # 机械臂 ID 列表, 同 lerobot 的 robot.id
    
    follower_arm: # 机械臂 id 为 "follower_arm" 的机械臂配置, 注意需要与上面的 robot_ids 中的 id 一致
      type: so101_follower # 机械臂类型, 同 lerobot 的 robot.type
      port: /dev/ttyACM0 # 机械臂端口号
      camera_names: [top, wrist] # 相机名称
      cameras: # 相机配置列表, 注意下面细分的每个相机名称也要与 camera_names 中的一致
        top:
           # 单个相机配置, 与 lerobot 的相机配置相同
           index: 0
           width: 640
           height: 480
           fps: 30
        wrist:
           index: 2
           width: 640
           height: 480
           fps: 60
```



参考 [turtlebot_policy_bridge.launch.py](./src/rosetta/launch/turtlebot_policy_bridge.launch.py) 进行快速配置, 如下举例:
```python
# turtlebot_policy_bridge.launch.py

return LaunchDescription([
   log_level_arg,
   Node(
      package='rosetta',
      executable='policy_bridge_node',
      name='policy_bridge',
      output='screen',
      emulate_tty=True,
      parameters=[
            ...
            # 配置模型推理设备, 在 ubuntu 上一般是 cuda, 在 oee 设备上一般是 npu
            {'policy_device': 'npu'},
            # 配置模型路径, 与 lerobot 训练出来的模型路径结构一致
            {'policy_path': '/path/to/pretrained_model'},
            ...
      ],
      arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
   ),
])

```

### 2.5.2. 详细配置
如需要配置更多参数, 详细配置方法可参考:
1. 硬件配置: 参考 [robot 启动参数配置](./src/robot_interface/ROS2_param.md)
2. 模型配置: 参考 [rosetta 官方文档](./src/rosetta/ROS2_param.md)

# 3. 运行

前文的配置完成后, 以 ACT 场景为例, 将按照以下命令启动:

> TODO: 后续会根据场景补充一键启动的 launch file 样例

## 3.1. 连接机械臂
新启动一个 terminal, 执行:
```shell
$ source startup.bash
$ ros2 run robot_interface robot_interface --ros-args --params-file install/robot_interface/share/robot_interface/config/single_arm_banana.yaml -p is_mock:=false

# 或可使用 mock 机械臂进行测试
$ source startup.bash
$ ros2 run robot_interface robot_interface --ros-args --params-file install/robot_interface/share/robot_interface/config/single_arm_banana.yaml -p is_mock:=true

# 完成后, 可通过如下命令查看是否能够正常获取到机械臂数据
$ ros2 topic echo /joint_states
```


## 3.2. 启动 rosetta 桥接服务
新启动一个 terminal, 执行:
```shell
$ source startup.bash
$ ros2 launch rosetta turtlebot_policy_bridge.launch.py log_level:=info

# 完成后, 可通过如下命令查看桥接服务是否启动
$ ros2 action list | grep run_policy
> /run_policy
```

## 3.3. 开启推理
新启动一个 terminal, 执行:
```shell
$ source startup.bash
$ ros2 action send_goal /run_policy rosetta_interfaces/action/RunPolicy  "{prompt: 'place the banana in the plate'}"

# 完成后, 能在 rosetta 启动的终端看到推理日志输出
```

## 3.4. 关闭推理
新启动一个 terminal, 执行:
```shell
$ source startup.bash
$ ros2 service call /run_policy/cancel std_srvs/srv/Trigger "{}"
```


# 4. Troubleshooting

## 4.1. 使用 `colcon build` 编译时报错:
```shell
Traceback (most recent call last):

File "/opt/ros/humble/lib/python3.11/site-packages/rosidl_adapter/resource/__init__.py", line 51, in evaluate_template
em.BUFFERED_OPT: True,
^^^^^^^^^^^^^^^

AttributeError: module 'em' has no attribute 'BUFFERED_OPT
module 'em' has no attribute 'BUFFERED_OPT'
```
解决方法: 降级安装安装 empy==3.3.2

## 4.2. 执行 `rosdep install` 相关指令报错
```shell
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
rosetta: Cannot locate rosdep definition for [ament_python, gz_math_vendor]
```
解决方法: 可以无视

## 4.3. 执行 `rosdep` 相关指令报错: OsNotDectected
解决方法: 执行 `export ROS_OS_OVERRIDE=rhel:8`

