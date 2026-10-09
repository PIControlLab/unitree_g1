# unitree_g1

宇树 G1 机器人的开发工作区，包含仿真、SDK、ROS2 通讯及强化学习控制等模块。

## 目录结构

```
unitree_g1/
├── unitree_mujoco/    # MuJoCo 物理引擎仿真（来自官网）
├── unitree_sdk2/      # 宇树 SDK2（来自官网）
├── unitree_ros2/      # ROS2 及相关中间件（来自官网）
├── unitree_g1_ws/     # 可选的 PyQt5 图形控制界面
├── unitree_g1_ws2/    # 强化学习策略部署
├── setup.sh           # 环境初始化脚本
└── start_gui.sh       # 图形化控制界面启动脚本
```

## 官网 workspace

### MuJoCo 仿真

- **仓库**：[unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)
- **目标**：启动物理引擎仿真，通过 DDS 或 ROS2 进行通讯控制

关键配置：

```bash
robot: "g1"                        # 机器人名称
robot_scene: "scene_23dof.xml"     # 场景文件
domain_id: 0                       # Domain ID
interface: "lo"                    # 网络接口
use_joystick: 0                    # 是否使用手柄模拟遥控器（0=关,1=开）
joystick_type: "xbox"              # 手柄类型：支持 "xbox" 和 "switch"
joystick_device: "/dev/input/js0"  # 手柄设备路径
joystick_bits: 16                  # 手柄精度（部分手柄仅支持 8 位）
print_scene_information: 1         # 打印机器人连杆、关节及传感器信息
enable_elastic_band: 1             # 虚拟弹性带，用于抬起 H1
```

### unitree_sdk2

- **仓库**：[unitreerobotics/unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)
- **目标**：运行个别例程，基于 SDK 进行二次开发

### unitree_ros2

- **仓库**：[unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)
- **目标**：保持中间件版本一致性，即 CycloneDDS 版本为 **0.9.2**

## 项目 workspace

### unitree_g1_ws

可选的 `g1_gui` ROS 2 Python 包，用于启动 `g1_ctrl`、操作 FSM
并通过共享内存 JSON 提供虚拟手柄输入。核心 Sim-to-Sim 链路不依赖该工作区。

### unitree_g1_ws2

存放训练好的强化学习策略及部署代码。

## 运行前准备

### 初始化环境

```bash
source ~/unitree_g1/setup.bash
```

## 运行

### 启动 MuJoCo 仿真

```bash
cd ~/unitree_g1/unitree_mujoco/simulate/build
./unitree_mujoco
```

### 方式一：命令行控制

**启动强化学习控制器：**

```bash
cd ~/unitree_g1/unitree_g1_ws2/deploy/robots/g1_23dof/build
./g1_ctrl
```

**施加控制指令：**

```bash
cd ~/unitree_g1/unitree_g1_ws2/deploy/robots/g1_23dof/build
python3 joystick_emu.py
```

### 方式二：图形化界面

```bash
ros2 run g1_gui launcher
```

或使用脚本：

```bash
cd ~/unitree_g1
./start_gui.sh
```
