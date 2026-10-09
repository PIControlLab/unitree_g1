# G1 Controller Workspace

Unitree G1 人形机器人 ROS 2 (Jazzy) 控制工作空间，包含双足行走、手臂遥操作与仿真集成。

## 目录结构

```
unitree_g1_ws/
├── build/                  # colcon 构建产物
├── install/                # colcon 安装产物
├── log/                    # 构建日志
├── src/
│   ├── g1_controller/      # C++ 运动控制包 (CPG 行走 + 键盘遥操 + 指令合并)
│   ├── g1_arm_controller/  # Python 右臂 5 关节位置阻抗控制器
│   └── g1_gui/             # PyQt5 右臂摇杆遥操作界面
├── setup.sh                # 仿真环境配置脚本
├── setup_real.sh           # 真机环境配置脚本
├── start_sim.sh            # 一键启动仿真
└── .gitignore
```

## 启动前
```bash
cd unitree_g1_ws
./compile.sh
source setup.sh
```

## 包说明

### g1_controller (C++)

| 节点 | 功能 |
|------|------|
| `g1_walk` | CPG 步态生成 + IMU 踝关节平衡，接收 `/cmd_vel`，发布 `/lowcmd_leg` |
| `g1_teleop` | 键盘遥操节点，读取键盘输入并发布 `/cmd_vel` (Twist) |
| `g1_arm_dev` | 右臂腕关节 (motor 26) 速度 PD → 力矩控制，接收 `/arm_vel` |
| `g1_merger` | 每关节独立锁止/透传合并器，合并 `/lowcmd_leg` + `/lowcmd_arm` → `/lowcmd` |

- **步态**: 正弦 CPG 模型，支撑/摆动相切换，前进速度自适应步频 (1.2–2.7 Hz)
- **平衡**: IMU 前馈踝关节+髋关节策略，PD 抗倾倒
- **起立**: 3 秒平滑斜坡插值，从当前姿态过渡到站立姿态
- **控制频率**: 500 Hz (2ms 周期)

### g1_arm_controller (Python)

| 节点 | 功能 |
|------|------|
| `arm_ctrl` | 右臂 5 关节 (motor 22-26) 位置阻抗控制器，50 Hz |

- **控制方法**: 位置阻抗插值 — 摇杆速度 → 限幅积分 → q_target，PD + 积分重力补偿
- **输入**: `/arm_vel` (geometry_msgs/Twist，来自 GUI 摇杆)，`cmd_vel_22..26` (独立关节速度)
- **输出**: `/lowcmd_arm` (LowCmd，发往 g1_merger 合并)
- **死区**: 0.005 m/s，超时 0.2s 自动锁止
- **调参**: kp=60, kd=1.5, ki=1~3，力矩限幅 25~100 Nm
- **正逆运动学**: 5 关节 FK + damped pseudo-inverse Jacobian (3×5 → 5 DOF)

### g1_gui (Python)

| 节点 | 功能 |
|------|------|
| `arm_teleop_node` | PyQt5 双摇杆界面，发布 `/arm_vel` |

- 左摇杆: v_x (水平) + v_y (垂直)
- 右滑块: v_z (前后)
- 量程: ±0.3 m/s，50 Hz 发布
- 松手自动回零

## 环境依赖

- **OS**: Ubuntu 24.04
- **ROS 2**: Jazzy
- **DDS**: CycloneDDS (`rmw_cyclonedds_cpp`)
- **Unitree SDK**: `unitree_hg`, `unitree_api` (ROS 2 msg 包)
- **仿真**: [unitree_mujoco](../unitree_mujoco/) (MuJoCo 物理引擎)
- **GUI**: PyQt5 (仅 g1_gui)
- **Python**: NumPy (仅 g1_arm_controller)

## 快速开始

### 构建

```bash
# 1. 先编译 Unitree ROS 消息包（仅首次需要）
cd ../unitree_ros2/cyclonedds_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# 2. 编译 G1 控制工作空间
cd ../../unitree_g1_ws
source /opt/ros/jazzy/setup.bash
source ../unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build --symlink-install
```

### 仿真运行

```bash
# 一键启动: 站立控制器 + 仿真器
bash start_sim.sh

# 新终端: 键盘遥操
source setup.sh
ros2 run g1_controller g1_teleop
```

按键映射:

| 按键 | 动作 |
|------|------|
| `W`/`S` | 前进/后退 |
| `A`/`D` | 左移/右移 |
| `Q`/`E` | 左转/右转 |
| `R`/`F` | 加速/减速 |
| `Space` | 停止 |
| `Esc` | 退出 |

### 手臂遥操作 (仿真)

```bash
# 终端 1: 手臂控制器
source setup.sh
ros2 run g1_arm_controller arm_ctrl

# 终端 2: GUI 摇杆
source setup.sh
ros2 run g1_gui arm_teleop
```

### 真机运行

```bash
# 根据实际网卡名替换 enp3s0
source setup_real.sh enp3s0
ros2 run g1_controller g1_walk
```

> **注意**: 真机与仿真使用不同的 ROS_DOMAIN_ID (真机=0, 仿真=1)，CycloneDDS 网络接口配置也不同。

## Topic 架构

```
键盘 → /cmd_vel (Twist) → g1_walk → /lowcmd_leg (LowCmd) ─┐
                                                           ├→ g1_merger → /lowcmd (LowCmd) → 机器人
摇杆 → /arm_vel (Twist) → arm_ctrl → /lowcmd_arm (LowCmd) ─┘
                              ↑
                 cmd_vel_22..26 (Float64, 独立关节)
```

## License

MIT
