# unitree_g1

Unitree G1 23DoF 的 MuJoCo 仿真与策略部署工作区。当前控制链路是
`g1_gui` 图形界面操控 `g1_ctrl`，后者通过 Unitree DDS 直接与仿真器或机器人通信。

## 目录

| 路径 | 用途 |
|---|---|
| `unitree_g1_ws2/deploy/` | G1 23DoF FSM、Velocity、Mimic、WBC、UFO 策略与模型 |
| `unitree_g1_ws/src/g1_gui/` | PyQt5 控制台，管理控制器进程和操作指令 |
| `unitree_g1_ws/src/g1_bridge/` | 保留的独立 ROS 2 合并器包；当前 GUI 控制链路不使用 |
| `unitree_mujoco/` | MuJoCo 仿真器，上游仓库 |
| `unitree_sdk2/`、`unitree_ros2/` | SDK 和 ROS 2 依赖，上游仓库 |
| `setup.sh`、`start_gui.sh` | 环境配置和 GUI 启动脚本 |

上游仓库目录由根目录 `.gitignore` 排除，新的检出环境需自行准备依赖。
构建与完整操作顺序见 [BEGIN.md](BEGIN.md)；GUI 架构见
[unitree_g1_ws/README.md](unitree_g1_ws/README.md)。

## 快速启动

从项目根目录分别打开两个终端。先启动 MuJoCo：

```bash
source ./setup.sh
cd unitree_mujoco/simulate/build
./unitree_mujoco -i 0 -r g1 -s scene_23dof.xml -t 1
```

再启动 GUI：

```bash
cd /home/hanlichen/projects/unitree_g1
./start_gui.sh
```

GUI 打开后点击“G1 控制器 (g1_ctrl)”卡片中的“启动”。
`start_gui.sh` 不会自动启动仿真器或控制器。当前配置使用 DDS Domain `0`
和 WSL `eth0`；控制器只应运行一个实例。

控制器启用 `Passive`、`FixStand`、`Velocity`、
`Mimic_Dance_102`、`WBC`、`UFO` 六个状态。
UFO 可从 Passive、FixStand 或 Velocity 进入；Tracking、Goal reaching
和 Reward inference 共用同一策略，仅切换预计算 z。
Tracking 结束后保持站立目标，不自动退出 UFO。
模型、任务与 z 文件见
[UFO 部署说明](unitree_g1_ws2/deploy/robots/g1_23dof/config/policy/ufo/README.md)。

先在 MuJoCo 中验证状态切换、动作和异常路径，再考虑实机运行。
