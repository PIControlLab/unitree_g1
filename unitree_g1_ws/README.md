# G1 GUI 工作空间

`unitree_g1_ws` 是 ROS 2 Jazzy 工作空间。当前 GUI 入口是 `g1_gui` 包中的
`launcher`，用于启动 G1 23DoF 控制器、切换 FSM 模式和发送操作指令。
控制策略和 UFO 的预计算 z 位于同一项目的 `unitree_g1_ws2/deploy`。

## 当前目录

```text
unitree_g1_ws/
├── src/
│   ├── g1_gui/       # 当前使用的 PyQt5 控制台
│   └── g1_bridge/    # 保留的独立 ROS 2 合并器包；当前 GUI 控制链路不启动它
├── compile.sh        # colcon 构建脚本
├── setup.sh          # 仿真环境配置
└── setup_real.sh     # 真机网络环境配置
```

`build/`、`install/` 和 `log/` 是构建产物目录。旧文档中的
`g1_controller`、`g1_arm_controller`、`arm_teleop` 和 `start_sim.sh`
在当前源码中不存在。

## GUI 与控制器的关系

```text
g1_gui/launcher
  ├─ 启动/停止 → unitree_g1_ws2/.../build/g1_ctrl
  ├─ 手柄与 FSM 指令 → /dev/shm/unitree_joystick_bridge.json → g1_ctrl
  ├─ UFO 任务选择 → /dev/shm/unitree_ufo_selection.json → g1_ctrl
  └─ 读取当前 FSM ← /dev/shm/unitree_fsm_state.json ← g1_ctrl

MuJoCo 或实机 ← DDS lowcmd ← g1_ctrl ← DDS lowstate ← MuJoCo 或实机
```

GUI 是进程管理器和操作界面，不是发布 `/arm_vel` 的手臂遥操作节点。
`g1_ctrl` 直接通过 Unitree DDS 发送 lowcmd；当前路径不经过
`g1_bridge/g1_merger`。GUI 的进程卡片只管理由该 GUI 启动的控制器，
因此启动前应确认没有另一个 `g1_ctrl` 正在运行。

GUI 启动后监听 UDP `0.0.0.0:9876`。外部发送数字 `1–6` 触发
Velocity 的两秒移动指令，`7`、`8`、`9` 分别请求进入 WBC、Dance、UFO；
状态不允许时请求不会生效。UDP 数字不直接选择 UFO z。

## 构建与启动

从项目根目录运行：

```bash
# 修改 GUI Python 包后重新构建
source /opt/ros/jazzy/setup.bash
source unitree_ros2/cyclonedds_ws/install/setup.bash
cd unitree_g1_ws
colcon build --symlink-install --packages-select g1_gui
cd ..

# 在单独终端启动 MuJoCo
source ./setup.sh
cd unitree_mujoco/simulate/build
./unitree_mujoco -i 0 -r g1 -s scene_23dof.xml -t 1
```

另开终端，从项目根目录启动 GUI：

```bash
./start_gui.sh
```

`start_gui.sh` 加载项目环境并执行 `ros2 run g1_gui launcher`。
它不会自动启动 MuJoCo；打开 GUI 后，还需点击“G1 控制器 (g1_ctrl)”
卡片中的“启动”。若只想直接启动 GUI，也可以先 `source ./setup.sh`，
再执行 `ros2 run g1_gui launcher`。项目和工作空间的 `setup.sh`
都会清除 `/dev/shm/unitree_*.json` 中的旧控制文件。

修改 `g1_ctrl` 的 C++ 源码时，应按项目根目录的 `BEGIN.md`
重新配置并编译控制器；只修改 GUI 的 Python 代码时，使用上面的
`g1_gui` 构建命令。

## 界面操作

左侧是 `g1_ctrl` 进程卡片、FSM 状态和模式按钮。当前状态由控制器写入
`/dev/shm/unitree_fsm_state.json`，GUI 根据实际状态启用可到达的按钮。
控制器初始状态为 Passive；通常先进入 FixStand，再进入 Velocity。

| 当前模式 | GUI 可请求进入 |
|---|---|
| Passive | FixStand、UFO |
| FixStand | Passive、Velocity、UFO |
| Velocity | FixStand、WBC、Dance、UFO |
| WBC、Dance | Velocity |
| UFO | Passive、FixStand、Velocity |

Velocity 模式显示平移摇杆和旋转滑块；松手归零。右侧还有前进、
后退、左右移和转弯按钮，每次发送约两秒的速度指令。
其他模式不会显示 Velocity 的移动按钮。

UFO 只有一个 FSM 状态。进入时先使用站立目标 z；右侧随后按
Tracking、Goal reaching、Reward inference 三组显示任务按钮。
这些按钮来自
`unitree_g1_ws2/deploy/robots/g1_23dof/config/policy/ufo/latent_catalog.json`，
点击后只切换预计算 z。Tracking 播放结束后回到站立目标；
Goal reaching 和 Reward inference 使用固定 z，直到选中别的任务。
运行时不推理 z。具体文件和模型契约见 UFO 目录中的 `README.md`。

## 环境与注意事项

项目默认使用 Ubuntu 24.04、ROS 2 Jazzy、CycloneDDS 和 PyQt5。
`./start_gui.sh` 所加载的项目 `setup.sh` 将 CycloneDDS 接口设为
`eth0`；控制器配置的 DDS Domain 为 `0`。GUI 依赖已构建的
`g1_gui` ROS 2 包和 `unitree_g1_ws2/deploy/robots/g1_23dof/build/g1_ctrl`。

先在 MuJoCo 中保持吊带、逐项验证状态切换和动作，再考虑实机。
关闭 GUI 时，它会停止自己启动的控制器进程。
