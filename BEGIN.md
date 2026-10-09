# Unitree G1 仿真：编译、启动与操作

适用于本项目的 **G1 23DoF + MuJoCo + `g1_ctrl` + GUI** 控制链路。
以下命令以 `/home/hanlichen/projects/unitree_g1` 为项目根目录。

## 1. 编译

### G1 控制器

修改 `unitree_g1_ws2/deploy` 中的 C++ 源码、FSM 注册或 CMake 配置后，
从项目根目录重新配置并编译：

```bash
cmake -S unitree_g1_ws2/deploy/robots/g1_23dof \
  -B unitree_g1_ws2/deploy/robots/g1_23dof/build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build unitree_g1_ws2/deploy/robots/g1_23dof/build --parallel 2
```

控制器通过 CMake `GLOB_RECURSE` 收集 `src/*.cpp`；新增源码文件后必须先运行
`cmake -S/-B`。仅修改部署 YAML、ONNX 或预计算 z 文件时，重启控制器即可。

### GUI

修改 `g1_gui` 的 Python 源码或 ROS 2 包后运行：

```bash
source /opt/ros/jazzy/setup.bash
source unitree_ros2/cyclonedds_ws/install/setup.bash
cd unitree_g1_ws
colcon build --symlink-install --packages-select g1_gui
cd ..
```

### MuJoCo

首次安装或修改 MuJoCo 源码时构建。UFO 部署本身不修改仿真器：

```bash
cmake -S unitree_mujoco/simulate \
  -B unitree_mujoco/simulate/build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build unitree_mujoco/simulate/build --parallel 2
```

## 2. 启动

当前 G1 控制器配置为 DDS Domain `0`；项目 `setup.sh` 将 CycloneDDS
绑定到 WSL `eth0`。仿真器、GUI 和控制器应使用相同环境。
同一时间只运行一个 `g1_ctrl`。

终端 A：启动 MuJoCo。

```bash
cd /home/hanlichen/projects/unitree_g1
source ./setup.sh
cd unitree_mujoco/simulate/build
./unitree_mujoco -i 0 -r g1 -s scene_23dof.xml -t 1
```

终端 B：启动 GUI。

```bash
cd /home/hanlichen/projects/unitree_g1
./start_gui.sh
```

GUI 不会启动 MuJoCo。打开后点击“G1 控制器 (g1_ctrl)”卡片中的“启动”，
等待日志出现 `Connected to robot.` 和 `FSM: Start Passive`。
GUI 通过共享内存 JSON 发送操作，`g1_ctrl` 直接通过 DDS 发送 lowcmd；
当前链路不使用 `g1_bridge/g1_merger`。

需要在终端直接运行控制器时：

```bash
cd /home/hanlichen/projects/unitree_g1
source ./setup.sh
cd unitree_g1_ws2/deploy/robots/g1_23dof/build
./g1_ctrl --log
```

直接运行时不要再从 GUI 启动第二个控制器。
GUI 的进程卡片只管理它自己启动的实例。

## 3. 建议的仿真操作顺序

1. 启动 MuJoCo，保持虚拟吊带开启。
2. 启动 `g1_ctrl`，确认进入 Passive。
3. 确认 GUI 的速度摇杆回中，进入 FixStand，等待至少 3 秒。
4. 在 MuJoCo 窗口用 `8` 或 `↓` 逐次放低机器人，让双脚接地。
5. 进入 Velocity，先保持零速度观察，再测试小速度。
6. 稳定后在 MuJoCo 窗口按 `9` 关闭吊带，再测试 WBC、Dance 或 UFO。

通常从 Velocity 用 `LT+Up` 返回 FixStand，再用 `LT+B`
返回 Passive；UFO 也可直接用 `LT+B` 返回 Passive。
停止控制器后再关闭 MuJoCo。机器人摔倒时先停控制器，
按 `Backspace` 重置 MuJoCo，再按上述顺序启动。

## 4. FSM 与 UFO 任务

实际转换以
`unitree_g1_ws2/deploy/robots/g1_23dof/config/config.yaml` 为准：

```text
Passive  ─ LT+Up ─→ FixStand
         └ LT+Y  ─→ UFO

FixStand ─ LT+B  ─→ Passive
         ├ RB+X  ─→ Velocity
         └ LT+Y  ─→ UFO

Velocity ─ LT+Up   ─→ FixStand
         ├ LT+Down ─→ Mimic_Dance_102
         ├ RB+Up   ─→ WBC
         └ LT+Y    ─→ UFO

Mimic_Dance_102 / WBC ─ RB+X 或动作结束 ─→ Velocity
UFO ─ LT+B  ─→ Passive
    ├ LT+Up ─→ FixStand
    └ RB+X  ─→ Velocity
```

| 模式 | 作用 |
|---|---|
| Passive | 控制器初始状态 |
| FixStand | 过渡到固定站立姿态 |
| Velocity | 摇杆速度跟踪 |
| Mimic_Dance_102 | 跟踪舞蹈参考片段 |
| WBC | 播放 WBC 全身参考轨迹 |
| UFO | 一个 UFO-FB policy，通过预计算 z 选择任务 |

UFO 可从 Passive、FixStand、Velocity 进入。进入时先使用站立目标 z；
进入 UFO 后，GUI 右侧显示 Tracking、Goal reaching、Reward inference
三组任务按钮。点击按钮只切换 z，不切换 FSM 状态。
Tracking 逐帧播放完后保持站立目标，不自动返回 Velocity；
Goal reaching 和 Reward inference 使用固定 z，直到切换任务。
当前任务清单为 5 条 Tracking 序列、15 个 Goal reaching 固定 z、
9 个 Reward inference 固定 z。详见
`unitree_g1_ws2/deploy/robots/g1_23dof/config/policy/ufo/README.md`。

## 5. 可选 UDP 触发

GUI 运行时监听 `0.0.0.0:9876`。可从另一终端使用：

```bash
cd /home/hanlichen/projects/unitree_g1/unitree_g1_ws2/deploy
python3 udp_send.py <运行GUI的电脑IP> 9876
```

`1–6` 为 Velocity 的两秒移动指令，`7=WBC`、`8=Dance`、
`9=UFO`。触发仍受当前 FSM 可达关系限制；UDP 编号不直接选择 UFO z。

## 6. 启动参数与排查

MuJoCo 命令中的 `-i 0` 指定 Domain 0，`-r g1` 指定机器人，
`-s scene_23dof.xml` 指定场景，`-t 1` 指定 `unitree_hg` IDL。
上游 `unitree_mujoco/simulate/config.yaml` 默认是 Go2、Domain 1；
本项目启动时通过参数覆盖。WSL `eth0` 已是该配置中的网络接口。

`g1_ctrl` 支持 `--help`、`--version`、`--log` 和
`--network <接口>`；网络接口默认 `eth0`。

- 一直等待 `rt/lowstate`：确认 MuJoCo 已启动，Domain 为 `0`，接口为 `eth0`。
- GUI 模式按钮不可用：先看“当前状态”；只有该状态可达的按钮会启用。
- UFO 任务按钮不可见：先进入 UFO；任务列表来自 `latent_catalog.json`。
- 控制命令重复：检查是否同时运行了两个 `g1_ctrl`。
- 出现 `llvmpipe`：当前是软件渲染，可在启动 MuJoCo 前重新加载项目 `setup.sh` 并检查图形环境。

首次实机运行前，仍需在 MuJoCo 中验证状态切换、动作收敛和异常保护。
