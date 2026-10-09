# Unitree G1 仿真：编译、启动与操作

适用于当前仓库的 **G1 23DoF + MuJoCo + `g1_ctrl` + GUI** 仿真链路。默认项目路径：

```text
/home/hanlichen/projects/unitree_g1
```

## 1. 编译

加入或修改 UFO 的 C++ 文件、FSM 注册或控制器 CMake 后必须重新配置并编译。只修改
ONNX、参考动作或 YAML 参数时不需要重编译控制器，重启 `g1_ctrl` 即可。

### 1.1 编译 G1 控制器（加入 UFO 后必须执行）

```bash
cd /home/hanlichen/projects/unitree_g1

cmake \
  -S unitree_g1_ws2/deploy/robots/g1_23dof \
  -B unitree_g1_ws2/deploy/robots/g1_23dof/build \
  -DCMAKE_BUILD_TYPE=Release

cmake --build \
  unitree_g1_ws2/deploy/robots/g1_23dof/build \
  --parallel 2
```

成功时最后应显示：

```text
Built target g1_ctrl
```

必须执行第一条 `cmake -S/-B`：本项目通过 CMake `GLOB` 搜索 `src/*.cpp`，只有重新配置后
新加入的 `State_UFO.cpp` 才会进入构建目标。

### 1.2 编译 GUI

加入或修改 GUI 按钮、ROS 2 Python 包后执行：

```bash
cd /home/hanlichen/projects/unitree_g1
source /opt/ros/jazzy/setup.bash
source unitree_ros2/cyclonedds_ws/install/setup.bash

cd unitree_g1_ws
colcon build --symlink-install --packages-select g1_gui
```

### 1.3 编译 MuJoCo（首次安装或修改 MuJoCo 源码时执行）

UFO 本身没有修改 MuJoCo 源码，已有可执行文件时可跳过本节。

```bash
cd /home/hanlichen/projects/unitree_g1

cmake \
  -S unitree_mujoco/simulate \
  -B unitree_mujoco/simulate/build \
  -DCMAKE_BUILD_TYPE=Release

cmake --build unitree_mujoco/simulate/build --parallel 2
```

## 2. 启动

默认通信配置为 Domain `0`、接口 `eth0`。不要同时启动两个 `g1_ctrl`。

### 2.1 终端 A：启动 MuJoCo

```bash
cd /home/hanlichen/projects/unitree_g1
source ./setup.sh

cd unitree_mujoco/simulate/build
./unitree_mujoco -i 0 -r g1 -s scene_23dof.xml -t 1
```

`eth0` 已是默认接口，不需要再传 `-n eth0`。

可在启动前检查 GPU 渲染：

```bash
glxinfo -B | grep -E 'direct rendering|Device:|Accelerated:|OpenGL renderer'
```

正确结果应包含 `D3D12 (NVIDIA GeForce RTX 3050 Ti Laptop GPU)` 和
`Accelerated: yes`。GPU 只加速窗口渲染，动力学计算仍主要使用 CPU。

### 2.2 终端 B：启动 GUI 和控制器

```bash
cd /home/hanlichen/projects/unitree_g1
./start_gui.sh
```

GUI 打开后，点击左侧 **G1 控制器（g1_ctrl）** 卡片中的“启动”。看到以下日志表示连接成功：

```text
Connected to robot.
FSM: Start Passive
```

如果需要直接观察控制器日志，可不用 GUI 启动控制器，改用：

```bash
cd /home/hanlichen/projects/unitree_g1
source ./setup.sh

cd unitree_g1_ws2/deploy/robots/g1_23dof/build
./g1_ctrl --log
```

手工启动 `g1_ctrl` 后不要再从 GUI 启动第二个控制器。

### 2.3 可选：UDP 触发

GUI 运行时监听 `0.0.0.0:9876`：

```bash
cd /home/hanlichen/projects/unitree_g1/unitree_g1_ws2/deploy
python3 udp_send.py <运行GUI的电脑IP> 9876
```

`1–6` 为速度指令，`7=WBC`、`8=Dance`、`9=UFO`。动作指令只在 `Velocity` 模式生效。

## 3. 正确操作顺序

1. 先启动 MuJoCo，保持虚拟吊带开启。
2. 启动 GUI 和 `g1_ctrl`，等待进入 `Passive`。
3. 确认速度摇杆回中，点击 `FixStand`，等待至少 3 秒。
4. 在 MuJoCo 窗口逐次按 `8` 或 `↓`，缓慢放低机器人至双脚接地。
5. 点击 `Velocity`，保持零速度观察 3–5 秒。
6. 稳定后在 MuJoCo 窗口按 `9` 关闭吊带。
7. 先测试小速度，再从 `Velocity` 进入 WBC、Dance 或 UFO。

```text
MuJoCo（吊带开启）
  -> g1_ctrl（Passive）
  -> FixStand，等待 3 秒
  -> 缓慢放低至脚掌接地
  -> Velocity，零速稳定 3–5 秒
  -> 按 9 关闭吊带
  -> 小速度测试
  -> WBC / Dance / UFO
```

正常停止：动作回到 `Velocity` → 速度归零 → `RB+Down` 回 `FixStand` → `LT+B` 回
`Passive` → 停止控制器 → 关闭 MuJoCo。

机器人摔倒时先停止控制器，再按 `Backspace` 重置 MuJoCo，然后从上述顺序重新开始。

## 4. 模式与跳转

```text
Passive
  └─ LT+Up ──> FixStand

FixStand
  ├─ LT+B ───> Passive
  └─ RB+X ───> Velocity

Velocity
  ├─ RB+Down ─> FixStand
  ├─ LT+Down ─> Mimic_Dance_102 ── RB+X / 动作结束 ──> Velocity
  ├─ RB+Up ───> WBC ────────────── RB+X / 动作结束 ──> Velocity
  └─ LT+Y ────> UFO_Walk ───────── RB+X / 动作结束 ──> Velocity
```

| 模式 | 效果 |
|---|---|
| `Passive` | 低刚度保护状态，也是控制器初始状态 |
| `FixStand` | 3 秒插值到固定站立姿态 |
| `Velocity` | 速度跟踪，可控制前后、横移和转向 |
| `Mimic_Dance_102` | 跟踪约 28 秒的舞蹈动作 |
| `WBC` | 播放当前 WBC 全身轨迹 |
| `UFO_Walk` | UFO-FB 双 ONNX 全身策略，播放约 10 秒参考片段 |

Dance、WBC 和 UFO 必须先进入 `Velocity`。倾倒或推理异常时策略可能自动进入
`Passive`。

## 5. 启动参数

### 5.1 MuJoCo

| 参数 | 作用 | 当前默认值 | 本项目启动值 |
|---|---|---|---|
| `-i`, `--domain_id` | DDS Domain | `1` | `0` |
| `-n`, `--network` | DDS 接口 | `eth0` | 默认即可 |
| `-r`, `--robot` | 机器人 | `go2` | `g1` |
| `-s`, `--scene` | 场景 | `scene.xml` | `scene_23dof.xml` |
| `-t`, `--idl_type` | `-1=自动`、`0=unitree_go`、`1=unitree_hg` | `-1` | `1` |

`unitree_mujoco/simulate/config.yaml` 还提供：

| 配置 | 默认值 | 作用 |
|---|---:|---|
| `use_joystick` | `0` | 使用 GUI 虚拟手柄；设为 `1` 时由 MuJoCo 读取物理手柄 |
| `joystick_type` | `xbox` | 可选 `xbox`、`switch` |
| `joystick_device` | `/dev/input/js0` | 物理手柄设备 |
| `joystick_bits` | `16` | 手柄精度，部分设备使用 `8` |
| `print_scene_information` | `1` | 打印模型信息 |
| `enable_elastic_band` | `1` | 启动时启用虚拟吊带 |

MuJoCo 快捷键：`9` 开关吊带，`7/↑` 增加支撑，`8/↓` 减少支撑，`Backspace` 重置。

### 5.2 控制器

| 参数 | 作用 | 默认值 |
|---|---|---|
| `-h`, `--help` | 显示帮助 | — |
| `-v`, `--version` | 显示版本 | — |
| `--log` | 写入旋转日志 | 关闭 |
| `-n`, `--network <接口>` | DDS 接口 | `eth0` |

## 6. 快速排查

- 一直等待 `rt/lowstate`：确认 MuJoCo 已启动，且 Domain 为 `0`、接口为 `eth0`。
- 出现 `llvmpipe`：当前是 CPU 软件渲染，重新 `source ./setup.sh` 后再启动 MuJoCo。
- UFO 按钮无效：先进入 `Velocity`。
- UFO 未出现在控制器中：重新执行第 1.1 节的 CMake 配置和编译。
- 重复或异常控制命令：确认只运行了一个 `g1_ctrl`。
