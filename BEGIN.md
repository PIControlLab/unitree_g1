# Unitree G1 仿真启动与操作指南

本文针对当前仓库的 **G1 23DoF + MuJoCo + `g1_ctrl` + GUI** 仿真链路。
以下命令默认项目位于：

```text
/home/hanlichen/projects/unitree_g1
```

本文只描述仿真启动，不适用于直接连接实机。不要同时启动两个 `g1_ctrl`，否则会有多个
`LowCmd` 发布者争夺控制权。

## 1. 模式效果与允许的跳转

控制器启动后的初始状态固定为 `Passive`。当前 FSM 的正常跳转关系如下：

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

部分策略检测到倾倒或推理异常时会自动进入 `Passive`，这属于保护路径，不是推荐的日常
切换方法。

| 模式 | 实现效果 | 进入方式 | 正常退出方式 |
|---|---|---|---|
| `Passive` | 位置刚度 `kp=0`，只保留阻尼并跟随当前关节位置；用于控制器初始状态和保护状态 | 控制器启动时自动进入；或在 `FixStand` 中按 `LT+B` | `LT+Up` 进入 `FixStand` |
| `FixStand` | 用 3 秒插值从当前姿态进入固定站立姿态 | 仅允许从 `Passive` 按 `LT+Up`，或从 `Velocity` 按 `RB+Down` | `RB+X` 进入 `Velocity`；`LT+B` 返回 `Passive` |
| `Velocity` | 50 Hz 速度跟踪策略；GUI 摇杆控制前后、横移和转向 | 仅从 `FixStand` 按 `RB+X`；动作策略结束后也会自动返回 | `RB+Down` 返回 `FixStand`，或进入下面三个动作状态 |
| `Mimic_Dance_102` | 跟踪 23DoF 舞蹈参考动作，配置播放区间约 28 秒 | 仅从 `Velocity` 按 `LT+Down` | `RB+X` 或动作结束后回 `Velocity`；严重倾斜会进入 `Passive` |
| `WBC` | 播放 WBC 全身参考轨迹，当前帧范围 100–300，约 4 秒 | 仅从 `Velocity` 按 `RB+Up` | `RB+X` 或动作结束后回 `Velocity` |
| `UFO_Walk` | UFO-FB 双 ONNX 全身策略，默认播放 `walk1_subject1` 的 130–140 秒片段，共 10 秒 | 仅从 `Velocity` 按 `LT+Y` | `RB+X` 或 10 秒结束后回 `Velocity`；倾倒或推理异常进入 `Passive` |

注意：当前 GUI 在 `FixStand` 状态点击 Dance 时会尝试发送 `RB+Y`，但当前 FSM 配置没有
`FixStand -> Dance` 这条跳转。因此跳舞、WBC 和 UFO 都应先进入 `Velocity`，再点击对应按钮。

## 2. 启动命令与参数

### 2.1 推荐的 DDS 配置

本机 WSL 内的 MuJoCo、控制器和 GUI 全部使用：

| 配置 | 本项目默认值 |
|---|---|
| DDS Domain | `0` |
| 网络接口 | `lo` |
| 机器人 | `g1` |
| 场景 | `scene_23dof.xml` |
| DDS IDL | `1`，即 `unitree_hg` |
| 机器类型 `mode_machine` | `4` |
| MuJoCo OpenGL 渲染 | WSLg D3D12，优先使用 NVIDIA RTX 3050 Ti |

项目根目录的 `setup.sh` 会加载 ROS 2 Jazzy、工程工作区和 Unitree 动态库，并把
CycloneDDS 限制到 `lo`。它还会清理旧的 `/dev/shm/unitree_*.json`，所以应在启动 GUI
和控制器之前执行，不要在 GUI 已运行后反复执行。脚本同时设置：

```bash
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
```

这会让 WSLg 的 Mesa OpenGL 通过 D3D12 使用 NVIDIA 显卡。它只加速 MuJoCo 窗口的
OpenGL 可视化，MuJoCo 动力学求解和控制器 ONNX 推理仍主要运行在 CPU 上。

每次启动前可用以下命令确认实际 renderer，不能只看 `nvidia-smi`：

```bash
cd /home/hanlichen/projects/unitree_g1
source ./setup.sh
glxinfo -B | grep -E 'direct rendering|Device:|Accelerated:|OpenGL renderer'
```

本机正确结果应包含：

```text
direct rendering: Yes
Device: D3D12 (NVIDIA GeForce RTX 3050 Ti Laptop GPU)
Accelerated: yes
OpenGL renderer string: D3D12 (NVIDIA GeForce RTX 3050 Ti Laptop GPU)
```

如果仍显示 `llvmpipe` 或 `Accelerated: no`，此时启动 MuJoCo 仍然是 CPU 软件渲染，
应先停止并修复图形环境。

### 2.2 终端 A：启动 MuJoCo

```bash
cd /home/hanlichen/projects/unitree_g1
source ./setup.sh
cd unitree_mujoco/simulate/build
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
  ./unitree_mujoco -i 0 -n lo -r g1 -s scene_23dof.xml -t 1
```

必须显式给出上面的参数，因为 `unitree_mujoco/simulate/config.yaml` 的上游基础默认值仍是
Go2、Domain 1。命令行参数会覆盖它们。

当前使用的是 C++ MuJoCo + GLFW 窗口。`MUJOCO_GL=egl` 主要用于 Python/headless
渲染，不能替代这里的 WSLg D3D12 OpenGL 配置。

MuJoCo 支持的命令行选项如下：

| 选项 | 含义 | 仓库 YAML 值 | 本项目推荐值 |
|---|---|---|---|
| `-h`, `--help` | 显示帮助 | — | 按需使用 |
| `-i`, `--domain_id` | DDS Domain ID | `1` | `0` |
| `-n`, `--network` | DDS 网络接口 | `lo` | `lo` |
| `-r`, `--robot` | 机器人类型 | `go2` | `g1` |
| `-s`, `--scene` | 对应机器人目录中的场景文件 | `scene.xml` | `scene_23dof.xml` |
| `-t`, `--idl_type` | `-1=自动`、`0=unitree_go`、`1=unitree_hg` | `-1` | `1` |

以下选项只能在 `unitree_mujoco/simulate/config.yaml` 中设置，不能由当前命令行覆盖：

| YAML 项 | 当前值 | 说明 |
|---|---:|---|
| `use_joystick` | `0` | 不让 MuJoCo 自己读取物理手柄；本项目默认由 GUI 写虚拟手柄数据 |
| `joystick_type` | `xbox` | `use_joystick=1` 时使用，可选 `xbox` 或 `switch` |
| `joystick_device` | `/dev/input/js0` | 物理手柄设备 |
| `joystick_bits` | `16` | 手柄精度；部分手柄需要 `8` |
| `print_scene_information` | `1` | 启动时打印连杆、关节和传感器信息 |
| `enable_elastic_band` | `1` | 启用虚拟吊带，推荐保留到站立策略稳定后再释放 |

MuJoCo 窗口中的相关快捷键：

| 按键 | 当前代码行为 |
|---|---|
| `9` | 启用/禁用虚拟吊带 |
| `7` 或 `↑` | 缩短吊带长度，增加向上支撑、吊起机器人 |
| `8` 或 `↓` | 加长吊带长度，减小向上支撑、放低机器人 |
| `Backspace` | 重置 MuJoCo 状态；重置前应先停止控制器，避免旧命令立即重新作用 |

### 2.3 终端 B：启动 GUI 和控制器

推荐使用项目脚本：

```bash
cd /home/hanlichen/projects/unitree_g1
./start_gui.sh
```

GUI 启动后，再点击左侧 **G1 控制器（g1_ctrl）** 卡片中的“启动”。GUI 本身只负责启动
控制器、写入虚拟手柄 JSON 和发送模式切换，不会启动 MuJoCo。

`start_gui.sh` 和 GUI 当前没有额外命令行选项。GUI 启动的 `g1_ctrl` 也不传参数，它依赖：

- `setup.sh` 设置的 `lo` CycloneDDS 接口；
- `config/config.yaml` 中的 `domain_id: 0` 和 `mode_machine: 4`；
- UDP 监听端口 `9876`。

如需在终端直接观察控制器日志，可以不用 GUI 启动控制器，改为：

```bash
cd /home/hanlichen/projects/unitree_g1
source ./setup.sh
cd unitree_g1_ws2/deploy/robots/g1_23dof/build
./g1_ctrl --network lo --log
```

`g1_ctrl` 可选参数：

| 选项 | 含义 | 默认/推荐 |
|---|---|---|
| `-h`, `--help` | 显示帮助 | 按需使用 |
| `-v`, `--version` | 显示版本 | 当前为 `1.0.0.1` |
| `--log` | 同时记录旋转日志 | 默认关闭；排查问题时推荐开启 |
| `-n`, `--network <接口>` | DDS 网络接口 | 程序默认空字符串；本项目推荐显式使用 `lo` |

手工启动 `g1_ctrl` 后不要再在 GUI 中点击“启动”，两者只能选一个。手工模式下如仍需 GUI
按钮，应让 GUI 只负责写手柄文件，且不要启动它的控制器进程。

### 2.4 可选：UDP 动作触发

GUI 运行时会监听 `0.0.0.0:9876`。另一终端或局域网电脑可运行：

```bash
cd /home/hanlichen/projects/unitree_g1/unitree_g1_ws2/deploy
python3 udp_send.py <运行GUI的电脑IP> 9876
```

其中 `1–6` 是速度动作，`7=WBC`、`8=Dance`、`9=UFO`。WBC、Dance、UFO 指令只有在
控制器实际处于 `Velocity` 时才会生效。

## 3. 正确的启动后操作顺序

### 3.1 首次安全启动

1. 先启动 MuJoCo，确认加载的是 `g1/scene_23dof.xml`、Domain 0、`lo` 和
   `unitree_hg`。此时保持虚拟吊带启用，不要立即按 `9`。
2. 再启动 GUI，并在 GUI 中启动 `g1_ctrl`。等待控制器连接成功；FSM 初始应为
   `Passive`。
3. 确认 GUI 速度摇杆位于中心、没有 UDP 端持续发送命令。
4. 点击 `FixStand`，对应 `LT+Up`。至少等待 3 秒，让机器人完成固定站立姿态插值。
5. 在 MuJoCo 窗口中逐次按 `8` 或 `↓`，缓慢加长吊带，使双脚接近并接触地面；不要一次
   大幅释放。
6. 点击 `Velocity`，对应 `RB+X`。保持所有速度指令为零，观察 3–5 秒，确认腿部没有持续
   抖动、姿态没有明显倾斜。
7. 在 MuJoCo 窗口按一次 `9`，禁用虚拟吊带。观察机器人是否能在零速度下独立稳定站立。
8. 先用很小的前进指令验证 `Velocity`，再逐步测试后退、横移和转向。
9. 只有在零速和小速度测试稳定后，才从 `Velocity` 进入 `WBC`、`Mimic_Dance_102` 或
   `UFO_Walk`。动作区域应留足空间，并准备随时按 `RB+X` 回到 `Velocity`。

推荐顺序可以概括为：

```text
启动 MuJoCo（吊带开启）
  -> 启动 g1_ctrl（Passive）
  -> FixStand，等待 3 秒
  -> 缓慢放低机器人至脚掌接地
  -> Velocity，零速度稳定 3–5 秒
  -> 按 9 关闭吊带
  -> 小速度测试
  -> WBC / Dance / UFO
```

### 3.2 正常结束

1. 等动作模式自动回到 `Velocity`，或按 `RB+X` 主动返回。
2. 将速度命令归零并等待机器人站稳。
3. 如需额外保护，可先按 `9` 重新启用吊带，再用 `7` 或 `↑` 逐步增加支撑。
4. 在 `Velocity` 中按 `RB+Down` 回到 `FixStand`。
5. 在 `FixStand` 中按 `LT+B` 回到 `Passive`。
6. 停止 GUI 中的 `g1_ctrl`，最后关闭 MuJoCo。

### 3.3 异常处理

- 机器人已经摔倒：停止控制器，不要在倒地姿态直接切换动作策略。需要重置时，先停
  `g1_ctrl`，在 MuJoCo 窗口按 `Backspace`，然后从完整启动顺序重新开始。
- 模式按钮无响应：确认当前状态是否允许该跳转。尤其是 Dance、WBC、UFO 必须从
  `Velocity` 进入。
- 控制器一直显示等待连接：检查 MuJoCo 是否先启动，以及两边是否同时使用 Domain 0、
  `lo` 和 `unitree_hg`。
- 机器人收到异常或重复命令：检查是否同时运行了多个 `g1_ctrl`、旧的 merger 或其他
  `LowCmd` 发布者。
- GUI 状态显示与机器人不一致：GUI 当前显示的是本地推测状态，不是控制器返回的真实
  FSM 状态；应以控制器日志和机器人实际行为为准。
