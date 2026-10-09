# unitree_g1 项目结构与 sim-to-sim 部署准备说明

> 审阅日期：2026-09-18  
> 项目位置：`~/projects/unitree_g1`  
> 文档重点：训练完成后的策略接入、MuJoCo 联调、控制链路验证与真机部署前验收；不展开 RL 训练方法。  
> 说明：本文严格区分“仓库当前实际存在的内容”和“README 描述但当前缺失的内容”。本项目来自 GitHub，上游作者机器上的绝对路径、外置依赖目录和本机不匹配是正常的移植现象；当前阶段只理解代码，不修改项目以适配本机。

## 1. 先说结论

这个仓库的核心不是训练，而是把已经导出的 ONNX 策略和参考动作接入一个面向 Unitree G1 23DoF 的 C++ 控制器，并通过与真机相同或接近的 DDS/LowCmd/LowState 接口，在 MuJoCo 中完成实机部署前的 sim-to-sim 验证。

当前主链路是：

```text
PyQt GUI / UDP
      │
      │ 写入虚拟手柄 JSON
      ▼
/dev/shm/unitree_joystick_bridge.json
      │
      ▼
g1_ctrl（1 kHz FSM；策略推理通常 50 Hz）
      │
      │ Unitree SDK2 DDS：LowCmd / LowState
      ▼
unitree_mujoco（G1 23DoF）
```

控制器包含五个 FSM 状态：`Passive`、`FixStand`、`Velocity`、`Mimic_Dance_102` 和 `WBC`。其中 `Velocity`、`Mimic`、`WBC` 各自加载 ONNX 或参考动作数据，最终都产生关节位置目标和 PD 参数。

当前 checkout 在这台机器上**尚不能开箱运行**。这不代表上游项目本身无法运行，而是 Git 仓库没有携带所有外部依赖，且作者环境与本机不同：

1. README 中的 `unitree_mujoco/`、`unitree_sdk2/`、`unitree_ros2/` 被 `.gitignore` 排除，当前工作树中实际不存在。
2. 当前机器没有 `/opt/ros/jazzy`、`/opt/ros/foxy`、`/opt/unitree_robotics`，也没有 `cmake`、`colcon` 和项目所需 C++ 开发包。
3. 仓库里的 ONNX Runtime 目录缺少 CMake 明确要求的 `libonnxruntime.so.1.22.0`。
4. 多个脚本仍硬编码旧路径 `~/unitree_g1` 或 `/home/ningwt/unitree_g1`，与现在的 `~/projects/unitree_g1` 不一致。
5. README 与当前代码已经发生较大偏移：文档中的若干包、脚本和节点在仓库中不存在。

因此，目前更准确的状态是：**部署控制器源码和策略产物已经在仓库中；本报告先解释它的设计与执行逻辑，环境补齐和本机适配留到后续共同修改阶段。**

## 2. 项目、数据、模型与报告的管理约定

后续建议统一采用以下布局：

```text
~/projects/unitree_g1/       # 代码、CMake、ROS 包、轻量配置
~/data/unitree_g1/           # 大体量动作数据、数据集、轨迹数据
~/runs/unitree_g1/           # checkpoint、ONNX、导出产物、实验运行结果
~/projects/mdreports/        # 学习/审阅用 Markdown 报告
```

当前仓库与该约定有冲突：约 18 MB 的 ONNX、NPZ、CSV 和 WBC 二进制轨迹目前都位于 `unitree_g1_ws2/deploy/robots/g1_23dof/config/policy/` 中。建议后续迁移为：

```text
~/runs/unitree_g1/velocity/policy.onnx
~/runs/unitree_g1/mimic/dance_102/policy.onnx
~/runs/unitree_g1/wbc/model.onnx

~/data/unitree_g1/mimic/dance_102/G1_Take_102.bvh_60hz.{csv,npz}
~/data/unitree_g1/wbc/motion_data/*.bin
```

源码仓库只保留轻量的 YAML 配置和一份“产物清单”（路径、版本、SHA256、输入输出维度、训练步长、关节顺序）。控制器可以通过配置中的绝对路径、环境变量展开，或只读符号链接访问这些产物。

本文只记录这一整改方向，没有擅自移动现有模型或数据，避免破坏当前相对路径。

## 3. 当前实际目录构成

```text
unitree_g1/
├── README.md
├── .gitignore
├── setup.sh
├── start_gui.sh
├── unitree_g1_ws/                    # ROS 2 Python 工作区（当前仅 2 个包）
│   ├── README.md
│   ├── compile.sh
│   ├── setup.sh
│   ├── setup_real.sh
│   └── src/
│       ├── g1_bridge/
│       │   └── g1_bridge/merger.py
│       └── g1_gui/
│           └── g1_gui/launcher.py
└── unitree_g1_ws2/deploy/            # C++ 策略部署主工程
    ├── include/
    │   ├── FSM/
    │   ├── isaaclab/
    │   ├── common/
    │   ├── param.h
    │   ├── unitree_articulation.h
    │   ├── UnitreeJoystickBridge.h
    │   ├── ArmBridge.h
    │   └── PolicyPublisher.h
    ├── robots/g1_23dof/
    │   ├── CMakeLists.txt
    │   ├── main.cpp
    │   ├── arm_ctrl.cpp
    │   ├── config/
    │   ├── include/
    │   └── src/
    ├── thirdparty/
    │   ├── cnpy/
    │   └── onnxruntime-linux-x64-1.22.0/
    └── udp_send.py
```

根目录 README 声称还应有以下官方工作区，但它们当前不存在，而且被 `.gitignore` 明确排除：

```text
unitree_mujoco/     # MuJoCo 仿真器
unitree_sdk2/       # Unitree SDK2
unitree_ros2/       # ROS 2 消息与 CycloneDDS 工作区
```

这意味着这些目录被设计为“每台机器本地准备的外部依赖”，不是本仓库的 Git 内容。

## 4. 各目录和关键文件说明

### 4.1 根目录

| 文件 | 当前作用 | 当前情况 |
|---|---|---|
| `README.md` | 项目总览、仿真和 GUI 的旧运行入口 | 架构方向仍有参考价值，但路径是旧的，命令中提到的 `joystick_emu.py` 不存在 |
| `.gitignore` | 忽略构建产物、Python 缓存及三个官方外部工作区 | 解释了为什么仿真器、SDK、ROS 2 源码不在当前 checkout 中 |
| `setup.sh` | 配置 ROS 2 Jazzy、CycloneDDS、Unitree 库和 loopback 接口 | 硬编码 `~/unitree_g1`；与当前路径不符；会清理 `/dev/shm/unitree_*.json` |
| `start_gui.sh` | source 环境后执行 `ros2 run g1_gui launcher` | 使用 ROS 2 Foxy，而其他脚本使用 Jazzy；同样硬编码旧路径 |

根目录脚本所体现的仿真通信意图是：`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，CycloneDDS 使用 `lo`，模拟器和控制器使用相同 Domain ID。

### 4.2 `unitree_g1_ws/`：ROS 2 辅助工作区

当前真实存在的包只有 `g1_bridge` 和 `g1_gui`。`unitree_g1_ws/README.md` 中描述的 `g1_controller`、`g1_arm_controller`、`start_sim.sh`、`g1_walk`、`g1_teleop`、`arm_teleop_node` 等均不在当前仓库里。

| 文件/包 | 作用 | 注意事项 |
|---|---|---|
| `compile.sh` | 执行 `colcon build --symlink-install` | 没有主动 source ROS 2 和 Unitree 消息工作区 |
| `setup.sh` | 仿真环境：Jazzy、loopback、Domain 0 | README 却写“仿真 Domain 1”，存在冲突；路径仍为 `~/unitree_g1` |
| `setup_real.sh` | 真机 ROS 2 CycloneDDS 单播发现配置 | 硬编码 `/home/ningwt/...` 和机器人 IP `192.168.12.103`，不能直接复用 |
| `g1_bridge/merger.py` | 500 Hz 读取策略/手臂 JSON，合并成 ROS 2 `LowCmd` | 属于旧的 ROS 合并架构；当前 `g1_ctrl` 已改为 DDS 直发，不应与它同时拥有控制通道 |
| `g1_bridge/setup.py` | 注册 `g1_merger` 和 `arm_ctrl_json` 命令 | `arm_ctrl_json.py` 实际缺失，对应命令不可用 |
| `g1_gui/launcher.py` | PyQt5 总控界面：进程管理、FSM 按钮、速度摇杆、UDP 动作触发 | 是当前实际的人机入口，但项目根目录硬编码为 `~/unitree_g1` |

`launcher.py` 的主要行为：

- 25 Hz 写 `/dev/shm/unitree_joystick_bridge.json`。
- 可以启动/停止 `g1_ctrl` 子进程，但不启动 MuJoCo。
- 提供 `Passive`、`FixStand`、`Velocity`、`WBC`、`Dance` 按钮。
- 监听 UDP `0.0.0.0:9876`，数字 1–8 触发六种速度动作、WBC 或跳舞。
- GUI 显示的 FSM 状态是本地推测值，不是从控制器读取的真实状态反馈。

### 4.3 `unitree_g1_ws2/deploy/`：部署核心

这是整个项目最重要的部分。

#### 通用 FSM 与接口层

| 文件 | 作用 |
|---|---|
| `include/FSM/BaseState.h` | FSM 状态基类、状态注册和转换检查的基础结构 |
| `include/FSM/CtrlFSM.h` | 以 1 ms 周期运行 FSM：更新状态、检查转换、执行控制、发布 LowCmd |
| `include/FSM/FSMState.h` | 共享 `LowCmd`、`LowState`、键盘、JSON 虚拟手柄和发布方式 |
| `include/FSM/State_Passive.h` | `kp=0`、保留阻尼并跟随当前位置，用作初始/保护状态 |
| `include/FSM/State_FixStand.h` | 从进入时姿态到预设站立姿态做 3 秒线性插值 |
| `include/FSM/State_RLBase.h` | Velocity 策略状态，管理 ONNX session、策略线程和控制增益 |
| `include/FSM/State_WBC.h` | WBC 模仿策略的观测、轨迹、ONNX 推理和恢复逻辑 |
| `include/param.h` | 解析 `--network`、`--log`、`--help`，根据可执行文件位置寻找 `config/config.yaml` |
| `include/unitree_articulation.h` | 把 Unitree `LowState` 转成策略使用的 IMU、投影重力、关节位置和速度 |
| `include/UnitreeJoystickBridge.h` | 读取 GUI 写出的 JSON，并覆盖 SDK joystick 状态 |
| `include/ArmBridge.h` | 从 JSON 读取独立手臂关节指令并覆盖 LowCmd；当前主程序未启用 |
| `include/PolicyPublisher.h` | 把完整 LowCmd 写入 JSON，供 ROS 合并器读取；当前主程序未启用 |
| `include/LinearInterpolator.h` | FixStand 的分段线性插值工具 |

#### 精简的 Isaac Lab 部署运行时

`include/isaaclab/` 不是完整 Isaac Lab，而是部署侧的轻量 C++ 复刻，包含：

- `ManagerBasedRLEnv`：按 YAML 组织机器人、观测、动作和算法。
- `ObservationManager`：拼接、缩放、裁剪并维护历史观测。
- `ActionManager`：将网络输出按 `scale + offset` 转成关节目标，并进行 clip。
- `OrtRunner`：通过 ONNX Runtime 在 CPU 上推理。
- `observations.h`：角速度、投影重力、速度指令、关节位置/速度、上一动作等。
- `joint_actions.h`：关节位置或速度动作的部署侧后处理。

这层代码决定 sim-to-sim 是否可信：训练时和部署时的观测顺序、历史堆叠、缩放、关节映射、动作缩放及默认姿态必须逐项一致。

#### G1 23DoF 控制器

| 文件 | 作用 | 当前情况 |
|---|---|---|
| `robots/g1_23dof/CMakeLists.txt` | 构建 `g1_23dof_controller_lib` 和 `g1_ctrl` | 硬编码 `/opt/unitree_robotics`、Eigen、静态 Boost/yaml-cpp、bundled ONNX Runtime |
| `main.cpp` | 初始化 SDK2 Channel、等待 LowState、创建 FSM 和虚拟手柄桥 | 当前直接调用 `lowcmd->unlockAndPublish()`，不经过 ROS merger |
| `config/config.yaml` | Domain、机器模式、FSM 状态、转换、站立姿态、三个策略路径 | 是部署总配置入口 |
| `src/State_RLBase.cpp` | Velocity 策略加载、倾倒检测、关节动作写回 | ONNX 50 Hz，LowCmd 由 FSM 1 kHz 发布 |
| `src/State_Mimic.cpp` | 舞蹈参考轨迹加载、23DoF 观测构造、ONNX 推理和自动退出 | 使用 CSV 参考动作，默认播放 0–28 秒 |
| `src/State_WBC.cpp` | 读取二进制动作轨迹，构造参考观测并运行 WBC ONNX | 使用 29DoF 模型，通过 dummy joint 适配 23DoF 机器人 |
| `include/State_Mimic.h` | CSV motion loader、29DoF 到 23DoF 关节筛选和插值 | 参考动作按 60 Hz 解释 |
| `include/Types.h` | 将 Unitree G1 SDK 的 LowCmd/LowState 类型设为项目别名 | 同时 include Go2 和 G1 wrapper，但实际使用 G1 |
| `arm_ctrl.cpp` | DDS 直连的右臂 5 关节独立控制器 | CMake 没有为它创建 executable；也不能与另一 LowCmd 发布者随意并发 |
| `include/keyboard.h` | 原始终端键盘读取器 | `keyboard.py` 是它的逐字副本，扩展名错误，不是 Python 文件 |

#### 策略与动作产物

| 产物 | 大小约 | 用途 |
|---|---:|---|
| `velocity/exported/policy.onnx` | 1.47 MB | 23DoF 速度跟踪策略 |
| `mimic/dance_102/exported/policy.onnx` | 0.93 MB | 舞蹈动作模仿策略 |
| `mimic/.../G1_Take_102...csv` | 0.60 MB | 部署时实际读取的 60 Hz 参考动作 |
| `mimic/.../G1_Take_102...npz` | 2.09 MB | 动作数据归档；当前 C++ 运行链路不读取 |
| `wbc/model.onnx` | 1.58 MB | WBC/模仿策略 |
| `wbc/motion_data/*.bin` | 合计约 11 MB | 位姿、四元数、速度、关节和 FPS 参考数据 |

#### 第三方代码

- `thirdparty/cnpy/`：NPY/NPZ 读写库和转换工具。当前 WBC 使用自定义 BIN reader，Mimic 使用 CSV，因此主运行链路对它的直接依赖有限。
- `thirdparty/onnxruntime-linux-x64-1.22.0/`：版本文件和 C/C++ 头文件存在，但 `lib/` 中只有 `pkgconfig/`，没有 CMake 要链接的动态库。
- `include/nlohmann/json.hpp`：单头 JSON 库，用于共享内存 JSON 桥。

## 5. 控制链路和时序

### 5.1 启动阶段

`g1_ctrl` 的启动逻辑如下：

1. `param::helper()` 从可执行文件位置推导工程目录并加载 `config/config.yaml`。
2. 读取 `domain_id`，再用 `--network` 指定的网卡初始化 SDK2 `ChannelFactory`。
3. 创建 LowCmd/LowState 对象，并阻塞等待仿真器或机器人发布 LowState。
4. 校验 `mode_machine`，当前配置值为 `4`。
5. 构造所有启用的 FSM 状态；这一步会立即加载 YAML、ONNX 和动作文件。
6. 从 `Passive` 状态启动 1 kHz FSM 线程。
7. 创建 `/dev/shm/unitree_joystick_bridge.json` 的读取桥。

由于策略和动作在 FSM 构造阶段加载，即使某个模式暂时不用，对应文件缺失也可能让整个控制器在启动时失败。

### 5.2 FSM 状态转换

以 `config/config.yaml` 为准，当前真实转换关系是：

```text
Passive -- LT+Up --> FixStand

FixStand -- LT+B --> Passive
FixStand -- RB+X --> Velocity

Velocity -- RB+Down --> FixStand
Velocity -- LT+Down --> Mimic_Dance_102
Velocity -- RB+Up --> WBC

Mimic_Dance_102 -- RB+X 或 28 s 结束 --> Velocity
Mimic/WBC 姿态异常 --> Passive 或 recovery

WBC -- 动作结束或 RB+X --> Velocity
```

GUI 注释声称可以从 FixStand 用 `RB+Y` 进入 Dance，但 YAML 中没有这条转换，所以当前 GUI 在 FixStand 点击 Dance 不会触发控制器状态切换。

### 5.3 各频率

| 环节 | 频率 |
|---|---:|
| FSM、LowState 更新、LowCmd 发布 | 1000 Hz |
| Velocity ONNX | `step_dt=0.02`，即 50 Hz |
| Mimic ONNX | `step_dt=0.02`，即 50 Hz |
| WBC ONNX | FSM 每 20 tick 推理一次，即 50 Hz |
| GUI 虚拟摇杆 | 25 Hz |
| 独立 `arm_ctrl.cpp` | 50 Hz |
| 旧 ROS `g1_merger` | 500 Hz |

sim-to-sim 必须检查 MuJoCo、LowState 时间戳和控制循环是否能稳定满足这些频率，而不只是观察机器人“看起来能走”。

### 5.4 23DoF 与电机索引

策略动作维数是 23，但 LowCmd/LowState 容器按 29 或 35 个槽位处理。Velocity 的 `joint_ids_map` 为：

```text
[0, 6, 12, 1, 7, 15, 22, 2, 8, 16, 23, 3, 9, 17, 24, 4, 10, 18, 25, 5, 11, 20, 27]
```

WBC 是 29DoF 模型适配 23DoF 机体，以下槽位被当作 dummy joint：

```text
13, 14, 20, 21, 27, 28
```

其中 Velocity 与 Mimic 的最后两个手臂/腕部映射并不完全相同。任何模型替换都必须确认：

- ONNX 输出顺序；
- `joint_ids_map`；
- `default_joint_pos` / action `offset`；
- stiffness、damping；
- MuJoCo XML actuator 顺序；
- 真机 SDK motor index。

这些是 sim-to-sim 最容易“能运行但控制含义已经错位”的地方。

## 6. 从策略产物到 sim-to-sim 的推荐完整流程

### 阶段 A：冻结部署契约

在接入任一新模型前，为模型保存一份部署清单：

1. ONNX 输入名、输入 shape、输出名、输出 shape。
2. 观测项顺序、每项维数、scale、clip 和 history length。
3. 动作关节顺序、scale、offset 和 clip。
4. 训练/部署控制周期，例如 20 ms。
5. 默认关节姿态、PD 增益和 torque limit。
6. 机器人资产版本、23DoF/29DoF 约定和导出 commit。
7. ONNX、YAML、动作数据的 SHA256。

`deploy.yaml` 已包含其中一部分，但目前没有集中记录版本和哈希。

### 阶段 B：补齐外部环境

目标环境从源码中的硬编码看，应满足：

- Ubuntu 上的 ROS 2 Jazzy（应统一，不再混用 Foxy）。
- CycloneDDS 和 `rmw_cyclonedds_cpp`。
- Unitree SDK2 安装在 `/opt/unitree_robotics/include` 和 `/opt/unitree_robotics/lib`，或同步修改 CMake。
- Eigen3、Boost program_options、yaml-cpp、fmt、pthread、rt。
- ONNX Runtime 1.22.0 的头文件和 `libonnxruntime.so.1.22.0`。
- CMake、C++17 兼容编译器、colcon、Python 3、PyQt5。
- Unitree ROS 2 消息包仅在使用 ROS bridge 时需要；DDS 直连主链路本身不依赖 ROS topic。

随后将官方依赖放在：

```text
~/projects/unitree_g1/unitree_mujoco
~/projects/unitree_g1/unitree_sdk2
~/projects/unitree_g1/unitree_ros2
```

这三个目录不会被本仓库提交，必须单独记录各自 commit。

### 阶段 C：统一配置

在编译前至少完成：

1. 把所有 `~/unitree_g1` 和 `/home/ningwt/unitree_g1` 改为基于脚本位置计算的路径。
2. 全部统一到 ROS 2 Jazzy。
3. 统一仿真 Domain ID；当前 `config.yaml` 和 `unitree_g1_ws/setup.sh` 都是 0，README 中“仿真=1”的说法应修正或落实到所有进程。
4. MuJoCo 使用 G1 `scene_23dof.xml`。
5. MuJoCo 的 DDS interface 设为 `lo`，Domain ID 与控制器相同。
6. 明确只使用一种 LowCmd 所有权架构：推荐当前 DDS 直连主链路。

### 阶段 D：构建

ROS 辅助工作区理论命令：

```bash
cd ~/projects/unitree_g1/unitree_g1_ws
source /opt/ros/jazzy/setup.bash
source ../unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build --symlink-install
```

C++ 控制器理论命令：

```bash
cd ~/projects/unitree_g1/unitree_g1_ws2/deploy/robots/g1_23dof
cmake -S . -B build
cmake --build build -j"$(nproc)"
```

这两组命令代表代码设计的构建方式，但在当前机器上不能成功执行：系统连 `cmake` 都未安装，并且前述 SDK/ONNX 动态库也缺失。

### 阶段 E：静态预检

启动物理仿真前检查：

```bash
test -x ~/projects/unitree_g1/unitree_g1_ws2/deploy/robots/g1_23dof/build/g1_ctrl
test -f ~/projects/unitree_g1/unitree_g1_ws2/deploy/robots/g1_23dof/config/config.yaml
test -f ~/projects/unitree_g1/unitree_g1_ws2/deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so.1.22.0
ldd ~/projects/unitree_g1/unitree_g1_ws2/deploy/robots/g1_23dof/build/g1_ctrl
```

还应写一个独立的 preflight 工具，自动核对 YAML 数组长度、ONNX shape、关节索引范围、策略文件哈希和所有引用路径。

### 阶段 F：启动 MuJoCo

先启动仿真器，因为 `g1_ctrl` 会等待 LowState：

```bash
cd ~/projects/unitree_g1/unitree_mujoco/simulate/build
./unitree_mujoco
```

MuJoCo 配置应明确为：

```yaml
robot: g1
robot_scene: scene_23dof.xml
domain_id: 0
interface: lo
```

启动后先确认模型静止、关节索引和传感器输出正常，再启动控制器。

### 阶段 G：启动控制器

命令行方式：

```bash
cd ~/projects/unitree_g1/unitree_g1_ws2/deploy/robots/g1_23dof/build
./g1_ctrl --network lo --log
```

不要从其他目录复制可执行文件单独运行；`param.h` 依赖可执行文件位于 `build/`，以便定位上一级 `config/`。

GUI 方式的预期流程是：

```bash
cd ~/projects/unitree_g1
./start_gui.sh
```

但当前必须先修复 `start_gui.sh` 和 `launcher.py` 的路径及 ROS 版本，否则 GUI 找不到 `g1_ctrl`。GUI 只负责启动控制器，不会自动启动 MuJoCo。

也不要同时手工启动 `g1_ctrl` 和再让 GUI 启动第二个实例，否则会出现多个 LowCmd 发布者。

### 阶段 H：按风险递增验证 FSM

推荐每一步都记录 LowState、目标关节、实际关节、推理时间和状态转换：

1. `Passive`：确认无位置刚度突变，仿真稳定。
2. `LT+Up → FixStand`：确认 3 秒插值连续，没有初始姿态跳变。
3. `RB+X → Velocity`：先零速度站立至少 30 秒。
4. 给小幅 `vx`，再分别测试 `vy`、`wz`；不要一开始满量程。
5. 验证松开输入后命令可靠归零。
6. 在 Velocity 中用 `LT+Down` 进入 Mimic，检查 28 秒结束后自动回到 Velocity。
7. 在 Velocity 中用 `RB+Up` 进入 WBC，检查参考帧 100–300 和退出衔接。
8. 在每个主动状态测试 `Passive`/倾倒保护/进程退出后的安全行为。

### 阶段 I：sim-to-sim 验收指标

至少保存以下量化结果：

- 1 kHz 控制循环和 50 Hz 推理循环的平均周期、P99、最大抖动。
- ONNX 单次推理平均/P99 耗时。
- 每个关节的目标与实际位置误差、速度峰值、扭矩峰值。
- base 高度、roll/pitch、投影重力和足底接触。
- 零命令漂移、速度跟踪误差和停止距离。
- FSM 切换前后的关节目标跳变量。
- 输入断联、GUI 崩溃、控制器崩溃、仿真重启时的行为。
- 同一模型多次运行的可重复性。

只有“机器人在窗口里能动”不算完成 sim-to-sim；必须证明接口、数值语义、时序和安全状态都与实机部署契约一致。

## 7. 阅读代码时应留意的事项（暂不修改）

以下内容分成两类：一类是从 GitHub 项目移植到本机时自然会遇到的环境差异；另一类是代码本身值得在进入真机前复核的控制与并发风险。本次只记录，不对源码做修改。

### 7.1 后续本机适配事项

1. **外部依赖缺失**：MuJoCo、SDK2、ROS 2 工作区不在当前 checkout；当前系统也没有对应安装。
2. **ONNX Runtime 动态库缺失**：CMake 指定的 `.so` 不存在，C++ 控制器无法链接。
3. **硬编码路径错误**：GUI 和 setup 脚本找不到当前 `~/projects/unitree_g1`。
4. `start_gui.sh` 使用 Foxy，而其他脚本和 README 使用 Jazzy，需要结合上游原始运行环境确认最终采用哪个版本。
5. README 说仿真 Domain ID 为 1，实际 setup 和 `config.yaml` 是 0，移植时要统一所有进程。
6. `g1_bridge/setup.py` 注册了当前 checkout 中不存在的 `arm_ctrl_json.py`；ROS workspace README 也描述了若干未随本仓库提供的包和脚本。
7. `arm_ctrl.cpp` 没有对应 CMake executable target，`keyboard.py` 则实际是 C++ 头文件的重复副本。这些可能是上游开发过程中保留的中间文件。

### 7.2 理解和验证控制逻辑时应重点复核

1. **输入没有失联归零**：`UnitreeJoystickBridge` 在 JSON 文件不再更新时会持续复用最后一次 payload，没有按时间戳或 mtime age 清零。若 GUI 在非零速度时退出，最后速度可能持续生效；真机前应验证或加入 watchdog。
2. **LowCmd 所有权存在多套候选架构**：代码同时保留 DDS 直连、ROS merger 和独立 arm DDS publisher。阅读时要分清当前主程序实际启用的是 DDS 直连，不要把三套链路误认为需要同时运行。
3. **FSM debug 残留表达式**：`CtrlFSM.h` 第 94 行存在未受 `has_value()` 保护的 optional 解引用，后续构建和运行前应复核。
4. GUI 从 FixStand 触发 Dance 的 `RB+Y` 不在当前 FSM YAML 中；GUI 状态也只是本地推测，没有控制器真实状态回传。
5. 策略线程和 1 kHz FSM 线程之间共享 action/env 状态，当前没有完整同步；理解执行时序时要意识到这里存在并发访问。
6. 倾角计算对 `acos()` 输入没有显式 clamp 到 `[-1, 1]`，需要在异常姿态测试中关注 NaN。
7. 配置和源码中同时出现 23、29、35 三种数量：分别接近有效策略 DoF、G1 电机槽位和消息容量，阅读时不能混为一谈。

### 7.3 后续工程整理方向

1. 大模型和动作数据迁移到 `~/runs/unitree_g1` 与 `~/data/unitree_g1`。
2. 记录外部仓库 commit、依赖版本和可复现环境。
3. 增加单元测试、接口回放、preflight 检查和自动 sim smoke test。
4. 将硬编码路径改成基于项目根目录或配置文件解析。

## 8. 真机前的放行门槛

以下条件全部通过前，不建议运行 `setup_real.sh` 或把 `g1_ctrl` 接到实机：

- sim-to-sim 连续运行达到预定时长，无 NaN、越界、推理异常和控制周期失速。
- 每一个 joint index 都通过人工和自动双重核验。
- 启动、站立、Velocity、Mimic、WBC、退出、急停路径均在仿真中重复通过。
- 输入 watchdog 已实现，并验证 GUI/网络/控制进程异常时命令归零或进入安全状态。
- 只有一个 LowCmd owner。
- 真机网络接口、Domain ID、机器人型号和 `mode_machine` 在启动前明确显示并要求确认。
- 关节位置、速度、力矩、PD 增益、动作变化率均有部署侧限幅。
- 初次真机测试使用悬挂/保护架、低增益、小动作、独立急停和现场监护。
- 真机日志能记录配置哈希、模型哈希、状态转换和故障原因。

## 9. 本次静态审阅实际验证了什么

- Git 工作树在审阅前是干净的，仓库只有一个初始化提交。
- 枚举并阅读了仓库中的主要脚本、ROS 包、CMake、FSM、策略接入代码和 YAML。
- `merger.py`、`launcher.py`、`udp_send.py` 可通过 Python AST 语法解析。
- `keyboard.py` 不能作为 Python 解析，因为它实际是 C++ 文件副本。
- 当前机器没有 `cmake`，因此未完成 C++ configure/compile。
- 当前机器没有 ROS 2、Unitree SDK 安装和必需的 C++ 开发依赖，因此未进行运行时验证。
- 没有启动 MuJoCo、控制器或任何可能发布 LowCmd 的进程。

## 10. 后续一起修改时的建议顺序

1. 先结合上游 README/运行环境确认依赖版本，再适配路径、ROS 版本和 Domain ID，做到本机可构建。
2. 明确 DDS 直连为唯一主架构，删除或隔离旧 merger 路径。
3. 实现虚拟手柄 watchdog 和唯一 LowCmd owner。
4. 新增 preflight 工具，自动检查模型、YAML、shape、关节映射和哈希。
5. 建立 MuJoCo 一键启动和最小 smoke test：Passive → FixStand → 零速 Velocity → Passive。
6. 再逐步加入速度跟踪、Mimic、WBC 和异常注入测试。
7. 将数据与模型迁移至 `~/data/unitree_g1`、`~/runs/unitree_g1`，配置中引用外部产物。
8. 完成量化 sim-to-sim 验收报告后，再单独编写真机部署 SOP。

---

这份仓库当前最有价值的部分，是已经把训练框架里的观测/动作定义压缩成了可部署的 C++ 运行时，并建立了 FSM、DDS、ONNX 和动作轨迹之间的连接。后续工作的重点应是把这条链路变成**可构建、可复现、可测量、有 watchdog、只有一个控制权所有者**的仿真验收系统。
