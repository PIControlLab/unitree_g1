# unitree_g1 环境安装与验证记录

> 操作日期：2026-09-18
> 项目目录：`/home/hanlichen/projects/unitree_g1`
> 系统：WSL2、Ubuntu 24.04
> 本报告第 1–10 节是当日安装与测试记录；其中“当前”“没有修改脚本”
> 等措辞均指 2026-09-18，不代表现在的代码。

## 0. 2026-09-24 源码与环境复核

当前源码结构、FSM 和操作顺序分别见同目录
《unitree_g1_sim_to_sim代码导读.md》和项目根目录 `BEGIN.md`。
本节说明历史记录与现状的差别：

- 控制器现有六个状态：Passive、FixStand、Velocity、Mimic、WBC、UFO；
  四个策略状态共用 `State_PolicyBase` 和 YAML schema 2。
- UFO 部署只有 policy ONNX 分片、catalog 和 29 个预计算 z NPZ；
  不再包含 backward encoder、原始参考动作或 z 生成脚本。
- 根目录 `setup.sh` 已按脚本位置定位 ROS 2 工作区，使用 Jazzy 和
  WSL `eth0`；`start_gui.sh` 加载该环境。GUI 管理 `g1_ctrl`
  并读取真实 FSM 状态。
- 本机已有官方依赖目录、ONNX Runtime 1.22.0 动态库和控制器构建产物。
  2026-09-24 的 `g1_ctrl` 编译通过，UFO catalog、29 个 z 文件及
  policy 校验值通过静态检查；尚未据此证明新的 DDS、FSM 动作或实机行为。
- 第 6 节的 Passive 短时 DDS 联调仅是 2026-09-18 的历史结果；
  第 7、8 节的手工环境配置和命令保留作当日记录。
  当前启动请以 `BEGIN.md` 和根目录 `setup.sh` 为准。

## 1. 2026-09-18 安装完成时的状态

环境安装和主要构建验证已经完成：

- ROS 2 Jazzy、CycloneDDS RMW、colcon 可用。
- Unitree SDK2 已安装到 `/opt/unitree_robotics`。
- MuJoCo 3.3.6 已安装，并成功加载 G1 23DoF 图形仿真。
- Unitree ROS 2 的 `unitree_api`、`unitree_go`、`unitree_hg` 均构建成功。
- 项目的 `g1_gui` 和 `g1_bridge` ROS 包构建成功。
- `g1_ctrl` 构建、链接和命令行加载成功。
- MuJoCo 与 `g1_ctrl` 已在 WSL `eth0` 上完成短时 DDS 联调，FSM 成功进入 `Passive`。
- 没有修改项目控制逻辑或原有脚本。

## 2. 安装前环境

安装前已有：

- GCC/G++ 13.3
- GNU Make
- Git
- Anaconda，默认 shell 会进入 Conda `base`
- WSLg 图形环境
- `libglfw3-dev`

安装前缺少：

- CMake、Ninja、pkg-config
- ROS 2、colcon、CycloneDDS RMW
- Unitree SDK2
- Unitree MuJoCo 和 MuJoCo C++ SDK
- Eigen、Boost、yaml-cpp、fmt、spdlog 等开发包
- 系统 Python 对应的 PyQt5
- 项目 CMake 所需的 ONNX Runtime 1.22.0 动态库

## 3. 对系统 apt 环境的修改

### 3.1 软件源

- 启用了 Ubuntu `universe` component。
- 安装了 `ros2-apt-source 1.3.0~noble`，用于管理 ROS 2 官方 apt 软件源和签名。
- 没有执行 `apt upgrade` 或 `apt dist-upgrade`。

安装过程中系统后台存在独立的 `unattended-upgrade` 记录；它不是本项目安装脚本显式执行的全系统升级。

### 3.2 直接请求安装的主要包

| 软件 | 已安装版本 |
|---|---|
| `ros-jazzy-ros-base` | `0.11.0-1noble.20260903.093452` |
| `ros-dev-tools` | `1.0.3` |
| `ros-jazzy-rmw-cyclonedds-cpp` | `2.2.4-1noble.20260902.041134` |
| `ros-jazzy-rosidl-generator-dds-idl` | `0.11.1-3noble.20260612.050127` |
| CMake | `3.28.3` |
| Ninja | `1.11.1` |
| GCC/G++ | `13.3.0` |
| Boost | `1.83.0` |
| Eigen | `3.4.0` |
| yaml-cpp | `0.8.0` |
| fmt | `9.1.0` |
| spdlog | `1.12.0` |
| GLFW | `3.3.10` |
| zlib | `1.3` |
| system Python | `3.12.3` |
| PyQt5 | `5.15.10`（Qt runtime 5.15.13） |
| `python3-colcon-common-extensions` | `0.3.0-100` |

ROS 2 和开发工具又自动安装了其依赖包，例如 `rclpy`、ament、rosidl、Fast DDS typesupport、CycloneDDS 0.10.5、Qt5 runtime 和 Python 构建工具。

第一次运行安装脚本时还将以下两个包从 `0.99.49.1` 更新到 `0.99.49.4`：

- `software-properties-common`
- `python3-software-properties`

## 4. Unitree 和仿真依赖

### 4.1 下载的官方仓库

三个仓库位于项目目录内，但被项目根目录 `.gitignore` 排除，不进入主项目版本控制。

| 目录 | Commit |
|---|---|
| `unitree_sdk2/` | `c753829882fba461ed07ba25aaabee0a25d83663` |
| `unitree_mujoco/` | `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d` |
| `unitree_ros2/` | `668d1ec5a05d1c38d3306bdca7d59f2ba3581a88` |

### 4.2 Unitree SDK2

使用以下配置构建并安装：

```text
BUILD_EXAMPLES=OFF
CMAKE_BUILD_TYPE=Release
CMAKE_INSTALL_PREFIX=/opt/unitree_robotics
```

主要安装结果：

```text
/opt/unitree_robotics/include/
/opt/unitree_robotics/lib/libunitree_sdk2.a
/opt/unitree_robotics/lib/libddsc.so*
/opt/unitree_robotics/lib/libddscxx.so*
```

安装目录占用约 44 MB。

### 4.3 MuJoCo 3.3.6

官方 Linux x86_64 包安装到：

```text
~/.mujoco/mujoco-3.3.6/
```

Unitree 仿真器使用以下符号链接：

```text
unitree_mujoco/simulate/mujoco
  -> ~/.mujoco/mujoco-3.3.6
```

下载包 SHA256：

```text
049204172901afad251070385a6badf46d795ebe47403d093f8469557eeeab5a
```

安装后的 `libmujoco.so.3.3.6` SHA256：

```text
b9173509d0c282a9b24b7f5825a40177a9967df0cd6395a9dc39522196e44495
```

### 4.4 ONNX Runtime 1.22.0

项目原有目录包含头文件，但缺少 CMake 指定的动态库。本次从 Microsoft 官方 1.22.0 Linux x64 release 补入：

```text
unitree_g1_ws2/deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib/
├── libonnxruntime.so
├── libonnxruntime.so.1
├── libonnxruntime.so.1.22.0
├── libonnxruntime_providers_shared.so
└── cmake/onnxruntime/...
```

下载包 SHA256：

```text
8344d55f93d5bc5021ce342db50f62079daf39aaafb5d311a451846228be49b3
```

`libonnxruntime.so.1.22.0` SHA256：

```text
3da6146e14e7b8aaec625dde11d6114c7457c87a5f93d744897da8781e35c673
```

这些 `.so` 文件受项目已有 `*.so*` ignore 规则管理，没有造成原 Git 跟踪文件变化。

## 5. 生成的构建产物

### 5.1 Unitree MuJoCo

```text
unitree_mujoco/simulate/build/unitree_mujoco
```

核心 `unitree_mujoco` target 构建成功，动态链接解析正常，使用：

- MuJoCo 3.3.6
- Ubuntu Boost 1.83
- Ubuntu yaml-cpp 0.8
- `/opt/unitree_robotics` 中的 CycloneDDS

完整默认构建还会编译附带的 `jstest`。该测试源码缺少 `<cstdint>`，因此 `jstest` 编译失败；它不影响 `unitree_mujoco` 主 target。为了不修改上游源码，本次只显式构建了主 target。

### 5.2 Unitree ROS 2 消息工作区

工作区：

```text
unitree_ros2/cyclonedds_ws/
```

成功构建：

- `unitree_api`
- `unitree_go`
- `unitree_hg`

构建产物位于该工作区的 `build/`、`install/`、`log/`。

### 5.3 项目 ROS 2 工作区

工作区：

```text
unitree_g1_ws/
```

成功构建：

- `g1_gui`
- `g1_bridge`

以下导入验证通过：

```text
rclpy
unitree_hg.msg
g1_gui.launcher
g1_bridge.merger
```

`g1_bridge` 注册的 `arm_ctrl_json` 命令无法执行，因为仓库中没有 `g1_bridge/arm_ctrl_json.py`。这是源码内容缺失，不是系统环境缺依赖。

### 5.4 G1 策略控制器

成功生成：

```text
unitree_g1_ws2/deploy/robots/g1_23dof/build/g1_ctrl
```

验证结果：

- `g1_ctrl --help` 正常。
- `g1_ctrl --version` 输出 `1.0.0.1`。
- SDK2、fmt 和 ONNX Runtime 动态库全部能解析。
- Velocity、Mimic 和 WBC 三个状态源码均成功编译。

## 6. 运行验证

### 6.1 MuJoCo 图形和模型

在 WSLg 中短暂启动了：

```bash
./unitree_mujoco -i 0 -n eth0 -r g1 -s scene_23dof.xml -t 1
```

验证通过：

- WSLg 图形窗口可启动。
- MuJoCo 版本为 3.3.6。
- G1 `scene_23dof.xml` 加载成功。
- 使用 `unitree_hg` DDS IDL。
- 29 个 actuator 以及 IMU、关节位置、速度和扭矩 sensor 正常枚举。

### 6.2 MuJoCo 与 g1_ctrl DDS 联调

第一次按项目 README 使用 `lo`。WSL 的 `lo` 标志只有 `LOOPBACK,UP,LOWER_UP`，没有 `MULTICAST`，CycloneDDS 输出：

```text
selected interface "lo" is not multicast-capable: disabling multicast
```

因此两个进程没有自动发现，控制器停在等待 `rt/lowstate`。

改用具备 `BROADCAST,MULTICAST` 的 WSL `eth0` 后联调成功：

```bash
./unitree_mujoco -i 0 -n eth0 -r g1 -s scene_23dof.xml -t 1
./g1_ctrl --network eth0
```

控制器验证到：

- 约 0.2 秒内连接 LowState。
- `mode_machine=4` 与 23DoF 仿真匹配。
- Velocity ONNX 加载成功，大小约 1.40 MB。
- Dance CSV 加载成功，识别时长 29.15 秒。
- Dance ONNX session 创建成功。
- WBC 轨迹加载成功，共 6574 帧。
- WBC ONNX 输入维数 439、动作维数 29。
- FSM 成功从 `Passive` 启动。

测试只保持 `Passive`，没有触发 FixStand、Velocity、Mimic 或 WBC，也没有连接实机。

## 7. 每个终端建议的环境初始化

当前默认 shell 会自动进入 Conda `base`。ROS 2 Jazzy 使用 Ubuntu 系统 Python 3.12；构建和运行 ROS 节点时建议先退出 Conda，避免 Python/CMake 优先找到 Conda 中的库。

```bash
conda deactivate

source /opt/ros/jazzy/setup.bash
source ~/projects/unitree_g1/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/projects/unitree_g1/unitree_g1_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:${LD_LIBRARY_PATH:-}
```

本次没有修改 `~/.bashrc`、Conda 配置或项目的 `setup.sh`。因此新终端中必须手动 source；等后续开始本机适配时，再统一项目脚本中的路径、ROS 版本和 WSL 网络接口。

未 source `/opt/ros/jazzy/setup.bash` 时直接执行 `/opt/ros/jazzy/bin/ros2` 可能找不到 Python package metadata，这是 ROS overlay 尚未加入环境导致的正常现象。

## 8. 当前建议的 WSL 仿真启动参数

终端 1：

```bash
cd ~/projects/unitree_g1/unitree_mujoco/simulate/build
./unitree_mujoco -i 0 -n eth0 -r g1 -s scene_23dof.xml -t 1
```

终端 2：

```bash
cd ~/projects/unitree_g1/unitree_g1_ws2/deploy/robots/g1_23dof/build
./g1_ctrl --network eth0
```

这里显式使用 `Domain 0 + eth0 + G1 + scene_23dof.xml + unitree_hg`，避免依赖当前上游 MuJoCo 默认配置（默认仍是 Go2、Domain 1）。

## 9. 占用空间

审阅时的近似值：

| 路径 | 占用 |
|---|---:|
| `/opt/ros/jazzy` | 125 MB |
| `/opt/unitree_robotics` | 44 MB |
| `~/.mujoco/mujoco-3.3.6` | 24 MB |
| `unitree_sdk2/` | 182 MB |
| `unitree_mujoco/`（含 build） | 427 MB |
| `unitree_ros2/`（含 build/install/log） | 52 MB |

apt 自动依赖、项目 ROS build 目录和系统缓存还会占用额外空间。

## 10. 2026-09-18 当次未做的修改

- 没有修改项目控制器、FSM、GUI 或配置源码。
- 没有把任何 ROS source 命令写入 `~/.bashrc`。
- 没有移动策略 ONNX 或动作数据。
- 没有运行 `apt autoremove`。
- 没有执行实机连接或主动关节控制。
- 没有修复 `jstest`、`arm_ctrl_json`、旧路径或项目脚本；这些留到后续本机适配阶段处理。

## 11. 参考来源

- [ROS 2 Jazzy Ubuntu 安装说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2)
- [Unitree MuJoCo](https://github.com/unitreerobotics/unitree_mujoco)
- [Unitree ROS 2](https://github.com/unitreerobotics/unitree_ros2)
- [MuJoCo releases](https://github.com/google-deepmind/mujoco/releases/tag/3.3.6)
- [ONNX Runtime releases](https://github.com/microsoft/onnxruntime/releases/tag/v1.22.0)
