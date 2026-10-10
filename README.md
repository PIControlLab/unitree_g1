# Unitree G1 23DoF

G1 23DoF 的策略部署与控制项目。控制器源码编译后生成可执行文件 `g1_ctrl`，
GUI 负责启动该程序并发送操作指令；`g1_ctrl` 通过 Unitree DDS 连接 MuJoCo 或 G1 真机。

## 目录

| 路径 | 内容 |
|---|---|
| `unitree_g1_ws2/deploy/` | 控制器源码、FSM、Velocity、Mimic、WBC、UFO 策略与模型 |
| `unitree_g1_ws/src/g1_gui/` | PyQt5 GUI 控制台 |
| `unitree_mujoco/` | [宇树官方 MuJoCo 仿真器](https://github.com/unitreerobotics/unitree_mujoco) |
| `unitree_ros2/` | [宇树官方 ROS 2 消息与 CycloneDDS 工作空间](https://github.com/unitreerobotics/unitree_ros2) |
| `unitree_sdk2/` | [宇树官方 Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2) |
| `setup.sh` | x86_64 Sim-to-sim 环境 |
| `unitree_g1_ws/setup_real.sh` | aarch64 Sim-to-real 环境 |
| `start_gui.sh` | x86_64 仿真 GUI 启动脚本 |

运行环境分为两套：

| 模式 | 平台 | 控制对象 | GUI 位置 |
|---|---|---|---|
| Sim-to-sim | x86_64 开发机 | MuJoCo G1 | x86_64 开发机 |
| Sim-to-real | aarch64 机载电脑 | G1 真机 | aarch64 机载电脑，远程桌面操作 |

两套环境分别构建，使用与目标平台匹配的依赖和二进制。

## 编译

以下命令均从项目根目录执行。先加载 ROS 2 和 Unitree ROS 2 环境：

```bash
source /opt/ros/jazzy/setup.bash
source unitree_ros2/cyclonedds_ws/install/setup.bash
```

### 编译控制器

```bash
cmake -S unitree_g1_ws2/deploy/robots/g1_23dof \
  -B unitree_g1_ws2/deploy/robots/g1_23dof/build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build unitree_g1_ws2/deploy/robots/g1_23dof/build --parallel 2
```

编译生成 `unitree_g1_ws2/deploy/robots/g1_23dof/build/g1_ctrl`，下文简称
`g1_ctrl`。

### 编译 GUI

```bash
cd unitree_g1_ws
./compile.sh
cd ..
```

### MuJoCo（仅 x86_64 Sim-to-sim 需要）

```bash
cmake -S unitree_mujoco/simulate \
  -B unitree_mujoco/simulate/build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build unitree_mujoco/simulate/build --parallel 2
```

Sim-to-sim 在 x86_64 开发机编译。Sim-to-real 在 aarch64 机载电脑编译，
并使用 aarch64 版本的 Unitree SDK、ROS 2 和 ONNX Runtime。

## Sim-to-sim：x86_64 + MuJoCo

终端 A 启动 MuJoCo：

```bash
cd <项目根目录>
source ./setup.sh
cd unitree_mujoco/simulate/build
./unitree_mujoco -i 0 -r g1 -s scene_23dof.xml -t 1
```

终端 B 启动 GUI：

```bash
cd <项目根目录>
./start_gui.sh
```

GUI 打开后：

1. MuJoCo 吊带开启，机器人悬空。
2. 点击“G1 控制器 (g1_ctrl)”中的“启动”，等待状态显示 Passive。
3. 点击 FixStand，等待 3 秒至机器人关节回正。注意此状态没有平衡保持能力。
4. 按键盘上的 `↓`，在 MuJoCo 中逐步放低机器人，使双脚接地。
5. 进入 Velocity 或者 UFO 状态，使机器人具有平衡保持能力。
6. 按 `9` 关闭吊带，再测试其他状态功能。

## Sim-to-real：aarch64 + G1 真机

通过远程桌面进入 G1 机载 aarch64 电脑，在机载终端运行：

```bash
cd <项目根目录>
source unitree_g1_ws/setup_real.sh eth0
ros2 run g1_gui launcher
```

GUI 打开后：

1. 点击“G1 控制器 (g1_ctrl)”中的“启动”，等待状态显示 Passive。
2. 进入 FixStand，确认机器人稳定。
3. 进入 Velocity/UFO 保持平衡，再测试其他状态。结束时，应确定机器人有吊带或其他支撑后，进入 Passive 模式释放控制器。
4. 关闭 GUI；GUI 会停止由它启动的 `g1_ctrl`。

## GUI 操作

| 当前状态 | 可以进入 |
|---|---|
| Passive | FixStand、UFO |
| FixStand | Passive、Velocity、UFO |
| Velocity | FixStand、WBC、Dance、UFO |
| WBC、Dance | Velocity |
| UFO | Passive、FixStand、Velocity |

- Velocity：左侧摇杆控制平移，旋转滑块控制转向；右侧方向按钮发送约两秒的动作。
- UFO：右侧选择 Tracking、Goal reaching 或 Reward inference 任务。


GUI 与控制器通过本机共享内存通信：

```text
GUI ──> /dev/shm/unitree_joystick_bridge.json ──> g1_ctrl
GUI ──> /dev/shm/unitree_ufo_selection.json  ──> g1_ctrl
GUI <── /dev/shm/unitree_fsm_state.json      <── g1_ctrl
```

## 可选：UDP 触发

GUI 启动后监听 UDP `0.0.0.0:9876`。另一台电脑与 GUI 主机网络互通时，可运行：

```bash
cd unitree_g1_ws2/deploy
python3 udp_send.py <运行GUI的电脑IP> 9876
```

| 数字 | 动作 | 生效状态 |
|---|---|---|
| 1–6 | 前进、后退、左右移、左右转弯，持续约两秒 | Velocity |
| 7 | 进入 WBC | Velocity |
| 8 | 进入 Dance | Velocity |
| 9 | 进入 UFO | Passive、FixStand 或 Velocity |

UDP 不启动 `g1_ctrl`，也不返回 FSM 状态。仅在可信网络中使用。

## 补充说明：MuJoCo 本地配置

以下配置位于宇树官方 `unitree_mujoco` 代码中，本项目为 WSL 仿真做了本地调整。

### DDS 网卡

WSL 使用 `eth0`，当前默认配置无需修改：

- `unitree_mujoco/simulate/config.yaml`：`interface: "eth0"`
- `setup.sh`：`NetworkInterface name="eth0"`
- `unitree_g1_ws2/deploy/include/param.h`：`default_value("eth0")`

原生 Linux 本机仿真使用 `lo`，将以上三个参数分别改为：

- `unitree_mujoco/simulate/config.yaml`：`interface: "lo"`
- `setup.sh`：`NetworkInterface name="lo"`
- `unitree_g1_ws2/deploy/include/param.h`：`default_value("lo")`

修改 `param.h` 后需要重新编译 `g1_ctrl`。

### 虚拟吊带

虚拟吊带同样在 `unitree_mujoco/simulate/config.yaml` 中配置：

```yaml
enable_elastic_band: 1
```

`1` 表示启用，`0` 表示禁用；修改后重启 MuJoCo 即可，无需重新编译。启用后，
按 `9` 连接或释放吊带，按 `7`/`↑` 吊高机器人，按 `8`/`↓` 放低机器人。
