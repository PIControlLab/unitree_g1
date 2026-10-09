# unitree_g1 当前代码导读与 sim-to-sim 核对

> 原报告日期：2026-09-18；本次按 2026-09-23 工作树复核。
> 范围：包含尚未提交的重构；最后一次提交不一定代表当前源码；本次已重新配置并编译当前 `g1_ctrl`。安装记录见同目录另一份报告。

## 1. 与原报告的关键差异

| 主题 | 当前工作树 |
|---|---|
| FSM | 六个状态：`Passive`、`FixStand`、`Velocity`、`Mimic_Dance_102`、`WBC`、`UFO`。 |
| 策略运行层 | 旧 `State_RLBase.h/.cpp` 已删除。四个策略状态使用 `State_PolicyBase`；YAML schema 2 描述观测、历史、动作、关节映射和模型。 |
| 参考数据 | Mimic 通过 `MotionReference` 读取 manifest/NPZ；WBC 通过 `ReferenceData` 读取 manifest/轨迹；UFO 通过 `UfoLatentReference` 读取 catalog/latent NPZ。旧报告中的“运行时直接读舞蹈 CSV / WBC BIN”已过时。 |
| GUI | 按文件位置定位项目根目录，读取真实 FSM 状态文件，提供 UFO 任务按钮；UDP 编号扩展到 1–9。 |
| 环境 | 根 `setup.sh` 使用相对脚本路径、ROS 2 Jazzy 和 WSL `eth0`。当前本机存在官方依赖目录、ONNX Runtime 动态库、CMake、colcon 及旧控制器构建产物。 |

## 2. 代码结构与通信链路

```text
GUI / UDP → /dev/shm/unitree_joystick_bridge.json
          → UnitreeJoystickBridge → g1_ctrl / CtrlFSM
          → Unitree SDK2 DDS LowCmd → unitree_mujoco 或机器人
g1_ctrl 状态切换 → /dev/shm/unitree_fsm_state.json → GUI
GUI UFO 任务按钮 → /dev/shm/unitree_ufo_selection.json → UFO latent
```

`CtrlFSM` 以 1 ms 周期刷新输入、检查转换、执行状态并发布 LowCmd；策略配置为 `policy_dt=0.02`（50 Hz）。GUI 虚拟摇杆每 40 ms 写入 JSON。主程序未启用 `PolicyPublisher`，旧 `g1_bridge/merger.py` 链路仍保留，但不应与当前 DDS 直连控制器争用 LowCmd。`main.cpp` 顶部“交由 Python merger 发布”的英文注释已经过时，应以实际调用为准。

部署重点文件：`unitree_g1_ws2/deploy/include/FSM/State_PolicyBase.h`、`robots/g1_23dof/src/State_PolicyBase.cpp`、`config/config.yaml`、`config/robot/g1_23dof.yaml`、各 `config/policy/*/deploy.yaml`、`include/policy/` 和 `unitree_g1_ws/src/g1_gui/g1_gui/launcher.py`。

## 3. 当前 FSM 转换

以 `config/config.yaml` 为准：

```text
Passive  -- LT+Up --> FixStand
Passive  -- LT+Y  --> UFO
FixStand -- LT+B  --> Passive
FixStand -- RB+X  --> Velocity
FixStand -- LT+Y  --> UFO
Velocity -- RB+Down --> FixStand
Velocity -- LT+Down --> Mimic_Dance_102
Velocity -- RB+Up   --> WBC
Velocity -- LT+Y    --> UFO
Mimic_Dance_102 -- RB+X 或片段结束 --> Velocity
WBC             -- RB+X 或片段结束 --> Velocity
UFO -- LT+B  --> Passive
UFO -- LT+Up --> FixStand
UFO -- RB+X  --> Velocity
```

UFO 的 tracking 片段结束后切换到 `goal_reaching/stand_pose` latent 并持续保持，**不会自动返回 Velocity**。根 README 和 `BEGIN.md` 中“10 秒后回 Velocity”“必须从 Velocity 进入 UFO”等文字与当前源码不符。GUI 按钮按可达关系启用；UFO 页面可在保持同一 FSM 状态时切换任务。UDP 1–6 是速度动作、7 是 WBC、8 是 Dance、9 是 UFO，触发仍受 GUI 当前状态约束。

## 4. 四类策略

| 状态 | 模型与参考 | 当前契约 |
|---|---|---|
| `Velocity` | `unitree_g1_23dof_velocity/exported/policy.onnx`；摇杆速度 | 23 关节，5 帧历史；进入时先运行一步；倾角保护启用。 |
| `Mimic_Dance_102` | `mimic/dance_102/exported/policy.onnx`；`reference/reference.yaml` 指向 NPZ | 23 关节；0–28 s 片段；结束回 Velocity；倾角保护启用。 |
| `WBC` | `wbc/model.onnx`；`reference/reference.yaml` | 29 维策略布局，4 帧历史，参考帧 100–300；参考重力差超过 0.9 转 Passive。 |
| `UFO` | 分片 `FBcprAuxModel.onnx.part00/part01`；latent catalog/NPZ | 23 关节；输入 631 维、动作 23 维；进入时 1 s 位置混合；tracking 结束保持站立目标。 |

四类状态共用 `State_PolicyBase` 的 ONNX runner、策略线程和动作写回。公共逻辑支持推理失败转 Passive、位置交接混合和配置指定的等效位置力矩限制；实际开关应逐个查看 YAML。当前 WBC/UFO 的 `bad_orientation.enabled` 为 `false`，UFO 的 `effort_limit_mode` 为 `equivalent_position`，另三类为 `none`。启用的倾角检查已对 `acos` 输入做 `[-1,1]` 限制。

UFO 的 tracking、goal reaching、reward inference 仅改变 256 维 latent `z`，共用一个 FSM 状态和同一 policy。`actor_obs[631]` 包含状态 52、上一动作 23、历史 300、`z` 256。当前 catalog 有 5 个 tracking、6 个 goal reaching、3 个 reward inference；文件缺失或 ID 无效时回退到 `tracking/walk`。三个 reward latent 用交付包中的安全动作片段生成，仓库没有对应的约 22 GB replay buffer。详见 `config/policy/ufo/README.md`。

不同策略的关节顺序并不相同，应以各自 `deploy.yaml` 的 `joint_names` 和 `config/robot/g1_23dof.yaml` 为准，不能沿用旧报告中的单一 `joint_ids_map`。

## 5. 构建与验证边界

当前本机可见 `unitree_mujoco/`、`unitree_sdk2/`、`unitree_ros2/`、ONNX Runtime 1.22.0 动态库和 `build/g1_ctrl`；系统能找到 CMake 和 colcon。原报告“当前缺少这些依赖，无法构建”的结论仅适用于安装前。当前策略重构修改了 C++ 源码，本次已重新配置 CMake 并完成 `g1_ctrl` 编译与链接；仍需在 MuJoCo 中验证六个状态。

仿真顺序参见 `BEGIN.md`：MuJoCo 使用 Domain 0、`eth0`、`g1`、`scene_23dof.xml`、`unitree_hg`；先启动 MuJoCo，再启动一个 `g1_ctrl`。GUI 不负责启动 MuJoCo。本机历史测试发现 `lo` 缺少组播能力，DDS 自动发现未成功；`eth0` 曾完成旧代码 Passive 联调。那次结果不能代替当前工作树的运动测试。

## 6. 仍需核对的风险

1. `UnitreeJoystickBridge` 复用最后一次 JSON payload，没有按文件年龄将速度归零；输入失联与 GUI 退出时的行为需验证。
2. `/dev/shm/unitree_fsm_state.json` 仅在 FSM 启动和切换时写入；文件存在不表示控制器仍存活。
3. `g1_bridge/setup.py` 注册的 `arm_ctrl_json` 对应文件仍缺失；`arm_ctrl.cpp` 仍未由当前 CMake 建成独立程序。
4. 官方外部仓库和 ONNX Runtime 动态库受忽略规则管理，另一个 clone 仍须单独准备。
5. 真机前需重新验证关节映射、1 kHz/50 Hz 时序、UFO 热切换和片段结束、异常保护、唯一 LowCmd 发布者及实际限幅。

本文说明当前源码；安装报告中的短时 Passive 验证是历史记录，不是本次重构的验收结论。
