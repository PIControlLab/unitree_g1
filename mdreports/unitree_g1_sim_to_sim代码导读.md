# unitree_g1 当前代码导读与 sim-to-sim 核对

> 原报告写于 2026-09-18；本版按 2026-09-24 的源码和配置更新。
> 环境安装与历史 Passive 联调记录见同目录的安装报告。
> 当前控制器已完成编译；本文不把编译结果当作 MuJoCo 动作或实机验收。

## 1. 当前结构

| 部分 | 当前实现 |
|---|---|
| FSM | `Passive`、`FixStand`、`Velocity`、`Mimic_Dance_102`、`WBC`、`UFO` 六个状态 |
| 策略运行层 | 四个策略状态共用 `State_PolicyBase`；YAML schema 2 定义观测、历史、动作和模型 |
| 参考数据 | Mimic 用 `MotionReference`，WBC 用 `ReferenceData`，UFO 用 `UfoLatentReference` 读取 catalog 与预计算 z NPZ |
| GUI | `g1_gui/launcher` 管理 `g1_ctrl`，读取真实 FSM 状态，显示 UFO 任务按钮，并监听 UDP 1–9 |
| 通信 | `g1_ctrl` 直接通过 Unitree DDS 收发 LowState/LowCmd；当前路径不经过 `g1_bridge/g1_merger` |

```text
GUI / UDP → /dev/shm/unitree_joystick_bridge.json
          → UnitreeJoystickBridge → g1_ctrl / CtrlFSM
          → Unitree SDK2 DDS LowCmd → MuJoCo 或机器人
g1_ctrl 状态切换 → /dev/shm/unitree_fsm_state.json → GUI
GUI UFO 任务按钮 → /dev/shm/unitree_ufo_selection.json → UfoLatentReference
```

GUI 的速度摇杆每 40 ms 更新共享文件。策略控制周期由各自的
`policy_dt` 配置；UFO 为 0.02 s。当前 `main.cpp` 没有启用
`PolicyPublisher`。部署入口是
`unitree_g1_ws2/deploy/robots/g1_23dof/main.cpp`，
FSM 配置位于同目录的 `config/config.yaml`。

## 2. FSM 转换

以 `config/config.yaml` 为准：

```text
Passive  -- LT+Up --> FixStand
Passive  -- LT+Y  --> UFO
FixStand -- LT+B  --> Passive
FixStand -- RB+X  --> Velocity
FixStand -- LT+Y  --> UFO
Velocity -- LT+Up   --> FixStand
Velocity -- LT+Down --> Mimic_Dance_102
Velocity -- RB+Up   --> WBC
Velocity -- LT+Y    --> UFO
Mimic_Dance_102 -- RB+X 或片段结束 --> Velocity
WBC             -- RB+X 或片段结束 --> Velocity
UFO -- LT+B  --> Passive
UFO -- LT+Up --> FixStand
UFO -- RB+X  --> Velocity
```

UFO 的 Tracking 播放结束后切换到
`goal_reaching/stand_pose` 并持续保持，**不会自动返回 Velocity**。
GUI 根据控制器报告的状态启用可到达按钮。UDP 1–6 是 Velocity 速度指令，
7、8、9 分别请求 WBC、Dance、UFO，仍受当前 FSM 状态约束。

## 3. 四类策略

| 状态 | 模型与运行时参考 | 行为 |
|---|---|---|
| Velocity | `unitree_g1_23dof_velocity/exported/policy.onnx` 与摇杆速度 | 23 关节速度策略 |
| Mimic_Dance_102 | `mimic/dance_102/exported/policy.onnx` 与 reference NPZ | 舞蹈片段结束后回 Velocity |
| WBC | `wbc/model.onnx` 与 reference NPZ | 29 维动作布局，参考片段结束后回 Velocity |
| UFO | `FBcprAuxModel.onnx.part00/part01` 与 latent catalog/NPZ | 23 维动作；任务只切换 256 维 z |

四个策略状态共用 `State_PolicyBase` 的 ONNX runner 和控制流程；
具体关节顺序、历史长度、力矩限制与保护开关以各自 `deploy.yaml`
及 `config/robot/g1_23dof.yaml` 为准，不能共用一份关节索引表。

UFO 的 actor observation 为 631 维：状态 52、上一动作 23、
历史 300、z 256。当前 catalog 有 **5 个 Tracking、15 个 Goal reaching、
9 个 Reward inference** 条目，全部指向预计算 NPZ。
Tracking 为逐帧序列，其他两组为固定 z。运行时不调用 backward encoder，
也不推理新的 z；编码器、原始 UFO 参考动作与生成脚本已从部署分支移除。
缺少 catalog 或 NPZ 会导致加载失败；选择文件中的无效 ID 会回退到
catalog 的 `default_id`（目前为 `tracking/walk`）。
详细文件和模型契约见
`unitree_g1_ws2/deploy/robots/g1_23dof/config/policy/ufo/README.md`。

## 4. 构建与验证边界

本机有 `unitree_mujoco/`、`unitree_sdk2/`、`unitree_ros2/`、
ONNX Runtime 1.22.0 动态库和 `build/g1_ctrl`。
2026-09-24 已成功编译 `g1_ctrl`，并校验 UFO 的 29 个 z 文件
均被 catalog 引用、形状和数值有效、policy 分片拼接校验值正确。
这些检查不等于已经完成 MuJoCo 动作测试。

仿真启动与操作顺序见项目根目录 `BEGIN.md`。使用 Domain 0、
WSL `eth0`、G1 23DoF 场景和 `unitree_hg` IDL。
GUI 不启动 MuJoCo；同一时间只启动一个 `g1_ctrl`。
安装报告中的 2026-09-18 Passive 短时 DDS 联调是历史结果，
不能代替当前策略的动作验证。

## 5. 后续验证重点

1. 在 MuJoCo 中逐一检查六个状态、可达转换、UFO 任务热切换和 Tracking 结束后的站立保持。
2. 检查输入失联、GUI 关闭、模型异常和策略保护时的控制器行为。
3. 真机前验证关节映射、控制时序、接触与限幅，并确保只有一个 LowCmd 发布者。
4. 新环境需重新准备受 `.gitignore` 排除的官方仓库与 ONNX Runtime 动态库。
