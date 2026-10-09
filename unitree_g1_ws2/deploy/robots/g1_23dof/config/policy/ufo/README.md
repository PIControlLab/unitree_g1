# UFO-FB G1 23DoF deployment

UFO 只有一个 FSM state。Tracking、Goal reaching 和 Reward inference 的差别只在
631 维 actor observation 最后的 256 维 `z`；它们共用同一个 policy、历史 observation、
动作变换和安全逻辑。

## 运行时选择

GUI 会读取 `latent_catalog.json`。进入 UFO state 后，右侧直接显示三组任务按钮；点击按钮会将
条目 id 原子写入：

```text
/dev/shm/unitree_ufo_selection.json
```

每次进入 `UFO` state 时，C++ 都先使用 `goal_reaching/stand_pose` 的 z，并将此前
选择文件中的任务视为已处理；因此退出 UFO 后再次进入也从站立目标开始。进入 UFO 后点击任务
按钮才会读取新选择并切换 z，Tracking 播放过程中也可立即切换。Tracking 条目逐帧读取 z，
播放结束后回到站立目标；Goal reaching 与 Reward inference 条目持续使用固定 z，直到另一
任务将其打断。三种功能始终共用一个 UFO FSM state。

手动退出 UFO 使用 LT+B 到 Passive、
LT+Up 到 FixStand，或 RB+X 到 Velocity。Passive 和 FixStand 均可使用
LT+Y 直接进入 UFO。


当前 catalog 包含：

- Tracking：walk、run、jump、dance、fall-and-get-up，原有逐帧 z 保持不变；
- Goal reaching：FixStand 推导的站立目标，以及 14 个来自 23DoF motion cache 的有效单帧姿态；
- Reward inference：9 个从约 22 GB replay buffer 推断的 locomotion/root 任务，包括静止、低姿态、前后左右移动和双向旋转。

五个 tracking 片段来自 Windows 交付包中的代表动作：dance 70–80 s、get-up 80–90 s、
jump 120–130 s、run 120–130 s、walk 130–140 s。站立目标的 observation 由
`config/state/fix_stand.yaml` 的目标关节姿态和 MuJoCo 静态正向运动学构造。
交付包没有这个 FixStand goal，因此保留原有的 `goal_reaching/stand_pose`；
`reward_inference` 内的静止任务是独立的 replay reward latent，不会取代该 goal。

## 服务器 latent 来源与导入

`source/` 保存服务器交付的安全 NPZ、manifest 和 goal 定义，不包含 22 GB replay
buffer、pickle 或重复的 126 MB ONNX。交付 checkpoint 为 102,498,304 environment steps；
每个 reward latent 基于 150,000 条采样 transition，目标帧速度设为 0。
`tools/install_replay_latents.py` 校验 NPZ、目标定义和现有 ONNX 的 SHA-256，
并验证名字、shape、float32 有限值及 L2 范数 16，随后生成 C++ 加载器需要的
`z[1,256]` 单任务 NPZ 与 GUI/C++ 共用的 catalog：

```bash
python unitree_g1_ws2/deploy/robots/g1_23dof/config/policy/ufo/tools/install_replay_latents.py
```

`tools/prepare_latents.py` 保留作旧的参考动作 z 预计算工具，但不能用于重新生成当前
replay reward/goal 集合，也不要对部署目录运行它，以免覆盖 tracking 文件。
新增 goal 的名称来自原始 motion/frame，不保证实机一定能到达；请先逐项在 MuJoCo
验证收敛、接触和关节限位。

## 模型契约

- policy input：`actor_obs[631] = state[52] + last_action[23] + history[300] + z[256]`
- policy output：23 维 action
- policy/control frequency：50 Hz
- z norm：16
- policy ONNX 分片 SHA-256（拼接后）：`d25b78ec8b224164c971fe60a27b6ee1263dd3a71ba2cae052bbc5b624909015`
- backward encoder SHA-256：`0ec233b7bffb90df761fc3ef40c5d5b6b1b09a4268bf02cc3a5403c2dac8b2cc`

这些策略均为动态全身控制。应先在 MuJoCo 中保留吊带、小范围验证，再上实机。