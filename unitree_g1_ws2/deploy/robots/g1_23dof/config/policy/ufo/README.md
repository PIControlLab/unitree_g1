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

- Tracking：walk、run、jump、dance、fall-and-get-up；
- Goal reaching：站立目标，以及上述五段参考动作的静态目标帧；
- Reward inference：forward 0.3/0.7/1.0 m/s（不再包含 stand）。

五个 tracking 片段来自 Windows 交付包中的代表动作：dance 70–80 s、get-up 80–90 s、
jump 120–130 s、run 120–130 s、walk 130–140 s。

## z 预计算

`tools/prepare_latents.py` 实现与 UFO FB 一致的三条变换：

1. Tracking：`B(s)` 后对未来 `seq_length=8` 帧求均值，再投影到范数 `sqrt(256)=16`；
2. Goal reaching：对一个目标 observation 计算 `B(s_goal)` 并投影；
3. Reward inference：`softmax(10*r)` 加权回归
   `sum_i r_i * softmax(10r)_i * B(s_i)`，再投影。

`goal_reaching/stand_pose` 的目标 observation 直接由 `config/state/fix_stand.yaml`
中的目标关节姿态和 MuJoCo 静态正向运动学构造，不再从任何 tracking 轨迹抽帧。

重新生成全部运行时 z：

```bash
python unitree_g1_ws2/deploy/robots/g1_23dof/config/policy/ufo/tools/prepare_latents.py \
  --encoder unitree_g1_ws2/deploy/robots/g1_23dof/config/policy/ufo/model/backward_encoder.onnx \
  --reference-dir unitree_g1_ws2/deploy/robots/g1_23dof/config/policy/ufo/reference \
  --output-dir unitree_g1_ws2/deploy/robots/g1_23dof/config/policy/ufo/latents \
  --fix-stand-config unitree_g1_ws2/deploy/robots/g1_23dof/config/state/fix_stand.yaml \
  --xml unitree_mujoco/unitree_robots/g1/g1_23dof.xml
```

官方高质量 reward inference 需要训练 replay buffer。当前交付包明确没有附带约 22 GB 的
replay buffer，因此仓库内的三个 reward z 使用随包五段安全 motion reference 作为 sample
corpus，并在 NPZ 中记录 sample count 和 reward 范围。拿到与该 checkpoint 匹配的 reduced
replay buffer 后，应先导出含 `z` 以及各 `reward_<task-id>` 数组的安全 NPZ，再通过
`--reward-samples` 重新生成；不要混用其他 checkpoint 的 replay buffer 或 z。

## 模型契约

- policy input：`actor_obs[631] = state[52] + last_action[23] + history[300] + z[256]`
- policy output：23 维 action
- policy/control frequency：50 Hz
- z norm：16
- policy ONNX 分片 SHA-256（拼接后）：`d25b78ec8b224164c971fe60a27b6ee1263dd3a71ba2cae052bbc5b624909015`
- backward encoder SHA-256：`0ec233b7bffb90df761fc3ef40c5d5b6b1b09a4268bf02cc3a5403c2dac8b2cc`

这些策略均为动态全身控制。应先在 MuJoCo 中保留吊带、小范围验证，再上实机。