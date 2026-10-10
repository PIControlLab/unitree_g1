# UFO-FB G1 23DoF deployment

UFO 只有一个运行时 FSM state。Tracking、Goal reaching 和 Reward inference 共用同一
UFO policy；区别是送入 631 维 actor observation 末尾的 256 维预计算 z。

## 运行所需文件

- `deploy.yaml`：控制、观测和模型配置。
- `model/FBcprAuxModel.onnx.part00` 与 `part01`：同一 policy ONNX 的两个连续分片，由控制器按顺序拼接加载。
- `latent_catalog.json`：GUI 和控制器共用的任务列表。
- `latents/*.npz`：29 个现成 z 文件，全部由 catalog 引用。其中 tracking 有 5 条逐帧序列（各 501 帧），goal reaching 有 15 个固定 z，reward inference 有 9 个固定 z。

运行时仅加载 policy ONNX 和 catalog 中的 z，不运行 backward encoder，也不生成或推断 z。此版本不包含离线编码器、原始参考动作、replay buffer 或 z 生成脚本。需要重新生成 z 时，请使用独立的训练或数据处理项目；不要覆盖这里已验证的部署文件。

## 选择与播放

GUI 从 `latent_catalog.json` 显示三组任务按钮，并把选中的条目 id 原子写入
`/dev/shm/unitree_ufo_selection.json`。进入 UFO state 时，控制器先加载
`goal_reaching/stand_pose`；进入后点击任务按钮才切换 z。

Tracking 按控制周期逐帧读取 z，结束后回到站立目标。Goal reaching 和 Reward inference 持续使用各自的固定 z，直到选中别的任务。UFO 运行中也可以切换任务。

可以用 LT+B 退出到 Passive、LT+Up 退出到 FixStand，或 RB+X 退出到 Velocity。Passive 和 FixStand 可用 LT+Y 直接进入 UFO。

## 模型契约

- policy input：`actor_obs[631] = state[52] + last_action[23] + history[300] + z[256]`
- policy output：23 维 action
- policy/control frequency：50 Hz
- z：float32、形状 `[N, 256]`，每帧 L2 范数约 16
- 拼接后的 policy ONNX SHA-256：`d25b78ec8b224164c971fe60a27b6ee1263dd3a71ba2cae052bbc5b624909015`

这些策略均为动态全身控制。先在 MuJoCo 中保留吊带，逐项验证，再考虑实机测试。
