# UFO-FB G1 23DoF deployment

本目录从 `ufo_fb_g1_23dof_sim2sim` 交付包迁移运行时所需资产。控制器按原始
contract 构造 631 维 observation，在 50 Hz 下运行 `FBcprAuxModel`，并使用
`backward_encoder` 从参考动作的 52+358 维特征预计算 256 维任务 latent。

默认参考是 `walk1_subject1.npz` 的 130–140 秒片段，与交付包本地 viewer
验证采用同一动作区间。`.uforef` 是不执行 pickle 的小端 float32 格式，可用：

```bash
python tools/prepare_reference.py \
  --reference reference/walk1_subject1.npz \
  --xml /path/to/ufo_fb_g1_23dof_sim2sim/robot/g1_23dof.xml \
  --start 130 --duration 10 \
  --output reference/walk1_subject1_130_140.uforef
```

源资产 SHA-256：

- `FBcprAuxModel.onnx`（两个 part 顺序拼接）：`d25b78ec8b224164c971fe60a27b6ee1263dd3a71ba2cae052bbc5b624909015`
- `backward_encoder.onnx`：`0ec233b7bffb90df761fc3ef40c5d5b6b1b09a4268bf02cc3a5403c2dac8b2cc`
- `walk1_subject1.npz`：`5b3ba2b7bdb68380223c53311fc2f99ffc326bb0fdb6df1e2db46449b324c8bd`

策略会执行 1 秒进入插值、训练契约中的逐关节 PD/力矩限幅、倾倒保护以及
推理异常回 Passive。它仍属于动态全身策略，必须先完成仿真验收。
