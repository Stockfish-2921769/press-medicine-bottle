# RL 学习 round 总结：PPO 真学按压微技能（两段课程 + 双几何 + 零样本）

> 报告日期：2026-09-09（训练/评测跑于 2026-09-08）
> 前置：混形双手 palm-down demo（`feasibility_mixed_press.md` §3/§6/§9）全链路 `USABLE`。本 round 把那条**纯规则脚本**（IK hover → 直线下压 → 触底判据）换成 **PPO 真学**的按压策略。
> 深入细节/中间诊断见 `feasibility_mixed_press.md` §10；本文件是自包含的**当前 RL 实验总结**（MDP 表 + 方法 + 结果 + 边界 + 复现）。

## 1. 一句话总结

**Stage A 两几何（s=1.0 与 s=0.7+垫座）都训到 100% 成功，且跨几何零样本双向 100%**（同一策略 s1↔s07 直接换任务、零训练）；**Stage B 的 reset-DR 课程把 s1 在加宽复位分布下从 93.3% 补到 100%**。诚实边界：宽 DR 初版不收敛（记录 + 缩 DR 后成立），s07 上 Stage A 本身已对缩后 DR 全鲁棒、Stage B 无增益（收益几何相关）。

## 2. MDP 一览表

场景 = `PressEnv(DirectRLEnv)`，每 env 一个独立副本：**Panda 长爪（西侧）钳住自由站异形瓶**（Ø42、弹簧喷嘴 K300/行程 5 mm，唯一 DoF=nozzle），**iiwa7+Shadow 掌心(pad)朝下压 cap 顶**。数值均以 `direct/press/press_env_cfg.py` / `press_env.py` 为准。

| 维度 | 值 | 说明 |
|---|---|---|
| 任务语义 | 学到「**压到底并稳住**」的按压微技能（nozzle 91% travel + 瓶漂移<2 mm）| Panda 钳瓶与 Shadow 手指**不学**（每步写回记录好的钳位目标）；抬回/回弹也不训（eval 补判，见 §8）|
| 并行 / 场景 | `num_envs=128, env_spacing=6.0, replicate_physics=True, clone_in_fabric=True` | 每 env：Presser/Panda/Bottle(自由)/Table(+Riser 仅 s07) |
| 节拍 | `sim.dt=1/120`，`decimation=2` → **决策/奖励 60 Hz** | 每个决策步内跑 2 个物理子步；`_apply_action` 每子步都执行 |
| episode | `episode_length_s=8` → 480 决策步（上限）| 收敛后成功首触 ~55 决策步（≈直线俯冲，与 demo 68–89 物理步同量级）|
| **动作** | **3-DoF 世界系 task-space 残差 (dx,dy,dz)**，`clip ±1`，`action_scale=0.0005 m`（0.5 mm/步 @60Hz → ~30 mm/s）| 累计进掌心命令位 `cmd_pos`；每子步 **1-step DLS IK** 映到 iiwa7 7 臂关节（关节增量 clamp ±0.09 rad）。几何无关 → 零样本跨几何的根因之一 |
| **观测** | **25 维，全相对**：nozzle `q,q̇`(2)；palm→瓶轴 xy err(2)；palm 高于 cap 平面 dz(1)；瓶漂移 xy(2)；7 臂关节相对 hover offset(7)+vel(7)；palm 垂速(1)；prev act(3) | 无任何绝对坐标 → 同网可在另一几何上直接开 |
| 力/行程代用 | nozzle 关节位 `q`（下压为负），力≈−K·q（K=300）；`cap_top_local=0.145`、`travel=0.005` | `ContactSensor` 不进多 env 训练场景（filter 脆弱）；真实 palm 力由 demo/eval 单 env 复算 |
| **Reward**（每决策步，q<0 为下压）| `+0.25/mm`·Δpress（仅新增下压行程 `clamp(q_prev−q,≥0)`）+ `+0.4`·[贴底带稳住]（`q∈[−0.0054,−0.0035]` ∧ drift<2 mm ∧ |q̇|<0.01）+ **`+10`** 稀疏成功（`q≤−0.0045` ∧ drift<2 mm，随即终止）+ **`−2`** fail 终止（行程没进就下探过头 / 瓶 drift>8 mm 被撞飞）+ **`−0.3/mm`**·drift（持续漂移罚）+ **`−0.02·Σa²`**（动作率）+ **`−0.01·mean|q̇_arm|`**（臂速）| 见 `_get_rewards` |
| Dones | `terminated = success \| fail(falloff\|drift_fail)`；`truncated = 480 步` | DirectRLEnv 在 step() 内自动 reset 终止子 env → 评测用 done 事件上的 reward 尖峰（`rew>2.0`）反推成功 |
| Reset | 离线平衡快照 `rl/snapshots/recenter_s{1.00,0.70}.npz`（`demo_mixed_press.py --snap_dir` 存：钳位形 + presser hover 关节 + panda 状态/目标）写回，速度清零、nozzle `q=0`，**命令位=当前 palm FK**（第一步从 hover 出发）| Stage A = 确定性复位 |
| DR（仅 Stage B 打开）| 瓶 root xy ±0.5 mm / yaw ±0.8°、presser 臂关节 ±0.02 rad、presser 基座 z ±4 mm、命令 xy ±1.5 mm | A/B **同一网络、同一 obs/act**，B = resume A 权重只放宽 reset 分布 |

> 观测-动作对（无记忆、部分可观测只靠当步量）下策略把「压到底并稳住」学成 ~55 步最优：头号激励是逐毫米 `+0.25` 的进展、触底带 `+0.4/步` 的稳住、以及触发即终止的 `+10`——reward 面根本没有让策略在底下长期徘徊的动机。

## 3. 方法：课程化两段 + 为什么这么设计

- **学什么**：只学按压微技能。episode 从离线平衡快照（已被钳、掌心 hover cap 上方）开始，策略输出小增量把 palm 沿 −z 压到 nozzle 到底并稳住、不把瓶推偏。Panda 钳位、Shadow 手指、抬回回弹都**不进** MDP（Stage A 保持奖励面良定义、收敛快）。
- **几何无关 MDP**：3-DoF task-space 残差 + IK warm-start 从当前 palm 出发 + 25 维全相对观测 → 一套网，s1 训练的权重直接 load 到 s07（0.7 臂 + 0.28 m 垫座、jacobian 尺度不同）零样本开，是「两几何都训」之外跨几何 gate 提前关闭的原因。
- **两段课程**：Stage A 在**确定性**复位上学精确按压（A：全 DR=0）；Stage B resume A 权重、只把 reset 分布放宽（B：上面 DR 档），让策略对「瓶被摆偏一点/臂起点偏一点」也稳。
- **宽 DR 初版为何不收敛（诚实记录，教训）**：先试了更宽的 reset（臂关节 ±0.06 rad、cmd-xy ±3 mm）——resume-A 直接崩（reward +3 → −50、critic value loss 80–180 发散），from-scratch 也 250 iter 仍近随机（std 1.4）。根因 = 该分布把直线下压技能放到「**2 mm 漂移奖励悬崖 + 长驻 timeout 负累积**」上，大横向 palm 偏移的重置让策略穿不过针眼。**按计划风险梯子缩 DR**（0.06→0.02 rad、3→1.5 mm，瓶/基座档保留）后即可学。判别法：先 eval「A 在 B 分布上零样本」定分布可学性，再决定 resume / from-scratch / 缩 DR。

## 4. PPO / 训练设置

- 网络：actor-critic MLP hidden `[512,256,128]`(ELU)，actor/critic **都 obs 归一**（EmpiricalNormalization）；连续高斯动作 `init_noise_std=1.0`。
- 超参（`direct/press/agents/rsl_rl_ppo_cfg.py`，镜 allegro_hand）：`num_steps_per_env=24`，`epochs=5`，`mini_batches=4`，`lr=3e-4`（adaptive，`desired_kl=0.012`），`clip=0.2`，`entropy_coef=0.005`，`gamma=0.99`，`lam=0.95`，`max_grad_norm=1.0`，`value_loss_coef=1.0` + clipped value loss。
- 运行：128 envs 单卡 **SPS≈3300–3700**（~1 s/iter）；`max_iterations=3000`、`save_interval=100`。**Stage A 收敛极快（<250 iter）**。
- 评测用 `get_inference_policy`（取均值、去随机）。
- run 目录（checkpoint 前体）：A_s1 `logs/rsl_rl/press_A_s1/2026-09-08_15-58-57/`（seed 0，m100 评测）、A_s07 `…/press_A_s07/2026-09-08_16-16-08/`（seed 0，m600）、B_s1 `…/press_B_s1/2026-09-08_16-39-35/`（resume A m900，m1400/1500）、B_s07 `…/press_B_s07/2026-09-08_16-52-43/`（resume A m600，m1200）。

## 5. 结果（成功率，done 尖峰计数）

**Stage A —— 两几何收敛 + 双向零样本 100%**：

| ckpt（训练） | eval task | episodes | success_rate |
|---|---|---|---|
| A_s1 m100（s1，seed0） | Isaac-Press-Direct-v0 (s1) | 2292 | **1.000**（首触 mean 55.4 med 55.0 min54 max57） |
| A_s1 m100 | Isaac-Press-07-Direct-v0 (s07，零样本) | 2016 | **1.000** |
| A_s07 m600（s07，seed0） | Isaac-Press-07-Direct-v0 (s07) | 2008 | **1.000** |
| A_s07 m600 | Isaac-Press-Direct-v0 (s1，零样本反向) | 2197 | **1.000** |

奖励曲线：s1 mean reward ~0 → **+3.9 @iter117 → 平台 ~+4.5**（ep 54.4）；s07 类似、平台 **~+6.5**（ep 64）。

**Stage B —— reset-DR 课程**：

| ckpt | eval task | episodes | success_rate |
|---|---|---|---|
| A_s1 m900（零样本，缩后 B 分布） | Isaac-Press-Direct-B-v0 | 1292 | 0.933 → **B_m1400: 1.000**（2241/2241） |
| A_s07 m600（零样本，缩后 07-B 分布） | Isaac-Press-07-B-v0 | 2079 | **1.000**（A 已全鲁棒）|
| B_s07 m1200 | Isaac-Press-07-B-v0 | 3041 | 0.988 |

→ **s1 两段课程成立**：A 漏掉的 ~7%「大横向偏移 reset」由 B 补到 100%。**s07 上 Stage A 本身已对缩后 DR 全鲁棒、Stage B 无增益**（诚实：课程收益是几何相关的，只在 A 会漏横向偏移的 s1 上体现）。

## 6. 视频（RL 策略滚动，已归档）

`media/rl_press_s1.mp4`、`media/rl_press_s07.mp4`、`media/rl_press_s1B.mp4` —— 1280×720 @60 fps、~3–4 s、每 clip 含 3 次成功按压（DirectRLEnv 成功即自动 reset → 连续多次）。像素校验通过（无黑帧、中心结构密度≈背景 ~9×、亮度/运动正常），机位 = demo 取景（eye=(0.15,0.95,1.35)/lookat=(0,0.02,0.52)）。录制脚本 `rl/record_press.py`。

## 7. 复现

```bash
cd <IsaacLab 根目录> && conda activate env_isaaclab   # 需已装本项目 direct/press 包（editable）
# Stage A(s1) 训练 + 自评 + 跨几何零样本
env -u DISPLAY ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Press-Direct-v0 \
  --num_envs 128 --headless --seed 0 --max_iterations 3000 --experiment_name press_A_s1
env -u DISPLAY ./isaaclab.sh -p rl/eval_press.py --task Isaac-Press-Direct-v0 --headless --num_envs 32 \
  --checkpoint logs/rsl_rl/press_A_s1/<run>/model_100.pt
env -u DISPLAY ./isaaclab.sh -p rl/eval_press.py --task Isaac-Press-07-Direct-v0 --headless --num_envs 32 \
  --checkpoint logs/rsl_rl/press_A_s1/<run>/model_100.pt        # 跨几何零样本
# Stage B(s1) resume（缩后 DR）
env -u DISPLAY ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Press-Direct-B-v0 \
  --resume --load_run <A-run> --checkpoint model_100.pt --num_envs 128 --headless --seed 1 \
  --max_iterations 1500 --experiment_name press_B_s1
# 录 RL 视频（headless RGB 需 --enable_cameras）
env -u DISPLAY ./isaaclab.sh -p rl/record_press.py --headless --enable_cameras \
  --task Isaac-Press-Direct-v0 --num_envs 1 \
  --checkpoint logs/rsl_rl/press_A_s1/<run>/model_100.pt --video media/rl_press_s1.mp4 --steps 200
```

代码落点：RL 环境 = `IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/press/`（`press_env.py` / `press_env_cfg.py` / `agents/rsl_rl_ppo_cfg.py` + 4 task id 注册）；项目侧 harness = `rl/{smoke_press,eval_press,record_press}.py`（`./isaaclab.sh -p` 直跑，`--headless` 需 `env -u DISPLAY`）；复位快照 `rl/snapshots/recenter_s{1.00,0.70}.npz`。阶段内完整训练/诊断叙述在 `feasibility_mixed_press.md` §10。

## 8. 边界 / 口径（诚实记录）

- **成功判据沿用 demo**：nozzle 到底（`≤−0.0045`，91% travel）+ 按压期瓶漂移 <2 mm。
- **力代理**：训练 env 用 nozzle q（`−K·q` 到底 ⇔ 反力 1.35–1.37 N，与 demo 触发量级同）；真实 palm 接触力由 demo/eval 单 env 用 ContactSensor 复算，RL 训练不装。
- **回弹/抬回不训**：奖励在「压到底并稳住」即终止；抬回与回弹由 demo/eval 判读脚本补（成功率高时回弹≈0 已在 demo 验证）。
- **Stage B 宽 DR 初版不收敛**：见 §3；若日后要更宽 reset 域，需先改「2 mm 漂移悬崖 + 长 timeout 负累积」的奖励面（如加 no-progress 终止、或训练期放宽 drift gate、eval 收紧）。
- **成功计数口径**：DirectRLEnv 自动 reset 终止子 env → 终止态事后读不到 → eval 以 done 事件的 reward 尖峰（`rew>2.0`）为准。
- **SPS 等训练侧数字**为 128 envs 单卡实测，非上界。
