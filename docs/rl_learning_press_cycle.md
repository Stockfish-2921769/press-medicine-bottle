# RL 学习 round 总结：闭环连续按压（PressCycle）—— 同瓶多拍相位 MDP，真学抬回/回弹

> 报告日期：2026-09-09（训练/评测均当日，T-1 天按 `rl_learning_press.md` 完成单拍微技能）
> 前置：单拍 PPO 真学按压（`rl_learning_press.md`）压到底即终止、抬回/回弹不训（eval 补判）。本 round 在**用户拍板**下把语义升级为「**闭环连续任务、可在自动化流水线部署**」：同一 clamp 自由瓶上**重复按压多拍**，每拍都需掌心抬离让弹簧完全回弹再压；episode 不因首触底终止。
> 代码 = `IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/press/`（`press_cycle_env.py` / `press_env_cfg.py` 的 `PressCycle*` + `__init__.py` 注册）；harness = `isaac_demo/rl/{eval_press_cycle,record_press_cycle,probe_release_latency}.py`。

## 1. 一句话总结

**s=1.0（s1）训到真学「下压→HOLD 稳住→抬回让弹簧完全回弹→再压」的连续循环策略**：最终 checkpoint 平均 **~15.9 拍/局（627 局共 9985 次完整 cycle）**，单次 cycle 完成瞬间弹簧回弹残余 `q≈-0.00028`（`|q|<0.0008` 完全回弹 ✓）、瓶漂移均值 **0.28 mm/拍**；录得**同一瓶 + 同一 clamp 连续 8 拍、0 次 reset** 的干净单 episode 视频。诚实边界：s1 策略**零样本跨几何到 s07 是正的部分迁移**（~2.4 拍/局，回弹干净、漂移 ~0.55 mm/拍，但吞吐显著下降）；canonical 25 mm 抬升档没训成正奖励（-15→-68 劣化，停机记录）；单拍旧任务回归仍 **100%**。

## 2. 语义：同瓶多拍闭环 + 3 相相位状态机（门控自动推进）

在单拍 `PressEnv`（Panda 长爪钳自由瓶 + Shadow 掌心压 cap）上叠 3 相。相位由**本步物理后状态**门控自动推进（策略职责 = 产出让门成立的驱动，含真学抬回；门无法伪造）：

| 相 | 进入 | 门控推进 → 下一相 | 超时 → fail |
|---|---|---|---|
| 0 PRESS | reset / cycle 完成（palm hover cap 上方 ~25–30 mm，q≈0）| `q ≤ bottom_q(-0.0045)` ∧ 瓶漂移<2 mm → HOLD | 250 步 |
| 1 HOLD | 到 底 | 在带 `q∈[-0.0054,-0.0035] ∧ |q̇|<0.02 ∧ drift<2 mm` 连续 `H_min=12` 步 → 给 lump、→ LIFT；q 回升 >-0.002 则 dwell 清零 | 60 步 |
| 2 LIFT | 稳住完成 | palm 抬到 `h ≥ h_return`（=palm_z−(root_z+0.145)，超 cap 平面）`∧ |q|<0.0008`（弹簧完全回弹）→ `cycle+=1`、回 PRESS | 200/100 步 |

- `dones`：`terminated = fail`（falloff / drift_fail>8 mm / 各相超时），**无 success 终止**；`truncated = 840 步 horizon`。→ 失败/到界自动 reset，cycle 缓冲跨相保留、**同 episode 累计多拍**。
- `episode_length_s=14`（840 决策步 @60 Hz ≈ 16 拍），一次 episode 经历多轮完整 cycle。
- **门参全为世界固定常量**（cap_top_local=0.145 / bottom_q / h_return / drift 阈值）→ cap/瓶物理在 s1/s07 完全一致，跨几何共享。
- **LiftEasy 档**：`h_return=0.012`、`lift_timeout=100`。物理上 12 mm 已足够——palm body 参考系下接触面在 ~11.2 mm 断开、弹簧在 ~11 mm 脱开，12 mm 门是**真抬离**且覆盖完整回弹（评测回弹 q≈-0.00028 ✓）。canonical 25 mm 只影响抬多高，不影响回弹是否发生。

## 3. MDP 一览

| 维度 | 值 | 说明 |
|---|---|---|
| 动作 | 3-DoF 世界系 task-space 残差 (dx,dy,dz)，clip ±1，`action_scale=0.0005 m` | 累计进掌心命令位 `cmd_pos`；每子步 1-step DLS IK 映到 iiwa7 臂。**Phase 过渡处把 `cmd_pos/cmd_quat` 重锚到掌心真实位姿**（见 §5 根因）|
| 观测 | 25（单拍相对 obs）+ **相位 one-hot 3** = **28 维** | one-hot 非泄漏：由传感器确定性可推的粗进度，消解「已到底须停压(HOLD)/须上升(LIFT)」歧义。不加 cycle_count（无界、归一化下漂移）|
| 奖励（cycle 档，单位每决策步）| 下压进展 `+0.25/mm`(仅负向行程增量, PRESS)；LIFT 抬升 `+0.6/mm`(仅向上增量, LIFT)；回弹 `+1.0/mm`(q 向 0 走, LIFT 塑造「松手」)；HOLD lump `+2.0`(满 H_min 一次性)；**cycle 完成 `+6.0`**；fail `-2.0`；漂移 `-0.3/mm`；动作 `-0.02·Σa²`；臂速 `-0.01·mean|q̇_arm|` | 防钻空：相内无稳态刷分、下压只奖新增负行程、各相 timeout 封死死锁、+cycle 远大于相内项 → 最快循环显式最优 |
| Dones | fail/horizon 自动 reset，无 success 终止 | 评测逐拍读 `env.cycle_count` 遥测（cycle 完成不是 done 事件）|

## 4. 训练课程（两段 resume，规避 from-scratch 稀疏 cycle 难学）

沿用单拍 A→B resume 成功先例：

- **S1 dive warm-start**（`Isaac-PressCycle-Dive`）：相位机锁 PRESS（one-hot 恒 [1,0,0]，obs 已 28 维），单拍旧奖励 + +10 到底终止。用已证明奖励面把「下压到底+稳住+drift<2 mm」训近 100%。
- **S2 cyclic（`Isaac-PressCycle-LiftEasy`）**：resume dive 权重（obs_normalizer 25:28 维清零置 1、action std 提到 1.0），开全相位机、加 HOLD lump / LIFT / cycle 奖励。收敛 ~+28..+42，final `model_3199`（+38.6）。
- canonical 25 mm 档尝试（resume m900 → `Isaac-PressCycle-Direct`）：**不收敛**（reward -15 → -68 劣化，停机）。原因：抬更高 → 单拍周期变长、漂移/动作成本抵消 cycle 增益，reward-per-step 转负；且 LiftEasy 12 mm 物理上已完全回弹，25 mm 非必需 → **以 LiftEasy 为主交付、canonical 记边界**。

## 5. 关键结构根因与修复（5 次训练停滞的共同根因，非奖励调参问题）

**命令积分债（command-integration debt）**：策略在压底过程中 `cmd_pos.z` 无条件向下积分（掌心物理被 cap 硬停挡住时也继续），把命令埋到掌心下方几十 mm。LIFT 抬回必须先爬完这段「幽灵深度」（掌心不动 → 抬升奖励为 0）才真抬——对 scalar-Gaussian PPO 是不可探索的连贯上冲 → 5 个先前配置全部停在下压吸引子（-13 到 -190）。
**修复（已 probe 验证）**：① 每个相位过渡把 `cmd_pos/cmd_quat` **重锚到掌心真实位姿**（HOLD→LIFT 入口债务归零）；② **LIFT 相专属命令深度下限**（`cmd_z ≥ palm_z − cmd_press_margin=0.0025`），防止 LIFT 里压进 cap 的停滞再次埋命令。下压相不受限（DLS IK 需 ~3.6 mm 命令赤字才有真行程，不能全局夹）。`probe_release_latency.py` 实测：压底 87 步/命令 0 赤字，80 步停滞后再抬 ~33 步完全回弹（修复前为不可达）。

## 6. PPO / 训练设置

- actor-critic MLP `[512,256,128]`(ELU)，obs 归一（EmpiricalNormalization），`init_noise_std=1.0`；超参同单拍（镜 allegro_hand，`PressPPORunnerCfg`）。
- 128 envs 单卡；S2 LiftEasy run：`logs/rsl_rl/presscycle_s1_s2fix_warm/2026-09-09_14-57-11/`（resume dive warm200w @model_200，3000+ iter，final `model_3199.pt`）。
- 评测 `get_inference_policy`（取均值）。

## 7. 结果

| ckpt（训练） | eval task | episodes(局) | 总 cycle | cycles/局 | 回弹 q(完成瞬间均值) | 瓶漂移(完成瞬间均值) |
|---|---|---|---|---|---|---|
| s1 LiftEasy `model_3199` | Isaac-PressCycle-LiftEasy-Direct-v0 (s1) | 627 | 9985 | **15.93** | **-0.00028**（完全回弹）| **0.28 mm/拍** |
| s1 LiftEasy `model_3199` | Isaac-PressCycle-07-LiftEasy-Direct-v0 (s07，**零样本**) | 1336 | 3188 | **2.39** | -0.00034 | 0.55 mm/拍 |
| 单拍回归 A_s1 `model_100` | Isaac-Press-Direct-v0 (s1) | 3415 | — | 成功 **1.000**（首触 55.6 步）| — | — |

- s1 每局 ~16 拍后以 **fail 门结束**（非 horizon trunc：0/627 到 840 步）。漂移 0.28 mm/拍随机游走约在 10–20 拍后穿越 8 mm fail 带 → **连续给药窗口约 16 拍/次 clamp**（流水线若需更长可 mid-run 重新对心/换瓶，或未来放宽 fail 包络）。这是多拍 clamp 保持力的真实边界，按计划如实记录。
- 平均每拍 ~51 决策步（录像 8 拍 409 步、0 reset）。

## 8. 视频（单 episode 连续多拍，RL 策略滚动）

`isaac_demo/outputs/rl_press_cycle_s1.mp4` —— **同一 clamp 自由瓶连续 8 拍、单 episode 0 次 reset**（1280×720 @60 fps、409 帧、~6.8 s；`record_press_cycle.py`）。像素校验通过：0 黑帧、亮度 117–125、中心/背景结构密度 ~2.1、帧间 |diff| 0.31（按压/抬离动作清晰）。

## 9. 复现

```bash
cd <IsaacLab 根> && conda activate env_isaaclab
# S1 dive warm-start（s1）
env -u DISPLAY ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-PressCycle-Dive-v0 \
  --num_envs 128 --headless --seed 0 --max_iterations 1500 --experiment_name presscycle_s1_s2fix_warm
# S2 cycle resume（LiftEasy 档）
env -u DISPLAY ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-PressCycle-LiftEasy-Direct-v0 \
  --resume --load_run warm200w --checkpoint model_200.pt --num_envs 128 --headless --seed 0 \
  --max_iterations 3000 --experiment_name presscycle_s1_s2fix_warm
# eval（s1 主档 / s07 零样本）+ 录像
env -u DISPLAY ./isaaclab.sh -p rl/eval_press_cycle.py --task Isaac-PressCycle-LiftEasy-Direct-v0 --headless --num_envs 64 \
  --checkpoint logs/rsl_rl/presscycle_s1_s2fix_warm/<run>/model_3199.pt --steps 8000
env -u DISPLAY ./isaaclab.sh -p rl/eval_press_cycle.py --task Isaac-PressCycle-07-LiftEasy-Direct-v0 --headless --num_envs 64 \
  --checkpoint logs/rsl_rl/presscycle_s1_s2fix_warm/<run>/model_3199.pt --steps 8000   # 零样本
env -u DISPLAY ./isaaclab.sh -p rl/record_press_cycle.py --task Isaac-PressCycle-LiftEasy-Direct-v0 --headless --enable_cameras \
  --checkpoint logs/rsl_rl/presscycle_s1_s2fix_warm/<run>/model_3199.pt --max_cycles 8 --video outputs/rl_press_cycle_s1.mp4
# 单拍回归
env -u DISPLAY ./isaaclab.sh -p rl/eval_press.py --task Isaac-Press-Direct-v0 --headless --num_envs 64 \
  --checkpoint logs/rsl_rl/press_A_s1/2026-09-08_15-58-57/model_100.pt --steps 3000
```

## 10. 边界 / 口径（诚实记录）

- **canonical 25 mm 抬升档未训成正奖励**（-15→-68）：见 §4/§5。LiftEasy 12 mm 已物理足够（接触 ~11.2 mm 断开、回弹完全），故 25 mm 非交付必需；若要求完整 hover 抬升需重设奖励（如按 LIFT 抬到 25 mm 另给 lump）。
- **多拍累计漂移**：单 clamp 连续 ~16 拍后越 8 mm fail 带（0.28 mm/拍 → 随机游走），s07 更明显（0.55 mm/拍 → ~2.4 拍/局）。流水线部署需 ~≤10 拍 re-dose/对心节奏，或未来按 cycle 放宽 fail 包络。这是「同瓶多拍连续」的核心物理边界，如实记录。
- **零样本 s07 为正的部分迁移**（2.39 vs 15.9 拍/局）：0.7 臂 + 0.28 m 垫座的 IK 动态不同，s1 学到的命令幅度未重调，吞吐下降；回弹与漂移质量仍干净。若需 s07 高吞吐，另训或退火。
- **训练无 ContactSensor**：palm 力以 nozzle q 代理（-K·q）；真实 palm 接触力由 demo/eval 单 env 复算。
- **相位门控=自动化状态机**：门由传感器确定性推进、策略负责真驱动与真抬回，非脚本；是 PLC/流水线状态机的合法实现。

---

> 前置单拍 round 见 `rl_learning_press.md`；混形 demo / 缩放几何见 `feasibility_mixed_press.md`。代码落点见文件头注释。
