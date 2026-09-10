# 流水线换瓶按压（PressLine）：剂量-配额 + RL 学夹爪换瓶 连续多瓶 RL（round 总结）

> 状态（2026-09-09）：**完成**。s1 nominal 几何训练出「自动流水线同工位连续多瓶」策略：一瓶按压达到剂量配额后，**RL 学到的长爪开/合（第 4 个 action DoF）自动换下一瓶**——单 episode 稳定换 4+ 瓶、0 reset（录像），批量评测 **5.95 瓶/局、每瓶剂量 3.21 拍（配额 3）**、87% episode 撑满 1200 步 horizon。诚实边界：**跨瓶高（zc0565/zc0605）与跨尺度（s07）零样本为负** —— 策略绑 nominal 几何，混合几何/混合工位单段连续仍不受 instanced-env 架构支持（见 §7/§8）。

代码 = `IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/press/`（`press_line_env.py` / `press_env_cfg.py` 的 `PressLine*` + `__init__.py` 注册）；harness = `isaac_demo/rl/{eval_press_line,record_press_line,smoke_press_line}.py`；设计 = `isaac_demo/docs/rl_press_line_design.md`。

## 1. 目标与本轮交付

用户口径（自动化流水线场景）：**夹爪抓住不同瓶子 → 机械臂+灵巧手按压 → 抬升 → 夹爪松开本瓶 → 抓下一瓶**，任务连续。上一轮 PressCycle 已真学「同 clamp 自由瓶连续多拍循环」（~15.9 拍/局）；本轮把「换瓶」也交给 RL：

- 每个瓶子上**剂量配额 `quota_per_bottle=3` 次完整按压 cycle**（PRESS→HOLD→LIFT，与 PressCycle 同门）后，机器强制进入换瓶。
- 换瓶 EXCHANGE（两子相）由策略驱动长爪开/闭（**真学，非脚本**）：3 EXCH_OPEN 把爪张到让旧瓶可脱离 → 新鲜瓶就位（conveyor 事件：写平衡位 + drift 归零）→ 4 EXCH_CLOSE 合爪抓稳新瓶 → 回 PRESS 继续下一瓶。
- 门全部由传感器（nozzle q / 刀缝 joint / palm 高度 / 瓶 drift）确定性推进，策略只负责产出让门成立的驱动。

s1 = full-size presser + nominal chosen 瓶（cap top 0.585 / cap_top_local 0.145）为主交付；跨瓶高与 s07 作零样本评测。

## 2. MDP（obs 31 / act 4）

| | 说明 |
|---|---|
| 动作 4 | `(dx,dy,dz)` 世界系任务空间残差 + 长爪标量 `a_jaw∈[-1,1]`（-1→目标开缝 `JAW_U=0.020`，+1→闭合 0；剂量相 0/1/2 锁死闭合，仅换瓶相解锁） |
| 观测 31 | base 26（原 25 + prev_actions 变 4 维）+ **5 相 one-hot** |
| 相 0 PRESS | `q≤bottom_q(-0.0045) ∧ drift<2mm` → HOLD |
| 相 1 HOLD | 底带 dwell `H_min=12` 连续步（`|q̇|<0.02`）→ LIFT；回升超带清零重计 |
| 相 2 LIFT | palm 抬过 cap 面 `h_return=0.012` ∧ `|q|<rebound_tol(0.0008)`（弹簧完全回弹）→ `cycle+=1`；未达额 → PRESS，达额（line）→ EXCH_OPEN |
| 相 3 EXCH_OPEN | 刀缝均值 `≥jaw_open_th(0.010)` ∧ palm 仍高于 cap `≥palm_clear(0.002)` → **写新鲜瓶就位** → EXCH_CLOSE |
| 相 4 EXCH_CLOSE | 刀缝 `≤jaw_close_th(0.0015)` ∧ drift 对新基准 <2mm，连续 `grip_dwell=5` 步 → `bottle_count+=1`、`_on_bottle=0` → PRESS |
| 超时 fail | PRESS 250 / HOLD stall 60 / LIFT 100 / EXCH_OPEN 150 / EXCH_CLOSE 250；另有 falloff、drift_fail>8mm |

奖励：下压进展 `+0.25/mm`（PRESS）、HOLD lump `+2.0`、LIFT 抬升 `+0.6/mm` + 回弹 `+1.0/mm`、cycle `+6.0`、fail `-2.0`、漂移 `-0.3/mm`、动作 `-0.02Σa²`、臂速 `-0.01`；line 侧再加：张爪缺口 `-0.08/mm×(jaw_open_th−jaw)`（EXCH_OPEN）、合爪盈余 `-0.08/mm×(jaw−jaw_close_th)`（EXCH_CLOSE）、**抓到新瓶 lump `+10.0`**。门阈世界固定（cap_top_local/bottom_q/h_return），obs 相位 one-hot 为合法自动化状态（传感器确定性可推）。**换瓶 is 事件**（arrival/抓到），bottle_count/cycle_count 只进遥测供 eval。

## 3. 训练课程（三段 resume，obs 全程 31 维保证同形）

| 段 | task id | 语义 | 结果 |
|---|---|---|---|
| S1 dive | `Isaac-PressLine-Dive-v0` | 相位锁 0、quota 关、jaw 锁闭合 = 单拍旧语义（到底 +10 终止） | reward 19.3 @799；`pressline_s1_s1/2026-09-09_17-25-49/model_799.pt` |
| S2 cyclic | `Isaac-PressLine-Cyclic-v0` | 开全 0/1/2 相（= PressCycle 循环），不换瓶 | 从 dive **model_200** resume（非 799）→ 收益 ~+40..60、ep 长 ~790/840；`2026-09-09_17-49-55_s2_cyclic_v2/model_1700.pt` |
| S3 line | `Isaac-PressLine-Direct-v0` | 开 quota + EXCHANGE，真学换瓶 | reward 峰值区 ~130–160（波动）；按直接 eval 选 **model_3000**（`2026-09-09_18-12-44_s3_line/`） |

> 复现要点：**S2 从已收敛 dive 的 model_799 resume 会卡在负区（~-13 plateau，策略过度固化「压到底即止」）**；改从**早期高熵 dive model_200** resume（沿用 PressCycle 成功先例）即恢复正收敛。S3 的逐段训练 reward 波动大（60↔160），**不能只看末尾 reward 选 ckpt**——对多段候选直接跑 `eval_press_line.py` 按 bottle_count 选，`model_3000` 胜出。奖励波动与 episode 在 horizon 附近被截断/失败混合有关。

## 4. 结果（s1 nominal 几何，model_3000）

`eval_press_line.py` 64 env × 20000 步（seed 0）：

```
bottles_total=6689  dose_cycles_total=21479  episodes_fail=142  episodes_trunc=982
bottles_per_episode=5.95   dose_cycles_per_episode=19.1   fail_frac=0.126
dose_cycles_per_bottle=3.21 (quota=3)  rebound_q_mean=-0.00028  drift_at_cycle=0.25mm
episodes_with_swap_frac=0.961
fail_types: falloff=105  timeout=37  drift=0
```

- **5.95 瓶/局**（每瓶 ≥1 次 clamp 换瓶），**每瓶 3.21 剂量拍 ≈ 配额 3**（正确「达额即换」，不多不少）；
- 87.4% episode **撑满 1200 步 horizon**（连续处理多瓶直到窗口到界 = 本 MDP 的「无失败长跑」）；真实失败仅 12.6%，其中 **falloff 105（9.3%）/ 超时 37（3.3%）/ drift_fail 0**；
- 96.1% episode 至少完成 1 次换瓶；回弹干净 `q≈-0.00028`、剂量瞬间漂移 0.25mm（同瓶连续换瓶不炸 drift）。

视频 `isaac_demo/outputs/rl_press_line_s1.mp4` —— **单 episode 连续 4 次换瓶、0 reset**：剂量配额 3/6/9/12 于 step 225/386/542/698 到位，换瓶于 231/392/548/704 完成（每瓶 ~160 步 = 3 拍 + 1 换）。1280×720 @60fps、704 帧、11.7 s；像素校验 0 黑帧、亮度 127.4、帧间 diff 0.21（按压/换瓶动作清晰）。末帧 `rl_press_line_s1_last.png`。

## 5. 训练收敛细节（诚实记录）

- **S2 首跑失败**：从 S1 `model_799`（已收敛、action std ~0.33）resume cyclic，reward 从 -59 爬到 -13 后 **plateau 250+ iter 不转正**（策略强固化「压到底即止」，抬回/回弹探索不足）→ 停机，改从 dive **model_200**（早期高熵）resume 即恢复。结论：cyclic/line 阶段要用**未完全收敛的高熵 dive** 做 init，否则 S1 的「到底终止」价值函数会锁死抬回学习。
- **S3 训练 reward 大幅波动**（~iter 1900-2100 高 ~160，~2530 一度回落到 ~45，~3000 又回 ~135）：选 ckpt 以直接 eval 的 bottle_count 为准，不追末尾 reward。可能机制：换瓶 lump +10 与 drift/超时成本的竞争在 128-env 批量里呈双峰，PPO 在峰间摆荡；为交付选了 reward 高区段且 eval 稳定的 `model_3000`。

## 6. 回归

旧单拍 `Isaac-Press-Direct-v0`（press_A_s1 model_900）：**success_rate 1.000**（2332/2332，首触底 ~55 步）。基类/旧 task/旧 eval 均未被本轮重构破坏。

## 7. 跨几何 / 跨尺度零样本（**为负 —— 诚实边界**）

同策略 model_3000 直接在 band 变体与 s07 上重放（64 env × 20000 步）：

| target | 几何 | 瓶/局 | 换瓶 episode 占比 | 失败主因 |
|---|---|---|---|---|
| nominal | chosen cap 0.585 | **5.95** | 96% | falloff/timeout 少量 |
| zc0565 | 短 0.565 (Δ-20mm) | 0.43 | 19% | 超时 4370（~95%） |
| zc0605 | 长 0.605 (Δ+20mm) | 0.07 | 2.9% | falloff 6685（85%） |
| s07 | s=0.7 + 0.28m riser | 0.01 | 0.6% | 超时 5629 |

设计 §7 曾「预期 band ±20mm 零样本高」——**未成立**。原因：换瓶门虽是几何相对量，但动作是**世界系任务空间残差**（积累式绝对下压/抬升幅度、爪与 palm 的绝对 hover 都按 nominal snap 校准）；瓶子变矮 palm 会按原幅度继续下潜撞台/击穿 cap（falloff），变高则够不着/回不到位（超时）。s07 缩臂的 IK 动力学更是彻底不同。**本轮交付口径收敛为：单几何（nominal）连续多瓶是真学且稳定；跨瓶高/跨尺度的「不同瓶子」需逐几何训练或加几何 DR，当前不零样本。**

混合几何在**单段连续 take** 内混不同 USD 仍不受支持（GPU instanced 同模板同几何 + 流水线串行单工位）；逐段换几何（各用该瓶 snap 重建 env）可做但每个 env 固定一种几何 —— 属架构限制，如实记录。

## 8. 边界与遗留

- **失败面**：~12.6% episode 以 falloff（105）/ 超时（37）告终，多数在 horizon 附近（~1200 步前）。falloff = 剂量相爪口仍抓瓶但 palm 越过 cap 面下潜（q 回升且 palm 低于 cap 面 10mm），多在换瓶后首批剂量偶发；未做针对性消解（可加「换瓶后 re-hover」门或把 falloff 判定收紧到连续步）。
- **换瓶瞬移建模**：离开/就位为 conveyor 事件（写平衡位 + drift 归零），sim 无传送带动画；爪开/合时机与抓稳仍为 RL 真学。
- **训练不稳定**：S3 段间 reward 双峰摆荡；换瓶 reward 面相对 dose 面是「稀疏 +10」，对 DR/扰动较脆。
- 长程 drift：单瓶剂量漂移 0.25mm/拍、换瓶后 drift 归零，8mm fail 带几乎不触发（drift_fail=0）；连续换瓶不炸漂移 ✓。

## 9. 复现命令（IsaacLab 根，`env_isaaclab`）

```bash
cd /home/ubuntu/press_demo/IsaacLab && conda activate env_isaaclab
# 训练（三段）
env -u DISPLAY ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-PressLine-Dive-v0 --num_envs 128 --headless --seed 0 --max_iterations 800 --experiment_name pressline_s1_s1
# S2 cyclic resume（从 dive model_200）
env -u DISPLAY ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-PressLine-Cyclic-v0 --num_envs 128 --headless --seed 0 --max_iterations 3000 --resume --load_run 2026-09-09_17-25-49 --checkpoint model_200.pt --experiment_name pressline_s1_s1 --run_name s2_cyclic_v2
# S3 line resume（从 S2 model_1700）
env -u DISPLAY ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-PressLine-Direct-v0 --num_envs 128 --headless --seed 0 --max_iterations 3000 --resume --load_run 2026-09-09_17-49-55_s2_cyclic_v2 --checkpoint model_1700.pt --experiment_name pressline_s1_s1 --run_name s3_line
# eval（s1 / band / s07）
env -u DISPLAY ./isaaclab.sh -p ../isaac_demo/rl/eval_press_line.py --task Isaac-PressLine-Direct-v0 --headless --num_envs 64 --checkpoint logs/rsl_rl/pressline_s1_s1/2026-09-09_18-12-44_s3_line/model_3000.pt --steps 20000
# 录像（s1，4 瓶）
env -u DISPLAY ./isaaclab.sh -p ../isaac_demo/rl/record_press_line.py --task Isaac-PressLine-Direct-v0 --headless --enable_cameras --checkpoint logs/rsl_rl/pressline_s1_s1/2026-09-09_18-12-44_s3_line/model_3000.pt --max_bottles 4 --video ../isaac_demo/outputs/rl_press_line_s1.mp4
```

## 10. 关键文件

- 环境/配置：`direct/press/press_line_env.py`（5 相 + jaw DoF + 换瓶事件 + fail_stat 遥测）、`direct/press/press_env_cfg.py`（`PressLineEnvCfg` + Dive/Cyclic + s07/zc0565/zc0605 变体）、`direct/press/__init__.py`（6 个 `Isaac-PressLine-*` id 注册）。
- Harness：`isaac_demo/rl/smoke_press_line.py`（可达性 oracle）、`eval_press_line.py`（bottle_count/cycle_count 遥测 + fail_type）、`record_press_line.py`（单 env 连续取景）。
- 资产 snap：`isaac_demo/rl/snapshots/recenter_{s1.00,s0.70,zc0565,zc0605}.npz`。
- 上游：`rl_learning_press_cycle.md`（同瓶多拍循环，本轮 dose 相位复用它）；`rl_press_line_design.md`（本轮设计）。
