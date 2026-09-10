# RL 设计 / 可行性结论：夹爪同步连续流水线（PressLine）—— 多瓶按压 + RL 学换瓶

> 轮次：2026-09-09（承 PressCycle `rl_learning_press_cycle.md`）。目标 = 用户口径「自动化流水线夹爪抓不同瓶 → 按压 → 抬手 → 夹爪松瓶抓下一瓶」的连续任务，夹爪开/闭换瓶由 RL 学（非脚本），混合瓶 = Ø42 高度族不同几何变体，s1 出主录像 + s07 零样本遥测。

## 1. 固定工位可行性结论（已实测，s1 长爪固定夹持 zgrab=0.515 + palm-down 按压臂）

`demo_mixed_press.py --usd <variant>` 扫 6 个 Ø42 自由瓶高度（全 USABLE/NOT_USABLE 判读）：

| 变体 | cap_top | body_h | clamp drift | hover扫掠 knock | 结果 | 机理 |
|---|---|---|---|---|---|---|
| zc0545 | 0.545 | 0.088 | 3.8mm | 1.8mm | **NOT_USABLE** | palm 下探 IK 下限 ~z0.563，够不到更低 cap（descend 停 0.563、contact=[]） |
| zc0565 | 0.565 | 0.108 | 3.5mm | 0.5mm | **USABLE** | |
| chosen | 0.585 | ~0.128 | 3.4mm | 1.2mm | **USABLE**（= 现 RL 几何，recenter_s1.00.npz） | |
| zc0605 | 0.605 | 0.148 | 3.6mm | 1.9mm | **USABLE** | |
| zc0625 | 0.625 | 0.168 | 3.6mm | **23.7mm** | **NOT_USABLE** | 高瓶在 hover 多 seed 扫掠下被撬翻（base 抬离 z0.455、nozzle 打偏到 x=-0.62） |
| zc0650 | 0.650 | 0.193 | 3.9mm | **28.3mm** | **NOT_USABLE** | 同上 tip-over |

- **固定工位可用带 cap_top ≈ [0.565, ~0.61]**；下界是按压臂最低 IK 行程，上界是高瓶重心高 + 低刀片夹持带的抗侧翻极限（夹持带 ~世界 z 0.47–0.52，脚盘 Ø52 小支点）。
- 宽体 Ø84（foot_r 0.052 变体）**不可夹**：长爪 open 嘴 ~80mm < 84。
- 因而**混合瓶集 = {zc0565, chosen, zc0605}**（cap 0.565/0.585/0.605、body 0.108/0.128/0.148），三瓶高度视觉可辨、全部在稳定+可达带内。zc0625/0650 记为边界（更高瓶需更高/第二夹持或托座，v1 不做）。

## 2. PressLine 语义：剂量配额 + EXCHANGE（同瓶多拍有上限，达额换瓶）

PressCycle 的问题是「同一 clamp 瓶无限循环 ~16 拍直到 drift fail」。流水线语义 = 每瓶给**固定剂量**后夹爪松开、下一瓶到工位、重新抓夹 → 每瓶都从新鲜基准重新按压（也规避多拍累计漂移上限，符合真实产线节拍）。

- **剂量配额 quota_per_bottle**（默认 3 次完整 press cycle/瓶）：当前瓶累计 dose 到额 → LIFT 完成不再回 PRESS，改进 **EXCHANGE**；额内完全同 PressCycle。
- **EXCHANGE（夹爪 RL 学开/闭）**，仍由传感器门控自动推进、策略真驱动长爪 joint：

| 子相 | 进入 | 门控推进 | 说明 |
|---|---|---|---|
| 3 EXCHOPEN | LIFT 清空 & 达额 | 实际刀缝 `mean(jaw_q) ≥ open_th(0.008, 嘴~56mm)` 持续 → 移走旧瓶/新瓶到位 → EXCHCLOSE | 策略要**真张开**；张开后做**换瓶事件**（旧瓶移离、新瓶写回工位平衡位 q=0、drift 基准归零）|
| 4 EXCHCLOSE | 新瓶到位 | 刀缝 `≤ close_th(0.0015)`（刀刃贴 Ø42 体，实测闭合滞 ~0.0004）且对新基准 drift<2mm 连续 dwell → `bottle_count+=1`、`on_bottle=0` → 相 0 | 策略要**真合拢抓稳**；给 EXCH lump |

- **换瓶事件**：几何固定训练下，= 把本 bottle articulation 写回该几何的 snap 平衡位、q=0、重设 drift 基准（视作产线把完成瓶带走、下一瓶送到工位）。录像时换不同几何的 USD（见 §7）。
- 5 相状态机（0 PRESS / 1 HOLD / 2 LIFT / 3 EXCHOPEN / 4 EXCHCLOSE）。观测里 one-hot **5 维**。
- **失败路径照旧**（falloff / drift_fail>8mm / 各相 timeout → fail -2，horizon trunc）。换瓶 reset drift 基准后，新瓶有全新 8mm 漂移包络 → 流水线可长跑，不再是单 clamp 16 拍就顶穿。

## 3. 为什么是 RL 而非脚本（口径）

同 PressCycle：门（q、刀缝、drift、palm 高度）由**本步物理后状态确定性推进**，无人能伪造；策略职责 = 产出让门成立的**真驱动**——下压到底、HOLD 稳住、抬回让弹簧回弹、**张开刀缝、对新瓶合拢抓稳**。夹爪开/闭是 policy 的连续动作（第 4 DoF），非 if/else 脚本。这是自动化流水线/PLC 状态机的合法实现。

## 4. MDP

| 维度 | 值 |
|---|---|
| 动作 | **4 DoF**：世界系 task-space 残差 (dx,dy,dz) + 长爪 scalar `a_jaw∈[-1,1]` → 刀片目标 joint `∈[0, JAW_U]`（a_jaw=-1 全开 / +1 全合）。**剂量相(0/1/2)强制合爪**(目标 0，忽略 a_jaw，防中途松瓶)；EXCHOPEN/CLOSE 才解锁 |
| 观测 | base 26（原 25，`_prev_actions` 3→4）+ **phase one-hot 5** = **31** |
| 奖励 | dose 段沿用 PressCycle（progress / HOLD lump / LIFT / rebound / cycle / fail / drift / action / armvel）；外加 EXCHOPEN 张缝进展、EXCHCLOSE 合缝进展、**EXCH 成功 lump**（抓稳新瓶）；dose 相保持合爪（无张爪奖励） |
| Dones | fail / horizon 自动 reset；换瓶数经 `env.bottle_count` 遥测 |

（初始数值；训练不收敛按风险梯子调，见 §9。）

## 5. 训练课程（三段，规避 from-scratch 稀疏；obs 全程 31 维保证 resume 同形）

沿用 dive→cyclic→line 成功先例，且与 PressCycle 完全同构：
- **S1 dive warm**：相位锁 0（one-hot 恒 [1,0,0,0,0]）、quota 关闭、jaw 锁定闭合 → 旧单拍奖励 +10 到底即终止。先把「下压到底+稳住」在 31 维 obs 下训近 100%。
- **S2 cyclic**：resume S1，开全 0/1/2 相位（不换瓶，= PressCycle 循环），学抬回/回弹/再压循环（31 维下复刻 ~16 拍/局能力）。
- **S3 line**：resume S2，开 quota + EXCHANGE（相 3/4 使能），真学「达额停压→张爪→新瓶→合爪→再压」。
- PPO cfg：复用 PressPPORunnerCfg 超参；obs 31 / act 4。
- s07 档（scale 0.7 + 0.28m riser + 同瓶）作为零样本目标 cfg 镜像，验证跨几何（与 PressCycle 相同口径）。

## 6. 集成落点

- `press/press_env_cfg.py`：新增 `PressLineEnvCfg`（含 quota/子相门参/换瓶奖励）+ `PressLineDiveCfg`/`PressCycleVariant(cyclic, line_mode off)`… 与 s07 / band（zc0565/zc0605，改 usd+cap_top_local+snapshot）子类；`observation_space=31`、`action_space=4`、`episode_length_s` 加大（~18–24 s）。
- `press/press_line_env.py`（新）：`class PressLineEnv(PressCycleEnv)` 扩展 5 相 + jaw DoF 处理 + 换瓶事件 + bottle_count 遥测。`__init__.py` 注册新 id（s1 / s07 / band 变体；保留旧 task 不动）。
- `press_env.py` `_panda_jaw_ids` 已有；开/闭目标由 snap `panda_jt`（闭合 0）推。
- 每个 band 变体需要各自平衡 snap：`demo_mixed_press.py --usd <v> --cap_z_local <v> --snap_dir <dir> --snap_only`（s1 主几何复用 `recenter_s1.00.npz`，不需新跑）。

## 7. 混合几何口径（诚实约束 + 交付）

GPU instanced 批量（replicate/clone_in_fabric）要求同 env 模板同几何，无法单批内混不同 USD。真实流水线也是**串行单工位**，故交付口径：
- 训练用 nominal chosen 几何；obs 全 cap/几何相对 + 门阈世界固定 → 策略应跨几何零样本（PressCycle 已验证 s1→s07 有正迁移；本 band ±20mm 只改瓶高、presser 完全同构，预期高）。
- **eval/录像跨 {zc0565, chosen, zc0605}**：逐几何同策略重放，各用自己 snap/hover；汇总 per-geometry 吞吐与换瓶成功率。
- **录像**：单 env 多拍连续 + 多次 EXCH，逐段换不同几何 USD（record 脚本在换瓶事件处重载下瓶资产 + 该瓶 hover snap），得到「同一工位连续夹不同瓶按压」的视觉证明。

## 8. 结果期望与量测

- S3 收敛后 eval：per-bottle 剂量成功率、**mean bottles/episode、per-EXCH 成功率**（张→到→抓→稳）、换瓶吞吐（steps·bottle⁻¹）、dose 回弹 q、换瓶后 drift 归零轨迹、失败类型分布。
- 跨几何 zero-shot：三瓶各自瓶数/局（≥额即可，越多越好）；s07 零样本单测（expect 吞吐降但结构与回弹质量仍在，记边界）。
- 回归：旧单拍 `Isaac-Press-Direct-v0` eval 仍 ~1.0；PressCycle `Isaac-PressCycle-LiftEasy` 不动。
- 视频：s1 同一工位 ≥2 次换瓶、几何逐段不同、0 reset；帧统计校验。

## 9. 风险与后备

- **S3 换瓶稀疏**（额满前 ~150+ 步才有 EXCH，EXCH 又 ~几十步）→ lump 放大 + 张/合进展奖励稠密化；仍卡则降 quota 至 1 先学「压一拍即换」，再抬 quota。
- **爪到新瓶对不准/夹空**（arrival 写平衡位但爪需合拢找瓶）→ 抓稳门用「刀缝≤close_th 且对新基准 drift<2mm 连续」；夹持力由 physics 决定。
- **换瓶事件被指太瞬移** → 口径：离开/到位是产线 conveyor 事件，demo sim 无传送带，以「写平衡位+drift 归零」建模；爪开/闭时机与抓稳仍为 RL 真学。
- **tip-over 高瓶边界**：z ≤0.61 稳定（实测 hover sweep 都撑住），更高不进混合集。
- **obs 31 / act 4 从零三段重训成本** → dive/cyclic 收敛快（同几何同奖励面），仅多 3 zero-ish 通道。
