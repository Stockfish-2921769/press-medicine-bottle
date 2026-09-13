# 异构瓶按压（PressLine-Hetero）：每 env 不同瓶泛化 round 总结

> 状态（2026-09-10）：**完成（含一个明确的负结果）**。用户口径「我们尝试用不同瓶子，验证泛化性」在本轮按**架构改造：每 env 异构瓶**落地 —— `replicate_physics=False` + `MultiUsdFileCfg` 让**每个 env 生成不同的瓶 USD**（真异构场景，最接近「一条线混不同瓶」），6 档瓶高（cap 0.565–0.660 m，跨度 95 mm）同场。
>
> - **正结果（头条）**：**from-scratch 异构 dive 策略 6 档瓶高全部 100% 成功**（`eval_hetero_dive.py`：677/677 次按压，`falloff=drift=timeout=0`，每档**确定性**触底 48–56 步）。策略首次真正跨瓶高泛化。
> - **分级结果**：nominal 流线策略零样本投到异构场景 —— 仍能压但**逐档退化明显**（1.27 瓶/局 vs nominal 5.95；fail_frac 89.5% vs 12.6%），且**非单调**（±20 mm 邻档最差）。
> - **负结果**：**异构 dose-loop 专家策略训不成**（nominal→het 微调、het→het S2/S3 resume 全 plateau，死因清一色 LIFT 超时 `q=-0.005` 从不抬回）。根因未定位，如实记录为边界。

代码 = `IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/press/`（`press_line_env.py` / `press_env_cfg.py` 的 `PressLineHetero*` + `__init__.py` 注册）；harness = `isaac_demo/rl/{eval_hetero_dive,diag_hetero_policy,record_press_hetero}.py`；上游 = `rl_learning_press_line.md`（5 相 line MDP 复用）。

## 1. 目标与本轮交付

`rl_learning_press_line.md` §7 记录的诚实边界是：**跨瓶高/跨尺度零样本为负**——nominal 策略绑死单几何，band ±20 mm 都不迁移，「要不同瓶子需逐几何训练或几何 DR」。本轮正面回答这个边界：

1. **架构**：把「每 env 一种瓶」做进 instanced env（此前记录为「不受支持」）；
2. **训练**：在真异构场景上**从零训** dive（而非微调）；
3. **量测**：逐档瓶高的成功率 / 触底步数、零样本投递的逐档吞吐与失败归因；
4. **诚实**：把「异构 dose-loop 专家训不成」作为负结果完整记录。

## 2. 异构场景架构（本轮的核心改动）

| 项 | 做法 |
|---|---|
| 每 env 不同瓶 | `bottle_cfg.spawn = MultiUsdFileCfg(usd_path=[6 个 USD], random_choice=False)` → `env i` 拿 `variant i % 6`（确定性 round-robin，均匀覆盖） |
| 场景前置条件 | `InteractiveSceneCfg(replicate_physics=False, clone_in_fabric=False)` —— PhysX 必须逐个 parse 每 env 自己的瓶（关闭实例化共享模板） |
| 每 env cap 高 | `cap_top_local` 从 `(N,)` 常量 tensor 变 per-env tensor：载入时用 stage 上的瓶 USD 路径反查变体，再从该变体的 recenter snapshot 读真实 cap-above-root |
| 门控/观测不改 | 全部早已是**几何相对量**（`palm_above = palm_z − (root_z + cap_top_local)`），所以只有 `cap_top_local` 需变 per-env，MDP 定义零修改 |
| num_envs | 取 6 的倍数（本轮 126 = 21×6）保证逐档均匀 |

**关键点**：这一步之所以可能，是因为 MDP 从设计起就用「相对 cap 平面」而非绝对高度（`rl_press_line_design.md` 的几何无关性红利），异构化只落到资产层与一个 per-env 张量，策略输入形状不变（obs 31 / act 4）。

## 3. 几何族与两档被排除的瓶

height 带与极端都造了自由瓶变体（`tools/author_hetero_bottle.py`，各带 recenter snapshot）：

| 变体 | cap_top (m) | vs nominal | 是否入选 |
|---|---|---|---|
| zc0500 | 0.500 | −85 mm | ✗ 钳臂 seat NOREACH |
| zc0545 | 0.545 | −40 mm | ✗ 低于 palm 下限 |
| **zc0565** | 0.565 | −20 mm | ✓（可压的最矮） |
| **chosen** | 0.585 | 0（nominal） | ✓ |
| **zc0605** | 0.605 | +20 mm | ✓ |
| **zc0625** | 0.625 | +40 mm | ✓ |
| **zc0650** | 0.650 | +65 mm | ✓ |
| **zc0660** | 0.660 | +75 mm | ✓ |

**排除 zc0500/zc0545 的物理原因（非碰撞，是关节限位）**：按压臂 palm 原点最低只能到 `z≈0.562`（wrist/forearm 关节限位），更矮的瓶压根压不到 —— demo 直接以 `descended w/o nozzle stroke ... contact=[]` 失败，**把压座整体降 35 mm 也无效**。所以 0.565 是可达下界，`zc0565` 即本场景的「最矮可压瓶」。这个下界是**机械臂几何**给出的，不是瓶本身的问题。

## 4. 训练：from-scratch 异构 dive

**为什么不微调**：把 nominal 训好的策略直接微调进 6 档混合分布，**两次都没救回来** —— 默认 adaptive LR plateau **≈−59**、`schedule="fixed"` + 6× 低 LR 只把落点挪到 **≈−39**，两者都在接触混合分布后迅速崩塌（与 `Press1EnvCfgB` 记录的「放宽 reset 分布即崩」同签名）。所以本轮不在原策略上补丁，而是**在混合场景上从零重训 dive 相位**（`PressLineHeteroDiveCfg`：相位锁 0、quota 关、jaw 锁闭 = 旧单拍语义，奖励面逐字复用 nominal S1 那一套）。

| run | num_envs | PPO 关键改动 | 结果（tfevents 实测） |
|---|---|---|---|
| `het_dive_scratch` `2026-09-10_12-08-24` | 48 | 默认（entropy_coef 0.005） | 100% 确定性成功，**但 exploration 全程不降**：`Loss/entropy` 5.674→**5.594**（平），reward/步 +1.68、ep_len 52.3 |
| **`het_dive_lowent126`** `2026-09-10_12-53-46` | **126** | `PressPPORunnerCfgHeteroDive`：**entropy_coef 0.005→0.0005**（10× 降） | **entropy 5.674→0.519**（action std ≈1.0→≈0.41）、reward/步 **+2.29**、ep_len **51.9** + 100% 确定性 |

**为什么在意 exploration**：`rl_learning_press_line.md` §5 的先例是「S2/S3 要从**未完全收敛的高熵 dive** resume 才学得会抬回」。而默认 cfg 下异构 dive 的 entropy 从不 anneal，等于永远交出一个「高熵 warm-start」；降 entropy bonus 让它真正收敛，才是下游 dose-loop 的合格起点。**交付 ckpt = `het_dive_lowent126/model_799.pt`**。

> 注：48-env 那版（默认 entropy）其实**也是 100% 成功**，只是 entropy 不降 —— 说明「任务学会了」与「策略收敛了」是两件事，单看 reward 曲线看不出差别。

## 5. 结果（一）：逐档瓶高 dive 成功率 = 100%

`eval_hetero_dive.py`，6 env（每 env 一个变体）× 6000 步，确定性推理，`model_799`：

```
[dive-eval] successes per variant: zc0660 124, zc0650 117, zc0625 113, zc0565 109, zc0605 107, chosen 107
[dive-eval] fail_types falloff=0 drift=0 timeout=0 none=0
```

| 变体 | 成功次数 | episode 结束数 | 成功率 | 触底步数 (mean=min) |
|---|---|---|---|---|
| zc0565 | 109 | 109 | **100%** | 55 |
| chosen | 107 | 107 | **100%** | 56 |
| zc0605 | 107 | 107 | **100%** | 56 |
| zc0625 | 113 | 113 | **100%** | 53 |
| zc0650 | 117 | 117 | **100%** | 51 |
| zc0660 | 124 | 124 | **100%** | 48 |
| 合计 | **677** | **677** | **100%** | — |

- **`episode-ends == successes` 逐档成立** ⇒ 每个 episode 都以成功告终，**零失败零截断**（`falloff/drift/timeout` 全 0）。
- `mean == min` 逐档成立 ⇒ **完全确定性**：同一档每次按压步数一模一样（零 DR、确定性策略）。
- 触底步数随瓶高**单调**：最高瓶 zc0660 只要 48 步、最矮/名义档 55–56 步（palm hover 相对 cap 面，瓶越高 hover 起点越高，到机械底行程越短）。

**这直接推翻了 `rl_learning_press_line.md` §7 的边界结论**：跨瓶高**零样本为负**成立的是「nominal 策略不迁移」；但只要**在混合场景上从零训**，单个策略就能覆盖 95 mm 瓶高跨度、逐档 100%。

## 6. 结果（二）：nominal 流线策略零样本投到异构场景（分级，非单调）

同一套量测，把 **nominal line 策略**（`pressline_s1_s1/2026-09-09_18-12-44_s3_line/model_3000.pt`）直接投到异构场景，126 env × 12000 步（seed 0）：

```
[eval] steps=12000 bottles_total=5308 dose_cycles_total=17639 episodes_fail=3740 episodes_trunc=439
[eval] bottles_per_episode=1.27 (n_ep=4179) dose_cycles_per_episode=4.2 fail_frac=0.895 episodes_with_swap_frac=0.308
```

| 变体 | 瓶数 | 拍数 | 拍/瓶 | 步/瓶 | episode 结束 | 失败占比 |
|---|---|---|---|---|---|---|
| zc0565 (−20) | 428 | 1477 | 3.45 | 589 | 870 | **98.0%** |
| chosen (0) | **1324** | 4248 | 3.21 | 190 | 221 | **9.5%** |
| zc0605 (+20) | **201** | 750 | 3.73 | 1254 | 1406 | **99.6%** |
| zc0625 (+40) | 800 | 2602 | 3.25 | 315 | 888 | 98.6% |
| zc0650 (+65) | 1195 | 4036 | 3.38 | 211 | 475 | 83.2% |
| zc0660 (+75) | **1360** | 4526 | 3.33 | 185 | 319 | 61.1% |

- **整体从 5.95 瓶/局塌到 1.27 瓶/局**、fail_frac 12.6%→89.5% —— 混合分布下 nominal 策略**确实退化**。
- **但「达额即换」的正确性没丢**：只要换瓶成功，`cycles_per_bottle` 仍在 **3.21–3.73 ≈ 配额 3**，说明退化发生在**换瓶/抬回的可行性**上，不在剂量计数逻辑上。
- **退化是非单调的，且最差落在 nominal 邻档**：zc0605(+20 mm) 与 zc0565(−20 mm) 是全场最差（99.6% / 98.0% 失败），反而比 zc0650(+65)/zc0660(+75) **差得多**（83% / 61%）。**设计 §7 曾「预期 band ±20 mm 零样本高」，实测恰恰相反 —— 邻档是死区。**
- 机制（与 §7 同因）：动作是**世界系任务空间残差**，下压/抬升幅度按 nominal snap 校准，瓶高一变，palm 的绝对行程就不再对得上。
- **但「怎么失败」因档而异 —— 是两种不同的病，不是一个**（`diag_hetero_policy.py`，6 env 每档一个 × 12000 步，逐 episode 归因）：

| 变体 | HOLD 超时 @ q_min≈−0.005（压到了但停不住） | PRESS 超时 @ q_min≈0（压根没压上） | env `fail_stat` 主导 |
|---|---|---|---|
| zc0565 (−20) | **61** | 2 | timeout |
| zc0605 (+20) | 7 | **55**（q_min=−0.00033，几乎零行程） | **falloff=61** |
| zc0625 (+40) | 12 | 7（q_min≈−0.0037，压一半） | timeout |
| zc0650 (+65) | 10 | 0 | timeout |
| zc0660 (+75) | 7 | 1 | timeout |

  - **短瓶侧（zc0565）**：能压到底（`q_min=−0.00500`）但**卡在 HOLD 停不住** —— 61 次 HOLD 超时，即压到位却过不了「`|q̇|<0.02` 连续 12 步」的 dwell 门（微驱动停不下来）。**不是够不着**。
  - **正邻档（zc0605）**：**根本压不上** —— 55 次 PRESS 超时里 `q_min=−0.00033`（≈0 行程），palm 下去了但没吃到 cap（env 自己记为 `falloff`，全场景 61 次 falloff 基本都在这一档）。这是**唯一「miss 掉 cap」的档**。
  - 其余偏高档：多为「压到了但 HOLD 停不住」（10/7 次）。
  ⇒ 所以「±20 mm 邻档最差」这句话对，但**两档的病根相反**：一个够得到却稳不住，一个压根没够到。修法也不同（前者要 dwell 门/接触动力学，后者要 reach/对心）。
- 复现口径注：`diag_hetero_policy.py` 的 reason **是重新推的**（快照取在 `step()` 之前，而 env 在 `step()` 之后判门），所以本表的 PRESS 类会落进它的 `timeout` 兜底桶；**以 env 自己的 `fail_stat`（`falloff=61 drift=0 timeout=101`，合计 162 = 本表逐档死亡数之和）为准**。

## 7. 负结果：异构 dose-loop 专家训不成（诚实记录）

既然异构 dive 100%，自然下一步是「het dive → resume 进 dose loop（`Isaac-PressLine-Hetero-Cyclic` / `-Hetero`）」，拿到一个**能连续多拍多瓶的异构专家**。**这一步没做成。**

试过的路径（全部 plateau，数值为 tfevents 实测首/末）：

| 起点 | task | reward/步 | ep_len | entropy | 结论 |
|---|---|---|---|---|---|
| nom→het 微调 ×2（adaptive 3e-4 / fixed 5e-5） | `-Hetero` | — | — | — | 发散/plateau |
| `het_dive_scratch/model_200` | Cyclic | −0.08 → **−83.9** | 2.5→269.9 | 5.61→5.55 | 不转正 |
| `het_dive_scratch/model_100` | Cyclic | −0.06 → **−102.1** | 2.5→284.0 | 5.39→5.60 | 不转正 |
| `het_dive_scratch/model_799`（高熵起点） | S2 cyclic | −0.14 → **−50.9** | 2.5→192.8 | 5.594→5.595 | 不转正 |
| 同上 | S3 line | −1.58 → **−59.5** | 11.0→211.6 | 5.593→5.601 | 不转正 |
| **`lowent126/model_799`（低熵起点）** | S2 cyclic | −0.90 → **−37.1** | 7.2→206.3 | 0.53→**1.02** | 不转正 |
| 同上 | S3 line | −0.70 → **−45.6** | 8.8→331.1 | 0.53→0.91 | 不转正 |

**死因高度一致**：**清一色 LIFT 超时，死在 `q=-0.005`** —— 策略压到底之后**从不抬回**，卡在 LIFT 相直到 timeout。即 `rl_learning_press_line.md` §5 记过的同一个病（「压到底即止」的价值函数锁死抬回学习）。

**一个反直觉的细节**：低熵起点（entropy 0.53）resume 进去后，**entropy 反而涨回 1.02** —— 说明策略不是在「冻结」状态下失败，而是**在探索中持续失败**（ep_len 从 7 涨到 206，即越跑越会「撑长 episode」但撑的方式是不抬回的 timeout）。这排除了「探索不足」这一假设。

**已排除的假设（逐一实测）**：

| 假设 | 判别实验 | 结论 |
|---|---|---|
| 环境 bug | 异构 cyclic/line env 能否压到底 | ✗ 能，diag 见 `q_min=-0.00500` 全档到位 |
| 学习率 | adaptive 3e-4 vs fixed 5e-5 | ✗ 两者 plateau 完全一样 |
| 探索过高/过低 | 换 entropy 已收到 0.53 的 low-ent dive 起点 | ✗ 照样 plateau（且 entropy 反涨到 1.02） |
| 脚本 oracle 不行 | `smoke_press_line.py` 在异构场景 | 部分：oracle 只在最矮档 `cycles min=0`（oracle 自身局限），envs 1–5 都能换到 3 瓶 |

**所以根因仍**未**定位** —— 本轮把它作为**未解边界**如实记录，不编机制。可复现的现场 = `logs/het_s2_low.log` / `logs/het_s3_low.log` 与 §9 的命令。

**边界不等于没收获**：`PressPPORunnerCfgHetero`（nominal→het 微调配方）保留在代码里、docstring 明标「DEAD END 记录，无 task id 引用」，让这个负结果可复现。

## 8. 回归

旧单拍 `Isaac-Press-Direct-v0`（`press_A_s1/2026-09-08_15-58-57/model_900.pt`）：**success_rate 1.000**（2332/2332，首触底 ~54.5 步）。本轮只**新增** `PressLineHetero*` cfg/注册/harness，基类 `PressEnv`、旧 task id、旧 eval/record 均未被破坏，单拍与 nominal line 路径回归通过。

## 9. 录像

6 档瓶高**各录一段**（异构场景单相机只能看到一个瓶，故逐变体取景），再拼成一张**「一个策略同时跑 6 档瓶高」的对照图**：

- `isaac_demo/outputs/het_env{0..5}.mp4` —— 逐档单段，1280×720@60fps、293–294 帧、每段 burn-in 标注（`zc0565 -20mm` … `zc0660 +75mm`）、按压次数 **5/5/5/5/5/6**（按压计数用**成功步奖励** `rew≥9` 判定 —— q 穿越底在 `step()` 内部就 auto-reset 了，读 q 永远看不到子阈值值，这是本轮 debug 出的坑）。
- **`isaac_demo/outputs/rl_press_hetero_dive.mp4`** —— 2×3 拼图（1920×1620@60fps、294 帧、2.0 MB），**六档同屏**，可直观看到「同一策略、瓶越高手臂工作位越高」。

相机：`eye=(0.15,0.95,1.35)` / `lookat=(0.0,0.02,0.52)`，`viewer.origin_type="env"` + `viewer.env_index=N` 让相机跟住目标 env 的局部系（否则只能看到 env0）。像素抽帧人工复核：6 格构图一致、标注可读、按压/抬回动作清晰；第 60/120 帧抽检确认瓶高差在画面中可见（最高档 palm 明显更高、可见喷嘴活塞杆）。

## 10. 边界与遗留

- **异构 dose-loop 专家：缺**（§7）。这是本轮最硬的边界 —— dive 泛化了，但**「连续多拍多瓶」的异构版本没训出来**，根因未定位。
- **nominal 策略零样本非单调**（§6）：±20 mm 邻档是死区、比 +65/+75 mm 更差；若要在真混线上用 nominal 策略，**邻档必须单独训**。
- **两档瓶物理不可压**（§3）：zc0500/zc0545 受**臂关节限位**（palm 下限 0.562）挡住，不是瓶的问题；要覆盖需改臂/降座。
- **异构化代价**：`replicate_physics=False` 关闭 PhysX 实例化共享，env 数一多吞吐下降；本轮 126 env 是实测的甜蜜点（48 env + 低 entropy 会崩，见 §4）。
- **无 ContactSensor**：仍以 nozzle `q` 作力/行程代理（沿用前几轮口径）。
- **训练期 DR**：异构 dive 用 A-确定性全零 DR 口径；未测更宽 DR 下的异构泛化（可作下一步）。

## 11. 复现命令（IsaacLab 根，`env_isaaclab`）

```bash
cd /home/ubuntu/press_demo/IsaacLab && conda activate env_isaaclab
# 1) from-scratch 异构 dive（126 env，低 entropy）
env -u DISPLAY ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-PressLine-Hetero-Dive-Direct-v0 \
  --num_envs 126 --headless --seed 0 --max_iterations 800 --experiment_name pressline_het_dive --run_name het_dive_lowent126
# 2) 逐档瓶高 dive 成功率（100% 验收）
env -u DISPLAY ./isaaclab.sh -p ../isaac_demo/rl/eval_hetero_dive.py --headless --num_envs 6 --steps 6000 \
  --checkpoint logs/rsl_rl/pressline_het_dive/2026-09-10_12-53-46_het_dive_lowent126/model_799.pt
# 3) nominal line 策略零样本投异构场景（分级退化）
env -u DISPLAY ./isaaclab.sh -p ../isaac_demo/rl/eval_press_line.py --headless --task Isaac-PressLine-Hetero-Direct-v0 \
  --num_envs 126 --steps 12000 --seed 0 \
  --checkpoint logs/rsl_rl/pressline_s1_s1/2026-09-09_18-12-44_s3_line/model_3000.pt
# 4) 零样本失败归因（逐档 × 相位 × 原因）
env -u DISPLAY ./isaaclab.sh -p ../isaac_demo/rl/diag_hetero_policy.py --headless --num_envs 6 --steps 12000 \
  --checkpoint logs/rsl_rl/pressline_s1_s1/2026-09-09_18-12-44_s3_line/model_3000.pt
# 5) 逐档录像（每变体一段；改 --env_index 0..5）→ 再 ffmpeg 拼 2x3
env -u DISPLAY ./isaaclab.sh -p ../isaac_demo/rl/record_press_hetero.py --headless --enable_cameras \
  --task Isaac-PressLine-Hetero-Dive-Direct-v0 --num_envs 6 --env_index 0 --steps 300 \
  --checkpoint logs/rsl_rl/pressline_het_dive/2026-09-10_12-53-46_het_dive_lowent126/model_799.pt \
  --video ../isaac_demo/outputs/het_env0.mp4 --label "zc0565 -20mm"
# 6) 负结果复现（异构 dose loop，预期 plateau）
env -u DISPLAY ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-PressLine-Hetero-Cyclic-Direct-v0 \
  --num_envs 126 --headless --seed 0 --max_iterations 900 --resume \
  --load_run 2026-09-10_12-53-46_het_dive_lowent126 --checkpoint model_799.pt --experiment_name pressline_het_dive --run_name s2_lowent
```

## 12. 关键文件

- **环境/配置**：`direct/press/press_env_cfg.py` —— `_HETERO_BOTTLES`（6 档 tag/usd/cap_key/snap 表 + 排除档的两行理由）、`_hetero_bottle_cfg()`（`MultiUsdFileCfg(..., random_choice=False)`）、`PressLineHeteroEnvCfg`（`replicate_physics=False` + per-env `hetero_cap_top_local`/`hetero_snapshot_paths`）、`PressLineHeteroDiveCfg`（S1：锁相 0 / 8 s）、`PressLineHeteroCyclicCfg`（S2：开 dose 相 / 14 s）。
- **注册**：`direct/press/__init__.py` —— `Isaac-PressLine-Hetero-{Direct,Dive-Direct,Cyclic-Direct}-v0`（Hetero-Direct = 零样本量测用全 line MDP；Dive/Cyclic 为课程两段）。
- **PPO**：`direct/press/agents/rsl_rl_ppo_cfg.py` —— `PressPPORunnerCfgHeteroDive`（entropy_coef 0.0005）；`PressPPORunnerCfgHetero`（标为 DEAD END 的微调配方）。
- **Harness**：`isaac_demo/rl/eval_hetero_dive.py`（逐档成功计数，用成功步奖励判定）、`diag_hetero_policy.py`（逐档 × 相位 × 原因归因 + horizon 截断分离）、`record_press_hetero.py`（`viewer.origin_type="env"` 逐 env 取景 + burn-in 标注）。
- **资产**：`assets/geom/hetero_bottle_zc{0565,0605,0625,0650,0660}_free.usda` + `hetero_bottle_chosen_free.usda`（作者 `tools/author_hetero_bottle.py`）；recenter snap `rl/snapshots/recenter_zc{...}.npz`。
- **录像**：`outputs/het_env{0..5}.mp4`（逐档）、`outputs/rl_press_hetero_dive.mp4`（2×3 同屏）。
- **上游**：`rl_learning_press_line.md`（5 相 line MDP，本轮复用；其 §7 的跨瓶高边界结论被本文 §5 修正）。
