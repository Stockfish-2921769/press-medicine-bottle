# 灵巧手按压药瓶喷嘴仿真 Demo —— 项目进展报告

> 报告日期：2026-09-17
> 任务性质：纯仿真（Isaac Sim 5.1 / Isaac Lab）Demo，独立项目，与 CerebVLA / SomaVLA 无关。
> 当前落地：**PressFlow 补强 round（真捕获判据 + 恒速捕获 A0/A1，完成）** —— 用户口径「流水线的位置和夹爪有穿模，同时应该**训练夹爪自己捕获在流水线上的药瓶**」（见 §14 / `docs/rl_learning_press_flow.md` §6.3–§6.5）。① **穿模先测再改**：新写 `rl/diag_press_flow_clip.py`（world AABB 逐轴间距 + 带↔刀片 15 轴 SAT 精确 MTD + 刀片↔瓶解析 + 生效 `contactOffset`），`--oracle`/`--policy` 双模式跑完**没有任何一对 `pen_depth` 转负** ⇒ 判定伪影、几何不改。② **B 段**：原相 4 门 `jaw≤th ∧ drift_ok` **不含「瓶在位」**、而刀片目标非 3/4 相恒为 0 ⇒ **空爪合拢也记一次 grasp**；加特权几何判据 `_presence_ok()`（刀片盒心投瓶偏移 → `|d.x/y/z|` + 直立度）后 **`empty_close=0`**，grasps 4.71→4.19(−11%) 而 **releases 3.86→3.89 持平**（剔掉的是假抓）、fail_frac 0.117→**0.035**。③ **A 段（带不许停、夹爪自己抓）**：暖启在 S5-B ckpt 上三次跑全部**冻结**在 episode 长度 497.04，确诊为相 4 jaw 动作均值 **+3.8 在 `±1` clip 之外 ⇒ 梯度恒 0**（reward shaping 救不了），改用**夹爪通道定向手术**（`rl/warmstart_center_jaw.py`：清零 jaw 输出行 + 设开通 bias/std + 清该三个张量的 Adam 动量），并配 `catch_grip_dwell=1` 与 `rew_capture_open 0.0035→0.025`（wedge 定价）；A0（带速 0.03）**捕获 3.89 瓶/局 = 静止基线 4.19 的 93%**。④ **A1（产线全速 0.12 m/s，完成）**：同一个 env 只把 `belt_speed` 调回 0.12（记账窗 40→10 步、相 4 deadline 自动 265→115 步），从 A0 ckpt 微调 ⇒ **grasps 4.28 瓶/局，反超静止基线 4.19**、`releases 4.06`、`fail_frac 0.082`（A0 的 1/3）、**`capture_timeout`/`reach_timeout` 双双归零**、`doses at release 5.00/5/5`（n=691）、抓取瞬间 **`bottle_speed mean 0.032 / p90 0.120 m/s`（瓶子以产线全速在动）**。上游：长程流水线（PressFlow）6 相 MDP 每次松开带满 5 拍（见 §13）；异构瓶泛化 6 档全 100%、dose-loop 训不成（见 §12）；流水线换瓶 5.95 瓶/局（见 §11）；闭环连续按压 ~15.9 拍/局（见 §9）；单拍微技能两几何 100%（见 §8）。
> 前序地面真值 demo：**混形双手方案**（Panda 长爪钳自由瓶 + Shadow 掌心朝下压 cap，触觉判读 `USABLE`）与**按压臂等比缩放 s=0.7 + 垫座**重验均通过。
> 明细文档：`docs/feasibility_*.md`（按轮次归档）+ **`docs/rl_learning_press_flow.md`（长程流水线按压）** / `docs/rl_learning_press_hetero.md`（异构瓶泛化） / `docs/rl_learning_press_cycle.md`（闭环连续 RL 总结） / `docs/rl_learning_press.md`（单拍 RL 总结）；脚本/资产/视频见仓库目录。

---

## 1. 项目目标与任务口径沿革

目标一路细化，均经用户拍板：

| 阶段 | 用户口径 / 任务语义 | 结果 |
|---|---|---|
| Phase 1 | 桌上一瓶身，用 KUKA iiwa7 + Allegro 16-DoF **单指（食指）指尖过顶垂直下压**弹簧喷嘴，到底后松指看回弹 | **完成**（base-fixed 口径），见 commit `0c0bdc2` 与 `media/press.mp4` |
| 演进方向 | 换**异形药瓶**（Ø42、参数化弹簧喷嘴），提高语义 =「一次握持内**环抱瓶身 + 按压喷嘴**」 | 逐步收敛出「**混形双手**」方案（见 §4 关键否定结论） |
| 混形双手 | 固定端 Panda 长爪**钳住瓶身**（夹持摩擦 + 桌面承反力），按压端 Shadow **掌心朝下压 cap 顶**，用 pad 接触力 + nozzle 行程做**触觉判读**（未来 palm tactel 代用），并**记录触发瞬间力** | **M1–M4 全链路跑通**（2026-09-08） |
| 缩放 round | 用户目视视频判「按压手所在机械臂过长」→ **等比缩小按压臂物理尺寸**；目标 = 尽量小但够得到原尺寸瓶口；**允许垫高基座**；只缩按压端 | `spawn.scale=0.7` + 0.28 m 垫座，**重验 USABLE**（2026-09-08，见 §6） |
| RL/PPO 学习按压（计划书 Phase 3） | 把规则下压脚本换成 **PPO 真学**按压微技能：课程化两段（A 精确按压 → B 放宽 reset-DR 续训）+ 双几何（s1/s07）都训 + 跨几何零样本 | **完成**（2026-09-08 训练 / 09-09 归档，见 §8 与 `rl_learning_press.md`）|
| 闭环连续按压（PressCycle） | 「**可在自动化流水线部署的连续任务**」：同一 clamp 自由瓶上 **PPO 真学重复多拍**（3 相相位机 PRESS→HOLD→LIFT，门控自动推进），每拍抬离让弹簧完全回弹再压，episode 不因首触底终止 | **完成**（2026-09-09，见 §9 与 `rl_learning_press_cycle.md`）|
| 流水线换瓶按压（PressLine） | 自动流水线上**夹爪抓不同瓶子 → 按压 → 抬升 → 松开本瓶 → 抓下一瓶**、任务连续；换瓶（长爪开/合）**交给 RL 真学**（第 4 DoF），瓶达剂量配额即换 | **完成**（2026-09-09，见 §11 与 `rl_learning_press_line.md`）|
| 异构瓶泛化（PressLine-Hetero） | 「我们尝试用不同瓶子，验证泛化性」→ 拍板**架构改造：每 env 异构瓶**（最高风险档），评测覆盖高度带 5 档 + 高/矮极端 | **部分完成**：每 env 异构瓶架构成立 + 异构 dive **6 档全 100%**；异构 dose-loop 专家**训不成**（负）（2026-09-10，见 §12 与 `rl_learning_press_hetero.md`）|
| 长程流水线按压（PressFlow） | 「没有去做**从发现自动化流水线的瓶子，到抓取到机械臂+灵巧手寻找药瓶并且按压，5 次按压后松开**的过程」→ 拍板：抓取端 = Panda 长爪、范围 = 接近 + **真实 conveyor 抓取**、发现 = **特权观测不加相机**、配额 3→**5** 且要有显式**松开** | **完成**（2026-09-12，见 §13 与 `rl_learning_press_flow.md`）|
| PressFlow 补强（真捕获 + 恒速捕获） | 「流水线的位置和夹爪有**穿模**，同时应该**训练夹爪自己捕获在流水线上的药瓶**」→ 拍板：捕获走 **两段课程 B→A**、穿模由**探针测** | **B / A0 / A1 全部完成**（2026-09-16/17，见 §14 与 `rl_learning_press_flow.md` §6.3–§6.5）：探针判定穿模为伪影、几何不改；B `empty_close=0`；A0 抓移动瓶 3.89 瓶/局 = 静止基线 93%；**A1 产线全速 0.12 m/s 下 4.28 瓶/局，反超静止基线 4.19**、两个超时桶归零 |

---

## 2. 里程碑总表

| 轮次 | 日期 | 对象 / 方法 | 结论 | 产出 |
|---|---|---|---|---|
| Phase 1 食指压 | 09-03/04 | Allegro 单指过顶压直立瓶，base-fixed | ✅ `hit_nozzle_bottom`、回弹≈0、无穿模 | `media/press.mp4`，commit `0c0bdc2` |
| `feasibility_grasp_thumb_press` | 09-04 | 同一次握持内环抱瓶 + **拇指压顶**扫描 | ❌ **几何不可行**（arm 不给「环绕 + 压顶」姿态族）→ Stage1 止损 | 报告 |
| `feasibility_hetero_press` | 09-04 | 方向修正 = **长指按压**，异形瓶扫描 + 选定瓶 | ✅ base-fixed 演示跑通；但「index 压顶 + 兄弟指贴瓶」仍不可达 | `assets/hetero_bottle_chosen.usda` |
| `geometry_envelope_hetero` | 09-05 | 帽顶高度 Z 带 / 横向摆瓶窗口 / 瓶宽敏感性 动态实测 | ✅ 得到几何可行域窗口；「压顶 + 兄弟指贴瓶」任意尺寸不可达（定量不变量） | 报告 |
| `feasibility_shadow_singlehand` | 09-07 | 换 Shadow 24-DoF 单臂单手「握持 + 拇指压」 | ❌ **负结论**（arm 可达族 × Shadow 直链手不匹配）；Option A 挂载旋转亦负；侧向平移解锁 wrap 半边但 thumb 压仍不成 | 报告 |
| `feasibility_mixed_press`（混形） | 09-08 | **双手**：Panda 长爪钳瓶 + Shadow palm 下压 | ✅ 全链路 `USABLE`（先 palm 朝上，用户目视判出 → 修正为 pad 朝下） | `media/mixed_press.mp4` |
| `feasibility_mixed_press`（缩放 round） | 09-08 | 按压臂 `spawn.scale=0.7` + 垫座 | ✅ 重验 `USABLE`（触觉判读同一判据链） | `media/mixed_press_scaled.mp4` |
| **RL/PPO 学习按压（§8 / `rl_learning_press.md`）** | 09-08/09 | PPO（rsl-rl）DirectRLEnv 真学按压微技能；两几何 s1/s07 × A/B | ✅ Stage A 两几何 100%、跨几何零样本双向 100%；Stage B 把 s1 DR 下 93.3→100%（s07 无增益） | `docs/rl_learning_press.md`、`media/rl_press_*.mp4` |
| **闭环连续按压（§9 / `rl_learning_press_cycle.md`）** | 09-09 | PressCycle 相位机 MDP（PRESS→HOLD→LIFT）同瓶多拍；dive→cycle 两段课程 + 命令积分债结构修复 | ✅ s1 ~15.9 拍/局（9985 cycle、回弹 q≈-0.00028、漂移 0.28 mm/拍）、录像单 episode 连续 8 拍 0 reset；s07 零样本部分迁移 ~2.4 拍/局；单拍回归 100% | `docs/rl_learning_press_cycle.md`、`media/rl_press_cycle_s1.mp4` |
| **流水线换瓶（§11 / `rl_learning_press_line.md`）** | 09-09 | PressLine 5 相 MDP：dose 配额 + RL 学长爪 EXCHANGE（换瓶第 4 DoF）；obs31/act4 三段 dive→cyclic→line | ✅ s1 nominal **5.95 瓶/局、3.21 拍/瓶（配额 3）**、87% episode 撑满 1200 步、drift_fail=0；录像单 episode 连续 4 次换瓶 0 reset；单拍回归 100%。边界：跨瓶高 / s07 零样本为负 | `docs/rl_learning_press_line.md`、`docs/rl_press_line_design.md`、`media/rl_press_line_s1.mp4` |
| **异构瓶泛化（§12 / `rl_learning_press_hetero.md`）** | 09-10 | **每 env 异构瓶**（`replicate_physics=False` + `MultiUsdFileCfg`）6 档瓶高同场；from-scratch 异构 dive | ◐ 架构成立；异构 dive **6 档全 100%**（677/677、0 失败、确定性 48–56 步）；零样本投递分级退化（5.95→1.27 瓶/局、fail_frac 89.5%、**非单调**）；**异构 dose-loop 专家训不成（负，根因未定位）** | `docs/rl_learning_press_hetero.md`、`media/rl_press_hetero_dive.mp4` |
| **长程流水线按压（§13 / `rl_learning_press_flow.md`）** | 09-12 | PressFlow 6 相 MDP（旧 5 相语义不变 + 追加 REACH）+ 真传送带 prim + HOME 位姿 + ckpt remap；S4 Reach（带关 quota 3）→ S5 Flow（真带 quota 5） | ✅ **每次松开带满配额 5 拍**（`doses at release 5.00/5/5`，n=2310）、`release_quota_frac=1.000`、drift_fail=falloff=grasp_timeout=0、3.86 瓶/局、fail_frac 11.7%；录像单 episode 全链 0 reset。过程性负结果已修：S5 首跑 99.2% `reach_timeout`（相 3/4 xy 命令无界 → 臂飞到 1.08 m 外起步） | `docs/rl_learning_press_flow.md`、`media/rl_press_flow.mp4` |
| **PressFlow 补强（§14 / `rl_learning_press_flow.md` §6.3–§6.5）** | 09-16/17 | ① 穿模**探针定位**（`diag_press_flow_clip.py`：world AABB 逐轴间距 + 15 轴 SAT MTD + 生效 `contactOffset`）；② **B 段**相 4 加特权几何「在位」判据 `_presence_ok()` + `rew_presence`；③ **A 段** `belt_catch` 恒速带 + 夹爪通道**定向手术**（暖启零梯度 wedge 修复），A0（0.03 m/s）→ A1（0.12 m/s 产线全速）微调 | ✅ 探针判定**无真穿模**（共面伪影，几何不改）；B：**`empty_close=0`**、grasps 4.71→4.19(−11%) 而 releases 3.86→3.89 **持平**、fail_frac 0.117→**0.035**、`doses at release 5.00/5/5` 保持；A0：**grasps 3.89 瓶/局 = 静止基线 4.19 的 93%**、`capture_timeout` 581/615→**2/110**、抓取瞬间 `bottle_speed 0.019 m/s`；**A1：grasps 4.28 瓶/局（反超静止基线 4.19）、`releases 4.06`、`fail_frac 0.082`（A0 的 1/3）、`capture_timeout`/`reach_timeout` 双双归零、`doses at release 5.00/5/5`（n=691）、抓取瞬间 `bottle_speed mean 0.032 / p90 0.120 m/s`**（A0 遗留的 `reach_timeout` 与 |bx| 外沿担忧**均被 A1 消解**） | `docs/rl_learning_press_flow.md`、`media/rl_press_flow_catch.mp4` |

> 注：任务计划书为 5 阶段（Phase 0 环境 / 1 场景 / 2 IK / 3 RL·PPO / 4 集成·视频·报告）。
> **Phase 3（RL/PPO 学习按压）已于 2026-09-09 落地**（真学策略，非规则脚本）：单拍微技能 MDP 表/结果/边界见 §8 与 `docs/rl_learning_press.md`；**闭环连续按压（PressCycle，同瓶多拍）见 §9 与 `docs/rl_learning_press_cycle.md`**；**流水线换瓶按压（PressLine，剂量配额 + RL 学长爪换瓶）见 §11 与 `docs/rl_learning_press_line.md`**；**长程流水线按压（PressFlow，真传送带 + 抓取 + 寻位 + 5 拍 + 放行）见 §13 与 `docs/rl_learning_press_flow.md`**。
> 上面的逐轮可行性迭代属于「动作/几何定义」阶段。详见本项目 memory `project_press_demo.md`。

---

## 3. 当前最终方案（混形双手，palm-down + 缩放 round）

### 3.1 场景布局（世界坐标）

| 角色 | 基座 xy | 朝向 | 说明 |
|---|---|---|---|
| Bottle（自由站） | (−0.545, −0.026) | — | `hetero_bottle_chosen_free.usda`：Ø42、cap 顶 world z≈0.585、弹簧 K300/maxForce6/行程 5 mm |
| Presser（iiwa7+Shadow） | s=1.0: (−0.05, −0.03)；**s=0.7: (−0.205, −0.026, 垫高 0.28)** | yaw=−90 | `kuka_shadow.usda`；palm 装 `ContactSensor`；**pad 外法向朝下** |
| Riser（缩放 round） | (−0.205, −0.026) | — | 静态 `CuboidCfg`(kinematic) 立柱，顶面 z=0.28 |
| Panda 长爪 | 西侧 (−1.119, −0.026) | 朝 +x | `panda_longjaw.usda`；从 −x 接近钳瓶身中段 z≈0.515 |

**双机方位是硬结论**：palm 下压时 wrist 落在瓶北(y+)柱，Panda 从北侧钳瓶会堵死 wrist 下探柱（err~70–200 mm NOREACH）→ Panda 放**西侧**，presser 从**东侧**下探，err 4–7 mm。

### 3.2 demo 序列（`scripts/demo_mixed_press.py`）

1. **Panda 钳瓶**：瓶暂移远 → 静态多 seed 刀片骑跨空座位 → 瓶瞬移回 home 进开爪 → 闭爪钳住（`CLAMP_OK`）。
2. **Palm hover（含 `[recenter]`）**：pad 朝下悬停 cap 正上方 30 mm（DLS 多 seed IK，err mm 级）。hover 扫掠会把被钳瓶撞偏 ~5 mm → **下探前重读实测瓶位重建 hover**。
3. **−z 下探压**：pad 压 cap → nozzle 行程到底 → 稳住 → 抬回看回弹；palm `ContactSensor` 全程读 pad 法向力，到底瞬间快照 `palm_force_at_trigger`。
4. **松爪撤走**：量瓶漂移。

### 3.3 关键量测（两轮对比）

| 量 | s=1.0（palm-down） | s=0.7 + 垫座 0.28 |
|---|---|---|
| 钳 | `CLAMP_LONGJAW` zgrab 0.515、seat_err 5 mm、钳移 3.4 mm | seat_err 0 mm、钳移 0.1 mm |
| hover | `PALM_IK` err 4 mm、cos_down +0.999 | err 1 mm、cos_down +0.997 |
| recenter | 扫掠撞偏 5.4 mm → re-aim err 3.9 mm | 5.0 mm → err 1.1 mm |
| 触底 | nozzle −0.00423（85% travel），漂移 0.02 mm | −0.00412，漂移 0.09 mm |
| 稳住 | nozzle −0.00455（91%），漂移 0.02 mm，**pad 力 1.19 N** | −0.00461（92%），漂移 0.21 mm，**pad 力 1.13 N** |
| 触发 | `TRIGGER` pad 1.19 N / 弹簧反力 1.37 N | pad 1.13 N / 弹簧反力 1.38 N |
| 抬回回弹 | −0.131 mm ≈ 0 | −0.149 mm ≈ 0 |
| 判决 | `PRESS_BOTTOM` 五项全 True → **`tactile_verdict=USABLE`** | 同 → **`USABLE`** |
| 收尾 | `[jaw_open]` 漂移 0.04 mm | 漂移 0.13 mm |

两轮同量级。**触觉判读判据**：pad 法向力 >0.5 N（压到 cap）→ nozzle 行程到底 ≥−travel×0.8 → 抬回回弹≈0（弹簧未卡死）→ 瓶漂移 <2 mm（夹持稳）→ `USABLE`；任一不满足即 `NOT_USABLE`（判据在代码，负例分支本轮未触发）。**触发力矩口径**：pad 法向接触力 + 弹簧回位反力 −K·q 同时打印，作为「未来 palm tactel 判断喷嘴可用性」的可验证落点。

---

## 4. 关键否定结论（为什么最终走到「混形双手」）

- **单手「一次握持内环抱 + 压顶」姿态族不存在**（Allegro 拇指压 / 长指压、Shadow 单臂拇指压全部负结论）：iiwa7 在瓶位附近只给「过顶 / 斜顶下压」姿态族，不给出「palm 低到瓶侧、指尖横过瓶轴的环绕」族；Shadow 又是 link7 伸出 ~0.46 m 的直链手，被压顶族带过去后指尖落在远高于瓶身的高处。单独换更灵巧的手修不了「臂工作空间 / 尺寸」类瓶颈。
- **放弃「一次握持内按压」**：改为**双手分工**——Panda 平爪真钳瓶（固定端，替代旧基线 `demo_bimanual.py` 里「近距扶」这种包不住 Ø42 的假 fence），Shadow 专注压顶。按压反力由夹持摩擦 + 桌面承担。
- **palm 朝向修正**：MCP 平面法向的拇指提示给的是手背外法向，首版 demo 出现「掌心(pad)朝上、手背压 cap」。修正为 `n_w=−n_w` 后 pad 朝下压；可达 cell 镜像切换，默认 `--yaw=-90`，三处脚本（demo/`probe_mixed_reach`/`probe_palm_press`）逐行一致。
- **hover 平面教训**：demo 下探 hover 平面是 **30 mm**，probe 默认 6 mm → 边界判定必须以 demo 实际平面扫（`--hover_mm 30`），否则假 FEAS（曾导致一次 NOREACH）。

---

## 5. 缩放 round 结论（为什么 s=0.7 是最小可行）

- **机制**：`UsdFileCfg.spawn.scale=(s,s,s)` 运行时等比缩放，无需重造 usda。Gate A 实证：body 位姿随 s 缩放（palm0 名义 z≈0.75·s）、joint/body 名不变、`ContactSensor` 正常建、pad 仍朝下。显式 `physics:mass` 不随 s 自动缩放（密度变大），demo 为 disable_gravity 准静态力驱，gate 判 >0.5 N 不受影响。
- **扫描**（`probe_mixed_reach.py` 扩 `--scale/--zlist/--hover_mm`，判据 pad-down err<18 mm 且 cos_down>0.95）：

| s | 基座垫高 bz | 基座离 cap r | 结论 |
|---|---|---|---|
| 1.0 | 0 | 0.50 | FEAS（对照） |
| 0.8 | 0.20 | 0.42 | FEAS |
| 0.7 | 0.20–0.28 | 0.34 | FEAS（hover 30 mm 平面；r≥0.38 NO） |
| 0.6 | 0.10–0.42 | 0.32–0.44 | NO_FEAS：够不到 + 垫高到 cap 高度时 pad-down 反转（palm 朝上） |

- **几何本质**：cap 固定原世界位、臂缩 s 后地板基座必够不到 → 基座要垫高并移近 cap；但垫到 cap 高度附近接近水平侧向、pad 无法朝下。**s=0.7 = 干净离桌挂载下能 hover(30 mm) 又能下探压的最小缩放**。
- **选定装具**：`--scale 0.7 --px -0.205 --py -0.026 --pz 0.28`（离桌东沿 ~40 mm），riser 顶面 z=0.28。质量按 s³ 标定、短臂 hover 扫掠撞偏等未专门处理（遗留，见 §7）。

---

## 6. 复现

环境：Isaac Sim 5.1.0.0 (pip) / Isaac Lab `main`，运行方式 `./isaaclab.sh -p <脚本>`；脚本内资产路径为相对路径（`scripts/` 与 `assets/` 同层），克隆后即可运行（需把脚本放到与 `assets/` 同层，或按仓库布局直接以绝对路径运行）。

```bash
cd <IsaacLab 根目录>   # 例如 /home/ubuntu/press_demo/IsaacLab
conda activate env_isaaclab

# A. 混形双手 palm-down（s=1.0 基线）
env -u DISPLAY timeout 1500 ./isaaclab.sh -p <repo>/scripts/demo_mixed_press.py --headless \
  --enable_cameras --ns 10 --step 260 --yaw=-90 --video media/mixed_press.mp4 > logs/demo_mixed_press.log 2>&1

# B. 缩放 round（按压臂 ×0.7 + 0.28 m 垫座）
env -u DISPLAY timeout 1500 ./isaaclab.sh -p <repo>/scripts/demo_mixed_press.py --headless \
  --enable_cameras --ns 10 --step 260 --yaw=-90 --px -0.205 --py -0.026 --pz 0.28 --scale 0.7 \
  --video media/mixed_press_scaled.mp4 > logs/demo_mixed_press_scaled.log 2>&1

# 判读 / 量测
grep -aE 'CLAMP_LONGJAW|PALM_IK aim|recenter|first contact|nozzle bottom|\[held\]|TRIGGER|PRESS_BOTTOM|tactile_verdict|\[jaw_open\]|DONE' logs/demo_mixed_press_scaled.log
# 期望: CLAMP_OK / PALM_IK ... yaw=-90 ... REACH / [recenter] ... re-aim err ~1mm /
#       first contact ... / nozzle bottom ... palm_force_at_trigger=1.13N /
#       [held] drift=0.21mm / TRIGGER ... palm_force=1.13N spring_reaction=1.38N /
#       PRESS_BOTTOM=True / tactile_verdict=USABLE / DONE
```

M1 可达性/缩放扫描：`scripts/probe_mixed_reach.py --headless --scale <s> --zlist ... --hover_mm 30`，输出 `VERDICT_SCALE`（详见 `feasibility_mixed_press.md` §9）。

> Isaac Sim 关停常挂：`DONE`/`[video]` 后数据已出但 close() 不返回，孤儿按 PID kill，重跑前清 GPU。demo `--headless` 视频渲染需离线环境（本机 `env -u DISPLAY`）。重跑两次数值一致（确定性）。

---

## 7. 产出与仓库布局

```
scripts/                        # 全部可运行脚本（含 Phase 1）
  demo_mixed_press.py           # 混形双手 palm-down demo（含 --scale/--px/--py/--pz）
  _kuka_shadow_cfg.py           # iiwa7+Shadow 单条 articulation 配置
  _panda_longjaw_cfg.py         # Panda + 自建 ALOHA 式长爪配置
  probe_mixed_reach.py          # 混形 / 缩放可达性扫描（--scale/--zlist/--hover_mm）
  phase1_press.py 等            # Phase 1 老脚本（历史弧线）
assets/
  kuka_shadow.usda              # 按压臂 = 薄引用 kuka.usd + shadow_hand.usd
  panda_longjaw.usda            # Panda 长爪
  hetero_bottle_chosen_free.usda# 自由站异形瓶（Ø42、弹簧喷嘴）
  medicine_bottle.usda          # Phase 1 直立瓶（保留）
tools/                          # USD authoring 工具（生成上述资产）
docs/
  PROGRESS.md                   # 本文（项目进展总览）
  rl_learning_press_cycle.md    # 闭环连续按压 round：同瓶多拍相位 MDP 总结（2026-09-09）
  rl_learning_press_hetero.md   # 异构瓶 round：每 env 异构瓶 + 6 档瓶高泛化 + dose-loop 负结果（2026-09-10）
  rl_learning_press.md          # RL/PPO 学习按压 round：MDP 一览表 + 总结（2026-09-09）
  feasibility_grasp_thumb_press.md     # 09-04 Allegro 握持+拇指压 → 负
  feasibility_hetero_press.md          # 09-04 异形长指按压 / 选定瓶 → 正(演示口径)
  geometry_envelope_hetero.md          # 09-05 几何可行域反推
  feasibility_shadow_singlehand.md     # 09-07 Shadow 单臂握持+按压 → 负
  feasibility_mixed_press.md           # 09-08 混形双手 + palm-down 修正 + 缩放 round + RL round §10
media/
  press.mp4                   # Phase 1 食指按压（20.5s 960×540）
  mixed_press.mp4             # 混形双手 palm-down（21.15s 960×540@20fps）
  mixed_press_scaled.mp4      # 缩放 round s=0.7（~20.8s 960×540@20fps）
  rl_press_s1.mp4 / rl_press_s07.mp4 / rl_press_s1B.mp4  # RL 单拍策略滚动（1280×720@60fps）
  rl_press_cycle_s1.mp4       # PressCycle 同瓶连续 8 拍 0 reset（1280×720@60）
  rl_press_line_s1.mp4        # PressLine 单 episode 连续 4 次换瓶 0 reset（1280×720@60）
  rl_press_hetero_dive.mp4    # PressLine-Hetero 同一策略 6 档瓶高同屏 2×3 拼图（1920×1620@60）
```

> 完整工作目录（含全部 58 个探针脚本、逐帧 PNG、扫描日志）在本机 `/home/ubuntu/press_demo/`（`isaac_demo/`、`outputs/`、`logs/`），仓库同步的是**关键产出**子集。

---

## 8. RL/PPO 学习按压 round（2026-09-09）

> 自包含总结 + **MDP 一览表**见 `docs/rl_learning_press.md`；阶段内完整训练/诊断叙述见 `feasibility_mixed_press.md` §10。这里只放结论与关键事实（数值以 done 尖峰计数，`rl/eval_press.py`）。

- **学什么**：把混形 demo 的规则下压换成 **PPO 真学**按压微技能 —— episode 从离线钳位快照 `rl/snapshots/recenter_s{1.00,0.70}.npz`（两几何各一）出发，策略输出 **3-DoF 世界系 task-space 残差**（clip±1、0.5 mm/步 @60 Hz，每步 1-step DLS→iiwa7 7 臂关节，手指锁名义）把掌心压到 nozzle 到底（`≤−0.0045`，91% travel）并稳住、不推偏瓶。**Panda 钳瓶 / Shadow 手指 / 抬回回弹不进训练**。obs **25 维全相对**；reward = 下压进展 `+0.25/mm` + 贴底带稳住 `+0.4` + 成功 `+10` + fail `−2` + 漂移 `−0.3/mm` + 动作率 `−0.02` + 臂速 `−0.01`（nozzle 关节位 q 作力/行程代理，力≈−K·q、K=300）。
- **结果**：Stage A 两几何都快速收敛（<250 iter，128 envs SPS≈3300–3700）：s1 自评 **1.000**（2292）/ s07 零样本 **1.000**（2016）；s07 自评 **1.000**（2008）/ s1 零样本 **1.000**（2197）→ **跨几何双向 100%**。Stage B resume-A、只放宽 reset-DR（瓶 xy±0.5 mm/yaw 0.8°、臂关节 ±0.02 rad、基座 z±4 mm、命令 xy±1.5 mm）：把 s1 在 B(DR) 分布下从 A 的 **93.3%** 补到 **100%**（B_m1400，2241/2241）；s07 上 A 本身已 100%（2079）、B ≈ 平（98.8%，3004/3041）→ **课程收益几何相关**（只在小横向偏移会漏的 s1 上补 A）。
- **教训（诚实）**：宽 DR 初版（臂关节 ±0.06 rad、cmd-xy ±3 mm）resume-A 崩 + from-scratch 也不学 —— 2 mm 漂移悬崖 + 长 timeout 负累积，直线压穿不过大横向偏移；**按计划风险梯子缩 DR 后成立**。判别法 = 先 eval「A 在 B 分布上零样本」定分布可学性，再决定 resume / from-scratch / 缩 DR。
- **代码落点（不入本仓库，依赖本机 IsaacLab editable 布局）**：env 包 `IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/press/`（4 task id：`Isaac-Press-{Direct,Direct-B,07-Direct,07-B}-v0`）；harness `rl/{smoke,eval,record}_press.py`。训练/评测/录视频复现命令见 `rl_learning_press.md` §7。

## 9. 闭环连续按压 round（PressCycle，2026-09-09）

> 自包含总结 + **MDP/相位机一览表**见 `docs/rl_learning_press_cycle.md`；根因诊断叙述见 `feasibility_mixed_press.md` §11。这里只放结论与关键事实（cycle 完成数 = `rl/eval_press_cycle.py` 逐拍读 `env.cycle_count` 遥测，非 done 事件）。

- **学什么**：把「压到底即终止」的单拍微技能升级为**闭环连续任务（用户口径：可在自动化流水线部署）**——同一 clamp 自由瓶上 **PPO 真学重复多拍**，每拍都需掌心抬离、弹簧完全回弹（`|q|<0.0008`）再压；episode 不因首触底终止。3 相相位机 PRESS→HOLD→LIFT 由**本步物理后状态门控自动推进**（策略职责 = 真驱动含真抬回，门无法伪造）。
- **结构与奖励修复（5 次停滞的共同根因）**：`cmd_pos.z` 在压底过程中无条件下积 → 命令埋到掌心下方几十 mm，LIFT 先爬「幽灵深度」才能真抬（对 PPO 不可探索）。修复 = ①每个相位过渡把命令位/姿态**重锚到掌心真实位姿**；②**LIFT 相专属命令深度下限**（`cmd_z ≥ palm_z − 0.0025`），压入 cap 的停滞不再埋命令。`probe_release_latency.py` 实测压底 87 步/0 赤字、80 步停滞后再抬 ~33 步完全回弹。
- **训练（两段 resume）**：S1 dive 单拍锁相 warm-start → S2 开全相位机（LiftEasy 档 `h_return=0.012`，物理上已足够：接触 ~11.2 mm 断开、回弹完全）。s1 收敛 ~+28..+42，final `model_3199`（+38.6）。
- **结果（s1）**：**~15.9 拍/局**（627 局共 9985 次完整 cycle）、cycle 完成瞬间回弹 `q≈-0.00028`（完全回弹 ✓）、瓶漂移 **0.28 mm/拍**；单 episode **同一瓶连续 8 拍 0 reset** 录像（`media/rl_press_cycle_s1.mp4`，1280×720@60、409 帧）。s1 策略零样本 → s07（同 easy-LIFT 门）：**~2.4 拍/局**（回弹干净 -0.00034、漂移 0.55 mm/拍，正的部分迁移）。单拍旧策略回归仍 **1.000**（3415/3415）。
- **代码落点（不入本仓库，依赖本机 IsaacLab editable 布局）**：`direct/press/press_cycle_env.py` + `PressCycle*` cfg（`press_env_cfg.py`）+ 新增 task id `Isaac-PressCycle-{Dive,Direct,LiftEasy,07-Direct,07-LiftEasy}-Direct-v0`；harness `rl/{eval_press_cycle,record_press_cycle,probe_release_latency}.py`。

## 10. 遗留 / 边界 / 建议下一步

- **PressFlow round 边界（详见 `rl_learning_press_flow.md` §9）**：① 成品瓶离场是 **kinematic 复演 + 单瓶 prim 复用**，不是真双瓶供应链（进料段是**摩擦拖动真物理**，倾倒余量 30×）；② 「发现」是**特权观测**（`drift` = 进料距离），**无相机**；③ 抓取仍是摩擦夹持（无 ContactSensor，力以 nozzle q 代理）；④ S5 放行密度低于 S4（3.86 vs 5.71 瓶/局，真带换瓶 ~475 步/瓶）；⑤ `reach_timeout` 仍是首位失败（53/599）；⑥ **s07（riser）未做**（带与 riser 几何要重排）；⑦ 训练后期方差 ⇒ ckpt 按直接 eval 选。
- **异构瓶 round 边界（详见 `rl_learning_press_hetero.md` §10）**：① **异构 dose-loop 专家缺** —— dive 泛化了，但「连续多拍多瓶」的异构版本**没训出来**（各路径全 plateau，死因清一色 LIFT 超时 `q=-0.005` 从不抬回），**根因未定位**；② **nominal 策略零样本非单调** —— ±20 mm 邻档是死区（fail 98%↑），比 +65/+75 mm 差得多，真混线用 nominal 策略须**邻档单独训**；③ **两档瓶物理不可压** —— zc0500/zc0545 受**臂关节限位**（palm 下限 z≈0.562）挡住，非瓶问题；④ 异构化代价 —— `replicate_physics=False` 关闭 PhysX 实例化共享，126 env 为本轮甜蜜点（48 env + 低 entropy 会崩）；⑤ 无 ContactSensor（nozzle q 作力代理）；⑥ 未测更宽 DR 下的异构泛化。
- **PressCycle 边界（详见 `rl_learning_press_cycle.md` §10）**：canonical 25 mm 抬升档未训成正奖励（12 mm 已物理足够，LiftEasy 为主交付）；**多拍累计漂移** —— 单 clamp 连续 ~16 拍后越 8 mm fail 带（0.28 mm/拍随机游走），s07 0.55 mm/拍 → 部署需 ~≤10 拍 re-dose/对心节奏，或未来按 cycle 放宽 fail 包络；s07 零样本为正的部分迁移（吞吐下降）；训练无 ContactSensor（nozzle q 作力代理）。
- **RL round 遗留边界（详见 `rl_learning_press.md` §8）**：Stage B 宽 DR 初版不收敛、缩 DR 后成立；训练用 nozzle q 作力代理（真实 palm 力需单 env ContactSensor 复算）；抬回/回弹不训、eval 补判；奖励面「2 mm 漂移悬崖 + 长 timeout 负累积」限制更宽 reset 域（若需更宽，加 no-progress 终止 / 训练期放宽 drift gate / eval 收紧）。
- **缩放 round 边界**：(a) 相机取景沿用 s=1.0 的 eye，短臂 + 0.28 m 垫座在帧内偏小，若想强调「臂变短」可拉近/降 eye 重出；(b) 质量未按 s³ 重标定（密度变大），gate 不受影响但接触力标定非本轮目标；(c) s≈0.6 需贴桌沿 + 高垫且 pad-down 退化，不取。
- **钳移 / approach 撞击**：jaw 闭合带 ~0.1–3.4 mm 钳移，hover 多 seed 扫掠把被钳瓶撞偏 ~5 mm（`[recenter]` 已把按压期漂移压到 <0.3 mm）；若要求全程漂移 <2 mm，需消除扫掠撞击。
- **palm 触觉粒度**：现为 `ContactSensor` 法向合力代用；真实 palm tactel 阵列→压力图建模未做，接口位已留。
- **Panda 未缩**：夹持口径（爪口/Ø42 瓶比）要求不变；若日后想整幅协调，Panda/瓶同步缩是另一口径。
- **相机取景**：本会话后端非多模态，无法逐帧目检构图；若需精调机位请人工抽帧确认。

## 11. 流水线换瓶按压 round（PressLine，2026-09-09）

> 用户口径：自动化流水线上**夹爪抓不同瓶子 → 按压 → 抬升 → 松开本瓶 → 抓下一瓶**、任务连续。设计 = `docs/rl_press_line_design.md`；自包含总结 + **5 相 MDP/奖励表** = `docs/rl_learning_press_line.md`。这里只放结论与关键事实（瓶/局、拍/瓶 = `rl/eval_press_line.py` 读 `bottle_count/cycle_count` 遥测）。

- **学什么**：在 PressCycle 的 dose 相位机（每瓶配额 `quota=3` 次完整按压 cycle）之上加 clamp **EXCHANGE**（3 EXCH_OPEN 张爪放瓶 → 新瓶就位 conveyor 事件 → 4 EXCH_CLOSE 合爪抓稳 → 回 PRESS），**长爪开/合是第 4 个真学 action DoF**（a_jaw∈[-1,1]；剂量相锁死闭合，仅换瓶相解锁）。门全部传感器确定性推进（nozzle q / 刀缝 joint / palm 高 / 瓶 drift），策略真学驱动换瓶，非脚本。obs 31 = base 26 + 5 相 one-hot；act 4。抓稳 = 刀缝≤close_th ∧ drift<2mm 连续 5 步 → `bottle_count++`、lump `+10`。
- **训练（三段 resume，obs 全程 31 维同形）**：S1 dive（相位锁 0、jaw 锁闭 = 单拍语义）→ S2 cyclic（开全 0/1/2 = PressCycle 循环）→ S3 line（开 quota + EXCHANGE）。**复现要点**：S2 从已收敛 dive 的 `model_799` resume 卡负区（~-13 plateau），改从**早期高熵 `model_200`** resume 才收敛（沿用 PressCycle 先例，未收敛 dive 更利于学抬回）；S3 训练 reward 双峰摆荡（60↔160），**以直接 eval 的 bottle_count 选 ckpt**（`model_3000` 胜出）。
- **结果（s1 nominal，model_3000，64 env×20000 步）**：**5.95 瓶/局**（6689 瓶 / 1124 局）、**3.21 拍/瓶（配额 3，达额即换）**、episodes_trunc 982（87.4%，撑满 1200 步 horizon）vs fail 142（falloff 105 / 超时 37 / **drift_fail 0**）、96.1% 局至少换 1 瓶、回弹 q≈-0.00028、剂量瞬间漂移 0.25 mm。录像 `media/rl_press_line_s1.mp4`：单 episode **连续 4 次换瓶、0 reset**（配额 3/6/9/12 于 step 225/386/542/698、换瓶 231/392/548/704；1280×720@60、704 帧）。单拍回归仍 **1.000**（2332/2332）。
- **边界（诚实）**：① 跨瓶高零样本为负 —— zc0565(短 20mm) 0.43 瓶/局(超时主因)、zc0605(长 20mm) 0.07 瓶/局(falloff 主因)；s07(0.7+riser) 0.01 瓶/局。策略的绝对任务空间残差按 nominal snap 校准，不跨几何/尺度泛化（要「不同瓶子」混线需逐几何训练或几何 DR）。② 混合几何单段连续 take 不受 instanced-env 架构支持（GPU instanced 同模板同几何 + 流水线串行单工位）。**→ 已于 09-10 异构瓶 round 推翻**：`replicate_physics=False` + `MultiUsdFileCfg` 可让每 env 一个不同瓶 USD（见 §12）。③ 换瓶「离/到」为 conveyor 事件建模（写平衡位 + drift 归零），sim 无传送带动画；爪开/合时机与抓稳仍真学。④ ~12.6% 局以 falloff/超时告终，多在 horizon 附近。⑤ 训练不稳定（S3 双峰摆荡）。
- **代码落点（不入本仓库，依赖本机 IsaacLab editable 布局）**：`direct/press/press_line_env.py` + `PressLine*` cfg（`press_env_cfg.py`）+ task id `Isaac-PressLine-{Dive,Cyclic,Direct,56-Direct,60-Direct,07-Direct}-v0`；harness `rl/{smoke,eval,record}_press_line.py`；瓶 snap `rl/snapshots/recenter_{s1.00,s0.70,zc0565,zc0605}.npz`。复现命令见 `rl_learning_press_line.md` §9。

## 12. 异构瓶泛化 round（PressLine-Hetero，2026-09-10）

> 用户口径：「我们尝试用不同瓶子，验证泛化性」→ 拍板走**架构改造：每 env 异构瓶**（最高风险档），评测覆盖 **高度带 5 档 + 高/矮极端**。自包含总结 = `docs/rl_learning_press_hetero.md`。这里只放结论与关键事实（逐档成功率 = `rl/eval_hetero_dive.py` 用成功步奖励判定；零样本吞吐 = `rl/eval_press_line.py` 读 `bottle_count/cycle_count`）。

- **架构（本轮核心）**：`InteractiveSceneCfg(replicate_physics=False, clone_in_fabric=False)` + `bottle_cfg.spawn = MultiUsdFileCfg(usd_path=[6 个瓶 USD], random_choice=False)` → **env i 拿 variant i%6**（确定性轮转、逐档均匀）。因 MDP 从设计起就用**几何相对量**（`palm_above = palm_z − (root_z + cap_top_local)`），异构化只落到资产层 + 把 `cap_top_local` 从常量变 per-env 张量，obs/act 形状与门控定义**零修改**。**推翻了 §11 的边界②**。
- **正结果（头条）**：**from-scratch 异构 dive 策略 6 档瓶高全部 100% 成功** —— `eval_hetero_dive.py` 6 env×6000 步：**677/677 次按压**、逐档 `episode-ends == successes`、`falloff=drift=timeout=0`、`mean==min` 完全确定性；触底步数随瓶高**单调**（最高 zc0660 48 步 → 名义/最矮 55–56 步）。**这修正了 `rl_learning_press_line.md` §7「跨瓶高零样本为负」**：负的是「nominal 策略不迁移」，只要在混合场景**从零训**，单策略即覆盖 95 mm 瓶高跨度、逐档 100%。
- **训练（from-scratch 而非微调）**：nominal→异构**微调两次都发散**（adaptive / fixed 低 LR 同样，同 `Press1EnvCfgB` 的「放宽分布即崩」签名）→ 改在混合场景从零重训 dive。**关键教训**：默认 cfg（entropy_coef 0.005）下 48 env 版**也是 100% 成功但 action std 从不 anneal**（entropy loss 平在 ~5.7）；把 entropy_coef 降 10× 到 0.0005 + 126 env，std 才 **0.99→0.29**（回到 nominal 收敛 regime），**交付 ckpt = `pressline_het_dive/2026-09-10_12-53-46_het_dive_lowent126/model_799.pt`**。「任务学会了」与「策略收敛了」是两件事，只看 reward 曲线看不出差别。
- **分级结果（零样本）**：nominal line 策略（`pressline_s1_s1/…/s3_line/model_3000.pt`）直接投异构场景，126 env×12000 步：**瓶/局 5.95→1.27**、fail_frac 12.6%→**89.5%**；**但「达额即换」正确性没丢**（换瓶成功时仍是 3.21–3.73 拍/瓶 ≈ 配额 3）。退化**非单调**：zc0605(+20mm) 201 瓶/99.6% 失败、zc0565(−20mm) 428/98.0% —— **±20 mm 邻档是死区，比 zc0650(+65)/zc0660(+75) 的 83%/61% 差得多**（设计 §7 曾预期邻档零样本高，实测恰恰相反）。**且两档病根相反**（`diag_hetero_policy.py`）：zc0565 **压得到底但卡在 HOLD 停不住**（61 次 HOLD 超时 @ `q_min=−0.005`），zc0605 **压根没压上**（55 次 PRESS 超时 @ `q_min=−0.00033`≈0 行程，env 记为 `falloff`，全场 61 次 falloff 基本都在此档）；其余偏高档多为「压到但 HOLD 停不住」。
- **负结果（诚实，根因未定位）**：**异构 dose-loop 专家训不成** —— nominal→het 微调 ×2、het dive→S2/S3 resume（起点 model_100/200/799）、乃至用 entropy 已收到 0.53 的 low-ent 起点，**全部 plateau**（reward/步 落到 **−37…−102**、ep_len 涨到 190–330），**死因清一色 LIFT 超时死在 `q=-0.005`：压到底后从不抬回**（同 `rl_learning_press_line.md` §5 那个病）。已逐一实测**排除**：环境 bug（diag 全档能压到底 `q_min=-0.00500`）、学习率（adaptive 3e-4 vs fixed 5e-5 一样）、探索（低熵起点照样，且 entropy 反涨 0.53→1.02，说明是「探索中持续失败」非冻结）、脚本 oracle（oracle 只最矮档 `cycles min=0`，env 侧无碍）。`PressPPORunnerCfgHetero` 保留并标为 DEAD END 记录。
- **两档瓶物理不可压**：zc0500(cap 0.500)/zc0545(0.545) 被排除 —— **臂关节限位**（palm 原点最低 `z≈0.562`）挡住，非碰撞、非瓶的问题（压座降 35 mm 也无效）；`zc0565` 即最矮可压瓶。
- **回归**：旧单拍 `Isaac-Press-Direct-v0`（`press_A_s1…model_900.pt`）**success_rate 1.000**（2332/2332，首触底 ~54.5 步）。本轮只新增 `PressLineHetero*`，基类/旧 task/旧 eval/record 未破。
- **录像**：`media/rl_press_hetero_dive.mp4` —— **同一策略 6 档瓶高同屏** 2×3 拼图（`zc0565 -20mm` → `zc0660 +75mm`，1920×1620@60、294 帧）；逐档源片段 `outputs/het_env{0..5}.mp4`（每段按压 5/5/5/5/5/6 次）。**踩坑**：按压计数不能用 q 穿越底判定 —— `step()` 内部就 auto-reset，读 q 永远看不到子阈值值，必须用**成功步奖励** `rew≥9`。
- **代码落点（不入本仓库，依赖本机 IsaacLab editable 布局）**：`direct/press/press_env_cfg.py`（`_HETERO_BOTTLES` 表 + `_hetero_bottle_cfg()` + `PressLineHetero{Dive,Cyclic,}EnvCfg`）、`__init__.py`（`Isaac-PressLine-Hetero-{Direct,Dive-Direct,Cyclic-Direct}-v0`）、`agents/rsl_rl_ppo_cfg.py`（`PressPPORunnerCfgHeteroDive`）；harness `rl/{eval_hetero_dive,diag_hetero_policy,record_press_hetero}.py`；资产 `assets/geom/hetero_bottle_zc{0565,0605,0625,0650,0660}_free.usda`；复现命令见 `rl_learning_press_hetero.md` §11。

## 13. 长程流水线按压 round（PressFlow，2026-09-12）

> 用户口径：「我们只训练了如何用夹爪抓握，和用机械臂+灵巧手按压药瓶，没有去做**从发现自动化流水线的瓶子，到抓取到机械臂+灵巧手寻找药瓶并且按压，5 次按压后松开**的过程」。前几轮的缺口是**结构性**的：`PressEnv._reset_idx` 直接把 presser 瞬移到瓶子上方 hover、把瓶瞬移到工位（episode 一开始就已夹住 + 已悬停）；「conveyor」只是 `_seat_fresh_bottle` 的瞬时瞬移，场景里没有带 prim、没有传感器。拍板：抓取端 = **Panda 长爪**（presser 只管寻位+下压）、范围 = **接近 + 真实 conveyor 抓取**、发现 = **特权观测不加相机**、配额 3→**5** + 显式**松开**。自包含总结 = `docs/rl_learning_press_flow.md`。这里只放结论与关键事实（拍数/瓶 = `rl/eval_press_flow.py` 读 `doses at release`）。

- **MDP（obs 32 / act 4，6 相）**：**旧相位索引保持原义，新相位只追加到 index 5**（0 PRESS / 1 HOLD / 2 LIFT 与 PressLine 逐字一致；3 EXCH_OPEN = 放行+撤臂+上料；4 EXCH_CLOSE = 抓取；**5 REACH 新** = presser 从 HOME 寻到 hover）。obs = base 26 + 5 维 PressLine one-hot + **1 维 REACH flag**；**REACH 上报 EXCH_CLOSE 的 one-hot**（自占第 6 槽会让前 5 维全零、落在 line 策略分布外，warm-start 会答成「张爪横扫」）。相 2 与 REACH **锁死闭爪**（REACH 语义 = 携带已夹瓶去工位）。动作 = 任务空间残差 3 维 + 长爪标量；**两档尺度**（0/1/2 相 0.5 mm/步旧值不动；3/4/5 相 4 mm/步，否则寻位吃满 horizon），相位切换 re-anchor 到 palm 真实 FK 清掉积分债。
- **HOME 位姿（新增）**：`home_joint_delta` 从 hover 平衡点偏移，`solve_press_flow_home.py` 用 **env 自己的差分 IK 伺服**解成 hover 的**纯平移**（第一版手挑 pitch/roll 让 HOME 姿态 ≠ hover，而 `_cmd_quat` 是 held fixed 的 ⇒ IK 被要求「到 hover 位置同时保持 HOME 姿态」这个不存在的位姿，靠**横向拽 palm** 妥协）。实测 HOME palm = env-local **(2.3, 3.7, 753.8) mm**（几乎正对工位上方），比 cap 高 **164 mm**。
- **真传送带**：`RigidObjectCfg` + `CuboidCfg` slab（kinematic、disable_gravity），顶面比桌面高 `BELT_LIFT=4 mm`；**进料 = 摩擦拖动真物理**（`_write_belt_pose` 同时写位姿钉轨 + 写速度给摩擦，`_drive_belt` 每**物理子步**推进）；倾倒阈值 ≈2.9 m/s² vs 工作 0.10 m/s²（**30× 余量，不需导轨**）。`flow_belt=False` 时**根本不注册**（不是关掉 —— slab 顶面高 4 mm 而旧 handoff 把瓶 re-seat 到桌面位姿，会插进去被 PhysX 弹飞，实测能毁掉 S4）。
- **训练课程**：S4 Reach（`Isaac-PressFlow-Reach-Direct-v0`，带关/quota 3，臂从 HOME 起必须自己寻位）从 remap 后的 line 早期 ckpt 起步 → **收敛**（`s4_xy/model_6600`）；S5 Flow（`Isaac-PressFlow-Direct-v0`，真带/抓取/放行/quota 5）从 S4 **早期高熵** ckpt（`warm_s5/model_5300`）resume → **收敛**（`s5_flow2/model_8299`）。obs 全程 32/act 4 保证逐段同形；ckpt remap = `rl/remap_ckpt_obs.py` 把 31→32 的首层**右侧补零列** + `EmpiricalNormalization` 统计补 0/1（零列 ⇒ 与旧策略逐位等价）。S4 五轮迭代的净改动：REACH 闭爪 / 上报 EXCH_CLOSE / `reach_vz_tol 0.03→0.10` / approach cone（xy 对齐成下探前置条件）/ 纯平移 HOME / `reach_xy_tol 0.003→0.006`。
- **结果 S4（带关 quota 3，32 env×30000 步，`model_6600`）**：episodes 690（fail 83 = 12.0%，trunc 607）、**`reach_timeout=0`**、grasps/局 6.75、releases/局 5.71、`doses at release mean=3.00`、`reach_steps` 均值 42.0（p10 37 / p90 50）。与继承的 PressLine 基线（5.95 瓶/局、3.21 拍/瓶）同量级，而**每轮循环都从 HOME 自己寻位**。
- **结果 S5（真带 quota 5，32 env×40000 步，`model_8299`）**：episodes 599（fail 70 = 11.7%，trunc 529，88.3% 撑满 2160 步 horizon）；**`doses at release mean=5.00 min=5 max=5`（n=2310）**、`release_quota_frac=1.000` —— 用户的「**5 次按压后松开**」逐次成立、不多不少；**`drift_fail=falloff=grasp_timeout=fetch_timeout=0`**；3.86 瓶/局、每瓶间隔 ~475 步、`reach_steps` 均值 82.7（< 400 窗口）。同 run 的 `model_8100` 核心性质一致（5.00/5/5、`release_quota_frac 1.000`、drift=falloff=grasp_timeout=0）但 fail_frac 0.171、3.66 瓶/局 ⇒ 修复**稳、非偶然**，交付选 8299。录像 `media/rl_press_flow.mp4`：单 episode **连续 2 瓶、0 reset**，每次 `doses on that bottle=5`（release 于 step 407 / 901；1280×720@60、901 帧 15.0 s；0 黑帧、亮度 241.8、帧间 diff 0.192）。
- **过程性负结果（已修，诚实记录）**：S5 首跑（`2026-09-13_04-40-58_s5_flow`）reward 在 −24…−45 摆荡、episode 长度 plateau 在 ~550，eval **99.2% 死在 `reach_timeout`**、`reach_steps` 均值 531.8（> 窗口 400）、**71% 的 env-step 花在相 5**。`diag_press_flow_drift.py` 加了瓶 root 的 env-local `bx`/`bz` 两列后一次定案：每帧 `bx≈2–3 mm`、`bz≈444 mm`（= `TAB+BELT_LIFT`，**瓶始终在工位**）而 palm 在相 4 死亡帧 `err_xy≈1078 mm`、`cmd_xy≈1195 mm` 且每步向外走 ~3 mm ⇒ **不是瓶丢了，是臂飞了**：相 3/4 只钳了 z（`up` 掩码），**xy 完全无界**；带一开相 3/4 被拉长到 ~150 步，粗尺度残差积分器把命令推到离站 1.19 m、palm 跟到 1.08 m，而 REACH 在 4→5 **re-anchor 到 palm 真实 FK** ⇒ 从 0.5–1.1 m 外起步。**修复 = 相 3/4 把 presser 的整个位姿（位置+四元数）命令回 HOME，并按 `flow_belt` 门控**：只钳 xy 不够（`_cmd_quat` 是 held 且按相位 re-anchor，IK 会为满足「(HOME xy, 相 2 姿态)」这个未必存在的位姿**放弃位置** —— 实测命令停 4.4 mm 而 palm 卡 147 mm 偏轴、48 局 0 局过相 4）；门控是因为带关时相 3/4 塌成几步、S4 本来就能应付，强加只会把它推出分布（实测 fail_frac 0.120→0.635、`reach_steps` 42.0→82.6）。修复后 REACH 入口 diag 实测 `above=165.4 mm`/`err_xy=4.0 mm`，与 `smoke_press_flow.py` 打印的 HOME `above_cap=+0.1642` **逐位吻合**；重训 reward **−58 → +45.9**、episode 长度 **428 → ~2050**。
- **回归**：旧单拍 `Isaac-Press-Direct-v0`（`press_A_s1…model_900.pt`）**success_rate 1.000**（2332/2332，首触底 mean 54.5）；旧流水线 `Isaac-PressLine-Direct-v0`（`s3_line/model_3000.pt`）**6.50 瓶/局、3.19 拍/瓶、drift_fail=0**（记录基线 5.95/3.21、drift 0）。本轮只**新增** `PressFlow*` + `Conveyor`，`press_env/cycle/line`、旧 task id、旧 eval/record 未破（两 task 的 env class 是 `PressFlowEnv` 的**父类**，结构上不可能受影响，仍实跑复核）。按 `flow_belt` 门控后 **S4 评测逐位复原**（690 局、fail_frac 0.120、`reach_steps` 42.0）。
- **边界（诚实）**：① 成品瓶离场是 **kinematic 复演 + 单瓶 prim 复用**，不是真双瓶供应链（进料段是摩擦拖动真物理）；② 「发现」是**特权观测**（`drift` 即进料距离），**无相机/无视觉检测**；③ 抓取仍是摩擦夹持（无 ContactSensor，力以 nozzle q 代理）；④ S5 放行密度低于 S4（3.86 vs 5.71 瓶/局）—— 真带换瓶本身 ~475 步/瓶；⑤ **训练后期方差**（同 S3/S4）⇒ **ckpt 必须按直接 eval 选**，不看末尾 reward；⑥ `reach_timeout` 仍是首位失败（53/599）。**s07（riser）本轮未做**（带与 riser 几何要重排）。
- **代码落点（不入本仓库，依赖本机 IsaacLab editable 布局）**：`direct/press/press_flow_env.py`（`PressFlowEnv(PressLineEnv)`：6 相机 + `_drive_belt`/`_write_belt_pose`/`_recycle_bottle` + `_home_arm` + 相 3/4 位姿钳位）、`press_env_cfg.py`（`_conveyor_cfg()` + `PressFlowEnvCfg`/`PressFlowReachCfg`/`PressFlowFullCfg`）、`__init__.py`（`Isaac-PressFlow-{,Reach-}Direct-v0`）、`agents/rsl_rl_ppo_cfg.py`（`PressFlowPPORunnerCfg`）；harness `rl/{remap_ckpt_obs,solve_press_flow_home,smoke_press_flow,diag_press_flow_drift,eval_press_flow,record_press_flow}.py`；复现命令见 `rl_learning_press_flow.md` §10。

## 14. PressFlow 补强 round（真捕获判据 + 恒速捕获 B/A0/A1，2026-09-16/17）

> 用户口径（承接 §13）：「这次结果很好，很符合流水线的定义，但是**流水线的位置和夹爪有穿模**，同时应该**训练夹爪自己捕获在流水线上的药瓶**的能力」。两件事分开处理：① 穿模**先测再改**；② 捕获走 **B→A 两段课程**（B 先修「真夹住」判据，A 再让带子不许停，A 内再分 A0 慢速 → A1 产线全速）。自包含总结 = `rl_learning_press_flow.md` §6.3/§6.4/§6.5/§9。

- **穿模：探针判定「无真穿模」，不改几何**。用户答「说不清，你写探针测」，于是新写 `rl/diag_press_flow_clip.py`（不复用 `diag_press_flow_outfeed.py` —— 后者把 `_phase=3` 钉死、只印 hand/blade 的 `[x,z]`、且把**刀片体原点**当盒心，沿 +Z 差 85 mm）。探针每控制步对候选 prim（`Conveyor`/`Table`/`Bottle`/`blade_L,R`/`panda_link6,7`/压臂 palm+基座）打印 **world AABB 逐轴间距 `gap_axis` 与 `pen_depth`**、**带↔刀片的 15 轴 SAT 精确 MTD**（刀片 45° 时 AABB 相交但盒不相交的假阳很多）、**刀片↔瓶的 2D 圆 vs 有向矩形解析**（叠 z 重叠）、以及**生效的 `physxCollision:contactOffset/restOffset` 与 `collisionEnabled`**。`--oracle` 与 `--policy` 双模式跑完，**没有任何一对的 `pen_depth` 转负** ⇒ 用户看到的是共面渲染伪影而非穿透。用户拍板「都不是，直接做 Part 2」。
- **B 段（相 4 真捕获判据）**：原相 4 门 = `jaw ≤ jaw_close_th ∧ drift_ok`，**不含「瓶在位」**，而刀片目标在非 3/4 相恒为 0 ⇒ **空爪合拢也满足 `grip_done`**，照样发 `rew_bottle=10.0`、照样被 `eval_press_flow.py` 记成一次 grasp。修法 = 特权几何判据 `_presence_ok()`：取两刀盒心 `c = p + quat_apply(q,[0,0,0.085])`、`mid=(cL+cR)/2`，把瓶子相对 `mid` 的偏移投到刀片系 `d`，判 `|d.x|<mouth_tol(6mm) ∧ |d.z|<along_tol(29mm=刀长/2 − body r) ∧ |d.y|<vert_tol(20mm) ∧ upright>0.95`；门加成 `m4 ∧ jaw≤th ∧ drift_ok ∧ present`，**`flow_belt=False` 时 `present` 恒 True ⇒ S4 逐位不变**；另加稠密项 `+rew_presence(0.015)×present×m4`。结果（`s5_presence/model_8299`，32 env×30000 步）：**`empty_close=0`**、`doses at release 5.00/5/5` 与 `release_quota_frac=1.000` **保持**、grasps 4.71→**4.19 瓶/局（−11%）** 而 **releases 3.86→3.89 持平**（= 剔掉的是空爪假抓，真抓全存活）、`fail_frac` 0.117→**0.035**。`grasp presence` 遥测给出物理解释：`mouth_mm 0.73`（= Ø42 身撑开内面的几何预期）、`margin_mm −5.9`、`bottle_speed 0.004`（抓取瞬间已停稳）。
- **A 段（恒速捕获）卡点 1：暖启起点在零梯度 wedge 里**。直接拿 S5-B ckpt 起 A0，三次跑全部**冻结**在 episode 长度 `497.04/497.04/497.05`（reward `−11.85/−28.44/−27.73`）几百个 iteration。`--policy --trace_ph4` 确诊：相 4 一开始 jaw 动作均值就是 **+3.8**，远在 env `±1` clip 之外 ⇒ `N(3.8,0.39)` 的**每个样本都 clip 成 +1** ⇒ 同一关节/同一 reward/同一 advantage ⇒ `E[(a−μ)/σ²·A] ≡ 0`。**reward shaping 救不了**（梯度项本身为零），要动的是**分布的均值**。修法 = 新脚本 `rl/warmstart_center_jaw.py`：把 actor 输出层 jaw 那**一行清零**、bias 设 `−0.2`（⇒ 12 mm 行程、64 mm 嘴 > Ø52 foot）、std 0.39→**0.5**、清掉**恰好这三个张量**的 Adam 动量（索引 0/7/8，带形状断言）、`iter`/LR 复位；其余（三条臂通道、critic、两个 obs 归一化器）原样不动。**整个干预 = 一层的「一行」**。
- **A 段卡点 2/3：dwell 与 wedge 定价**。① `grip_dwell=5` 在 `jaw` 是**实测**关节（滞后 ~3 步）的前提下实义为「连续 5 步闭合**命令**」，起点均值开着 + std 0.5 时连续五步约 1e-4 ⇒ 抓取 bonus 一次收不到、通道永远没有指向它的梯度 —— 捕获模式降到 **1**（`catch_grip_dwell`），安全性由 B 段的 `present` 门承担。② `rew_capture_open` 原设 0.0035/mm/步 时 **wedge 比「开着等」更便宜**（0.0035×12×150 = 6.3 vs 开窗成本 ~34）—— 策略被**付钱**坐进让捕获不可能的状态；提到 **0.025**（wedge 45 > 34）后每个失败模式只剩一个单调下坡：从 wedge 出来是「张开」，从开着不动出来是「合拢」。
- **结果 A0（`a0_catch4/model_1500`，32 env×12000 步，deterministic）**：`capture_timeout` **581/615 → 194/288 → 2/110**；**grasps 3.89 瓶/局 vs B 段静止抓取 4.19 = 93%**、releases 3.60 vs 3.89、**`doses at release 5.00/5/5`（n=396）一字不差**、`episodes_with_grasp_frac=0.982`、`drift=0`。`grasp presence` 的 **`bottle_speed mean=0.019 p90=0.031`** —— 抓取瞬间瓶子**在动**（带巡航 0.03），这是「真捕获」与 B 段「等它停稳再夹」最直接的分野。训练曲线 episode 长度 `497 → 2610 → 3313`、reward `−68 → −25.9 → +65.3`。
- **结果 A1（`a1_catch/model_3500`，产线全速 0.12 m/s，32 env×12000 步，deterministic）**：A1 = `PressFlowCatchFastCfg`（新 task id `Isaac-PressFlow-CatchFast-Direct-v0`），只把 `belt_speed` 0.03→**0.12**、`episode_length_s` 60→36，记账窗自动 **40→10 步**、相 4 deadline 自动 **265→115 步**（cfg 里是**推导**出来的，不是写死的）。**`grasps 4.28 瓶/局`（> 静止基线 4.19、> A0 3.89）、`releases 4.06`（vs B 3.89 / A0 3.60）、`fail_frac 0.082`（A0 的 1/3）、`capture_timeout` 与 `reach_timeout` 双双归零、`doses at release 5.00/5/5`（n=691）一字不差**；抓取瞬间 `bottle_speed mean=0.032 / p90=0.120 m/s` —— `p90` 正是 `belt_speed` 本身，即相当一部分抓取发生在带子**还没被锁停**的那一步内。**录像 `media/rl_press_flow_catch.mp4`**（`--task …CatchFast`、`model_3500`、`--max_bottles 2`）：单 episode 2 次捕获+松开 0 reset，相 `3→4 @ step 90 (bx=+0.089)`、`4→5 @ step 133 (bx=+0.003)`，两轮都在瓶子**行进中**合爪。
- **A1 的起步点与「零样本」诊断**：A0 的 ckpt 直接跑 A1，**捕获完美迁移甚至更准**（`capture_timeout=1/936`、`empty_close=0`、抓取瞬间 `|bx|` mean **3.4 mm**，优于 A0 的 8.0），但**捕获之后全链塌**（`drift=891/934`、`reaches=0`）。单 env 逐帧 trace 定案：瓶子在 `bx=+0.0005` 当场停死、`drift` 全程 <0.7 mm、**卡在相 5**（`stall` 涨到 112+、900 步 0 reach）⇒ 不是瓶子丢了也不是捕获失败，是暖启的 REACH 对「全速捕获位姿」出分布。同速度下**规则 oracle 可解**（`smoke_press_flow.py --task …CatchFast`：4 瓶、`aborts 0/4`、`doses [5,5,5,5]`）⇒ A1 是**微调**问题，不是重新设计。起步 ckpt 选 A0 的 **`model_2100`** 而非 §14 头条的 `model_1500`：两者抓取相当（3.86 vs 3.89），但 `model_2100` 的 REACH 条件好得多（`reach_timeout` **0 vs 21**、`reach_steps` **75.0 vs 104.9**），而 A1 要修的正是 REACH。（顺带否掉 A0 后期 reward ~116 的 `model_3300`：直接 eval 全面更差 —— grasps 3.29、fail_frac 0.574、`|bx|` 贴容差外沿。**reward 高 ≠ 指标好**。）
- **遗留缺陷（诚实）**：① A0 的 `reach_timeout=21/26` 是首位失败（捕获**之后**的寻位偶发超窗，暖启 REACH 对「捕获位姿」出分布，不是捕获问题，`drift=0`）—— **A1 已把它修掉（归零）**。② 抓取瞬间 |bottle_x| 在 A0 落在 `mean 7.7 / p90 9.9 mm`（容差 10），源于奖励在 `near` 处的不连续（`_close_gate` 与 `_inbound` 同点翻转）；原担心提速后「贴边闭爪」会**速度脆弱**，**A1 结论：担忧没有兑现** —— 全速下反而更靠窗心（mean **7.0** / p90 **8.5 mm**）。③ A 段的 `empty_close` 语义与 B 段不同：B 段是「空爪被记账」的 loophole 计数，A 段门含 `present`（构造上不可能空爪记账）⇒ 退化成**漏抓计数**。④ A1 剩余的 `timeout=12/170`（7%）是**单相超时**，既非捕获也非寻位。
- **代码落点（不入本仓库）**：`direct/press/press_flow_env.py`（`_presence_ok()`、相 4 门、`catch_grip_dwell`、`_inbound`/`_close_gate`、`_hold_belt`、`_drive_belt` 恒速+`_belt_locked`、`drift` 在抓取事件重基线、`_N_FAIL_BUCKETS=8`）、`press_env_cfg.py`（`presence_*`/`rew_presence`/`rew_capture_open`/`catch_grip_dwell` + `PressFlowCatchCfg`(A0) + `PressFlowCatchFastCfg`(A1，只改 `belt_speed`/`episode_length_s`，相 4 deadline 由 `belt_speed` 推导）、`__init__.py`（`Isaac-PressFlow-Catch{,Fast}-Direct-v0`）；harness 新增 `rl/{warmstart_center_jaw,diag_press_flow_clip}.py`；复现命令见 `rl_learning_press_flow.md` §10 第 7–8 步。
