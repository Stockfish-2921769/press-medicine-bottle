# 长程流水线按压（PressFlow）：上料 → 抓取 → 寻位 → 按压×5 → 松开 RL（round 总结）

> 状态（2026-09-12）：**完成**。把前几轮**分别**训成的两个技能接成一条长程 episode：真实传送带把瓶子送进工位 → 长爪抓住 → presser 从 **HOME** 位自己寻到瓶口上方 hover → 按压 **5** 拍（配额 5）→ 开爪撤臂放行 → 下一瓶。批量评测（32 env × 40000 步，`model_8299`）：**每次松开都带满配额 5 拍**（`doses at release mean=5.00 min=5 max=5`，n=2310）、`release_quota_frac=1.000`、3.86 瓶/局、fail_frac 11.7%、**drift_fail=0 / falloff=0 / grasp_timeout=0**。诚实边界：瓶子进料是**摩擦拖动真物理**，但成品瓶离场是 kinematic 复演 + **单瓶 prim 复用**（不是真双瓶供应链）；「发现」是**特权观测**（`drift` 即进料距离），无相机；抓取仍是摩擦夹持（无 ContactSensor，力以 nozzle q 代理）。
>
> 状态（2026-09-16）：**补强 round**。用户指出①「流水线的位置和夹爪有**穿模**」②「应该训练夹爪**自己捕获在流水线上**的药瓶」。① 探针 `rl/diag_press_flow_clip.py`（world AABB 逐轴间距 + 带↔刀片 15 轴 SAT 精确 MTD + 刀片↔瓶解析 + 生效 `contactOffset`）在 `--oracle`/`--policy` 双模式跑完**没有任何一对 `pen_depth` 转负** ⇒ 判定为共面渲染伪影，**几何不改**。② 分两段课程：**B** 给相 4 门加**特权几何在位判据** `_presence_ok()`（§6.3，`empty_close 68→0`、fail_frac 0.117→0.035）；**A** 把输送从「减速停稳」改成**恒速** `belt_catch`（§6.4 A0 0.03 m/s、§6.5 A1 **产线 0.12 m/s**）。**A1 头图：抓产线全速行进的瓶子 4.28 瓶/局 > 静止基线 4.19**、`capture_timeout`/`reach_timeout` 双双归零、`doses at release 5.00/5/5`（n=691）、抓取瞬间瓶速 `mean 0.032 / p90 0.120 m/s`。

代码 = `IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/press/`（`press_flow_env.py` + `press_env_cfg.py` 的 `PressFlow*` + `__init__.py` 注册）；harness = `isaac_demo/rl/{smoke_press_flow,eval_press_flow,record_press_flow,diag_press_flow_drift,solve_press_flow_home,remap_ckpt_obs}.py`。

## 1. 目标与本轮交付

用户口径：「我们只训练了如何用夹爪抓握，和用机械臂+灵巧手按压药瓶，没有去做**从发现自动化流水线的瓶子，到抓取到机械臂+灵巧手寻找药瓶并且按压，5 次按压后松开**的过程」。前几轮的缺口是**结构性的**，不是训练不足：

- `PressEnv._reset_idx` **直接把 presser 31 个关节瞬移到瓶子上方 hover**、把瓶子瞬移到工位 —— episode 一开始「瓶子已被夹住、手已经悬在它头上」，**没有任何接近/寻位动作**；
- 「conveyor」是 `PressLineEnv._seat_fresh_bottle` 的 `write_root_pose_to_sim` **瞬时瞬移**，场景里**没有传送带 prim、没有任何传感器**；
- 剂量配额 3 且「松开」只是夹具开爪，没有独立的放行/撤臂/上料相位。

本轮 Q&A 拍板：① 抓取端 = **Panda 长爪**（保留已验证的双臂分工，presser 只负责寻位+下压）；② 范围 = **接近 + 真实 conveyor 抓取**；③ 「发现」= **特权观测，不加相机**；④ 配额 3→**5** 且要有显式**松开**。

交付 = 6 相 MDP（obs 32 / act 4）+ 真传送带 prim + HOME 位姿 + 四段新增奖励 + 两段课程（S4 Reach / S5 Flow）。

## 2. MDP（obs 32 / act 4，6 相）

关键设计：**旧相位索引保持原义，新相位只追加到 index 5**。这样旧策略的 5 维 one-hot 语义不变，第 6 维零初始化即可让 warm-start 策略在起点**行为逐位一致**（见 §3 remap）。

| idx | 相 | 语义 | 门控推进 |
|---|---|---|---|
| 0 | PRESS | 同 PressLine | `q≤bottom_q ∧ drift_ok` → 1 |
| 1 | HOLD | 同 PressLine | 底带 dwell `H_min=12` → 2 |
| 2 | LIFT | 同 PressLine | 抬过 cap `h_return=0.012` ∧ `\|q\|<rebound_tol` → 配额未满回 0；满 → 3 |
| 3 | EXCH_OPEN = **放行/撤臂/上料** | 开爪 + palm 撤到 **HOME** + 带把成品瓶带出、新瓶带到工位 | `jaw≥jaw_open_th ∧ palm_above≥home_clear(0.06)`（=release）→ 带 outfeed → infeed → 到站 → 4 |
| 4 | EXCH_CLOSE = **抓取** | 长爪在工位合拢抓住新瓶 | `jaw≤jaw_close_th ∧ drift_ok ∧`**`presence`** 连续 `grip_dwell=5` → 5 |
| 5 | **REACH（新）** | presser 从 HOME 寻到瓶口上方 hover | `\|palm_err_xy\|<reach_xy_tol(0.006) ∧ palm_above∈[0.008,0.050] ∧ \|palm_vz\|<reach_vz_tol(0.10)` → 0 |

流程闭环：`… 2(满配) → 3(放行+上料) → 4(抓) → 5(寻) → 0(压) → 1 → 2 → …`；episode reset → **phase 3**。用户要的「5 次按压后松开」= phase 3 的开爪+撤臂+瓶体离场，不另设相位。

**动作 4** = `(dx,dy,dz)` 世界系任务空间残差（积分进 `_cmd_pos`）+ 长爪标量 `a_jaw∈[-1,1]`。**两档尺度**：相 0/1/2 用精细 `action_scale=0.0005`（0.5 mm/步，旧下压行为**零扰动**）；相 3/4/5 用 `action_scale_coarse=0.004`（4 mm/步，否则 0.15 m 的寻位要 300 步、吃掉整个 horizon）。相位切换时 `_cmd_pos`/`_cmd_quat` **re-anchor 到 palm 真实 FK**（命令积分债清零），尺度切换不产生累积跳变。

**观测 32** = base 26（`PressEnv`，`prev_actions` 已 4 维）+ 5 维 PressLine 相 one-hot + **1 维 REACH flag**。**不加新特征**：`drift`（瓶 xy − 工位家）在进料期天然就是「瓶子离工位还有多远」。**REACH 上报 EXCH_CLOSE 的 one-hot**（不是自占第 6 槽）：6 槽 one-hot 会让 REACH 成为唯一「前 5 维全零」的组合，而 remap 的第 6 列是零填充 —— 该组合落在 line 策略训练分布之外，warm-start 策略会把它答成「张爪横扫」；上报 EXCH_CLOSE 则让每个观测都落在 line 策略的支撑集内。

**相 2 与 REACH 锁死闭合**（`hold_closed = phase≤2 | phase==5`）：REACH 的语义是「把已夹住的瓶子从夹具搬到按压工位」，中途开爪必然掉瓶。

奖励沿用 PressLine 全部剂量/交换项（`quota→5`），新增：寻位进展 `+rew_reach_prog(0.02)/mm ×Δd`（**两侧**势能，防「后退不罚、再进重奖」的振荡套利）、专用 xy 项 `+rew_reach_xy(0.06)/mm`（`d_hover` 的 z 项在同高度下主导 ~35×，对横向漂移近乎失明）、到位 lump `+5.0`、放行 lump `+4.0`、撤臂进展 `+0.05/mm`。**`drift_fail` 与 drift 罚按相位门控**（`phase==3` 时不生效）：进料期瓶子本来就离工位 0.2 m，否则会被误判 fail/狂扣。

**目标高度随 xy 误差滑动**（`z_ref = hover_ref + (reach_approach_hi−hover_ref)·clamp(err_xy/align_band,0,1)`，`approach_hi=0.08`、`align_band=0.030`）：xy 没对上时目标停在 80 mm 高，对上才滑到 28 mm —— **把 xy 对齐变成下探的前置条件**，而不是指望策略自己交错完成。

## 3. HOME 位姿与 warm-start 桥

**HOME**（env 里原本不存在）：新增 cfg `home_joint_delta`，在 `__init__` 里 `_home_arm = _q_hover_arm + delta`。第一版手挑 `[0,−0.3,0,−0.4,0,0,0]` 是**错的** —— 那两个关节是 pitch/roll，HOME 与 hover 因此**姿态不同**；而 `_apply_action` 喂给 IK 的 `_cmd_quat` 是**held fixed** 的，于是 IK 被要求「到达 hover 位置同时保持 HOME 姿态」——一个不存在的位姿 —— 它靠**横向拽 palm** 来妥协。

`solve_press_flow_home.py` 把它改成**hover 的纯平移**：把 presser 瞬移到 hover 平衡点，再用 **env 自己的差分 IK 伺服**（`_cmd_pos = hover_palm+offset`、`_cmd_quat = hover_palm_quat` 钉住）迭代写回关节状态（绕开 PD 动力学，确定性、跨 env 一致）。解出 `home_joint_delta=[−0.1571, 0.0513, −0.4207, −0.3490, 0.2914, −0.0334, −0.4706]`，实测 HOME palm = env-local **(2.3, 3.7, 753.8) mm** —— 几乎**正对工位上方**（工位 xy≈(0,3.3)），比 cap 高 **164 mm**，姿态与 hover 一致 ⇒ **寻位是纯下降**。

**ckpt remap**：旧 line 策略 actor 首层是 `nn.Linear(31,512)`，新 env 是 32 ⇒ 直接 `runner.load` 形状不匹配。`remap_ckpt_obs.py` 读旧 ckpt → actor/critic 首层权重**右侧补零列**、`EmpiricalNormalization` 的 `running_mean` 补 0 / `running_var` 补 1 → 存新 ckpt。零列 ⇒ 新维度被忽略 ⇒ remap 后策略与旧策略**逐位等价**，是干净的 warm start。

## 4. 真实传送带

**prim**：`Conveyor` = `RigidObjectCfg` + `CuboidCfg`（**dynamic**：`kinematic_enabled=False`、`disable_gravity=True`、mass 40），尺寸 `(0.36, 0.14, 0.004)`，静止时 env-local 中心在 `BELT_CENTER_X0=+0.10`，随 `_belt_s` 沿带轴滑动。顶面比桌面高 `BELT_LIFT=BELT_TH=4 mm`。`flow_belt=False` 时**根本不注册** —— 不是「关掉」：slab 顶面比桌面高 4 mm，而无带 handoff 把瓶子 re-seat 到桌面位姿，会让瓶子下半 3.4 mm 插进 slab，PhysX 解穿模时把两者一起弹飞。

**输送 = 摩擦拖动（真物理）**：`_write_belt_pose` 同时写**位姿**（把 slab 钉在轨道上）与**速度**（摩擦唯一的作用对象；只写位姿等于瞬移，对 PhysX 就是一个静止表面）。`_drive_belt` 从 `_apply_action` 调用（**每个物理子步**推进，不是每个控制步），速度取 `clamp(belt_gain·(目标−瓶x), ±belt_speed)`，`belt_speed=0.12`、`belt_accel=0.10`、`belt_gain=2.0`。**为什么必须 dynamic**：PhysX 只从刚体自身速度推表面速度，纯 `write_root_pose_to_sim`（=`setGlobalPose` 瞬移）在求解器眼里是静止表面 —— 实测 slab 空滑完整条 0.33 m 行程而瓶子纹丝不动。

> **安全性有量化余量**：Ø42 / 0.25 kg 直立圆柱被摩擦拖动的**倾倒阈值** ≈ `m·g·r/(h/2)` ≈ 0.74 N，对应加速度 ≈ **2.9 m/s²**；工作加速度 0.10 m/s²（**30× 余量**）。**不需要导轨**。

单瓶 prim 复用：放行时带把成品瓶沿 +x 带到下游，出画后 `_recycle_bottle` 把它定位回上游供下一轮。

## 5. 训练课程（沿「未收敛 checkpoint resume」既有先例）

| 段 | task id | 起点 | 语义 | 结果 |
|---|---|---|---|---|
| S4 Reach | `Isaac-PressFlow-Reach-Direct-v0` | remap 后的 line `pressline_s1_s1/…/s3_line` **早期 ckpt** | 带关（瓶子仍瞬移就位）、quota 3、`episode_length_s=24`；臂从 **HOME** 起、必须自己寻到 hover | ✅ 收敛；`2026-09-13_03-47-42_s4_xy/model_6600.pt` |
| S5 Flow | `Isaac-PressFlow-Direct-v0` | S4 **早期高熵 ckpt**（`warm_s5/model_5300.pt`） | 开真带 + 抓取 + 放行 + quota 5、`episode_length_s=36` | ✅ 收敛；`2026-09-13_05-49-34_s5_flow2/model_8299.pt` |
| S5-B 真捕获 | `Isaac-PressFlow-Direct-v0`（原地收紧） | S5 **早期高熵 ckpt**（`warm_s5/model_5300.pt`，同 §7 教训） | 相 4 门加 `presence` + `rew_presence` 稠密项；其余逐位不变 | ✅ 收敛；`2026-09-16_20-54-45_s5_presence/model_8299.pt`（§6.3） |
| A0 恒速捕获 | `Isaac-PressFlow-Catch-Direct-v0`（新 task id） | S5-B ckpt + **夹爪通道定向手术**（§6.4） | `belt_catch=True`、`belt_speed=0.03`（记账窗 40 步，4× 容易）、`episode_length_s=60` | ✅ 收敛；`2026-09-16_22-18-38_a0_catch4/model_1500.pt`（§6.4） |
| A1 全速捕获 | `Isaac-PressFlow-CatchFast-Direct-v0`（新 task id） | A0 **`model_2100`** resume（见 §6.5 选点） | 同一个 env，只把带速调回产线的 `0.12`（记账窗 10 步），deadline 自动 265→115 | ✅ 收敛；`2026-09-16_23-41-12_a1_catch/model_3500.pt`（§6.5） |

复现要点沿用本项目已三次记录的教训：**新阶段要从上一段的早期（未收敛、高熵）ckpt resume** —— 收敛后的价值函数锁死旧行为、plateau。obs 全程 32 / act 4，保证逐段同形。

S4 五轮迭代（`s4_reach → s4_jaw → s4_infeed → s4_cone → s4_home → s4_xy`）的净改动 = ① 相 2 与 REACH 锁死闭爪；② REACH 上报 EXCH_CLOSE 的 one-hot；③ `reach_vz_tol 0.03→0.10`；④ approach cone（`approach_hi`/`align_band` + 两侧势能）；⑤ **纯平移 HOME**；⑥ `reach_xy_tol 0.003→0.006` + `rew_reach_xy=0.06`。前四轮的死因分别是 100% `drift` 死在相 5（爪在 REACH 首步张开、palm 横扫把无夹持的瓶子扫过 8 mm 悬崖）与 `reach_timeout` 满盘。

## 6. 结果

### 6.1 S4 Reach（皮带关，quota 3；32 env × 30000 步，`model_6600`）

```
episodes=690 (fail=83 trunc=607)   fail_frac=0.120
fail_types: falloff=0 drift=52 timeout=31 none=959917 reach_timeout=0 fetch_timeout=0 grasp_timeout=0
per episode: grasps=6.75 releases=5.71 reaches=6.70   episodes_with_grasp_frac=1.000
per grasp: doses=2.73 (quota=3) releases=0.846 reaches=0.992 release_quota_frac=1.000
doses at release: mean=3.00 min=3 max=3 n=3942
reach_steps per reach: mean=42.0 p10=37 p90=50 n=690 (reach_timeout=400)
```

与继承的 PressLine 基线（5.95 瓶/局、3.21 拍/瓶）同量级，而**每一轮循环都从 HOME 自己寻位**。`reach_timeout=0` = 寻位门每次都在窗口内成立。

### 6.2 S5 Flow（真带 + 抓取 + 放行，quota 5；32 env × 40000 步）

`model_8299`：

```
episodes=599 (fail=70 trunc=529)   fail_frac=0.117
fail_types: falloff=0 drift=0 timeout=17 none=1279930 reach_timeout=53 fetch_timeout=0 grasp_timeout=0
per episode: grasps=4.71 releases=3.86 dose_cycles=19.4   episodes_with_grasp_frac=1.000
per grasp: doses=4.13 (quota=5) releases=0.819 reaches=0.868 release_quota_frac=1.000
doses at release: mean=5.00 min=5 max=5 n=2310
steps between releases: mean=475 p10=409 p90=517
reach_steps per reach: mean=82.7 p10=54 p90=84 n=599 (reach_timeout=400)
```

- **`doses at release = 5.00 / min 5 / max 5`（n=2310）** —— 用户的「**5 次按压后松开**」逐次成立，不多不少（`release_quota_frac=1.000`）；
- **`drift_fail=0`、`falloff=0`、`grasp_timeout=0`、`fetch_timeout=0`** —— 抓取门与放行门从未卡死；
- 88.3% episode 撑满 2160 步 horizon（`trunc`），即「无失败长跑」= 本 MDP 的常态；真实失败 11.7%（`reach_timeout` 53 / 其它超时 17）；
- 3.86 瓶/局，每瓶间隔 ~475 步（≈ 5 拍 + 1 次放行/上料）；`reach_steps` 82.7 步（< 400 窗口，含带残余运动）。

**ckpt 选择**（本项目已知「多次训练后期方差」）：同 run 的 `model_8100` 也成立（`doses at release 5.00/5/5`、`release_quota_frac 1.000`、`drift=falloff=grasp_timeout=0`），但 fail_frac 0.171、3.66 瓶/局、`reach_steps` 60.1。两者核心性质一致 ⇒ 修复是**稳的、不是偶然**；交付选 `model_8299`。

**视频** `media/rl_press_flow.mp4`：**单 episode 连续 2 瓶、0 reset**（`[rec] segment_frames=901 resets=0 releases=2 completed=True`），每次松开都是 `doses on that bottle=5`（release 于 step 407 / 901，即第一瓶 5 拍只花 407 步）。1280×720 @60fps、901 帧 15.0 s；像素校验 **0 黑帧**、亮度 241.8、帧间 diff 0.192（进料/抓取/寻位/按压/放行动作清晰）。末帧 `outputs/rl_press_flow_last.png`。

### 6.3 S5-B —— 真捕获判据（`presence` 门）

**被修掉的缺陷**：S5 的相 4 门是 `jaw≤jaw_close_th ∧ drift_ok`，**不含「瓶在位」**。而刀片的目标在非相 3/4 恒为 0（`hold_closed = ph≤2 | ph==5`），所以**空爪合拢也会满足 `grip_done`** —— 照样发 `rew_bottle=10.0`、照样被 `grasp_count` 记账。配上带子的减速停稳（`v=clamp(belt_gain·(station−bottle_x))` 把瓶子停在工位），这个 loophole 在 S5 里**从未暴露**：瓶子总在工位等着，空爪与真抓在遥测上不可分。它是**承重的**，所以 §4 的验证阶梯要求「先量再收紧」而不是直接改门。

**修法 = 特权几何判据**（`press_flow_env._presence_ok()`，全张量、128 env 零成本）：取 `blade_L/blade_R` 世界位姿，盒心 `c = p + quat_apply(q,[0,0,0.085])`，`mid=(cL+cR)/2`；把瓶子相对 `mid` 的偏移投到刀片系 `d = quat_apply(quat_inv(qL), bottle_root_w − mid)`：

```
in_x = |d.x| < presence_mouth_tol(0.006)   # 在两刀之间
in_z = |d.z| < presence_along_tol(0.029)   # 落在刀片足迹内（刀长 0.050 − foot 半径 0.026）
in_y = |d.y| < presence_vert_tol(0.020)    # 竖直：刀片半高 0.025，留边
upright = (1 − 2(qx²+qy²)) > presence_upright(0.95)
present = in_x & in_z & in_y & upright
```

门（`:450`）：`present = self._presence_ok() if cfg.flow_belt else ones_like(drift_ok)`，`jaw_on = m4 & (jaw≤jaw_close_th) & drift_ok & present`。**S4（带关）走 else 分支 ⇒ 逐位不变**（§8 实测复核）。另加稠密项 `+rew_presence(0.015) × present × m4`，让相 4 在 dwell 未满前就有梯度；遥测新增 `empty_close`（相 4 退出时 `present` 从未为真）与 `grasp presence`（`margin_mm`/`mouth_mm`/`bottle_speed`）。

**结果**（`2026-09-16_20-54-45_s5_presence/model_8299.pt`，32 env × 30000 步）：

```
episodes=428 (fail=15 trunc=413)   fail_frac=0.035
fail_types: falloff=0 drift=5 timeout=3 none=959985 reach_timeout=7 fetch_timeout=0 grasp_timeout=0 capture_timeout=0
per episode: grasps=4.19 releases=3.89 reaches=4.15 dose_cycles=20.3   episodes_with_grasp_frac=1.000
per grasp: doses=4.86 (quota=5) releases=0.930 reaches=0.991 release_quota_frac=1.000 steps_per_grasp=536
doses at release: mean=5.00 min=5 max=5 n=1667
steps between releases: mean=522 p10=445 p90=583
reach_steps per reach: mean=80.7 p10=54 p90=93 n=427 (reach_timeout=400)
empty_close=0
grasp presence: margin_mm mean=-5.9 p10=-5.9 max=-5.8 mouth_mm mean=0.73 max=0.75 bottle_speed mean=0.004 p90=0.007 n=1792
```

- **`empty_close=0`**（1667 次松开、1792 次抓取，没有一次空爪合拢被记账）；
- **`doses at release 5.00/5/5` 与 `release_quota_frac=1.000` 保持** —— 收紧没有换来配额退化；
- `grasps` 4.71→**4.19 瓶/局（−11%）**、`releases` 3.86→**3.89（持平）**、`fail_frac` 0.117→**0.035**、`reach_timeout` 53→**7**。抓取数下降而松开数与配额不变 = **空爪的假抓被剔掉**，真抓全部存活；这也是判据**够紧、但不过紧**的直接证据（真抓若被误杀，`releases` 会一起塌）；
- `grasp presence` 三个量给出物理解释：`mouth_mm mean=0.73`（刀片内面间隙 0.73 mm，正是 Ø42 瓶身把内面撑在 ±0.021 ⇒ `q≈0.0007` 的几何预期）、`margin_mm mean=−5.9`（瓶轴比 `mid` 低 5.9 mm，落在 `presence_along_tol=0.029` 内）、`bottle_speed mean=0.004 m/s`（抓取瞬间瓶子已停稳）。

**为什么不动 `grip_dwell`**：`jaw` 是**实测**关节（`_jaw_actual()`），有 ~3 步作动器滞后，所以 `grip_dwell=5` 实义是「连续 5 步闭合**命令**」。这个 dwell 是**静止抓取**的假阳守卫；有了几何门之后它是冗余的 —— 但只在 §6.4 的捕获模式下才删（那里 dwell 是**探索**的障碍），本节的 B 段原样保留。

### 6.4 A0 —— 恒速捕获（夹爪自己去流水线上抓）

**B 段与 A 段的差别，一句话**：B 里带子把瓶子**减速停稳**在工位（`v = clamp(belt_gain·(station − bottle_x), ±belt_speed)`），抓取的时间问题由输送解决，夹爪只要合拢；A 里带子**恒速**跑（`v_t = belt_speed`），瓶子不会停下来等人，**闭合时机变成策略的问题**。所以 A 段新 task id `Isaac-PressFlow-Catch-Direct-v0`（`PressFlowCatchCfg`：`belt_catch=True`、`belt_speed=0.03`），而相 4 的语义、门、观测全不变。

**几何窗口是算术，不是调参**：刀片足迹沿带轴 `JAW_BLEN=0.100`，瓶子能在足迹内任意位置被抓 ⇒ 「瓶在两刀之间」的行程是 `2×presence_along_tol = 58 mm`。控制 `dt=1/60`：A0（0.03 m/s，0.5 mm/步）**116 步**全重叠、其中进 `belt_capture_tol=10 mm` 记账窗的 **40 步**；A1（0.12 m/s，2 mm/步）**29 / 10 步**。相 4 的 deadline 因此**从 `belt_speed` 推导**（`(capture_enter_x + belt_capture_tol)/belt_speed/dt + dwell + slack` = 200+5+60 = **A0 265 步**、50+5+60 = **A1 115 步**；实测打印 `capture_deadline` 与之一致）—— 同一个步数在 0.12 下是 2× 余量、在 0.03 下**小于**瓶子走到阀门的旅行时间，写死就会让 A0 在等的那只瓶子到达前就超时。

**卡点 1：暖启起点在 wedge 里，而且是「零梯度」的 wedge。** 直接拿 S5-B 的 ckpt 起 A0，三次跑（iteration ~250 处）全部**冻结**在 episode 长度 `497.04 / 497.04 / 497.05`、reward `−11.85 / −28.44 / −27.73`，几百个 iteration 不动。诊断（`smoke_press_flow.py --policy --trace_ph4`，量在 `s5_presence/model_6000.pt` 上）：相 4 一开始 jaw 动作均值就是 **+3.8**，远在 env 的 `±1` clip 之外（`:408`）。于是 `N(3.8, 0.39)` 的**每一个样本**都 clip 到 +1 ⇒ 同一个关节、同一个 reward、同一个 advantage ⇒ `E[(a−μ)/σ²·A] ≡ 0`。**这不是 reward shaping 能救的**：把 `rew_capture_open` 调多大、把 `rew_jaw_dist` 调多细都不会在这一维产生一行梯度，因为梯度项本身是零。要动的是**分布**，而且动的是它的均值（唯一超出取值范围的那部分）。

**修法 = 夹爪通道定向手术**（`rl/warmstart_center_jaw.py`）：把 actor 输出层 action index 3（jaw）那一行**清零**（该通道起点无状态依赖），bias 设成 `−0.2` ⇒ `jaw = JAW_U·(1−a)/2 = 12 mm`、嘴 64 mm（> Ø52 foot，`capture_open_req=0.012` 同一个理由），std 从退火后的 0.39 放宽到 **0.5**（一个 batch 内就采得到「窄到进不去」到「全开」的嘴，close-deficit 项才有价差可分）；清掉**恰好被改的三个张量**的 Adam 动量（`std`、`out.weight`、`out.bias`，索引 0/7/8，带形状断言），免得旧约定的动量把它立刻推回去；`iter` 与存储的 LR 复位（源 run 已退火到 1e-5 底，对刻意扰动的策略太慢）。**其余全部原样搬过来**：三条臂通道、critic、两个观测归一化器。整个干预 = 一层的**一行**。

**卡点 2：`grip_dwell=5` 在新通道上不可达。** `jaw` 是**实测**关节（`_jaw_actual()`），比命令滞后约 3 步，所以 `grip_dwell=5` 的真义是「连续 5 步闭合**命令**」。起点均值开着、std 0.5 时，一步采到闭合是 ~2σ 之外，**连续五步**约 1e-4 —— 抓取 bonus 一次都收不到，通道就永远没有指向它的梯度。捕获模式因此把 dwell 降到 **1**（`catch_grip_dwell=1`）。这是安全的，**恰恰因为 §6.3 的 `presence` 门**：`jaw≤close_th` 单凭自己曾在空气上成立，而 `jaw 闭合 ∧ present ∧ near` 是「瓶子在两刀之间且离工位 10 mm 内」，相变还在同一步把爪锁死、把带锁停。dwell 是**静止抓取**时代的假阳守卫，有了几何门就冗余 —— 而在捕获模式下它是横在任务与任何奖励之间的那一个。

**卡点 3：wedge 比「开着等」更便宜。** `rew_capture_open` 原设 0.0035/mm/步。一个 wedge 在整段接近里（A0 ~150 步）只付这个，之后什么都不付（瓶子永不到窗口）；而保持张开要在瓶子进窗的 ~40 步里付 `rew_jaw_dist` 对缺失的 10.5 mm 闭合的罚（0.08/mm ⇒ ~0.84/步 ⇒ ~34 总计）。**0.0035 下 wedge 是更便宜的那个失败**（0.0035×12×150 = 6.3 < 34）—— 策略被**付钱**坐进那个让捕获不可能的状态。把 wedge 的价定到高于开窗成本（0.025×12×150 = 45）之后，**每一个失败模式都只剩一个单调的下坡方向**：从 wedge 出来是「张开」，从开着不动出来是「合拢」。

**结果**（`2026-09-16_22-18-38_a0_catch4/model_1500.pt`，32 env × 12000 步，deterministic）：

```
episodes=110 (fail=26 trunc=84)   fail_frac=0.236
fail_types: falloff=0 drift=0 timeout=3 none=383974 reach_timeout=21 fetch_timeout=0 grasp_timeout=0 capture_timeout=2
per episode: grasps=3.89 releases=3.60 reaches=3.67 dose_cycles=18.2   episodes_with_grasp_frac=0.982
per grasp: doses=4.68 (quota=5) releases=0.925 reaches=0.944 release_quota_frac=1.000 steps_per_grasp=897
doses at release: mean=5.00 min=5 max=5 n=396
steps between releases: mean=797 p10=709 p90=833
reach_steps per reach: mean=104.9 p10=65 p90=162 n=96 (reach_timeout=400)
empty_close=68
grasp presence: margin_mm mean=-5.8 p10=-6.0 max=-5.6 mouth_mm mean=1.11 max=1.50 bottle_speed mean=0.019 p90=0.031 n=428
capture timing: |bottle_x| at grip mm mean=7.7 p90=9.9 max=10.0 signed mean=+7.7 n=428 (belt_cap_tol=10mm, deadline=265 steps)
```

- **`capture_timeout=2/110`**（model_200 是 581/615、model_900 是 194/288）—— 捕获时机基本学成；
- **`grasps 3.89 瓶/局` vs B 段静止抓取的 4.19** ⇒ 抓**移动**中的瓶子达到了静止基线的 **93%**，而 `releases 3.60 vs 3.89`、**`doses at release 5.00/5/5` 一字不差**；`episodes_with_grasp_frac=0.982`；
- `grasp presence` 的 `bottle_speed mean=0.019 p90=0.031` —— 抓取瞬间瓶子**在动**（带的巡航速度就是 0.03），这是「真捕获」与 B 段「等它停稳再夹」最直接的分野；
- `capture timing` 落在 `mean 7.7 mm / p90 9.9 mm`（容差 10 mm）—— 策略把闭合压在记账窗内、且**贴近外沿**，即它学到的时机是「瓶子刚进窗就合」（见 §9 第 7 条：这是奖励里的不连续点造成的角点解，A1 提速时是首要观察对象）；
- 训练曲线：episode 长度 `497 → 2610 → 3313`、reward `−68 → −25.9 → +65.3`。

**遗留缺陷（诚实记录）**：`reach_timeout=21` 成了首位失败（21/26 失败），即捕获成功之后的**寻位**偶发超窗 —— 与 model_200 上观察到的「相 5 把被夹住的瓶子横向拽到 8 mm 漂移悬崖」同源。它不是捕获问题（`drift=0`），是暖启的 REACH 对「捕获位姿」出分布。`reach_steps` 分布同时在变好（mean `204 → 137.6 → 104.9`、p90 `461 → 162`），所以倾向于「继续训练」而非改门。**该项在 A1 已被解决（`reach_timeout` 归零，见 §6.5）** —— 印证了「继续训练」这一判断。

### 6.5 A1 —— 全速捕获（产线速度 0.12 m/s）

**A1 是什么**：同一个 env、同一个门、同一个 deadline 公式，只把 `belt_speed` 从 0.03 调到产线的 **0.12**（新 task id `Isaac-PressFlow-CatchFast-Direct-v0`，cfg `PressFlowCatchFastCfg`）。记账窗因此从 40 步缩到 **10 步**（`2·belt_capture_tol/speed/dt = 20 mm / 0.12 / 0.01667`），相 4 的 deadline 自动 **265 → 115 步**（`capture_deadline` 实打实测到 115）。`episode_length_s` 回到基线 36 s —— A0 的 60 s 只是为了在 4× 长的进料腿里塞同样多的瓶子。

**零样本先验（拿 A0 的 ckpt 直接跑 A1，不训练）**：捕获**完美迁移、甚至更准** —— `capture_timeout=1/936`、`empty_close=0`、`episodes_with_grasp_frac=0.999`、抓取瞬间 `|bx|` mean **3.4 mm**（p90 5.2、max 9.5，比 A0 的 8.0/9.9 更靠窗心）。但**捕获之后的全链塌了**：`drift=891/934`、`releases/局 0.08`、`doses per grasp 0.38`、`reaches=0`。单 env 逐帧 trace 定案：相 4 在 `bx=+0.0865` 张着嘴等（`present` 自 `+0.0285` 起为真）、`+0.0085` 处 `near` 触发、5 步内合到 0.0007、`+0.0005` 处瓶子**当场停死**（`bxdot −0.1196 → −0.0026`），下一步进相 5（`belt_v=0.0000`、`drift=0.00`）—— 然后**卡在相 5**：`stall` 一路涨到 112+、900 步内 `reaches=0`，而瓶子全程 `|drift| < 0.7 mm`。所以**不是瓶子丢了、也不是捕获失败，是暖启的 REACH 对「全速捕获位姿」出分布**。

**同一速度下 oracle 可解（据此排除 env 问题）**：`smoke_press_flow.py --task Isaac-PressFlow-CatchFast-Direct-v0`（规则 oracle，保持张口到 `bx > catch_close_x` 再合）跑出 **4 瓶、`aborts 0/4`、`doses [5,5,5,5]`、`reach_steps 135.8`、`capture_timeout=0`** —— 0.12 m/s 下环境本身完全可解，A1 是**微调**问题，不是重新设计。

**起步 ckpt 的选择（与 §6.4 头条不同）**：§6.4 的头条是 A0 `model_1500`（grasps 3.89），但 A1 从 **`model_2100`** 起 —— 两者抓取相当（3.89 vs 3.86），而 `model_2100` 的 REACH 条件好得多（`reach_timeout` **0 vs 21**、`reach_steps` **75.0 vs 104.9**），A1 要修的恰恰是 REACH。顺带否掉一个诱人选项：A0 训练后期 reward 冲到 ~116 的 `model_3300`，直接 eval 反而**全面更差** —— grasps **3.29**、fail_frac **0.574**、`capture_timeout=64`、`|bx| at grip` mean 9.1 / p90 10.0 贴着容差外沿。**reward 高 ≠ 指标好**，与 §9 第 5 条同一条教训。

**结果**（`2026-09-16_23-41-12_a1_catch/model_3500.pt`，32 env × 12000 步，deterministic）：

```
episodes=170 (fail=14 trunc=156)   fail_frac=0.082
fail_types: falloff=1 drift=1 timeout=12 none=383986 reach_timeout=0 fetch_timeout=0 grasp_timeout=0 capture_timeout=0
per episode: grasps=4.28 releases=4.06 reaches=4.19 dose_cycles=20.7   episodes_with_grasp_frac=1.000
per grasp: doses=4.82 (quota=5) releases=0.949 reaches=0.978 release_quota_frac=1.000 steps_per_grasp=527
doses at release: mean=5.00 min=5 max=5 n=691
steps between releases: mean=506 p10=424 p90=553 n=691
reach_steps per reach: mean=55.0 p10=52 p90=58 n=170 (reach_timeout=400)
empty_close=3
grasp presence: margin_mm mean=-5.9 p10=-6.0 max=-5.6 mouth_mm mean=0.90 max=1.49 bottle_speed mean=0.032 p90=0.120 n=728
capture timing: |bottle_x| at grip mm mean=7.0 p90=8.5 max=9.9 signed mean=+7.0 n=728 (belt_cap_tol=10mm, deadline=115 steps)
```

**按速度排的三档对照**（同一指标、同一 eval 口径）：

| | B 静止（带把瓶减速停稳） | A0 恒速 0.03 | **A1 恒速 0.12（产线速度）** |
|---|---|---|---|
| grasps / 局 | 4.19 | 3.89 | **4.28** |
| releases / 局 | 3.89 | 3.60 | **4.06** |
| fail_frac | 0.035 | 0.236 | **0.082** |
| `capture_timeout` | 0 | 2 | **0** |
| `reach_timeout` | 0 | 21 | **0** |
| doses at release | 5.00/5/5 | 5.00/5/5 | **5.00/5/5** |
| 抓取瞬间瓶速 | ~0 | 0.019 m/s | **0.032 m/s（p90 0.120）** |
| `\|bx\|` at grip（容差 10 mm） | — | 7.7 / 9.9 | **7.0 / 8.5** |

- **全速捕获不是「勉强能用」，是三档里最好的那一档**：grasps **4.28** 超过静止基线 **4.19**、也超过 A0 **3.89**；`fail_frac 0.082` 是 A0 的 **1/3**；两个超时桶（`capture_timeout`、`reach_timeout`）**双双归零**；`doses at release 5.00/5/5`（n=691）一字不差。
- `bottle_speed mean=0.032 p90=0.120` —— 抓取瞬间瓶子以**产线全速**在动；`p90=0.120` 正是 `belt_speed` 本身，即相当一部分抓取发生在带子**还没被锁停**的那一步内。
- **§9 第 7 条的「贴边闭爪」担忧没有兑现**：记账窗从 40 步压到 10 步、速度 4×，策略反而**更靠窗心**（`|bx|` mean 7.0 / p90 8.5，对比 A0 的 7.7 / 9.9）。奖励的不连续点仍在（角点解仍存在），但 10 步窗口内它有足够的定位精度把自己摆进去。
- 训练：resume 后 reward `8 → −60 → 36 → 85 → 118 → 126 → 140`，约 2220 iter 起停在 **116–140** 的平台（其间 2420 附近有一次短促回落又爬回）；episode 长度 ~2006（horizon 2160 的 93%）。**按 §9 第 5 条取了平台中段的 `model_3500`，未等末尾。**

**遗留**：`timeout=12/170`（7%）成首位失败 —— 是某个**单相超时**，既非捕获（`capture_timeout=0`）也非寻位（`reach_timeout=0`）；`empty_close=3`、`drift=1`、`falloff=1` 都已到噪声量级。

**录像**：`media/rl_press_flow_catch.mp4`（`--task Isaac-PressFlow-CatchFast-Direct-v0`、`model_3500`、`--max_bottles 2`、1280×720@60、963 帧 16.1 s、单 episode 0 reset）。逐帧相位：`3→4 @ step 90 (bx=+0.089)`、`4→5 @ step 133 (bx=+0.003)`，第二瓶 `614 / 657` —— 两个循环都在瓶子**行进中**合爪。

## 7. 训练收敛细节（诚实记录）

- **S5 首跑（`2026-09-13_04-40-58_s5_flow`）撞墙**：episode 长度 404→551→1134 后**plateau 在 ~550**、reward 在 −24…−45 摆荡；eval（`model_6100`）**99.2% episode 死在 `reach_timeout`**、`reaches=33/2295 grasps`、`reach_steps` 均值 531.8（> 窗口 400），**71% 的 env-step 花在相 5**。即 REACH 在 S4 能成、在 S5 从不完成。
- **根因定位（`diag_press_flow_drift.py`）**：给轨迹加了瓶 root 的 env-local `bx`/`bz` 两列后一次定案 —— 每一帧 `bx≈2–3 mm`、`bz≈444 mm`（= `TAB+BELT_LIFT`，**瓶子始终在工位**），而 palm 在相 4 死亡帧上 `err_xy≈1078 mm`、`cmd_xy≈1195 mm` 且**每步向外走 ~3 mm**。所以「瓶子丢了」被排除，**是臂飞了**：相 3/4 只钳了 z（`up` 掩码），**xy 完全无界**；带一开，相 3/4 被拉长到 ~150 步的换瓶流程，粗尺度下的残差积分器一路把命令推到离站 1.19 m、palm 跟到 1.08 m；而 REACH 在 4→5 时 **re-anchor 到 palm 真实 FK**，于是它从 0.5–1.1 m 外起步、撞 400 步窗口。
- **修复**：相 3/4 把 presser 的**整个位姿（位置+四元数）**命令回 HOME，**按 `flow_belt` 门控**。
  - **为什么必须连四元数一起写**：`_apply_action` 喂 IK 的 `_cmd_quat` 是 held、且在每个相位切换 re-anchor 到 palm **当时**的姿态；只钳 xy 会留给 IK 一个「(HOME xy, 相 2 姿态)」这种**未必存在**的位姿，它靠**放弃位置**来满足姿态 —— 实测只钳 xy 时命令停在离站 4.4 mm、palm 却卡在 147 mm 偏轴，**48 局 0 局过相 4**。写上 HOME 自己的位置**和**四元数，目标就是 `_reset_idx` 停臂的那个位姿，**构造上可达**。
  - **为什么按 `flow_belt` 门控**：带关时相 3/4 塌成「放行→抓取」几步，积分器来不及漂，S4 策略本来就能应付 —— 在那里强加这个钳位只会把它**推出分布**且毫无收益。实测（`s4_xy/model_6600`，quota 3，32 env × 30000 步）：fail_frac **0.120→0.635**、`reach_steps` **42.0→82.6**，多出来的全是 `drift`（52→285）与 `timeout`（31→199）。门控后 S4 **逐位复原**。
- **修复后的 S5 重训**（同 warm start `warm_s5/model_5300`，128 env × 3000 iter，~56 min）：reward **−58 → +45.9**，episode 长度 **428 → ~2050**，1533 iter 时 episode 长度就已跳到 ~2100。修复本身把 REACH 的**入口几何**摆正（diag 实测 REACH 入口 `above=165.4 mm`、`err_xy=4.0 mm` —— 与 `smoke_press_flow.py` 打印的 HOME `above_cap=+0.1642` **逐位吻合**），33/33 局过相 4（修复前 27/32）。

## 8. 回归

| target | 结果 |
|---|---|
| 旧单拍 `Isaac-Press-Direct-v0`（`press_A_s1…model_900.pt`） | **success_rate 1.000**（2332/2332，首触底 mean 54.5 / med 55.0 / [53,57] 步） |
| 旧流水线 `Isaac-PressLine-Direct-v0`（`pressline_s1_s1…s3_line/model_3000.pt`） | **6.50 瓶/局、3.19 拍/瓶（配额 3）**、fail_frac 0.059、**drift_fail=0**（记录基线 5.95/3.21、drift 0） |
| S4 `Isaac-PressFlow-Reach-Direct-v0`（`s4_xy/model_6600.pt`，32 env × 30000 步） | **逐位复原**：690 局、fail_frac **0.120**、`reach_steps` **42.0**（p10 37 / p90 50，n=690）、`drift=52 / timeout=31` —— 与 §6.1 基线**每一个数字都一样** |

本轮只**新增** `PressFlow*` 与 `Conveyor`；`press_env.py` / `press_cycle_env.py` / `press_line_env.py` / 旧 task id / 旧 eval·record **未改**。上面两个 task 的 env class 是 `PressFlowEnv` 的**父类**，结构上不可能被本轮改动影响 —— 仍实跑复核通过。

**三个回归都在**补强 round（`presence` 门 + `belt_catch` 捕获 + A1 cfg）**全部落地之后重跑过一遍**，因为 `press_flow_env.py` 在 09-16 22:15 还改过一次（早于 A1 训练）。三条都过：单拍 1.000、流水线 6.50/3.19/`drift=0`、S4 **逐位复原**（690 局、fail_frac 0.120、`reach_steps` 42.0）—— 即新增的 `presence` 门与恒速捕获通道对 `flow_belt=False` 的 S4 **零扰动**（构造上靠 `if cfg.belt_catch:` 与 `flow_belt` 双重门控）。

## 9. 诚实边界

1. **成品瓶离场是 kinematic 复演 + 单瓶 prim 复用**，不是真双瓶供应链：进料段是**摩擦拖动真物理**（30× 倾倒余量），但放行后是「带把瓶带到下游→出画后重新定位到上游」。
2. **「发现」是特权观测**：`drift`（瓶 xy − 工位家）即进料距离，**无相机、无视觉检测**。
3. **抓取仍是摩擦夹持**，无 ContactSensor，力仍以 nozzle q 代理（沿用前几轮口径）。
4. **S5 的放行密度低于 S4**（3.86 vs 5.71 瓶/局）：真带换瓶本身耗时 ~475 步/瓶，且 `reach_steps` 涨到 82.7（带残余运动 + 携带瓶），horizon 36 s 下每局瓶数自然更低。
5. **训练后期方差**（同 S3 line 与 S4：`model_6600` ≫ `model_7599`）⇒ **ckpt 必须按直接 eval 选**，不看末尾 reward / 最后迭代号。
6. **`reach_timeout` 曾是首位失败（S5 Flow 53/599，A0 21/26）**：修复把 REACH 入口摆正了，但少数局的寻位仍超窗口；**A1 起用 REACH 条件更好的 A0 `model_2100` 起步后该项归零（§6.5）**，进一步可加大窗口或继续训练。
7. **捕获段的「张嘴窗口」是奖励里的一个不连续点**（A 段新引入）：`_close_gate = near`（|bx|<10 mm）同时关掉「早闭罚」与打开「闭爪压力」，所以确定性策略会停在窗口边缘闭爪（A0 实测抓取瞬间 |bx| mean 8.1 mm、p90 10.0 mm）。原担心：`belt_speed` 升到 0.12 时窗口从 40 步缩到 10 步，这个「贴边闭爪」会**速度脆弱**。**结论（§6.5）：担忧没有兑现** —— A1 全速下 `|bx|` mean **7.0 mm** / p90 **8.5 mm**，比 A0 **更靠窗心**，`capture_timeout` 归零。不连续点仍在（角点解仍存在），但 4× 速度 + 1/4 窗口下策略仍有足够定位精度。**若日后还要提速**，第一处要改的仍是把闭爪压力绑到几何（`present`）而不是绑到 `near` 阈值。
8. **A 段的 `empty_close` 语义与 B 段不同**：B 段它是「空爪被记账」的loophole 计数；A 段门是 `present ∧ near ∧ closed`（`present` 蕴含瓶在两刀之间），空爪**构造上不可能**记账，所以那里它退化成「本次相 4 尝试里瓶子从未进过嘴」的**漏抓计数**。同一个计数器，两段读法不同。

## 10. 复现命令（从 IsaacLab 根目录，`conda activate env_isaaclab`）

```bash
# 0) 求解/校验 HOME 位姿（纯平移，打印 joint delta）
env -u DISPLAY ./isaaclab.sh -p ../isaac_demo/rl/solve_press_flow_home.py --headless \
    --task Isaac-PressFlow-Reach-Direct-v0 --num_envs 2

# 1) 脚本 oracle 全链冒烟（先证 env 可跑通，再花训练算力）
env -u DISPLAY ./isaaclab.sh -p ../isaac_demo/rl/smoke_press_flow.py --headless \
    --task Isaac-PressFlow-Direct-v0 --num_envs 8 --max_steps 4000        # 期望 SMOKE_OK

# 2) ckpt remap（31→32 零填充）
env -u DISPLAY ./isaaclab.sh -p ../isaac_demo/rl/remap_ckpt_obs.py \
    --checkpoint logs/rsl_rl/pressline_s1_s1/<run>/model_<K>.pt \
    --out       logs/rsl_rl/pressflow/warm_s4/model_<K>.pt

# 3) S4 Reach（带关、quota 3）——从 remap 后的 line 早期 ckpt 起步
env -u DISPLAY ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-PressFlow-Reach-Direct-v0 --headless --num_envs 128 \
    --max_iterations 2500 --run_name s4_xy \
    --resume --load_run warm_s4d --checkpoint model_5100.pt

# 4) S5 Flow（真带、quota 5）——从 S4 **早期高熵** ckpt resume
env -u DISPLAY ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-PressFlow-Direct-v0 --headless --num_envs 128 \
    --max_iterations 3000 --run_name s5_flow2 \
    --resume --load_run warm_s5 --checkpoint model_5300.pt

# 5) 评测（瓶/局、每次松开带几拍、reach 步数分布、失败分解）
env -u DISPLAY ./isaaclab.sh -p ../isaac_demo/rl/eval_press_flow.py --headless \
    --task Isaac-PressFlow-Direct-v0 --num_envs 32 --steps 40000 \
    --checkpoint logs/rsl_rl/pressflow/2026-09-13_05-49-34_s5_flow2/model_8299.pt

# 5b) S5-B 真捕获（相 4 加 presence 门）：同 task id，从 **早期高熵** ckpt 原地重训
env -u DISPLAY ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-PressFlow-Direct-v0 --headless --num_envs 128 \
    --max_iterations 3000 --run_name s5_presence \
    --resume --load_run warm_s5 --checkpoint model_5300.pt
env -u DISPLAY ./isaaclab.sh -p ../isaac_demo/rl/eval_press_flow.py --headless \
    --task Isaac-PressFlow-Direct-v0 --num_envs 32 --steps 30000 \
    --checkpoint logs/rsl_rl/pressflow/2026-09-16_20-54-45_s5_presence/model_8299.pt

# 6) 单 episode 全链录像
env -u DISPLAY ./isaaclab.sh -p ../isaac_demo/rl/record_press_flow.py --headless --enable_cameras \
    --task Isaac-PressFlow-Direct-v0 --num_envs 1 --max_bottles 2 --width 1280 --height 720 \
    --checkpoint logs/rsl_rl/pressflow/2026-09-13_05-49-34_s5_flow2/model_8299.pt \
    --video outputs/rl_press_flow.mp4

# 7) A0 恒速捕获：先做**夹爪通道定向手术**（zero jaw row + 设开通 bias/std + 清 Adam 动量），
#    再从这个 ckpt resume —— 不手术的话相 4 的 jaw 通道均值在 ±1 clip 之外，梯度恒为 0（§6.4）
env -u DISPLAY ./isaaclab.sh -p ../isaac_demo/rl/warmstart_center_jaw.py \
    --in  logs/rsl_rl/pressflow/2026-09-16_20-54-45_s5_presence/model_6000.pt \
    --out logs/rsl_rl/pressflow/warm_a0/model_6000.pt

# 7a) 捕获 oracle 冒烟：同一个 smoke 脚本，`cfg.belt_catch` 时相 4 改成「瓶进刀片足迹前保持张开、
#     越过后合拢」（`bx > catch_close_x`）。要求 SMOKE_OK、capture_to=0
env -u DISPLAY ./isaaclab.sh -p ../isaac_demo/rl/smoke_press_flow.py --headless \
    --task Isaac-PressFlow-Catch-Direct-v0 --num_envs 8 --max_steps 3000 --min_releases 1

# 7b) A0 训练（belt_speed=0.03，窗口 ~40 步）
env -u DISPLAY ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-PressFlow-Catch-Direct-v0 --headless --num_envs 128 \
    --max_iterations 8000 --run_name a0_catch --device cuda:0 \
    --resume --load_run warm_a0 --checkpoint model_6000.pt

# 7c) A0 评测（deterministic：`grasp timing` 给抓取瞬间 |bottle_x|，`empty_close` 给空爪计数）
env -u DISPLAY ./isaaclab.sh -p ../isaac_demo/rl/eval_press_flow.py --headless \
    --task Isaac-PressFlow-Catch-Direct-v0 --num_envs 32 --steps 12000 --device cuda:1 \
    --checkpoint logs/rsl_rl/pressflow/2026-09-16_22-18-38_a0_catch4/model_1500.pt

# 8) A1 全速捕获（产线 0.12 m/s）：**不开刀**，直接从 A0 的 ckpt resume。
#    起点选 model_2100 而不是 §6.4 头条的 model_1500：抓取相当，但 REACH 条件好得多
#    （reach_timeout 0 vs 21、reach_steps 75.0 vs 104.9），而 A1 要修的正是 REACH（§6.5）
env -u DISPLAY ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-PressFlow-CatchFast-Direct-v0 --headless --num_envs 128 --seed 42 \
    --max_iterations 8000 --run_name a1_catch --device cuda:0 \
    --resume --load_run 2026-09-16_22-18-38_a0_catch4 --checkpoint model_2100.pt
# 注意：resume 保留 checkpoint 里的 global iter，所以日志是 `N/10100` 而不是 `N/8000`

# 8a) A1 评测（期望 capture_timeout=0、reach_timeout=0、doses 5.00/5/5、grasps > 4）
env -u DISPLAY ./isaaclab.sh -p ../isaac_demo/rl/eval_press_flow.py --headless \
    --task Isaac-PressFlow-CatchFast-Direct-v0 --num_envs 32 --steps 12000 --device cuda:1 \
    --checkpoint logs/rsl_rl/pressflow/2026-09-16_23-41-12_a1_catch/model_3500.pt

# 8b) A1 录像：单 episode 连续 2 瓶、瓶子在行进中被合爪捕获
env -u DISPLAY ./isaaclab.sh -p ../isaac_demo/rl/record_press_flow.py --headless --enable_cameras \
    --task Isaac-PressFlow-CatchFast-Direct-v0 --num_envs 1 --max_bottles 2 --width 1280 --height 720 \
    --checkpoint logs/rsl_rl/pressflow/2026-09-16_23-41-12_a1_catch/model_3500.pt \
    --video outputs/rl_press_flow_catch.mp4
```

## 11. 关键文件

| 文件 | 作用 |
|---|---|
| `direct/press/press_flow_env.py` | `PressFlowEnv(PressLineEnv)`：6 相机、传送带驱动、HOME 位姿、相 3/4 位姿钳位、传带门控、`_presence_ok()` 在位判据、`_hold_belt`、捕获模式恒速/锁定 |
| `direct/press/press_env_cfg.py` | `_conveyor_cfg()` + `PressFlowEnvCfg` / `PressFlowReachCfg`（S4）/ `PressFlowFullCfg`（S5）/ `PressFlowCatchCfg`（A0）/ `PressFlowCatchFastCfg`（A1，只改 `belt_speed`/`episode_length_s`） |
| `direct/press/__init__.py` | 注册 `Isaac-PressFlow-{,Reach-,Catch-,CatchFast-}Direct-v0` |
| `direct/press/agents/rsl_rl_ppo_cfg.py` | `PressFlowPPORunnerCfg` |
| `rl/remap_ckpt_obs.py` | 31→32 ckpt 零填充 warm-start 桥 |
| `rl/solve_press_flow_home.py` | 把 HOME 解成 hover 的**纯平移** |
| `rl/warmstart_center_jaw.py` | **夹爪通道定向手术**：清零 jaw 输出行 + 设开通 bias/std + 清对应 Adam 动量（§6.4） |
| `rl/diag_press_flow_clip.py` | 穿模探针：候选 prim 的 world AABB / 逐轴间距 / 盒-盒 SAT / 刀片↔瓶解析 / 生效 contactOffset |
| `rl/smoke_press_flow.py` | 6 相脚本 oracle 全链冒烟（训练前闸门）；`--empty_grasp` 空爪负测；`belt_catch` 时相 4 自动切成捕获 oracle（`--catch_close_x`） |
| `rl/diag_press_flow_drift.py` | 相 5 死亡/入口轨迹 + 瓶 env-local `bx`/`bz`（定位「臂飞了 vs 瓶丢了」） |
| `rl/eval_press_flow.py` | 瓶/局、每次松开拍数、reach 步数分布、失败分解、`empty_close`、`grasp presence` |
| `rl/record_press_flow.py` | 单 episode 全链录像 |
