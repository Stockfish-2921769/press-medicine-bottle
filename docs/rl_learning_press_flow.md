# 长程流水线按压（PressFlow）：上料 → 抓取 → 寻位 → 按压×5 → 松开 RL（round 总结）

> 状态（2026-09-12）：**完成**。把前几轮**分别**训成的两个技能接成一条长程 episode：真实传送带把瓶子送进工位 → 长爪抓住 → presser 从 **HOME** 位自己寻到瓶口上方 hover → 按压 **5** 拍（配额 5）→ 开爪撤臂放行 → 下一瓶。批量评测（32 env × 40000 步，`model_8299`）：**每次松开都带满配额 5 拍**（`doses at release mean=5.00 min=5 max=5`，n=2310）、`release_quota_frac=1.000`、3.86 瓶/局、fail_frac 11.7%、**drift_fail=0 / falloff=0 / grasp_timeout=0**。诚实边界：瓶子进料是**摩擦拖动真物理**，但成品瓶离场是 kinematic 复演 + **单瓶 prim 复用**（不是真双瓶供应链）；「发现」是**特权观测**（`drift` 即进料距离），无相机；抓取仍是摩擦夹持（无 ContactSensor，力以 nozzle q 代理）。

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
| 4 | EXCH_CLOSE = **抓取** | 长爪在工位合拢抓住新瓶 | `jaw≤jaw_close_th ∧ drift_ok` 连续 `grip_dwell=5` → 5 |
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

**prim**：`Conveyor` = `RigidObjectCfg` + `CuboidCfg`（kinematic、`disable_gravity`），尺寸 `(0.60, 0.14, 0.02)`，顶面比桌面高 `BELT_LIFT=4 mm`，env-local 中心在工位处。`flow_belt=False` 时**根本不注册** —— 不是「关掉」：slab 是 dynamic 且顶面比桌面高 4 mm，而无带 handoff 把瓶子 re-seat 到桌面位姿，会让瓶子下半 3.4 mm 插进 slab，PhysX 解穿模时把两者一起弹飞。

**输送 = 摩擦拖动（真物理）**：`_write_belt_pose` 同时写**位姿**（把 slab 钉在轨道上）与**速度**（摩擦唯一的作用对象；只写位姿等于瞬移，对 PhysX 就是一个静止表面）。`_drive_belt` 从 `_apply_action` 调用（**每个物理子步**推进，不是每个控制步），速度取 `clamp(belt_gain·(目标−瓶x), ±belt_speed)`，`belt_speed=0.12`、`bel_accel=0.10`、`belt_gain=2.0`。

> **安全性有量化余量**：Ø42 / 0.25 kg 直立圆柱被摩擦拖动的**倾倒阈值** ≈ `m·g·r/(h/2)` ≈ 0.74 N，对应加速度 ≈ **2.9 m/s²**；工作加速度 0.10 m/s²（**30× 余量**）。**不需要导轨**。

单瓶 prim 复用：放行时带把成品瓶沿 +x 带到下游，出画后 `_recycle_bottle` 把它定位回上游供下一轮。

## 5. 训练课程（沿「未收敛 checkpoint resume」既有先例）

| 段 | task id | 起点 | 语义 | 结果 |
|---|---|---|---|---|
| S4 Reach | `Isaac-PressFlow-Reach-Direct-v0` | remap 后的 line `pressline_s1_s1/…/s3_line` **早期 ckpt** | 带关（瓶子仍瞬移就位）、quota 3、`episode_length_s=24`；臂从 **HOME** 起、必须自己寻到 hover | ✅ 收敛；`2026-09-13_03-47-42_s4_xy/model_6600.pt` |
| S5 Flow | `Isaac-PressFlow-Direct-v0` | S4 **早期高熵 ckpt**（`warm_s5/model_5300.pt`） | 开真带 + 抓取 + 放行 + quota 5、`episode_length_s=36` | ✅ 收敛；`2026-09-13_05-49-34_s5_flow2/model_8299.pt` |

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

本轮只**新增** `PressFlow*` 与 `Conveyor`；`press_env.py` / `press_cycle_env.py` / `press_line_env.py` / 旧 task id / 旧 eval·record **未改**。上面两个 task 的 env class 是 `PressFlowEnv` 的**父类**，结构上不可能被本轮改动影响 —— 仍实跑复核通过。另：新钳位按 `flow_belt` 门控后，S4（带关）评测与本轮改动前**逐位一致**（690 局、fail_frac 0.120、`reach_steps` 42.0）。

## 9. 诚实边界

1. **成品瓶离场是 kinematic 复演 + 单瓶 prim 复用**，不是真双瓶供应链：进料段是**摩擦拖动真物理**（30× 倾倒余量），但放行后是「带把瓶带到下游→出画后重新定位到上游」。
2. **「发现」是特权观测**：`drift`（瓶 xy − 工位家）即进料距离，**无相机、无视觉检测**。
3. **抓取仍是摩擦夹持**，无 ContactSensor，力仍以 nozzle q 代理（沿用前几轮口径）。
4. **S5 的放行密度低于 S4**（3.86 vs 5.71 瓶/局）：真带换瓶本身耗时 ~475 步/瓶，且 `reach_steps` 涨到 82.7（带残余运动 + 携带瓶），horizon 36 s 下每局瓶数自然更低。
5. **训练后期方差**（同 S3 line 与 S4：`model_6600` ≫ `model_7599`）⇒ **ckpt 必须按直接 eval 选**，不看末尾 reward / 最后迭代号。
6. **`reach_timeout` 仍是首位失败（53/599）**：修复把 REACH 入口摆正了，但少数局的寻位仍超窗口；进一步可加大窗口或继续训练。

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

# 6) 单 episode 全链录像
env -u DISPLAY ./isaaclab.sh -p ../isaac_demo/rl/record_press_flow.py --headless --enable_cameras \
    --task Isaac-PressFlow-Direct-v0 --num_envs 1 --max_bottles 2 --width 1280 --height 720 \
    --checkpoint logs/rsl_rl/pressflow/2026-09-13_05-49-34_s5_flow2/model_8299.pt \
    --video outputs/rl_press_flow.mp4
```

## 11. 关键文件

| 文件 | 作用 |
|---|---|
| `direct/press/press_flow_env.py` | `PressFlowEnv(PressLineEnv)`：6 相机、传送带驱动、HOME 位姿、相 3/4 位姿钳位、传带门控 |
| `direct/press/press_env_cfg.py` | `_conveyor_cfg()` + `PressFlowEnvCfg` / `PressFlowReachCfg`（S4）/ `PressFlowFullCfg`（S5） |
| `direct/press/__init__.py` | 注册 `Isaac-PressFlow-{,Reach-}Direct-v0` |
| `direct/press/agents/rsl_rl_ppo_cfg.py` | `PressFlowPPORunnerCfg` |
| `rl/remap_ckpt_obs.py` | 31→32 ckpt 零填充 warm-start 桥 |
| `rl/solve_press_flow_home.py` | 把 HOME 解成 hover 的**纯平移** |
| `rl/smoke_press_flow.py` | 6 相脚本 oracle 全链冒烟（训练前闸门） |
| `rl/diag_press_flow_drift.py` | 相 5 死亡/入口轨迹 + 瓶 env-local `bx`/`bz`（定位「臂飞了 vs 瓶丢了」） |
| `rl/eval_press_flow.py` | 瓶/局、每次松开拍数、reach 步数分布、失败分解 |
| `rl/record_press_flow.py` | 单 episode 全链录像 |
