# 灵巧手按压药瓶喷嘴仿真 Demo —— 项目进展报告

> 报告日期：2026-09-09
> 任务性质：纯仿真（Isaac Sim 5.1 / Isaac Lab）Demo，独立项目，与 CerebVLA / SomaVLA 无关。
> 当前落地：**流水线换瓶按压（PressLine）round** —— PPO 真学「剂量配额多拍 + 长爪开/合换瓶（第 4 DoF）」单工位连续多瓶（单 episode 连续 4 次换瓶 0 reset 视频、s1 nominal 5.95 瓶/局、3.21 拍/瓶，见 §11 / `docs/rl_learning_press_line.md`，视频 `media/rl_press_line_s1.mp4`）。上游闭环连续按压（PressCycle）：同瓶多拍 s1 ~15.9 拍/局（见 §9 / `rl_learning_press_cycle.md`）；单拍微技能：Stage A 两几何 100%、Stage B 把 s1 DR 下 93.3% 补到 100%（见 §8 / `rl_learning_press.md`）。
> 前序地面真值 demo：**混形双手方案**（Panda 长爪钳自由瓶 + Shadow 掌心朝下压 cap，触觉判读 `USABLE`）与**按压臂等比缩放 s=0.7 + 垫座**重验均通过。
> 明细文档：`docs/feasibility_*.md`（按轮次归档）+ **`docs/rl_learning_press_cycle.md`（闭环连续 RL 总结）** / `docs/rl_learning_press.md`（单拍 RL 总结）；脚本/资产/视频见仓库目录。

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

> 注：任务计划书为 5 阶段（Phase 0 环境 / 1 场景 / 2 IK / 3 RL·PPO / 4 集成·视频·报告）。
> **Phase 3（RL/PPO 学习按压）已于 2026-09-09 落地**（真学策略，非规则脚本）：单拍微技能 MDP 表/结果/边界见 §8 与 `docs/rl_learning_press.md`；**闭环连续按压（PressCycle，同瓶多拍）见 §9 与 `docs/rl_learning_press_cycle.md`**；**流水线换瓶按压（PressLine，剂量配额 + RL 学长爪换瓶）见 §11 与 `docs/rl_learning_press_line.md`**。
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
  rl_press_s1.mp4 / rl_press_s07.mp4 / rl_press_s1B.mp4  # RL 策略滚动（1280×720@60fps）
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
- **边界（诚实）**：① 跨瓶高零样本为负 —— zc0565(短 20mm) 0.43 瓶/局(超时主因)、zc0605(长 20mm) 0.07 瓶/局(falloff 主因)；s07(0.7+riser) 0.01 瓶/局。策略的绝对任务空间残差按 nominal snap 校准，不跨几何/尺度泛化（要「不同瓶子」混线需逐几何训练或几何 DR）。② 混合几何单段连续 take 不受 instanced-env 架构支持（GPU instanced 同模板同几何 + 流水线串行单工位）。③ 换瓶「离/到」为 conveyor 事件建模（写平衡位 + drift 归零），sim 无传送带动画；爪开/合时机与抓稳仍真学。④ ~12.6% 局以 falloff/超时告终，多在 horizon 附近。⑤ 训练不稳定（S3 双峰摆荡）。
- **代码落点（不入本仓库，依赖本机 IsaacLab editable 布局）**：`direct/press/press_line_env.py` + `PressLine*` cfg（`press_env_cfg.py`）+ task id `Isaac-PressLine-{Dive,Cyclic,Direct,56-Direct,60-Direct,07-Direct}-v0`；harness `rl/{smoke,eval,record}_press_line.py`；瓶 snap `rl/snapshots/recenter_{s1.00,s0.70,zc0565,zc0605}.npz`。复现命令见 `rl_learning_press_line.md` §9。
