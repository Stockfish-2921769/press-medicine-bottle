# press-medicine-bottle

灵巧手按压药瓶喷嘴仿真 Demo（独立任务，与 CerebVLA / SomaVLA 项目无关）。纯仿真，环境 = Isaac Sim 5.1 / Isaac Lab。

**当前状态（2026-09-12）：长程流水线按压（PressFlow）round 完成** —— 用户口径「没有去做**从发现自动化流水线的瓶子，到抓取到机械臂+灵巧手寻找药瓶并且按压，5 次按压后松开**的过程」按 6 相 MDP + **真传送带 prim** 落地：瓶子随带**摩擦拖动**进料 → 长爪在工位**抓住** → presser 从 **HOME** 位**自己寻位**到瓶口上方 → 按压 **5** 拍 → **开爪撤臂放行** → 下一瓶。**头条结果：每次松开都带满配额 5 拍**（`doses at release mean=5.00 min=5 max=5`，n=2310、`release_quota_frac=1.000`）、`drift_fail=falloff=grasp_timeout=0`、3.86 瓶/局、fail_frac 11.7%；录像单 episode 连续 2 瓶、0 reset。**过程性负结果（已修，诚实记录）**：S5 首跑 99.2% episode 死在 `reach_timeout` —— 根因是**真带把相 3/4 拉长到 ~150 步而 presser 的 xy 命令无界**，臂飞到离站 1.08 m 外、而 REACH 在相 4→5 re-anchor 到 palm 真实 FK 就从那里起步；改为相 3/4 把**整个位姿**命令回 HOME（按 `flow_belt` 门控，S4 评测逐位不受影响）后收敛。上游：异构瓶泛化（PressLine-Hetero）**异构 dive 6 档全 100%**（677/677），零样本投递分级退化（1.27 瓶/局、非单调），异构 dose-loop 专家训不成（负，根因未定位）；PressLine round：s1 nominal **5.95 瓶/局、每瓶 3.21 拍（配额 3）**、视频单 episode 连续 4 次换瓶 0 reset；PressCycle round：同瓶多拍循环 s1 ~15.9 拍/局；单拍微技能 Stage A 两几何各 100%。
详细进展报告见 **`docs/PROGRESS.md`**；逐轮可行性结论见 `docs/feasibility_*.md`；**长程流水线按压总结见 `docs/rl_learning_press_flow.md`**，异构瓶泛化总结见 `docs/rl_learning_press_hetero.md`，流水线换瓶 RL 总结见 `docs/rl_learning_press_line.md`（设计 `docs/rl_press_line_design.md`），闭环连续按压总结见 `docs/rl_learning_press_cycle.md`，单拍 RL 总结见 `docs/rl_learning_press.md`。

## 现状一句话

Panda 长爪（西侧，ALOHA 式 2 指平爪）**钳住自由站异形药瓶**（Ø42、弹簧喷嘴 K300/行程 5mm），
iiwa7+Shadow **掌心(pad)朝下压 cap 顶**，nozzle 行程到底（91% travel）→ pad `ContactSensor` 法向力 + 弹簧反力作
**触觉判读** → `tactile_verdict=USABLE`，并记录**触发瞬间力**（pad 1.13–1.19 N / 弹簧反力 ~1.37 N）。
随后按用户口径把**按压臂等比缩小 s=0.7 + 0.28 m 垫座立柱**重验通过（`USABLE`，`media/mixed_press_scaled.mp4`）。
RL round 把规则下压换成 **PPO 真学**（单拍微技能 → 闭环同瓶多拍连续循环，面向自动化流水线部署）。

视频：
- `media/rl_press_flow.mp4` —— **长程流水线全链**（PressFlow）：RL 策略单 episode **连续 2 瓶、0 reset** —— 瓶子随传送带进料 → 长爪抓取 → presser 从 HOME 寻位 → **按压 5 拍** → 开爪撤臂放行 → 下一瓶（每次松开 `doses on that bottle=5`，release 于 step 407/901），1280×720@60fps、901 帧 15.0 s
- `media/rl_press_hetero_dive.mp4` —— **异构瓶泛化**（PressLine-Hetero）：**同一策略 6 档瓶高同屏** 2×3 拼图（`zc0565 -20mm` → `zc0660 +75mm`，瓶高跨度 95 mm），每格都是该档瓶的一个完整下压序列，1920×1620@60fps、294 帧（逐档源片段 `het_env{0..5}.mp4`）
- `media/rl_press_line_s1.mp4` —— **流水线换瓶按压**（PressLine，s1 nominal）：RL 策略单 episode 同一工位**连续 4 次换瓶**（每次 3 拍达额后爪张开放瓶→新瓶→合爪抓稳）、0 reset，1280×720@60fps、704 帧
- `media/rl_press_cycle_s1.mp4` —— **闭环连续按压**（PressCycle，s1 LiftEasy）：RL 策略单 episode 同一 clamp 瓶连续 8 拍、0 reset，1280×720@60fps、409 帧
- `media/rl_press_s1.mp4` / `media/rl_press_s07.mp4` / `media/rl_press_s1B.mp4` —— RL 单拍策略滚动（Stage A 两几何 + Stage B DR，1280×720@60fps，各含 3 次成功按压+自动 reset）
- `media/mixed_press.mp4` —— 混形双手 palm-down（s=1.0，21.15s 960×540）
- `media/mixed_press_scaled.mp4` —— 缩放 round（s=0.7 + 垫座，~20.8s 960×540）
- `media/press.mp4` —— Phase 1 食指按压（历史）

## 目录结构

```
scripts/                  # 可运行脚本（与 assets/ 同层，资产相对路径，克隆即用）
  demo_mixed_press.py     # 混形双手 palm-down demo（--yaw=-90；缩放 --scale/--px/--py/--pz）
  probe_mixed_reach.py    # 混形/缩放可达性扫描（--scale --zlist --hover_mm；输出 VERDICT_SCALE）
  _kuka_shadow_cfg.py     # iiwa7+Shadow 单条 articulation 配置
  _panda_longjaw_cfg.py   # Panda + 自建 ALOHA 式长爪配置
  phase1_*.py 等          # Phase 1（Allegro 食指压直立瓶）历史脚本
assets/
  kuka_shadow.usda        # 按压臂（薄引用 kuka.usd + shadow_hand.usd）
  panda_longjaw.usda      # Panda 长爪
  hetero_bottle_chosen_free.usda  # 自由站异形瓶（当前 demo 用）
  medicine_bottle.usda    # Phase 1 直立瓶（历史）
tools/                    # USD authoring（生成上列资产）
docs/
  PROGRESS.md             # ★ 项目进展总览（里程碑/当前方案/复现/遗留）
  rl_learning_press_flow.md     # 长程流水线 round：6 相 MDP + 真传送带 + HOME 寻位 + 5 拍松开（2026-09-12）
  rl_learning_press_hetero.md   # 异构瓶 round：每 env 异构瓶架构 + 6 档瓶高 dive 100% + 零样本分级 + dose-loop 负结果（2026-09-10）
  rl_learning_press_line.md     # 流水线换瓶 round：剂量配额 + RL 学夹爪换瓶 MDP + 总结（2026-09-09）
  rl_press_line_design.md       # 流水线换瓶 round：设计（几何可行域 → 5 相 MDP/课程）（2026-09-09）
  rl_learning_press_cycle.md    # 闭环连续按压 round：同瓶多拍相位 MDP + 总结（2026-09-09）
  rl_learning_press.md    # RL/PPO 学习按压 round：MDP 一览表 + 总结（2026-09-09）
  feasibility_mixed_press.md    # 混形双手 + palm-down 修正 + 缩放 round + RL round §10（2026-09-08/09）
  feasibility_shadow_singlehand.md  # Shadow 单臂握持+按压 → 负（2026-09-07）
  geometry_envelope_hetero.md    # 几何可行域反推（2026-09-05）
  feasibility_hetero_press.md    # 异形长指按压/选定瓶 → 正(演示口径)（2026-09-04）
  feasibility_grasp_thumb_press.md # Allegro 握持+拇指压 → 负（2026-09-04）
media/
  rl_press_flow.mp4       # 长程流水线全链（PressFlow）：单 episode 连续 2 瓶、每次 5 拍后松开、0 reset（见 rl_learning_press_flow.md §6）
  rl_press_hetero_dive.mp4 # 异构瓶泛化（PressLine-Hetero）：同一策略 6 档瓶高同屏 2×3 拼图（见 rl_learning_press_hetero.md §9）
  rl_press_line_s1.mp4    # 流水线换瓶（PressLine，s1）：单 episode 连续 4 次换瓶 0 reset（见 rl_learning_press_line.md §4）
  press.mp4               # Phase 1 食指按压
  mixed_press.mp4         # 混形双手 palm-down（s=1.0）
  mixed_press_scaled.mp4  # 缩放 round（s=0.7 + 垫座）
  rl_press_cycle_s1.mp4   # 闭环连续按压（PressCycle，s1）：同瓶连续 8 拍 0 reset
  rl_press_s1.mp4 / rl_press_s07.mp4 / rl_press_s1B.mp4  # RL 策略滚动（见 rl_learning_press.md §6）
```

## 运行环境与复现

- Isaac Sim 5.1.0.0 (pip) / Isaac Lab `main`，运行 `./isaaclab.sh -p <脚本>`。
- 资产相对路径（`scripts/` 与 `assets/` 同层），克隆后即可运行。
- 完整复现命令与期望量测见 `docs/PROGRESS.md` §6（demo 两口径：s=1.0 基线 / s=0.7 缩放）。

```bash
conda activate env_isaaclab
cd <IsaacLab 根目录>
# 混形双手 palm-down
env -u DISPLAY ./isaaclab.sh -p /path/to/press_repo/scripts/demo_mixed_press.py --headless \
    --enable_cameras --ns 10 --step 260 --yaw=-90 --video /path/out.mp4
# 缩放 round（按压臂 ×0.7 + 0.28 m 垫座）
env -u DISPLAY ./isaaclab.sh -p /path/to/press_repo/scripts/demo_mixed_press.py --headless \
    --enable_cameras --ns 10 --step 260 --yaw=-90 --px -0.205 --py -0.026 --pz 0.28 --scale 0.7 \
    --video /path/out_scaled.mp4
```

## 任务沿革（简述）

| 轮次 | 日期 | 结论 |
|---|---|---|
| Phase 1：Allegro 食指过顶压直立瓶 | 09-03/04 | ✅ 按压到底 + 回弹（`media/press.mp4`，commit `0c0bdc2`） |
| Allegro「握持 + 拇指压顶」 | 09-04 | ❌ 几何不可行 |
| 异形长指按压 + 选定瓶 | 09-04 | ✅ base-fixed 演示口径 |
| 几何尺寸可行域反推 | 09-05 | ✅ 可行域窗口 + 定量不变量 |
| Shadow 单臂「握持 + 拇指压」 | 09-07 | ❌ 负结论 |
| **混形双手 palm-down** | 09-08 | ✅ `USABLE`（`media/mixed_press.mp4`） |
| **按压臂缩放 s=0.7 + 垫座** | 09-08 | ✅ `USABLE`（`media/mixed_press_scaled.mp4`） |
| **RL/PPO 学习按压（Phase 3）** | 09-09 | ✅ 两几何 100% + 双向零样本 100% + Stage B 课程（`docs/rl_learning_press.md`，视频 `media/rl_press_*.mp4`） |
| **闭环连续按压 PressCycle** | 09-09 | ✅ 同瓶多拍循环 s1 ~15.9 拍/局（`docs/rl_learning_press_cycle.md`，视频 `media/rl_press_cycle_s1.mp4`） |
| **流水线换瓶 PressLine** | 09-09 | ✅ s1 nominal 5.95 瓶/局、3.21 拍/瓶（配额 3）；跨瓶高/跨尺度零样本为负（`docs/rl_learning_press_line.md`，视频 `media/rl_press_line_s1.mp4`） |
| **异构瓶泛化 PressLine-Hetero** | 09-10 | ◐ **每 env 异构瓶**架构成立；from-scratch 异构 dive **6 档瓶高全 100%**（677/677）；nominal 零样本分级退化（1.27 瓶/局、非单调）；异构 dose-loop 专家**训不成**（负，根因未定位）（`docs/rl_learning_press_hetero.md`，视频 `media/rl_press_hetero_dive.mp4`） |
| **长程流水线按压 PressFlow** | 09-12 | ✅ 真传送带进料 → 长爪抓取 → HOME 寻位 → **5 拍** → 开爪放行；**每次松开带满 5 拍**、drift_fail=falloff=grasp_timeout=0、3.86 瓶/局；录像单 episode 连续 2 瓶 0 reset（`docs/rl_learning_press_flow.md`，视频 `media/rl_press_flow.mp4`） |

> 任务计划书为 5 阶段；Phase 3（RL/PPO 学习按压）本轮已完成（当前是计划书中"真学按压策略"的落地，非规则脚本）。
> 遗留/边界与建议下一步详见 `docs/PROGRESS.md` §10。
