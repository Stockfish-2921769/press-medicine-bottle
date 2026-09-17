# press-medicine-bottle

灵巧手按压药瓶喷嘴仿真 Demo（独立任务，与 CerebVLA / SomaVLA 项目无关）。纯仿真，环境 = Isaac Sim 5.1 / Isaac Lab。

**当前状态（2026-09-17）：PressFlow 补强 round（真捕获判据 + 恒速捕获 B/A0/A1，完成）** —— 用户口径「流水线的位置和夹爪有**穿模**，同时应该**训练夹爪自己捕获在流水线上的药瓶**」。① **穿模先测再改**：新写探针 `rl/diag_press_flow_clip.py`（world AABB 逐轴间距 + 带↔刀片 **15 轴 SAT 精确 MTD** + 刀片↔瓶解析 + 生效 `contactOffset`），`--oracle`/`--policy` 双模式跑完**没有任何一对 `pen_depth` 转负** ⇒ 判定为共面渲染伪影，**几何不改**。② **B 段（相 4 真捕获判据）**：原门 `jaw≤th ∧ drift_ok` **不含「瓶在位」**、而刀片目标非 3/4 相恒为 0 ⇒ **空爪合拢也记一次 grasp**；加特权几何判据 `_presence_ok()` 后 **`empty_close=0`**、grasps 4.71→**4.19 瓶/局(−11%)** 而 releases 3.86→**3.89 持平**（剔掉的是假抓）、fail_frac 0.117→**0.035**、`doses at release 5.00/5/5` 保持。③ **A 段（带不许停，夹爪自己在流水线上抓）**：暖启在 S5-B ckpt 上三次跑**冻结**在 episode 长度 497.04 —— 确诊相 4 jaw 动作均值 **+3.8 在 `±1` clip 之外 ⇒ `E[(a−μ)/σ²·A] ≡ 0`，梯度恒零**（reward shaping 救不了，要动的是**分布的均值**）；改用**夹爪通道定向手术** `rl/warmstart_center_jaw.py`（清零 jaw 输出行 + 设开通 bias/std + 清该三张量的 Adam 动量 + `iter`/LR 复位），配 `catch_grip_dwell=1` 与 `rew_capture_open 0.0035→0.025`（wedge 必须贵于「开着等」）。**A0（带速 0.03）头条结果：抓移动中的瓶子 3.89 瓶/局 = 静止基线 4.19 的 93%**、`capture_timeout` **581/615 → 2/110**、**`doses at release 5.00/5/5`（n=396）一字不差**、抓取瞬间 **`bottle_speed 0.019 m/s`（瓶子在动，非停稳后夹）**。④ **A1（产线全速 0.12 m/s）**：同一 env 只把 `belt_speed` 调回 0.12（记账窗自动 40→10 步、相 4 deadline 自动 265→115 步），从 A0 `model_2100` 微调 ⇒ **`grasps 4.28 瓶/局` 反超静止基线 4.19、`releases 4.06`、`fail_frac 0.082`（A0 的 1/3）、`capture_timeout`/`reach_timeout` 双双归零、`doses at release 5.00/5/5`（n=691）**、抓取瞬间 **`bottle_speed mean 0.032 / p90 0.120 m/s`**。A0 的两条担忧（`reach_timeout` 首位失败、抓取 `|bx|` 贴 10 mm 容差外沿）**全速下都没有兑现** —— A1 反而更靠窗心（mean 7.0 / p90 8.5 mm）。上游：长程流水线（PressFlow）每次松开带满 5 拍；异构瓶泛化 **异构 dive 6 档全 100%**（677/677），零样本投递分级退化，异构 dose-loop 训不成（负，根因未定位）；PressLine s1 **5.95 瓶/局、3.21 拍/瓶**；PressCycle s1 ~15.9 拍/局；单拍微技能 Stage A 两几何各 100%。
详细进展报告见 **`docs/PROGRESS.md`**；逐轮可行性结论见 `docs/feasibility_*.md`；**长程流水线按压总结见 `docs/rl_learning_press_flow.md`**，异构瓶泛化总结见 `docs/rl_learning_press_hetero.md`，流水线换瓶 RL 总结见 `docs/rl_learning_press_line.md`（设计 `docs/rl_press_line_design.md`），闭环连续按压总结见 `docs/rl_learning_press_cycle.md`，单拍 RL 总结见 `docs/rl_learning_press.md`。

## 现状一句话

Panda 长爪（西侧，ALOHA 式 2 指平爪）**钳住自由站异形药瓶**（Ø42、弹簧喷嘴 K300/行程 5mm），
iiwa7+Shadow **掌心(pad)朝下压 cap 顶**，nozzle 行程到底（91% travel）→ pad `ContactSensor` 法向力 + 弹簧反力作
**触觉判读** → `tactile_verdict=USABLE`，并记录**触发瞬间力**（pad 1.13–1.19 N / 弹簧反力 ~1.37 N）。
随后按用户口径把**按压臂等比缩小 s=0.7 + 0.28 m 垫座立柱**重验通过（`USABLE`，`media/mixed_press_scaled.mp4`）。
RL round 把规则下压换成 **PPO 真学**（单拍微技能 → 闭环同瓶多拍连续循环，面向自动化流水线部署）。

视频：
- `media/rl_press_flow_catch.mp4` —— **流水线全速捕获**（PressFlow A1，`belt_speed=0.12 m/s`）：RL 策略**在带子不停的前提下**自己捕获进料中的药瓶（相 3→4 @ step 90 `bx=+0.089`、4→5 @ step 133 `bx=+0.003`，抓取瞬间瓶子仍以产线速度在动）→ 寻位 → 5 拍 → 放行 → 第二瓶（614/657），单 episode 2 次松开 0 reset，1280×720@60fps、963 帧
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
  rl_learning_press_flow.md     # 长程流水线 round：6 相 MDP + 真传送带 + HOME 寻位 + 5 拍松开（2026-09-12）；补强：真捕获 B 段 + 恒速捕获 A0/A1（2026-09-16/17）
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
  rl_press_flow_catch.mp4 # 流水线全速捕获（PressFlow A1，带速 0.12 m/s）：单 episode 2 次捕获+松开、0 reset（见 rl_learning_press_flow.md §6.5）
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
| **PressFlow 补强（真捕获 + 恒速捕获）** | 09-16/17 | ✅ 穿模探针判定**无真穿模**（共面伪影）；**B**：相 4 加「瓶在位」几何门 ⇒ **`empty_close=0`**、grasps 4.71→4.19(−11%) 而 releases 持平、fail_frac 0.117→**0.035**；**A0**（带速 0.03）：抓**移动中**的瓶子 **3.89 瓶/局 = 静止基线 4.19 的 93%**、`capture_timeout` 581/615→**2/110**；**A1**（产线全速 0.12）：**grasps 4.28 瓶/局（反超静止基线）**、`fail_frac 0.082`、`capture_timeout`/`reach_timeout` **双双归零**、`doses at release 5.00/5/5`（n=691）（`docs/rl_learning_press_flow.md` §6.3–§6.5，视频 `media/rl_press_flow_catch.mp4`） |

> 任务计划书为 5 阶段；Phase 3（RL/PPO 学习按压）本轮已完成（当前是计划书中"真学按压策略"的落地，非规则脚本）。
> 遗留/边界与建议下一步详见 `docs/PROGRESS.md` §10。
