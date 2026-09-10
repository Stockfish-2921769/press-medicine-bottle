# press-medicine-bottle

灵巧手按压药瓶喷嘴仿真 Demo（独立任务，与 CerebVLA / SomaVLA 项目无关）。纯仿真，环境 = Isaac Sim 5.1 / Isaac Lab。

**当前状态（2026-09-09）：流水线换瓶按压（PressLine）round 完成** —— 在剂量-配额多拍基础上，**长爪开/合（换瓶）也交给 RL 真学**（第 4 个 action DoF）：一瓶达额 3 拍即自动张爪释放 → 新瓶就位 → 合爪抓稳 → 再压下一瓶，**单工位连续多瓶**。s1 nominal **5.95 瓶/局、每瓶 3.21 拍（配额 3）**、87% episode 撑满 1200 步、drift_fail=0；视频 = **单 episode 连续 4 次换瓶、0 reset**。诚实边界：跨瓶高 / 跨尺度（s07）零样本为负（策略绑 nominal 几何）。上游 PressCycle round：同瓶多拍循环 s1 ~15.9 拍/局；单拍微技能 Stage A 两几何各 100%。
详细进展报告见 **`docs/PROGRESS.md`**；逐轮可行性结论见 `docs/feasibility_*.md`；**流水线换瓶 RL 总结见 `docs/rl_learning_press_line.md`**（设计 `docs/rl_press_line_design.md`），闭环连续按压总结见 `docs/rl_learning_press_cycle.md`，单拍 RL 总结见 `docs/rl_learning_press.md`。

## 现状一句话

Panda 长爪（西侧，ALOHA 式 2 指平爪）**钳住自由站异形药瓶**（Ø42、弹簧喷嘴 K300/行程 5mm），
iiwa7+Shadow **掌心(pad)朝下压 cap 顶**，nozzle 行程到底（91% travel）→ pad `ContactSensor` 法向力 + 弹簧反力作
**触觉判读** → `tactile_verdict=USABLE`，并记录**触发瞬间力**（pad 1.13–1.19 N / 弹簧反力 ~1.37 N）。
随后按用户口径把**按压臂等比缩小 s=0.7 + 0.28 m 垫座立柱**重验通过（`USABLE`，`media/mixed_press_scaled.mp4`）。
RL round 把规则下压换成 **PPO 真学**（单拍微技能 → 闭环同瓶多拍连续循环，面向自动化流水线部署）。

视频：
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

> 任务计划书为 5 阶段；Phase 3（RL/PPO 学习按压）本轮已完成（当前是计划书中"真学按压策略"的落地，非规则脚本）。
> 遗留/边界与建议下一步详见 `docs/PROGRESS.md` §9。
