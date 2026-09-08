# 异形药瓶「长指按压」可行性扫描与选定瓶演示

> 状态：**可行性已定性 + 演示跑通（base-fixed 口径）**
> 日期：2026-09-04
> 评估对象：`KUKA_ALLEGRO_CFG`（KUKA iiwa7 + 16-DoF Allegro 手）。任务语义仍为「在桌面上对一瓶身（仿真参数化、触发口径可放开）完成一次握持/接触 + 按压其喷嘴」。
> 用户拍板（2026-09-04）：**放弃「拇指压顶」→ 改用「长指按压」**（index/middle）。瓶形与弹簧口径均仿真自定。

---

## 0. 结论摘要

1. **长指按压成立**：index（食指）长指在过顶姿态族里能把指尖送到瓶顶正中、z 轴转竖直（径向 ≤ ~7 mm、cos_z 0.93–1.00），垂直下压参量化瓶的弹簧喷嘴到 ~72% 行程（nozzle −0.00359 m）、稳住、松开回弹至 ~0（日志 SUMMARY，§4）。
2. **「一指压顶 + 另一长指同时贴瓶」在本几何下不成立**：扫 index/middle 为 PRESSER、其余长指 curl 0.6–1.2，凡是 PRESSER 能稳居中压顶的位形，兄弟长指全部落到离瓶轴 ≥ ~78 mm 之外，够不到任何瓶壁；唯一能两指同时贴 Ø42 瓶壁的位形（curl 全 0.8）里没有任何一根手指空闲压顶。这与上一轮拇指版结论同源：**Allegro 过顶族 + 等长平行指 = 一次性「抓握中按压」不可得**，可得的只是「按压」本身。
3. **触发口径放开后，压到底的力量预算被满足**：弹簧由直立瓶的 K=2000（压 4–5 mm 需 ~8–10 N、Allegro 无裕量）放松到 **K=300 N/m、行程 5 mm、maxForce 6 N（到底仅需 ~1.5 N）**，index 指尖之力绰绰有余（这也是新瓶能按压到底的直接成因）。
4. 选定演示（M4）：`phase1_press.py`（已泛化 index 过顶下压引擎）+ `assets/hetero_bottle_chosen.usda`（参数化瓶，Ø42、瓶高 0.128、瓶顶/帽顶 world 0.585 = 直立瓶同高），`--nozzle_bottom 0.0035` 判定到位。base 默认 FixedJoint 固定。

---

## 1. 方向修正（为何从「拇指压顶」改「长指按压」）

前一报告（`feasibility_grasp_thumb_press.md`）与首版异形瓶扫描（thumb 作 PRESSER）均指向同一硬约束：Kuka 臂在瓶位附近只给「过顶、手指朝下」姿态族，而 **thumb 在过顶族里天然落在手掌对侧**（与三长指几乎不共面），对中 Ø22 喷嘴残差 ~54 mm、不可压。相反，**长指（index/middle）在过顶族里与瓶轴天然共面**——index 正是 Phase1 已验证能稳定压直立瓶帽顶的那根。

因此 re-scope 只问一句：在 index/middle 作竖直 PRESSER 的前提下，其余长指能否在**同一姿态**里贴住瓶壁（即「长指按 + 长指贴」）？本扫描回答：不能（§2 判定）。

## 2. 长指按压可行性扫描（M2，`scripts/scan_hetero_press.py`，headless）

### 方法（全 kinematic，只装 桌 + 手；瓶身 = 桌面 (cx,cy)=(-0.545,-0.026) 上的虚拟圆柱）
- PRESSER = `index_link_3` 或 `middle_link_3`（保持 nominal 指形），DLS-IK 送到 (cx,cy,hover=0.612) 且 z 轴竖直朝下（Phase1 同款 down-quat）。
- 对每个 PRESSER，把其余两根长指（brace）joint_1..3 扫 curl ∈ {0.6,0.8,1.0,1.2}，IK 复位 PRESSER 居中后记录 brace 各指中节(1→2)+末节(2→3)段端点 world 位。
- 离线段判定（离线网格）：段两端 z 落在 [ttop, ztop]，到瓶轴水平最小距离 ≤ body_r + pad_tol(12 mm) 记「贴瓶」。
- PRESSER 有效需 位置残差 < 60 mm 且 径向 ≤ press_tol(30 mm) 且 竖直度 cos_z ≥ 0.93。

### 结果（`logs/scan_hetero_press.log`，节选关键行）

| PRESSER | brace curl (middle,ring 或 index,ring) | align err mm | PRESSER 径向 mm | cos_z | 兄弟指贴瓶 |
|---|---|---|---|---|---|
| **index** | 任取 0.6–1.2 | 3–24 | 0.7–6.8 | 0.934–1.000 | **始终 None**（middle/ring 径向 ≥ ~78 mm） |
| middle | (0.6, 0.6) | 32 | 8.8 | 0.939 | index 段径向 9.8（**穿入** body_r 15–24 内，非外贴） |
| middle | (0.6, 0.8..1.2), (0.8, 0.6) | 37–51 | 5.5–9.4 | 0.83–0.91 | 不达 cos_z 门槛 / 残差超限 |

### 判定（诚实结论）
- index 是唯一稳健的中心竖直 PRESSER（align 好、可压）；middle 仅在最松 curl 下勉强居中且竖直度差。
- index 压顶时，middle/ring 即使收到 1.2 也**外抬**不贴瓶（等长平行指、过顶族固有几何，同上一报告 §3）。
- middle 压顶那 12 个「FEAS」行里唯一接触指 = index 段径向 9.8 mm **小于** body_r(15–24 mm)，是几何判据把「穿入圆柱」误计为贴瓶——改成「径向 ≥ body_r」真外贴判定后归零。
- → **本手 + 本姿态族下，不存在「竖直压顶 + 兄弟指贴瓶壁」同时成立的位形**。这与拇指版一致：按压与握持只能二选一，取按压。

## 3. 选定瓶（M1，`tools/author_hetero_bottle.py` → `assets/hetero_bottle_chosen.usda`）

参数化文本拼串（模板 schema 与 `medicine_bottle.usda` 一致，可 diff）：

| 项 | 值 | 说明 |
|---|---|---|
| body_shape / body_r | cyl / 0.021 m（Ø42） | 与直立瓶同径，利于复用已验证压高 |
| body_h | 0.128 m | 瓶身到顶 = 0.128；帽顶 local 0.145 → **world 0.585**（= 直立瓶帽顶，index 已验证压位） |
| 喷嘴 | noz_r 0.011 / noz_h 0.010，居中 (0,0) | re-scope 后不再需要偏心 lug |
| 行程 / 弹簧 | travel 0.005 m(lower −0.005)、**K=300**、D=2、maxForce=6、upper +0.001 | 到底 ~1.5 N < index 指尖力 → 触发口径放开的主要收益 |
| base | mass 0.25，`base_to_world` FixedJoint（默认固定） | M3 自由站 gate 本轮不做（base-fixed 为验收口径，自由站留待下一步） |

## 4. M4 演示（`phase1_press.py` + chosen.usda，headless，可选 --video）

引擎 = Phase1 已泛化的 index 过顶下压：A 原地转竖直（EE 命令 z 保持 hover、不下探）→ D 沿 −Z 接触驱动下压（nozzle 离开 0 记首触，到 −0.0035 判到位）→ H 稳住 → R 抬回释放看回弹。瓶身 fixed，全程测量 nozzle_joint。

命令：
```bash
env -u DISPLAY ./isaaclab.sh -p ../isaac_demo/scripts/phase1_press.py --headless \
  --enable_cameras --usd ../isaac_demo/assets/hetero_bottle_chosen.usda \
  --nozzle_top_home 0.145 --video /home/ubuntu/press_demo/outputs/hetero_press.mp4
```

量测（logs/demo_hetero_press.log SUMMARY；无视频复跑 logs/phase1_chosen.log 同值）：

| 量 | 值 |
|---|---|
| align_residual（转竖直后与 −Z 夹角） | 0.364 rad（≈ 近竖直；完全竖直会进腕奇异、下不去） |
| first_contact | EE z 0.6391，nozzle −0.00027 |
| nozzle_at_press_end | **−0.00359**（到位判据 −0.0035，≈72% 行程） |
| nozzle_held | −0.00344 |
| nozzle_after_release | −0.00027（**回弹 ≈ 0**） |
| 压到底实测反力 | ~K·0.0036 ≈ 1.1 N（maxForce 6 裕量大） |

视频 `outputs/hetero_press.mp4`：ffprobe 校验 + 抽帧复核按压瞬间（见 §5 命令）。

## 5. 验证 / 复现

```bash
cd /home/ubuntu/press_demo/IsaacLab && conda activate env_isaaclab
# M1 生成（已生成 chosen）
python ../isaac_demo/tools/author_hetero_bottle.py --tag chosen --body_shape cyl --body_r 0.021 \
  --body_h 0.128 --noz_x 0 --noz_y 0 --noz_r 0.011 --noz_h 0.010 --travel 0.005 \
  --K 300 --D 2 --max_force 6 --base_mass 0.25
# M2 扫描
timeout 1100 ./isaaclab.sh -p ../isaac_demo/scripts/scan_hetero_press.py --headless \
  > /home/ubuntu/press_demo/logs/scan_hetero_press.log 2>&1
# M4 演示（含视频）
env -u DISPLAY timeout 1500 ./isaaclab.sh -p ../isaac_demo/scripts/phase1_press.py --headless \
  --enable_cameras --usd ../isaac_demo/assets/hetero_bottle_chosen.usda \
  --nozzle_top_home 0.145 --video /home/ubuntu/press_demo/outputs/hetero_press.mp4 \
  2>&1 | tee /home/ubuntu/press_demo/logs/demo_hetero_press.log
ffprobe -v error -show_entries stream=width,height,nb_frames,duration -of csv \
  /home/ubuntu/press_demo/outputs/hetero_press.mp4
grep -E 'nozzle_at_press_end|nozzle_held|nozzle_after_release|DONE' \
  /home/ubuntu/press_demo/logs/demo_hetero_press.log
```

## 6. 相对 `feasibility_grasp_thumb_press.md` 的改动点

| 上轮结论（拇指压顶 / 直立瓶） | 本轮处理 |
|---|---|
| 直立 Ø44 瓶 + 弹簧 K=2000：压到底需 ~8–10 N、Allegro 无裕量 | **放开触发口径**：仿真参数化瓶 K=300 / 行程 5 mm / maxForce 6 → 到底 ~1.1–1.5 N，按压成为稳定可达动作 |
| 喷嘴比瓶口高 45 mm 逼手抬高、其余指够不到瓶身 | **接受「按压与贴瓶不可兼得」**，改为只按不握；瓶帽顶仍定在 index 已验的世界 0.585 |
| 拇指过顶对中喷嘴不可信（残差 ~54 mm） | **改用长指（index）按压**（与瓶轴天然共面，Phase1 已验证） |
| 「抓握中按压」姿态族不存在 | 本扫描再证：index 压顶位形里兄弟长指恒外抬不贴瓶；报告停在定性结论 + 可选按的演示，不硬造「抓+按」 |

日志：`/home/ubuntu/press_demo/logs/{scan_hetero_press,demo_hetero_press}.log`（另有无视频复跑 `phase1_chosen.log`）。

---

### 后续（不在本轮）
- M3 自由站 gate（`--free` 去掉 base_to_world）与「index 压顶时以中指作稳定限位」的组合仍待评估；若要走「真实握持中按压」，需换手/换瓶位整线（上轮 §5/§7 的 A–D 路线）。
- Phase 2 RL（PPO 训出「过顶接近 → index 压顶 → 回弹」策略）为独立确认项。
