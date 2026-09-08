# 混形双手 Demo：ALOHA 平爪固定瓶 + Shadow 掌心垂直下压（掌心朝下修正 + 触发力矩记录）

> 状态：**M1–M4 全链路跑通（掌心朝下压 cap → nozzle 行程到底 91% → 稳住 → 回弹 ≈0 → 松爪；按压期瓶漂移 0.02 mm；触发力矩已记录；USABLE；视频）**
> 日期：2026-09-08（本轮修正：palm 掌心朝上 → 朝下 + 记录触发瞬间的力；`probe_palm_press_o90.log` 对照 2026-09-07）
> 评估对象：`demo_mixed_press.py` —— 固定端 = Panda iiwa/2 指长爪（`_panda_longjaw_cfg.py`，ALOHA 式平爪代用），按压端 = `kuka_shadow.usda`（iiwa7 + Shadow 灵巧手），自由站参数化瓶 `hetero_bottle_chosen_free.usda`（Ø42、cap 顶 world 0.585、弹簧 K300 / 行程 5 mm / maxForce 6）。
> 语义：为「未来用掌心触觉阵列判断喷嘴可用性」搭落点 —— 用 **掌心(pad)朝下** 压 cap 顶，palm `ContactSensor` 法向合力 + nozzle 行程作触觉判读代用，输出 `tactile_verdict`，并在 nozzle 触底时记录触发力矩。
> **缩放 round（2026-09-08 追加）**：用户目视按压臂过长 → 按压臂 `spawn.scale=0.7` 等比缩小 + 0.28 m 垫座立柱，重验 pad-down 压成立（USABLE）。见 §9。

---

## 0. 结论摘要

1. **掌心朝下压成立（本轮修正核心）**：panda 长爪从**西侧(-x)** 钳住自由 Ø42 瓶身中段（`CLAMP_OK`，钳移 3.4 mm），iiwa7+Shadow 从**东侧(+x)** 基座把 **palm 掌心（pad 接触面）** 摆成水平朝下、置于 cap 正上方（`PALM_IK yaw=−90`：err 4 mm、cos_down +0.999）。整臂沿 −z 下探：**pad 触 cap 顶** → nozzle 行程到底 −0.00423 → 稳住 −0.00455（≈91% travel）→ 抬回回弹 −0.131 mm ≈ 0 → 长爪松开（`[jaw_open]` 后瓶漂移 0.04 mm）。
2. **触发力矩已记录**：nozzle 到底瞬间 palm ContactSensor 法向力 **`palm_force_at_trigger=1.19 N`**（稳住期同值），与之对抗的弹簧反力 `spring_reaction=−K·q=1.37 N`（K=300，q=−0.00455）。`TRIGGER nozzle_held=−0.00455 (4.55/5 mm travel) palm_force=1.19N spring_reaction=1.37N`。
3. **按压期瓶被真固定**：**下探/稳住/抬回全程 瓶 root 漂移 0.02 mm**（以重定心后实测瓶位为参考）。注意 hover 多 seed 扫掠本身会把被钳自由瓶撞偏 5.4 mm —— 本轮加 `[recenter]`：下探前重读实测瓶位重建 hover 对准，把 approach 撞击从「按压期漂移」口径里排除。
4. **触觉判读输出 USABLE**：pad 接触 + 法向力 1.19 N > 0.5 + 行程到底 + 回弹 ≈0 + 漂移 0.02 mm → `tactile_verdict=USABLE`。
5. **视频**：`outputs/mixed_press.mp4`，960×540、423 帧、21.15 s @20 fps（ffprobe 校验），`_last.png` 抽帧非空（378 KB）。

关键量（`logs/demo_mixed_press.log`）：

| 阶段 | 量 | 值 |
|---|---|---|
| 钳 | `CLAMP_LONGJAW` | zgrab=0.515, seat_err=5mm, 钳移 drift=3.4mm, CLAMP_OK |
| hover | `PALM_IK` yaw=−90 | err=4mm palm=(−0.550,−0.023,+0.614) cos_down=+0.999 REACH |
| recenter | `[recenter]` | hover 扫掠撞偏 5.4mm → 重建 hover, re-aim err=3.9mm |
| 下探 | first contact step 75 | z_cmd=0.5925 nozzle=−0.00031 bodies=[palm] |
| 到底 | nozzle bottom step 89 | z_cmd=0.5883 nozzle=−0.00423 drift=0.02mm **palm_force_at_trigger=1.19N** slip=(+0.0,−0.0)mm |
| 稳住 | `[held]` | nozzle=−0.00455 drift=0.02mm palm force=1.19N |
| 触发 | `TRIGGER` | nozzle_held=−0.00455 (4.55/5 mm) palm_force=1.19N spring_reaction=1.37N |
| 抬回 | `[release]` | nozzle_after=−0.00013 rebound=−0.131mm |
| 判决 | `PRESS_BOTTOM` 五项 | True / contact / 1.19N / drift_ok / rebound_ok |
| 判决 | `tactile_verdict` | **USABLE** |
| 松爪 | `[jaw_open]` | q_read=+0.0200 drift=0.04mm |
| 收尾 | `DONE` | 日志完整结束（flush 修正后） |

---

## 1. 语义 / 布局（混形依据）

用户拍板「混形」：固定端用 **ALOHA 式 2 指平行夹爪**（仿真无 ALOHA/WidowX → 以 **Franka Panda + 自建长爪** `_panda_longjaw_cfg.py` 代用），按压端用 **Shadow 灵巧手**。动机：以后要用灵巧手**掌心的触觉传感器判断喷嘴可用性** → 本轮演示要把「**掌心(pad)向下压 → pad 触面读接触力 + nozzle 行程 → 判 可用/不可用**」的落点搭出来。上一轮基线 `demo_bimanual.py` 的「侧向指尖扶」只是近距 fence、大号 Allegro 包不住 Ø42 扶不住 —— 平行爪真钳住瓶身是正解，按压反力由夹持摩擦 + 桌面承担。

布局（世界坐标，瓶 home=(−0.545,−0.026,0.44)）：

| 角色 | 基座 xy | 朝向 | 说明 |
|---|---|---|---|
| Bottle（自由站） | (−0.545, −0.026) | — | `hetero_bottle_chosen_free.usda`，cap 顶 world 0.585 |
| Presser（iiwa7+Shadow） | (−0.05, −0.03) | rot φ≈179°（朝瓶，东侧） | `kuka_shadow.usda`，palm 装 ContactSensor |
| Panda 长爪 | **西侧** (−1.119, −0.026) | φ_p≈0°（朝瓶，+x） | 爪从 −x 接近钳瓶身中段 z≈0.515 |

**双机方位是硬结论**（§4 诊断）：presser 掌心下压时 wrist 落在瓶**北(y+)**柱（指尖朝南、腕在掌心北），若 panda 从 **+y(北)** 侧钳瓶，其前臂正好堵住 wrist 下探柱（实测 err~70–200mm NOREACH）；panda 放 **−x(西)** 侧后整列留空，presser 从东侧可达 err 4 mm。M1 扫描默认只装了 presser（无 panda 实体），故未预判双机同柱 —— demo 实测才暴露，属布局修正而非判据放宽。

## 2. 掌平面 frame 与 palm-down（pad-down）姿态解

Shadow 手 `robot0_palm` 局部系未知。掌平面 = 过 4 根长指 MCP 近节（`robot0_{ff,mf,rf,lf}proximal`）的平面：

- **n_w**：两对角叉积 `cross(M_mf−M_ff, M_rf−M_lf)`（归一），拇指提示翻符号（`thb−centroid` 投影符号）定外法向；
- **fwd**：`mid_mf − mcp_mf`（沿中指轴，剔掉 n_w 分量）；
- **centroid / palm origin**：palm0 到 MCP centroid 沿 palm 局部 z 偏移 **c_local=(0,0,+0.094)**。

**本轮关键修正（palm 朝上 → 朝下）**：拇指提示给出的 MCP 平面法向实测是 **手背朝外** 的法向 —— 旧代码把它对齐世界 −z，得到 palm pad 朝上、手背贴 cap 压（用户目视 M3 视频判出「掌心朝上」，美中不足）。修正：量完拇指翻面后再 `n_w = −n_w`，使 n_w 变为 **pad(掌心接触面) 外法向**，之后对齐 −z 即 **pad 朝下** 压 cap。取反后可达性镜像：原 yaw=+90（pad-up 可达）变 NOREACH，**yaw=−90** 为 pad-down 可达 cell（err 4–8 mm，cos_down +0.999…1.000）。demo/`probe_mixed_reach.py`/`probe_palm_press.py` 三处同一 `n_w=−n_w` 修正保持逐行一致。名义位实测 `palm0=(+0.495,−0.014,+0.750) n·(−z)=+1.000`（pad 外法向朝下）。

**要点**：frame 三常数 n_local/f_local/c_local 是手刚体固有量，必须在干净 kqnom 位测一次、整段复用；不要在 panda 合拢（~6000 步）后再量 —— 彼时 palm 体被扰动/姿势不同，n_w 符号（近平行 MCP 对角叉积对噪声敏感）会翻错面。

## 3. M1 混形可达性扫描（`probe_mixed_reach.py`）

扫 presser 基座 r=0.50 各方位 × 挂载 × yaw，pad-down 判据 err<18mm 且 cos_down>0.95。东侧 cell 命中（`logs/probe_mixed_reach_default4.log` 旧 pad-up / `logs/probe_mixed_reach_paddown.log` pad-down）：

```
# pad-down(n_w 取反后) —— 选定行: 东侧 presser base (-0.05,-0.03), yaw=-90
base_r=0.50 ang=  0 yaw=-90 | palm_err=    4mm cos_down=+0.999 FEAS base=(-0.05,-0.03)
base_r=0.50 ang=180 yaw=-90 | palm_err=  ... mm cos_down=... FEAS base=(-1.05,-0.03)   # 与西侧 panda 冲突, 未取
```

M1 为 presser-only 解析，**未实体装 panda** —— 双机不撞是 demo 实测（§4）才收敛出的布局；若日后改挂载/瓶位，应把两臂实体都装进 M1 再扫。

## 4. 组件与 demo 序列（`demo_mixed_press.py`）

场景装配 = 桌 + 自由瓶 + Panda 长爪 + iiwa7+Shadow + ContactSensor + 相机。序列：

① **Panda 钳瓶（M2）**：瓶暂移远处 → 静态多 seed 让刀片骑跨空座位（seat_err 5mm）→ 瓶瞬移回 home 进开爪 → 闭爪钳住瓶身（`CLAMP_OK`，钳移 3.4mm）。
② **Palm hover（M3，含 recenter）**：palm pad-down 到位（cap 正上方 30 mm、pad 法向水平朝下，err 4 mm）。hover 多 seed 扫掠会把被钳自由瓶撞偏（实测 5.4mm）→ **`[recenter]` 下探前重读实测瓶位、重建 cap_goal/hover 对准（re-aim err 3.9mm）**，使按压期漂移以重定心后瓶位为参考。
③ **−z 下探压**：pad 压 cap 顶 → nozzle 行程到底（−0.00423）→ 稳住 −0.00455（≈91%）→ 抬回看回弹（palm ContactSensor 全程读 pad 法向力；到底瞬间记 `palm_force_at_trigger`）。
④ **松爪撤走**：jaw 张开（`JAW_U`），量瓶漂移。

量测印桩：`[frame]`/`CLAMP_LONGJAW`/`PALM_IK`/`[recenter]`/`[descend] 首触与到底`/`[held]`/`TRIGGER`/`[release]`/`[jaw_open]` + `tactile_verdict`。日志以 `DONE` 收尾（本轮把 `DONE`/`[video]` 补 `flush=True`，避免关停挂起时缓冲丢失）。

**本轮定位的修正**：(a) palm 朝上（手背压）→ 加 `n_w=−n_w` 变 pad 朝下；(b) 可达 cell 镜像切换，demo 默认 `--yaw=-90`；(c) hover 扫掠撞偏 5.4mm → 加 `[recenter]` 重定心；(d) 关停挂起时 `DONE` 因无 flush 丢失 → 补 flush。

## 5. 触觉判读（palm ContactSensor = 未来 tactel 代用）与触发力矩

palm 本体 + 4 近节挂 `ContactSensor`（filter `/World/Bottle/base`,`/World/Bottle/nozzle`），读按压期 pad 法向合力。判决链条：

1. pad 有力（1.19 N > 0.5）→ 压到 cap；
2. nozzle 开始动（首触 −0.00031）→ 行程到底 −0.00423/−0.00455（≥ −travel×0.8）；
3. 抬回回弹 −0.131 mm ≈ 0 → 弹簧未卡死；
4. 瓶漂移 0.02 mm（重定心参考）→ 夹持稳。

**触发力矩（用户口径「记录触发的力矩」）**：nozzle 到底瞬间即「触发」，记录两条力 —— pad 法向接触力（ContactSensor `force_matrix_w` 归一求和，到底 break 时快照）`palm_force_at_trigger=1.19 N`；与之平衡的**弹簧回位反力** `spring_reaction=−K·q=−300×(−0.00455)=+1.37 N`（模型内建弹簧 K300 N/m / travel 5mm / maxForce 6，q<0 为下压行程）。打印 `TRIGGER nozzle_held=... palm_force=... spring_reaction=...`。四条件齐 → **`tactile_verdict=USABLE`**；若 pad 压但喷嘴不动/回弹不回 → NOT_USABLE（判据在代码里，同一引擎负例分支，本轮未触发）。

真实 palm tactel 阵列建模（接触网格→压力图）不在这轮；ContactSensor 法向合力是**可验证代用**，接口位留在 demo 判读处，后续可换真 tactel 读 pressure map 后给更高粒度判决。

## 6. 验证 / 复现

```bash
cd /home/ubuntu/press_demo/IsaacLab && source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
# M3 demo（pad 朝下压: panda 钳住自由瓶 + palm 掌心朝下压 cap, yaw=-90）
env -u DISPLAY timeout 1500 ./isaaclab.sh -p ../isaac_demo/scripts/demo_mixed_press.py --headless \
  --enable_cameras --ns 10 --step 260 --yaw=-90 --video ../outputs/mixed_press.mp4 \
  > /home/ubuntu/press_demo/logs/demo_mixed_press.log 2>&1
# 判读 / 量测
grep -aE 'CLAMP_LONGJAW|PALM_IK aim|recenter|first contact|nozzle bottom|\[held\]|TRIGGER|PRESS_BOTTOM|tactile_verdict|\[jaw_open\]|DONE' /home/ubuntu/press_demo/logs/demo_mixed_press.log
# 视频
ffprobe -v error -show_entries stream=width,height,nb_frames,duration -of csv ../outputs/mixed_press.mp4
# 期望: CLAMP_OK / PALM_IK ... yaw=-90 ... REACH / [recenter] hover扫掠后瓶漂移=5.4mm ... re-aim err=3.9mm /
#       first contact / nozzle bottom ... palm_force_at_trigger=1.19N / [held] drift=0.02mm /
#       TRIGGER ... palm_force=1.19N spring_reaction=1.37N / PRESS_BOTTOM=True ... drift_ok=True /
#       tactile_verdict=USABLE / [jaw_open] drift=0.04mm / DONE
#       ffprobe -> 960,540,423,21.15
```

Isaac Sim 关停常挂（`DONE`/`[video]` 后数据已出但 close() 不返回），孤儿按 PID kill，重跑前清 GPU（~17MiB 空闲）。重跑两次数值一致（确定性）。

## 7. 相对前几份报告的改动点

| 上几轮 | 本轮处理 |
|---|---|
| 基线 `demo_bimanual.py`：自由瓶 + 一手 Allegro 过顶指尖压 + 一手**近距扶**（fence，包不住 Ø42、扶=不抓） | **换真固定**：Panda 长爪从西侧钳住瓶身，按压反力由夹持摩擦 + 桌面承担 |
| Allegro 单指指尖压 cap 顶 | **Shadow pad-down 正压**：掌心(pad)平面朝下压 cap 顶，pad 即未来 tactel 触面 |
| **上一版 `demo_mixed_press.py` palm 朝上（手背压 cap）**（用户目视判出美中不足；根因 = MCP 平面法向拇指提示给的是手背外法向） | **本轮修正：`n_w=−n_w` → pad 朝下压 cap**；可达 cell 镜像，`--yaw=-90`；三处脚本同步取反 |
| hover 多 seed 扫掠把被钳自由瓶撞偏（实测 5.4mm），按压期漂移口径含此撞击 | **`[recenter]` 下探前重读实测瓶位重建 hover**，按压期漂移 0.02 mm（排除 approach 撞击，同 b_fixed「压前回 home」口径） |
| base-fixed 瓶口径（`--b_fixed`，瓶固定） | 自由瓶 + 被爪钳住是主口径；`--b_fixed` 保留为回退 gate |
| 触发「力矩」只到行程/接触判读 | **记录触发瞬间力**：`TRIGGER` 打印 pad 法向力 1.19N + 弹簧反力 1.37N |
| 触觉只有指尖/接触日志 | pad ContactSensor 法向合力 → `tactile_verdict=USABLE`；关停挂起丢失 DONE → 补 flush |

## 8. 遗留 / 边界

- 自由瓶被爪钳住时 jaw 闭合带 3.4 mm 钳移；hover 多 seed 扫掠再把瓶撞偏 5.4 mm —— 本轮 `[recenter]` 让按压期漂移仅 0.02 mm，但钳移/approach 撞击仍在（判据落在「按压期漂移」而非全过程）。若要求全程漂移也 <2 mm，需消除扫掠撞击（改单 seed hover 或加门闸保护）。
- 触发力矩 = pad 法向接触力 1.19N + 弹簧反力 1.37N（数值上 pad 力略小于弹簧反力，因接触有部分力被 cap/nozzle 摩擦承担）；真实 palm tactel 阵列压力图建模不在本轮，接口位已留。
- 名义 cap 顶 world 0.585 与 palm hover 30mm → 下探到 pad 触 cap 时 nozzle 已近底；回弹 −0.131mm 为 pad 抬离瞬间 nozzle 的弹性回位，非塑料变形。
- M1 扫描 presser-only（未装 panda 实体），双机同柱由 demo 实测暴露并收敛到 west 布局 —— 若日后改挂载/瓶位，应把两臂实体都装进 M1 再扫。

## 9. 缩放 round：`spawn.scale=0.7` 等比缩小按压臂 + 垫座立柱

> 用户口径（AskUserQuestion 拍板）：看视频判**按压手所在机械臂太长** → **等比缩放物理尺寸**；目标 = **尽量小但要够得到**桌面原尺寸瓶口（cap 顶 world z≈0.585）；**允许把基座垫高**（静态立柱）。只缩按压端 iiwa7+Shadow，**Panda 长爪不缩**（否则破坏其对 Ø42 瓶的夹持口径），桌面/瓶/瓶位全不动。

### 机制（M1 Gate A）：运行时 `spawn.scale` 生效，无需重造 usda
- `kuka_shadow.usda` = 薄引用层引用远端 `kuka.usd`(iiwa7+Allegro)+`shadow_hand.usd`。运行时缩放直接用 `UsdFileCfg.spawn.scale=(s,s,s)`（`from_files_cfg.py`），demo/probe 加 `--scale <s>` 后 `kcfg.spawn.scale=(s,s,s)`。
- **Gate A 实证**：s=0.6/0.7/0.8 装到干净位，body 位姿随 s 缩放（palm0 名义 z≈0.75·s，如 s=0.7 → 0.525），joint/body 名不变、ContactSensor 正常建、`n·(−z)=+1` pad 仍朝下。物理单位不变（metersPerUnit=1），只是几何缩小。
- 显式 `physics:mass` 不随 s 自动缩放（scaled 后质量不变→密度变大）；demo 为 disable_gravity quasi-static 力驱，只影响接触力幅值标定，gate 判 >0.5N 不受影响（本轮不按 s³ 调质量）。

### M1 最短 s 扫描（headless，`probe_mixed_reach.py` 扩展 `--scale/--zlist/--hover_mm`）
判据：palm 原点 err<18mm（pad-down origin-aim）且 cos_down>0.95，`VERDICT_SCALE`。网格扫描结果：

| s | 基座垫高 z(bz) | 基座离 cap r | 结论 |
|---|---|---|---|
| 1.0 | 0 | 0.50 | FEAS（原口径对照） |
| 0.8 | 0.20 | 0.42 | FEAS（base=(−0.13,−0.03)） |
| 0.7 | 0.20–0.28 | 0.34–0.36 | FEAS；r=0.42 NO |
| 0.7 | hover **30mm**(demo 实际平面) | **r≤0.34** | FEAS err 4–7mm（bz 0.20–0.28）；r=0.38 起 NO |
| 0.6 | 0.10–0.42 | 0.32–0.44 | **NO_FEAS**：够不到 + 垫到 cap 高时 pad-down 反转(palm 朝上 cos≈−0.9) |

**几何结论**：cap 固定在原世界位、臂缩 s 后地板基座必够不到 → 基座要垫高并移近 cap；但垫到 cap 高度附近时接近变成「近水平侧向」，pad 无法朝下。**s=0.7 是「干净离桌挂载」下能同时 hover(30mm) 又能下探压的最小缩放**（s≈0.65/0.6 需 r<0.34 贴桌沿 + 高垫，代价大且 pad-down 退化）。**demo hover 平面是 30mm，probe 默认 6mm —— 边界上必须用 demo 实际平面扫描**（r=0.34/6mm 与 30mm 差出整格）。

**选定装具**：`--scale 0.7`，基座 `--px=-0.205 --py=-0.026 --pz=0.28`（cap x −0.545 → r=0.34，离桌东沿 ~40mm），riser 立柱顶面 z=0.28。悬空 hover err 1mm、cos_down 0.997。

### M2 改动点（demo/probe）
- demo：加 `--scale`(默认1.0)/`--pz`(默认0)；装载处 `kcfg.spawn.scale=(s,s,s)`、`kcfg.init_state.pos=(px,py,pz)`；`pz>0.05` 时在基座下装静态 `CuboidCfg`(kinematic) 垫座立柱（size 0.24×0.30×pz，顶面 z=pz）。判读/descend/recenter 全用世界 cap 目标，不改。
- probe：加 `--scale`（`cfg.spawn.scale`）、`--zlist`（基座 z 网格）、`set_root(bx,by,bz,φ)`、`--hover_mm` 对齐 demo 平面；输出 `VERDICT_SCALE`。

### M3 物理 gate（自由瓶被 Panda 长爪钳住 + 缩放手压）—— USABLE
`logs/demo_mixed_press_scaled2.log`，`outputs/mixed_press_scaled.mp4`（416 帧 960×540 ~20.8s @20fps，`_last.png` 抽帧 198 KB）：

| 阶段 | 量 | 值 |
|---|---|---|
| 钳 | `CLAMP_LONGJAW` | zgrab=0.515 seat_err=0mm 钳移 drift=0.1mm CLAMP_OK |
| hover | `PALM_IK` yaw=−90 | err=1mm palm=(−0.544,−0.025,+0.615) cos_down=+0.997 REACH |
| recenter | `[recenter]` | hover 扫掠撞偏 5.0mm → 重建 hover, re-aim err=1.1mm |
| 下探 | first contact step 50 | z_cmd=0.6000 nozzle=−0.00031 bodies=[palm] |
| 到底 | nozzle bottom step 68 | z_cmd=0.5946 palm_z=0.5999 nozzle=−0.00412 drift=0.09mm **palm_force_at_trigger=1.13N** slip=(−0.1,+0.0)mm |
| 稳住 | `[held]` | nozzle=−0.00461 drift=0.21mm max_force palm:1.13N |
| 触发 | `TRIGGER` | nozzle_held=−0.00461 (4.61/5 mm) palm_force=1.13N spring_reaction=1.38N |
| 抬回 | `[release]` | nozzle_after=−0.00015 rebound=−0.149mm |
| 判决 | `PRESS_BOTTOM` 五项 | True / contact / 1.13N / drift_ok / rebound_ok |
| 判决 | `tactile_verdict` | **USABLE** |
| 收尾 | `[jaw_open]`+`DONE` | q_read=+0.0200 drift=0.13mm |

> 与 s=1.0 基线同量级（基线 palm_force 1.19N/spring 1.37N/drift 0.02mm；s=0.7 → 1.13/1.38/0.21mm）。触发力矩记录保持同一判读链。

### 复现（缩放 variant）
```bash
cd /home/ubuntu/press_demo/IsaacLab && source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
env -u DISPLAY timeout 1500 ./isaaclab.sh -p ../isaac_demo/scripts/demo_mixed_press.py --headless \
  --enable_cameras --ns 10 --step 260 --yaw=-90 --px -0.205 --py -0.026 --pz 0.28 --scale 0.7 \
  --video ../outputs/mixed_press_scaled.mp4 > /home/ubuntu/press_demo/logs/demo_mixed_press_scaled2.log 2>&1
grep -aE 'CLAMP_LONGJAW|PALM_IK aim|recenter|first contact|nozzle bottom|\[held\]|TRIGGER|PRESS_BOTTOM|tactile_verdict|\[jaw_open\]|DONE' /home/ubuntu/press_demo/logs/demo_mixed_press_scaled2.log
ffprobe -v error -show_entries stream=width,height,nb_frames,duration -of csv ../outputs/mixed_press_scaled.mp4
```
M1 扫描日志：`logs/{probe_scale_s08,probe_scale_s07,probe_scale_s07_h30}.log`（另 `probe_scale_ctl_s10.log` s=1.0 对照、`probe_scale_s06.log` NO_FEAS）。

### 遗留 / 边界（缩放 round）
- **相机取景沿用 s=1.0**：eye=(−0.395,0.924,1.35)。短臂 + 0.28 垫座在帧内偏小；若想强调「臂变短」，可拉近/降 eye 重出（本轮未调，视频已可用）。
- 垫座立柱 0.24 深中心在 px=−0.205，西缘探入桌板东沿下 ~80mm；桌板底 z0.38 > 柱顶 0.28 无碰撞，视觉被桌沿遮住，可接受。
- s≈0.6 需贴桌沿(r<0.34)+高垫、pad-down 退化，不取；质量按 s³ 标定与短臂 hover 扫掠撞偏(5.0mm)同基线，未做专门处理。
- Panda 未缩（夹持口径要求不变）；若日后想整体协调，Panda/瓶同步缩是另口径，未做。

日志/视频：缩放 round 见上；对照 s=1.0 基线 `logs/demo_mixed_press.log` + `outputs/mixed_press.mp4`。
