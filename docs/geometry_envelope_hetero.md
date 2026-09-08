# 异形瓶「几何尺寸可行域」反推报告（动态实测）

> 状态：**可行域窗口已动态实测反推完成（base-fixed 口径）**
> 日期：2026-09-05
> 回答用户问题：「能否更改瓶子的几何尺寸，甚至反推出在什么几何尺寸范围内，这个能成功？」
> 方法：几何/运动学定性 + **边界用真实 `phase1_press.py` 动态实跑**（每个配置独立实例，poll `^DONE` 后杀进程组）。引擎与瓶模型见 `docs/feasibility_hetero_press.md`。
> 依赖：瓶 = `tools/author_hetero_bottle.py` 参数化圆柱（K=300 N/m、行程 5 mm、maxForce 6 N、base FixedJoint）；按压 = index 过顶竖直下压（hover 0.665 起）。

---

## 0. 结论摘要（可直接引用）

成功判定口径（两档）：

- **(a) 纯按压（只压顶、不看兄弟指）**——只要 **帽顶 world 高度 ∈ [0.50, 0.65] m**（等价 **body_h ∈ [0.043, 0.193] m**、帽顶 local = body_h+0.017 ∈ [0.060, 0.210]），Ø42 圆柱瓶 index 过顶下压**全部压到底（nozzle −0.0035）+ 松开回弹 ≈ 0**。上边界在 **帽顶 0.66（body_h 0.203）失效**：能压到底（nozzle −0.005 满行程）但**抬不起来、卡死**。
- **(b) 横向摆瓶窗口（帽顶 0.585）**——瓶心相对 index 名义 (−0.545,−0.026) 可平移 **x 方向 0…+120 mm、y 方向 ±50 mm 全部压成**；±120/±50 之外未再探（IK 残差已 0.57 rad 仍下压成功，真实容差远大于运动学收敛预示）。
- **(c) 瓶宽不敏感**——Ø84（body_r 0.042）@帽顶 0.585 与 Ø42 表现**逐值一致**（按压只被顶部接近间隙限，不被宽度限，因弹簧 K=300 到底只需 ~1.5 N）。
- **(d) 「压顶 + 兄弟指同时贴瓶」在任意瓶尺寸下定量不可达**——index 压顶时 middle/ring/thumb **全部悬在帽顶上方 ≥ +30 mm**（middle 指尖 ~帽顶+60 mm、ring ~帽顶+110 mm），而瓶壁最高只到 **body_top = 帽顶 − 0.017** < 帽顶：兄弟指与瓶壁**永无 z 重叠**。这是比上轮「径向 ≥ 78 mm」更强的通用不变量（与瓶形无关）。

→ 一句话窗口：**把瓶做到「帽顶 world 0.50–0.65（推荐 0.56–0.62）、摆位偏离 index 名义 ≤ +120 mm/±50 mm、宽 ≤ Ø84」都能压成**；**凡是想要「压的同时另一根指贴瓶壁」，改瓶尺寸无用，必须改动作/手/位形**。

---

## 1. 瓶参数与测量口径

- 表：`assets/geom/hetero_bottle_zc<WWW>.usda`，`<WWW>` = 帽顶 world 高度 ×1000。几何：cyl Ø42（body_r 0.021）除 wide585（Ø84）；帽顶 local = body_h + 0.017（= travel 0.005 + bottom_gap 0.002 + noz_h 0.010 的 pad 顶高出瓶顶的量）；帽顶 world = table(0.44) + body_h + 0.017。
- 引擎 `phase1_press.py`：A 转竖直（EE 命令 z 停在 hover）→ D 沿 −Z 下压（判据 nozzle ≤ −0.0035）→ H 稳 → R 抬回看回弹。
- **判定 PASS** = `hit_nozzle_bottom True`（nozzle_at_press_end ≈ −0.0035，≈70% 行程）且 `nozzle_after_release ≈ −0.00027`（回弹≈0）。
- 日志 `logs/geom_*.log`；SUMMARY `first_contact_ee_z` 因 resting noise（nozzle=−0.00027）在读下压起始即触发、**不可信**，真接触以 `press_end` 的 ee z（= 帽顶附近）与 nozzle 行程为准。

## 2. (a) 帽顶高度可行域（Z 带，动态网格）

hover 0.665、瓶心 (−0.545,−0.026)、Ø42：

| tag / 帽顶world / body_h | align_residual | press_end ee z | nozzle@press_end | nozzle_held | nozzle_after_release | 判定 |
|---|---|---|---|---|---|---|
| zc0500 / 0.500 / 0.043 | 0.364 | 0.5095 | −0.00356 | −0.00498 | −0.00027 | **PASS** |
| zc0545 / 0.545 / 0.088 | 0.364 | 0.5550 | −0.00357 | −0.00498 | −0.00027 | **PASS** |
| zc0565 / 0.565 / 0.108 | 0.364 | 0.5757 | −0.00350 | −0.00392 | −0.00027 | **PASS** |
| zc0585 / 0.585 / 0.128 (chosen) | 0.364 | 0.5963 | −0.00359 | −0.00344 | −0.00027 | **PASS** |
| zc0605 / 0.605 / 0.148 | 0.364 | 0.6168 | −0.00355 | −0.00432 | −0.00027 | **PASS** |
| zc0625 / 0.625 / 0.168 | 0.348 | 0.6375 | −0.00354 | −0.00496 | −0.00027 | **PASS** |
| zc0650 / 0.650 / 0.193 | 0.096 | 0.6604 | −0.00353 | **−0.00500(触底)** | −0.00029 | **PASS（临界）** |
| zc0660 / 0.660 / 0.203 | 0.085 | 0.6689 | **−0.00500** | **−0.00500** | **−0.00500** | **FAIL（卡死）** |

- 窗口上边界机制：帽顶 ≈0.65 时接近已几乎全竖直（residual 0.096），而 index 竖直自然落点/workspace 顶 ≈0.65–0.66；帽顶 ≥0.66 时指尖即使压到满行程，**也无法抬到帽顶之上脱离** → 释放后仍 −0.005 卡死。0.65 行已临界（hold 触满行程、仅靠弹簧回弹 −0.00029 险过）。
- 下边界物理下限：body_h→0 时帽顶 ≈0.457（表 0.44 + pad 0.017）；实测最低 0.50 即已全竖直/近奇异仍成，未再往下探（0.50 以下更接近桌面的矮身瓶按同一机制只会更好够到）。

## 3. (b) 横向摆瓶窗口（帽顶 0.585、Ø42，--bottle_x/--bottle_y 平移瓶心）

| run | 瓶心 (x,y) | 相对名义偏移 | align_residual | 判定 |
|---|---|---|---|---|
| zc0585 基准 | (−0.545, −0.026) | 0 / 0 | 0.364 | **PASS** |
| lateral_x15 | (−0.530, −0.026) | x +15 mm | 0.379 | **PASS** |
| lateral_x30 | (−0.515, −0.026) | x +30 mm | 0.511 | **PASS** |
| lateral_x45 | (−0.500, −0.026) | x +45 mm | 0.412 | **PASS** |
| lateral_x120 | (−0.425, −0.026) | x +120 mm | 0.574 | **PASS** |
| lateral_y20 | (−0.545, −0.006) | y +20 mm | 0.42 | **PASS** |
| lateral_y50 | (−0.545, +0.024) | y +50 mm | 0.495 | **PASS** |
| lateral_y_50 | (−0.545, −0.076) | y −50 mm | 0.272 | **PASS** |

- x 最远 +120 mm 时残差 0.574 rad、press_end z 轴 (−0.25,−0.29,−0.92)——仍能稳定下压到底。**真实横向容差远大于既有 kinematic 扫描预示**（之前把「IK 收敛残差大」当不可达，属过度悲观）。
- 窗口表述为「≥ 已测范围」：x ∈ [−0.545,−0.425]（即瓶心在 index 名义正下到 +120 mm 前方）、y ∈ [−0.076, +0.024]（±50 mm）全部可压；更远处未测（若需更大横向裕量可再补扫）。

## 4. (c) 瓶宽不敏感

`wide585`（body_r 0.042 → Ø84，帽顶 0.585）：align_residual 0.364、press_end ee z 0.5964、nozzle −0.00354、回弹 −0.00027，**与 Ø42 chosen 逐值一致**。→ 圆柱越宽不影响按压成功，只会在极端 Ø 时受「指尖接近路径间隙」限制；力预算由弹簧 K=300（到底 ~1.5 N ≪ index 指尖裕量）保证，与瓶宽无关。

## 5. (d) 「压顶 + 兄弟指贴瓶」为何任意尺寸都不可达（定量不变量）

两种证据互为印证：

**运动学诊断**（`scan_hetero_geom.py`，机器人+桌，index 转竖直于命令 zc）：middle/ring 各节径向/高度呈「水平蜷在帽顶上方」：
| presser 命令 zc | middle L1/L2/L3 z | ring L1/L2/L3 z | middle 径向 L1/L2/L3 mm | ring 径向 L1/L2/L3 mm |
|---|---|---|---|---|
| 0.575 | 0.616/0.619/0.620 | 0.660/0.668/0.672 | 92/39/1 | 96/43/9 |
| 0.600 | 0.641/0.645/0.646 | 0.686/0.694/0.698 | 92/38/1 | 96/43/9 |
| 0.630 | 0.671/0.674/0.675 | 0.716/0.723/0.727 | 91/38/1 | 95/42/9 |

**真实压顶动态**（各帽顶的 press_end）：middle_link_3 ≈ **帽顶 + 0.062**（0.585→0.647；0.50→0.562；0.65→0.712）、ring ≈ 帽顶 + 0.113、thumb ≈ 帽顶 + 0.03。全部 ≥ 帽顶。

两者合一 ⇒ 对任意帽顶 C：兄弟指最低 (thumb ≈ C+0.03) > C = 瓶壁最高 (body_top = C−0.017)。**瓶身不存在能碰到任何兄弟指的 z 带**，改瓶宽/瓶高只会让瓶壁最高仍停在 C−0.017 < 兄弟指下缘，永不重叠。因此 (b) 判据「压的同时 ≥1 兄弟指贴瓶壁」= 几何不可达，是**与瓶尺寸无关的不变量**（比上轮「径向 ≥78 mm / 穿入误计」更干净、更根本）。

- 上轮 `feasibility_hetero_press.md §2` 的「middle L3 径向 9.8 < body_r 穿入误计为贴瓶」在此框架下同样解释为：穿入段恰好位于 C 之下但该 z 已无壁（或径向落在柱体内），非外贴。

## 6. 结论与操作建议

1. **可行设计区间**：帽顶 world **0.50–0.65**（推荐稳定段 0.56–0.62，避 0.65 临界与 ≥0.66 卡死），瓶心放 index 名义 (−0.545,−0.026) 或偏离 ≤ x+120/±y50，瓶宽任意 ≤Ø84。此区间内 `phase1_press.py` 引擎原样可压（改 `--usd` + `--nozzle_top_home`=帽顶 local 即可）。
2. 若目标是「真握持中按压 / 压时另一指贴瓶」——**本手 + 本姿态族在几何上不可能**，不必再试改瓶。要突破需整线换动作（换手/侧向进入/低位 grab），上轮报告 A–D 路线仍适用；或者接受「纯按压」作为最终语义。
3. 弹簧口径一旦回到 K≈2000（直立瓶式强触发），按压力预算重新吃紧（§0 结论依赖 K=300/maxForce6/行程5mm 的放开口径）——改几何不影响这一点，力预算由弹簧参数单独决定。

## 7. 验证 / 复现

```bash
cd /home/ubuntu/press_demo/IsaacLab && source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
# 生成某个帽顶的瓶（示例帽顶 world 0.60 → body_h 0.148）
python ../isaac_demo/tools/author_hetero_bottle.py --tag zc0600 --out assets/geom \
  --body_shape cyl --body_r 0.021 --body_h 0.148 --noz_r 0.011 --noz_h 0.010 \
  --travel 0.005 --K 300 --D 2 --max_force 6 --base_mass 0.25
# 动态实跑（cap_local = body_h+0.017；瓶心平移加 --bottle_x/--bottle_y）
setsid bash -c "env -u DISPLAY PYTHONUNBUFFERED=1 timeout 200 ./isaaclab.sh -p ../isaac_demo/scripts/phase1_press.py \
  --headless --usd ../isaac_demo/assets/geom/hetero_bottle_zc0600.usda --nozzle_top_home 0.165 \
  > /home/ubuntu/press_demo/logs/geom_zc0600.log 2>&1" & pgid=$!; \
  for i in $(seq 1 200); do grep -q '^DONE' /home/ubuntu/press_demo/logs/geom_zc0600.log 2>/dev/null && break; sleep 1; done; \
  kill -- -$pgid 2>/dev/null
grep 'SUMMARY' /home/ubuntu/press_demo/logs/geom_zc0600.log   # hit_nozzle_bottom True, after_release ≈ -0.00027
```

- 复跑驱动脚本（含全部 Z/横向/wide 行的网格）参考：`/home/ubuntu/press_demo/run_geom_grid.sh`。
- 全部网格日志：`/home/ubuntu/press_demo/logs/geom_{zc05*,zc06*,lateral_*,wide585}.log`；SUMMARY 汇总已在 §2–§4 表格列出。
- IsaacSim teardown 挂起处理：`^DONE` 后 kill 进程组（每实例 ~20 s 完成），孤儿实例按 PID pkill，重跑前清 GPU。
