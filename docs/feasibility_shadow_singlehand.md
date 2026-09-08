# Shadow 单臂+单手「抓握瓶身 + 拇指按压」几何可行性（2026-09-07 负结论）

> 状态：**可行性扫描完成，当前挂载（fingers-up）下为负结论；数据齐全，待用户拍板下一方向**
> 日期：2026-09-07
> 评估对象：`KUKA_SHADOW_CFG`（KUKA iiwa7 + 24-DoF Shadow Hand，`assets/kuka_shadow.usda`，34 bodies / 31 joints）。任务语义：对桌上一瓶身完成「同一次握持内环抱瓶身 + 拇指按压」。
> 用户拍板（2026-09-07）：单臂+单手路线选用**拟人的 Shadow Hand 上臂换装**（AskUserQuestion 单臂二选一），替代 Allegro。

---

## 0. 结论摘要

1. **当前挂载（Shadow 以「指尖朝上」焊在 iiwa7 link_7 正上延伸、无旋转）下，环绕一瓶直立小瓶瓶身 + 拇指压顶 在几何上不可达。** 三重证据互洽：
   - **palm 指向工作空间**：front 网格里 palm 竖直朝下 `err ≥ 232 mm`（全格），低带（z 0.20–0.40）`≥ 386 mm`；最好的斜下（diag_xn）收敛格孤立且零星（47 / 111 / 137 mm @ 高位），低带最好 125 mm（x−0.70）仍超 60 mm 门槛。
   - **可达位形下指尖落点实测**：即便取 DIAG 可达锚点，palm 实际停在 z 0.85–0.98、四长指尖落在 **z 0.73–0.94**，悬在一只直立瓶身（瓶顶 ~0.585）上方 0.15 m 以上——指尖永远够不到瓶身。
   - **屈曲不救场**：Shadow 四长指平行，全屈（J1=J2=+1.57）指尖也只回收到 palm 前 ~71 mm（伸直 ~160 mm），**永远收不回 palm**；无「指尖反掌包圆」能力，对竖直圆柱没有逐指环绕可能。
2. **根因 = 手臂可达族 × 手几何不匹配**：iiwa7 在瓶位附近只给「过顶 / 斜顶下压」族（与 Allegro 各轮负结论同源）；而 Shadow 是 link7 伸出去 ~0.46 m 的**直链手**，要环抱直立瓶需要「palm 低到瓶侧、指尖横过瓶轴」的姿态——恰好是 arm 给不出的族。被顶族带过去时，0.46 m 直链把指尖推到远高于瓶身的高处。
3. **这不是 Shadow 资产失败**：合并 articulation 已跑通（几何/屈曲/IK 全部可驱动，§2/§3），失败在「该挂载姿态」下 wrap 语义不可达。
4. **可选下一步（§7）**：换挂载旋转（把直链横过来送向瓶侧）重扫；或接受可达的「顶部/斜顶爪握」语义 + 偏心按压嘴；或止损回退到已跑通的异形瓶长指按压。

---

## 1. 背景

- 上轮（Allegro 各口径）已多次证：iiwa7 在瓶附近只给顶/斜下顶进族、水平侧捏永不出现；且「一指压顶时兄弟指够不到瓶身」。
- 用户要拟人 + 泛化，换装更拟人的 Shadow Hand。合并资产 recipe 见 memory 2026-09-07 段：引用 kuka `/kuka` → `/KukaShadow`，摘 Allegro，非实例化 shadow_hand.usd 作 `iiwa7_link_*` 平级 sibling，去 ArticulationRootAPI / rootJoint，FixedJoint 焊 `iiwa7_link_7`→`robot0_hand_mount`（缺省 link_7 原点、**无旋转**）。

## 2. 资产与 home 几何（`probe_shadow_merged_curl.py` / `probe_merged_load.py`）

| 量 | 值 |
|---|---|
| 合并骨架 | 34 bodies / 31 joints（iiwa7 7 + Shadow 24） |
| home | link_7 @ (0,0,1.221)；palm ≈(0,0.01,1.511)；ffdistal z 1.676；thdistal (0.0835,0.0009,1.5895) |
| 手直链长 | link7→palm ~0.29 m，→指尖 ~0.46 m（mount→forearm→wrist→palm→指尖同轴） |
| 四指尖横宽 | ~67 mm（ff/mf/rf/lf distal rel palm: x +33/+11/−11/−34 mm） |
| thumb 短指 | thumb distal 距指尖 ~87 mm 短（rel palm z +78.5 vs 长指 ~+160） |

## 3. Shadow 屈曲语义（实测 `probe_shadow_sign.py`，正负号曾误判后校准）

- **正命令 = 屈曲**（lim 元数据 [−1.571,0] 不可信，已实测弃用）；J1/J2 可至 +1.57，末端 J3 截顶于 **+0.349**；fan（J0）不被 'fingers' actuator 驱动。
- 屈曲只把指尖沿掌前方向**回收缩短**：curl 0.5→指尖 z_relpalm ~145 mm、1.1→~103 mm、全屈→~71 mm。**永不越过 palm** → 无法像人那样「指垫裹圆」。
- 与 Allegro 关键差异：Allegro 手只有 ~0.2 m 短、palm 有横移；Shadow 是 0.46 m 直链 + 更长指尖，对「压到低矮对象」反而更难。

## 4. palm 可达图（fingers-up 挂载，`probe_merged_palm_map.py`，EE=palm，DLS-IK，前侧网格 x∈[−0.34..−0.74] × z∈[0.46..1.00]，y=−0.026）

| 朝向 (ez) | 结果（pos_err mm） |
|---|---|
| down（指尖竖直下） | **全格 ≥ 232**（min 232 @ x−0.34 z0.90）→ 顶族不可得 |
| diag_xn（斜前下 −x） | 仅零星低位收敛：**47** @ x−0.34 z0.90、111 @ x−0.58 z0.46、137 @ x−0.58 z0.54；余 ≥ ~160 |
| diag_yn（斜 −y） | 全部 ≥ 146（min @ x−0.42 z0.46） |

低带探针（`probe_low_band.py`，z∈{0.20..0.40}）：down 全 ≥ 386（最差 597）；diag_xn 最好 **125 mm** @ x−0.70 z0.32（127 @ z0.26、166 @ z0.40）——**没有任何格 <60 mm**。

## 5. 可达锚点下的指尖落点（`probe_merged_fingers.py --step 140`，`probe_nbh.log`）

| 锚点 | pos_err mm | 实际 palm | 4-tip bbox | thumb z |
|---|---|---|---|---|
| nbh47（目标 (−0.34,−0.026,0.90)） | 295 | (−0.271,−0.308,0.848) | z [0.753,0.783] | ~0.84 |
| nbh47_y2（同点 ex=−y） | 144 | (−0.453,−0.065,0.980) | x[−0.617,−0.584], z[0.887,0.943] | ~1.02 |

对照：直立异形瓶帽顶 world 0.585、瓶身 z ≲0.57。**可达位形下指尖最低 0.73、一般 0.85–0.94**，离瓶身还有 0.15 m+；且 DLS 收敛态 palm 落在 z0.85–1.0 高位，与命令差 144–295 mm，说明 IK 在低斜处陷于姿态死角。

## 6. 根因（与 Allegro 各负结论同源、且更重）

iiwa7 在瓶位附近的前工作空间 = **顶/斜顶下压族**。Shadow 要「环绕直立瓶 + thumb 压顶」需要手掌侧对瓶（palm 低、指尖横过瓶轴）——arm 给不出；而给的出的是把 0.46 m 直链朝下/斜下探，指尖落点因此恒比瓶身高一截。屈曲轴平行 + 指尖收不回掌，更排除了对圆柱的逐指环绕。→ **换手本身没有解锁 wrap**，只是换了更拟人的手指；要让拟人 wrap 成立，得让 arm 的顶族把手**横向**送到瓶侧，或改对象/语义。

## 7. 决策选项（请拍板）

| 方向 | 做法 | 风险 / 代价 |
|---|---|---|
| **A. 挂载旋转重扫（推荐先试）** | 用 `author_kuka_shadow.py` 已有 `--mx/--my/--mz/--rot_deg`，把 0.46 m 直链焊到 ⊥ link7（或全 3 轴扫几个角度），把 arm 的过顶斜下族变成「手横担到瓶侧」，重跑 palm 图 + 指尖/thumb 落点扫描 | 可能仍负；长手横担观感不拟人；每角度一次 GPU 重扫 |
| B. 接受顶/斜顶族：拟人「顶部爪握/盖握」小物 + 偏心压嘴 | 对象做成紧凑小物（Ø~35–45 mm、矮），Shadow 自上方以 4 长指盖握瓶身上沿+帽、thumb 在侧；压嘴偏心放 thumb 落点（沿用异形瓶 author 工具思路） | 需先扫「顶握各指是否贴体 + thumb 落点」新几何；握持语义弱于侧抱 |
| C. 止损回退 | Shadow 方向记为负（本报告留存数据/资产）；回退到**已跑通**的异形瓶长指按压（Allegro `chosen.usda` + `phase1_press.py`，nozzle −0.0035） | 放弃拟人 Shadow；直接进 Phase 2 RL |

## 8. 复现

```bash
cd /home/ubuntu/press_demo/IsaacLab && conda activate env_isaaclab
# palm 可达图
./isaaclab.sh -p ../isaac_demo/scripts/probe_merged_palm_map.py --headless > /home/ubuntu/press_demo/logs/probe_merged_palm_map.log 2>&1
# 低带
./isaaclab.sh -p ../isaac_demo/scripts/probe_low_band.py --headless > /home/ubuntu/press_demo/logs/probe_low_band.log 2>&1
# 可达锚点指尖落点
./isaaclab.sh -p ../isaac_demo/scripts/probe_merged_fingers.py --headless --step 140 > /home/ubuntu/press_demo/logs/probe_nbh.log 2>&1
# 渲染（可选，目视）：env -u DISPLAY ... --enable_cameras
grep -E 'err=|4-tip' logs/probe_merged_palm_map.log logs/probe_nbh.log
```

日志：`logs/{probe_merged_palm_map,probe_low_band,probe_nbh,probe_shadow_sign,probe_shadow_merged_curl,scan_merged_press}.log`。

---

## 9. Option A（挂载旋转）实测补充（2026-09-07 追加，负）

用户拍板走 A。`author_kuka_shadow.py` 扩成支持绕 link_7 局部 x/y/z 组旋转（`--rotx/--roty/--rotz`），产变体 `assets/kuka_shadow_rx90.usda`（rotx=+90，home 处链朝下）、`rx-90`、`ry-90`。新探针 `scripts/probe_mount_layout.py`（--usd/--orient/网格）在 front 网格上按不同链方向命令 palm、量指尖/拇指相对虚拟瓶的径向与 z 重叠。结果：

| 测试 | 结果 |
|---|---|
| rx90 链朝下（palm ez=−z）over-front 网格（x−0.60..−0.20 × z0.64..0.95, 20 格） | **0 可达**，全 err ≥ 295 mm |
| ry−90 链朝下 同网格 | **0 可达**，全 err ≥ 97 mm（仅 x−0.20 z0.95 高空 97 mm，其余 ≫） |
| identity 链水平 −x（fingers 横指瓶侧，palm z0.50..0.62 x−0.35..−0.17，12 格） | 11 格 err≥144，仅 1 格 err85；可达那格 palm 实际 (−0.117,−0.036,0.686)，指尖径向 116–141 mm、z 668（**瓶身 band [480,600] 上方 68mm**），touch=0 → 指尖到不了瓶身 |

判读：挂载旋转只把"同一套臂可达位形"重贴标签（rx90 让 reach-over 位形的链转向侧向，而不是让链在对象上方竖直）；把链竖直或水平送到瓶身高度仍要 arm 在低斜处摆出直立/横担姿态，正是其到不了的那族。→ **A（挂载旋转）实测未解锁 wrap+thumb-press**，与 §0 根因同向。变体资产已留存，可复现。

---

## 10. 「换平面」（瓶放到臂 sagittal 之外）实测补充（2026-09-07，部分解锁：wrap 半边首次成立，thumb 压仍未成）

用户提出：**别把瓶/臂/手放在同一平面**。此前全在臂前(sagittal 附近)扫；先导 `probe_multiseed.py --mode sidey` 发现链 ∓y 侧入能把 palm 降到 z~0.55（sagittal 扫从未那么低）。据此扫 lateral 网格 `scripts/probe_lateral.py`（多 seed IK；瓶位 xlist −0.52/−0.44/−0.36 × ylist +0.12..+0.30；palm z∈瓶身带；每瓶两侧 ±y 侧入）。瓶 TOP0.60 / body [0.48,0.60] / r22。

### 10.1 结果（`logs/probe_lateral.log`）

- **收敛全在 x=−0.36 列 + y− 侧入**（sgn=−1：palm 在瓶 −y 侧、链 +y 指向瓶）：11 格 err<80（best 22–69mm）；**y+ 侧入 0 格收敛**（全 ≥118）。
- **真实瓶身接触首次出现（本报告 campaign 首个）**：

| 瓶位 (x,y) | palm 目标 z | err | 实际 palm | touch 判定 |
|---|---|---|---|---|
| (−0.36,+0.12) | 0.53 | 61 | (−0.377,−0.003, **0.589**) | 段径向 11/11/32/55mm → 3 段 <r+pad34 |
| (−0.36,+0.18) | 0.53 | 69 | (−0.361,+0.070, 0.597) | 8/30/52/75 → 2 段 |
| (−0.36,+0.24) | 0.61 | 66 | (−0.349,+0.106, 0.552) | 1 段 |

   palm z 目标 0.53 时 DLS 只把 palm 降到 ~0.55–0.59（垂直残差为主）——恰在瓶身带 → 长指从低位 palm 横向伸到瓶身带近侧。

- **详细复核（`probe_lateral_view.py`，复现 (−0.36,+0.12) zc0.53 y−，多轮取全局最小 err 53，palm 钉定 (−0.376,−0.055,**0.552**)；注：该 probe 里"瓶"为纯可视圆柱，无碰撞——低 wrap 时指段可穿入瓶体，故把段径向 < r 判为 PEN（穿），[r, r+pad) 判为 surf（真贴））**：
  - **surf=2 pen=0**：mf=26mm、rf=28mm（=真贴瓶表 r22+~4–6mm 的近侧）；ff=43、lf=44mm（21–22mm 离表）。指尖 y≈+0.092–0.098 ≈ 瓶近侧表面线(y=0.098)，z 0.527–0.551（瓶身带）→ **四指到瓶近侧表面、未绕过远侧，无对掌**。palm 位比 probe_lateral 的 0.589 略低（0.552），指段不再穿瓶 → 是**单侧"指尖+指垫贴瓶面"的钩靠，不是包圆**。
  - **thumb 恒远离且不达顶**：此解 THtip 距轴 133–148mm、dz −36..−85（瓶身带/低于顶）。lateral 网格里 thumb_r 最小 ~49–61mm（瓶 y0.18–0.30）仍在表外 27–40mm、z 在瓶身带或刚过顶——**没有任何解让 thumb 位于可压的点而手指同时贴瓶**。

### 10.2 判读

1. **用户"换平面"直觉部分证实**：把瓶摆到臂 sagittal 之外 + 从近侧 y− 入，**arm 第一次把 palm 送到瓶身带高度**且指尖/指垫真触瓶面——这是此前 front 网格 / 挂载旋转 / 高位 alias 全都没有过的（这是"低位手族开在臂侧面而非正前方"的正面证据）。
2. **但 wrap 是单侧钩靠，非环绕**：Shadow 四平行长指没有对掌拇指配合时，只能让指尖到瓶**近侧**表面；palm z 抬高到让指段"包过远侧"会穿瓶(palm 0.589 版 ff/mf 径向 11 < r22 = 穿)，降到不穿(palm 0.552)则 mf/rf 贴表、ff/lf 差 2cm。真"环抱"仍无：要绕远侧需指尖横越轴后能弯回来，指节屈曲做不到。
3. **thumb-press 与 body-contact 依旧互斥**（与 §0 同构、只是换到 lateral 平面）：贴瓶时 thumb 50–150mm 外、≤ 顶高；thumb 上到顶(dz>0)时 palm 太高 → 指不贴瓶。加高 curl 只把 thumb 往下/回收，够不到顶中心或 rim。
4. **渲染尝试未果**：headless EGL 下此场景（GroundPlane+DomeLight+无 table 的红圆柱+合并 Shadow）相机出图全白（rgb 非白 ~0%），demo 式 mp4 需带桌面/异形瓶 USD 的场景；本阶段几何结论以数值为准，无逐帧视觉佐证。

### 10.3 决策建议

数据不支持继续在同一硬件+直立小瓶上追"同次握持内贴瓶 + thumb 压顶"。可选：
- **(1) 接受单侧低位钩靠为"握"，改任务语义**：用 2 指（mf/rf）贴瓶近侧作稳定 + 放弃 thumb——不再是"按压"任务；不符合用户原始语义。
- **(2) 把可压的东西放到 thumb 自然落点（偏心嘴），thumb 从旁压、不压顶**：thumb 落点在瓶身带上部一侧(r~49–61、dz~−27..−54)→ 嘴做成瓶侧凸台，thumb 沿其滑台轴（非竖直）侧压。仍要"mf/rf 贴瓶 + thumb 侧压"同一 palm 高度能否共处，值得一次扫描；但"拇指按水泵"的拟人语义从"压顶"变"压侧"。
- **(3) 止损**：Shadow+iiwa7 单臂对直立小瓶的"抓瓶+thumb 压顶"记录为负（本报告）；回退到已跑通的**异形瓶长指按压**（Allegro chosen.usda + phase1_press.py，nozzle −0.0035，已进 Phase 2 RL 可行）。
- 不推荐再烧扫描找"贴瓶同时 thumb 压顶"的解——两目标在 palm 高度上互斥，已多平面证实。

日志：`logs/{probe_lateral,probe_lateral_view,probe_multiseed}.log`（复现命令行同 §8 + `--ox -0.36 --oy 0.12` / probe_lateral_view 加 `--enable_cameras` 与 `env -u DISPLAY`）。
