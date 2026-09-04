# press-medicine-bottle

灵巧手按压药瓶喷嘴仿真 Demo（独立任务，与 CerebVLA / SomaVLA 项目无关）。

Isaac Lab + Allegro Hand（KUKA iiwa7 机械臂 + 16-DoF 灵巧手）环境下，驱动食指垂直下压药瓶喷嘴、
喷嘴弹簧被压到底后松指回弹。当前为 **Phase 1** 验收成果：场景加载 + 接触按压（无穿模）。

## 目录结构

```
scripts/
  phase1_press.py         # 核心：原地转向竖直 → 垂直下压到底 → 稳住 → 抬指看回弹；可选录制 mp4
  phase1_scene_setup.py   # 场景装配 + 400 步稳定性检查（加载信息 / 底座不漂移 / 喷嘴稳定）
  probe_robot.py          # 机器人加载与名义位形检查（探索用）
  probe_finger_flex.py    # 手指屈曲探测（探索用）
  probe_reach.py          # index_link_3 可达性探测（决定药瓶摆放位，探索用）
  test_bottle_asset.py    # 药瓶资产解析 + 喷嘴压底回弹验证（早期）
assets/
  medicine_bottle.usda    # 药瓶资产：底座固定到世界 + 喷嘴 1-DOF 弹簧滑台(Z, 下极限 -0.004)
media/
  press.mp4               # Phase 1 成功按压的可视化回放（20.5s, 960x540@30fps）
```

## 运行环境

- Isaac Sim 5.1.0.0 (pip) / Isaac Lab `main` / `KUKA_ALLEGRO_CFG`（isaaclab_assets 自带）
- 运行方式：

```bash
conda activate env_isaaclab
cd <IsaacLab 根目录>
./isaaclab.sh -p /path/to/press_repo/scripts/phase1_press.py --headless

# 录制可视化视频(需离线渲染;若本机有不可用的 X DISPLAY,先 env -u DISPLAY)
env -u DISPLAY ./isaaclab.sh -p /path/to/press_repo/scripts/phase1_press.py \
    --headless --enable_cameras --video out.mp4
```

脚本内资产路径为相对路径（`scripts/` 与 `assets/` 同层），克隆后无需改动即可运行。

## 关键几何约束（源自可达性探测）

- KUKA 名义位形下 `index_link_3` 原点 ≈ `(-0.545, -0.026, 0.67)`。把食指 z 轴转到竖直(-Z)的
  **高位姿态在机械臂 workspace 不成立**；可达的近似竖直姿态总落在该 x/y 附近、EE 高度 ~0.63–0.65。
  因此药瓶默认摆在指尖名义正下方（`--bottle_x -0.545 --bottle_y -0.026`），不让 IK 做大平移。
- A 阶段 EE 命令高度取 hover（`--hover_z 0.665`），**不要下探**；否则 EE 侧向漂移导致下压落在喷嘴旁。
- DLS IK 每步关节 clamp 需 ≥ 0.08 rad（0.05 会卡在不良姿态）。

## Phase 1 验收结果

```
SUMMARY {'bottle_center_used': (-0.545, -0.026), 'align_residual_rad': 0.364,
         'first_contact_ee_z': 0.6061, 'hit_nozzle_bottom': True,
         'nozzle_at_press_end': -0.0035278, 'nozzle_held': -0.0037854,
         'nozzle_after_release': -2.16e-05}
```

食指接触喷嘴表面并压至 **≈ -0.0035 ~ -0.0038 m**（下极限 -0.004，未过冲 = 无穿模），
松开后弹簧回弹至 ~0。可视化见 `media/press.mp4`。
