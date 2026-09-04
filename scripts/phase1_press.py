"""Phase 1 按压验证：工作台+药瓶+Kuka/Allegro;驱动食指(index_link_3)朝下对准喷嘴下压、松开回弹。

方法要点(基于 reach 探测校准):
  * Allegro 名义指形固定;7 个 iiwa 关节做 DLS IK。竖直朝下(index 直指 -Z)的高位姿态在机械臂
    workspace 里不成立——可达的"近似竖直"姿态总是落在食指名义 x/y 附近、EE 高度 ~0.65。因此
    默认把药瓶自动摆到 index_link_3 名义正下方(bx,by = 名义 EE x/y),避免让臂去做不可达的平移。
  * A 阶段:在瓶位正上方(EE z 保持 hover 高度,不命令下探)原地转到竖直朝下,只允许小幅回落,
    期望 residual ~0.2-0.3 rad(≈85% 竖直,足以垂直下压)。
  * D 阶段:沿 -Z 接触驱动下压;nozzle_joint 离开 0 记首触,到 ~下极限判触发。
  * H 稳住,R 抬回释放看弹簧回弹。

药瓶位置可用 --bottle_x/--bottle_y 覆盖(自动时为 None)。四元数一律 wxyz。
"""

import argparse
import os

import numpy as np
import torch

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--table_top", type=float, default=0.44)
parser.add_argument("--bottle_x", type=float, default=-0.545, help="药瓶 x(默认=index_link_3 名义正下方,reach 校准值)")
parser.add_argument("--bottle_y", type=float, default=-0.026)
parser.add_argument("--hover_z", type=float, default=0.665, help="A 阶段对准时 EE 命令 z(≈名义 EE z,别下探)")
parser.add_argument("--descend_dz", type=float, default=0.0003, help="每步垂直下压增量(m)")
parser.add_argument("--nozzle_bottom", type=float, default=-0.0035, help="视为到位的 nozzle 行程")
parser.add_argument("--video", default=None, help="可选: 输出 mp4 路径;给出则录制按压全过程")
parser.add_argument("--video_every", type=int, default=4, help="每隔 N 个物理步抓一帧(120Hz 下 4 → 30fps)")
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.utils import math as m_math
from isaaclab_assets import KUKA_ALLEGRO_CFG

BOTTLE_USD = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets", "medicine_bottle.usda")
NOZZLE_TOP_HOME = 0.145


def main():
    dt = 1 / 120.0
    dev = "cuda:0"
    sim = SimulationContext(SimulationCfg(dt=dt, device=dev, gravity=(0.0, 0.0, -9.81)))
    table_top = args_cli.table_top

    gnd_cfg = sim_utils.GroundPlaneCfg()
    gnd_cfg.func("/World/GroundPlane", gnd_cfg)
    light_cfg = sim_utils.DomeLightCfg(intensity=800.0)
    light_cfg.func("/World/Light", light_cfg)

    bx, by = args_cli.bottle_x, args_cli.bottle_y
    th = 0.06
    table = RigidObject(
        RigidObjectCfg(
            prim_path="/World/Table",
            spawn=sim_utils.CuboidCfg(
                size=(0.6, 0.9, th),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.55, 0.42, 0.28)),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True),
                collision_props=sim_utils.CollisionPropertiesCfg(),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(bx, by, table_top - th / 2)),
        )
    )
    bottle = Articulation(
        ArticulationCfg(
            prim_path="/World/Bottle",
            spawn=sim_utils.UsdFileCfg(usd_path=BOTTLE_USD),
            init_state=ArticulationCfg.InitialStateCfg(pos=(bx, by, table_top)),
            actuators={},
        )
    )
    robot = Articulation(KUKA_ALLEGRO_CFG.replace(prim_path="/World/Robot"))

    # 视频录制状态(相机传感器必须在 sim.reset() 前创建,否则不会在 PLAY 时初始化)
    cam = None
    vid_every = max(1, args_cli.video_every)
    vid_cnt = [0]
    vid_frames = []
    if args_cli.video:
        from isaaclab.sensors import Camera, CameraCfg

        cam = Camera(
            CameraCfg(
                prim_path="/World/Cam",
                update_period=0.0,
                height=540,
                width=960,
                data_types=["rgb"],
                spawn=sim_utils.PinholeCameraCfg(
                    focal_length=24.0,
                    focus_distance=400.0,
                    horizontal_aperture=20.955,
                    clipping_range=(0.1, 1.0e5),
                ),
            )
        )

    sim.reset()
    table.reset()
    bottle.reset()
    robot.reset()

    jn = list(robot.joint_names)
    qpos = torch.zeros(1, robot.num_joints, device=dev)
    for pattern, val in KUKA_ALLEGRO_CFG.init_state.joint_pos.items():
        import re as _re
        rx = _re.compile("^" + pattern.replace("(", "(?:") .replace(")", ")") + "$")
        for i, n in enumerate(jn):
            if rx.match(n):
                qpos[0, i] = val
    qvel0 = torch.zeros_like(qpos)

    # 瞬移名义位形并保持(不运动,避免扫过药瓶)
    robot.write_joint_state_to_sim(qpos, qvel0)
    robot.set_joint_position_target(qpos)
    for _ in range(12):
        robot.write_data_to_sim()
        bottle.write_data_to_sim()
        table.write_data_to_sim()
        sim.step()
        table.update(dt)
        bottle.update(dt)
        robot.update(dt)

    # 录像相机对准按压点(按压中心 (bx,by), 高度 ~ nozzle 顶)
    if cam is not None:
        eye = torch.tensor([[bx + 0.32, by + 0.62, 0.95]], device=dev, dtype=torch.float32)
        tgt = torch.tensor([[bx, by, 0.58]], device=dev, dtype=torch.float32)
        cam.set_world_poses_from_view(eye, tgt)
        print(f"[video] camera eye={eye[0].tolist()} target={tgt[0].tolist()}")

    names = list(robot.body_names)
    tip_i = names.index("index_link_3")
    arm_ids = [i for i, n in enumerate(jn) if n.startswith("iiwa7")]
    arm_ids_t = torch.tensor(arm_ids, device=dev)
    other_tips = {k: names.index(k) for k in ["middle_link_3", "ring_link_3", "thumb_link_3"] if k in names}

    def body_pose(idx):
        return robot.data.body_pos_w[:, idx].float(), robot.data.body_quat_w[:, idx].float()

    def zaxis_w(idx):
        ez = torch.zeros((1, 3), device=dev)
        ez[:, 2] = 1.0
        return m_math.quat_apply(robot.data.body_quat_w[:, idx].float(), ez)

    pump_top = table_top + NOZZLE_TOP_HOME
    print(f"pump_top(z)={pump_top:.4f}  bottle center=({bx:.4f},{by:.4f})")

    za0 = zaxis_w(tip_i)[0]
    print(f"[nominal] index_link_3 z-axis = ({za0[0]:+.3f},{za0[1]:+.3f},{za0[2]:+.3f})")

    # 目标姿态: 指尖 z 轴 → (0,0,-1)
    ez = torch.zeros((1, 3), device=dev)
    ez[:, 2] = 1.0
    q_cur = robot.data.body_quat_w[:, tip_i].float()
    v = m_math.quat_apply(q_cur, ez)
    w = -ez
    cross = torch.linalg.cross(v, w)
    c = torch.clamp((v * w).sum(-1), -1, 1)
    ang = torch.acos(c)
    q_rot = m_math.quat_from_angle_axis(ang, cross)
    q_tgt = m_math.quat_mul(q_rot, q_cur)
    q_tgt = q_tgt / torch.linalg.norm(q_tgt, dim=-1, keepdim=True)
    print(f"    need rotate index z by {float(ang):.2f} rad toward straight-down")

    ik_cfg = DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls")
    ik = DifferentialIKController(ik_cfg, num_envs=1, device=dev)

    def track(command, n=1):
        for _ in range(n):
            ee_pos, ee_quat = body_pose(tip_i)
            jac = robot.root_physx_view.get_jacobians()[:, tip_i, :, arm_ids_t]
            jp = robot.data.joint_pos[:, arm_ids_t].float().clone()
            ik.set_command(command)
            jp_des = ik.compute(ee_pos, ee_quat, jac, jp)
            jp_des = torch.clamp(jp_des, min=jp - 0.08, max=jp + 0.08)
            robot.set_joint_position_target(jp_des, arm_ids)
            robot.write_data_to_sim()
            bottle.write_data_to_sim()
            sim.step()
            bottle.update(dt)
            robot.update(dt)
            if cam is not None:
                vid_cnt[0] += 1
                if vid_cnt[0] % vid_every == 0:
                    cam.update(dt, force_recompute=True)
                    vid_frames.append(np.ascontiguousarray(cam.data.output["rgb"][0].cpu().numpy()))

    def move_track(ctarget, n, tag, every=200):
        p0, q0 = body_pose(tip_i)
        tp_, tq_ = ctarget[:, :3], ctarget[:, 3:]
        for it in range(n):
            tau = (it + 1) / n
            pc = p0 + (tp_ - p0) * tau
            qc = m_math.quat_slerp(q0[0], tq_[0], tau).unsqueeze(0)
            command = torch.cat([pc, qc], dim=1)
            track(command)
            if every and (it + 1) % every == 0:
                ep, _ = body_pose(tip_i)
                za = zaxis_w(tip_i)[0]
                print(f"    {tag} {it+1:4d}: pos_err={float(torch.linalg.norm(ep - tp_)):.4f}"
                      f" zaxis=({za[0]:+.2f},{za[1]:+.2f},{za[2]:+.2f})"
                      f" nozzle={float(bottle.data.joint_pos[0,0]):+.5f}")
        report(tag)

    def report(tag):
        tp, _ = body_pose(tip_i)
        za = zaxis_w(tip_i)[0]
        nz = float(bottle.data.joint_pos[0, 0])
        others = ", ".join(f"{k}={float(robot.data.body_pos_w[0, v, 2]):+.4f}" for k, v in other_tips.items())
        print(
            f"[{tag}] ee=({float(tp[0,0]):+.4f},{float(tp[0,1]):+.4f},{float(tp[0,2]):+.4f})"
            f" zaxis=({za[0]:+.2f},{za[1]:+.2f},{za[2]:+.2f}) nozzle={nz:+.5f} | {others}"
        )

    # ---- A: 原地转向竖直(EE 命令高度=hover,不往下探) ----
    cmd_hover = torch.cat([torch.tensor([[bx, by, args_cli.hover_z]], device=dev, dtype=torch.float32), q_tgt], dim=1)
    print("\n[A] rotate toward straight-down over bottle center ...")
    move_track(cmd_hover, 1100, "align")
    track(cmd_hover, 250)
    report("align_settle")
    za = zaxis_w(tip_i)[0]
    residual = float(torch.acos(torch.clamp(-za[2], -1, 1)))
    print(f"    angle from straight-down = {residual:.3f} rad")

    # ---- D: 接触驱动垂直下压 ----
    print("\n[D] vertical descend ...")
    command = cmd_hover.clone()
    first_contact_z = None
    hit_bottom = False
    n_steps = 0
    while True:
        command[:, 2] = float(command[0, 2]) - args_cli.descend_dz
        zc = float(command[0, 2])
        track(command)
        n_steps += 1
        nz = float(bottle.data.joint_pos[0, 0])
        ee_z = float(robot.data.body_pos_w[0, tip_i, 2])
        if nz < -0.0002 and first_contact_z is None:
            first_contact_z = ee_z
            print(f"    >> first contact  z_cmd={zc:.4f} nozzle={nz:+.5f} ee_z={ee_z:.4f}")
        if nz <= args_cli.nozzle_bottom:
            hit_bottom = True
            print(f"    >> nozzle bottom  z_cmd={zc:.4f} nozzle={nz:+.5f} ee_z={ee_z:.4f}")
            break
        if zc < pump_top - 0.02 and nz > -0.0002:
            print(f"    !! EE command far below nozzle top w/o stroke (nozzle={nz:+.5f}); abort")
            break
        if n_steps > 6000:
            print("    !! too many descend steps")
            break
        if n_steps % 120 == 0:
            print(f"    descend {n_steps:4d}: z_cmd={zc:.4f} ee_z={ee_z:.4f} nozzle={nz:+.5f}")
    report("press_end")
    nz_press = float(bottle.data.joint_pos[0, 0])

    # ---- H: 稳住 ----
    for _ in range(50):
        track(command)
    nz_held = float(bottle.data.joint_pos[0, 0])
    print(f"\n[held] nozzle = {nz_held:+.5f}")

    # ---- R: 抬回 hover(撤力,看回弹) ----
    print("\n[R] lift back ...")
    move_track(cmd_hover, 700, "release")
    for _ in range(120):
        track(cmd_hover)
    nz_final = float(bottle.data.joint_pos[0, 0])
    print(f"\n[released] nozzle = {nz_final:+.5f}")

    if args_cli.video and vid_frames:
        import os

        import cv2

        def _to_bgr(fr):
            if fr.dtype != np.uint8:
                fr = np.clip(fr * 255.0 if fr.max() <= 1.01 else fr, 0, 255).astype(np.uint8)
            return cv2.cvtColor(fr, cv2.COLOR_RGB2BGR)

        os.makedirs(os.path.dirname(os.path.abspath(args_cli.video)) or ".", exist_ok=True)
        H, W = vid_frames[0].shape[:2]
        fps = int(round(1.0 / dt / vid_every))
        writer = cv2.VideoWriter(args_cli.video, cv2.VideoWriter_fourcc(*"mp4v"), fps, (W, H))
        for fr in vid_frames:
            writer.write(_to_bgr(fr))
        writer.release()
        print(f"\n[video] {len(vid_frames)} frames {W}x{H} @ {fps}fps -> {args_cli.video}")

    summary = {
        "bottle_center_used": (round(bx, 4), round(by, 4)),
        "align_residual_rad": round(residual, 3),
        "first_contact_ee_z": round(first_contact_z, 4) if first_contact_z else None,
        "hit_nozzle_bottom": hit_bottom,
        "nozzle_at_press_end": nz_press,
        "nozzle_held": nz_held,
        "nozzle_after_release": nz_final,
    }
    print(f"\nSUMMARY {summary}")
    print("DONE")
    simulation_app.close()


if __name__ == "__main__":
    main()
