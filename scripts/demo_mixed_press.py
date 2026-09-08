"""混形双手 demo (M2 夹 + M3 掌心压) : Panda 长指平爪钳住自由瓶, Shadow 掌心原点正压 cap 顶,
用 palm ContactSensor 读按压期掌心法向力 + nozzle 行程, 输出 tactile_verdict (USABLE / NOT_USABLE)。

布局: 瓶 home (BX,BY,table) 自由站桌面; panda 长爪从瓶 -x(西)侧钳住瓶身中段(zgrab);
iiwa7+Shadow(presser) 基座在瓶 +x 侧绕 z 朝向瓶, 多 seed DLS 把 robot0_palm 原点摆到 cap 正上方、
掌心(pad 面, 已取反手背外法向)朝下 (origin-aim, yaw=-90), 然后整臂沿 -z 下探: 掌心压 cap 顶 -> nozzle 行程到底 -> 稳住 ->
抬回看回弹 -> 爪松开。全程量瓶 root 漂移 / nozzle / palm 法向力。

用法: cd IsaacLab && env -u DISPLAY ./isaaclab.sh -p ../isaac_demo/scripts/demo_mixed_press.py \
      --headless --enable_cameras --video ../outputs/mixed_press.mp4
      [--bx --by --table --ns --step --hover_mm --descend_vel --nozzle_bottom --zgrab --bdist]
      [--b_fixed]  回退: 瓶 base-fixed 版(不钳, 直接压)
输出: CLAMP_* / PALM_IK REACH / descend 接触/到底 / [held] / rebound / drift / tactile_verdict / DONE
"""

import argparse
import math
import os
import re

import torch

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--video", default=None)
parser.add_argument("--video_every", type=int, default=3)
parser.add_argument("--bx", type=float, default=-0.545)
parser.add_argument("--by", type=float, default=-0.026)
parser.add_argument("--table", type=float, default=0.44)
parser.add_argument("--cap_z_local", type=float, default=0.145)
parser.add_argument("--hover_mm", type=float, default=30.0)
parser.add_argument("--px", type=float, default=-0.05, help="presser 基座 x")
parser.add_argument("--py", type=float, default=-0.03, help="presser 基座 y")
parser.add_argument("--pz", type=float, default=0.0, help="presser 基座世界 z(垫高), 需配 --scale 缩短后仍够到 cap")
parser.add_argument("--scale", type=float, default=1.0, help="presser 整机等比缩放(spawn.scale), 1.0=不缩(旧口径)")
parser.add_argument("--yaw", type=str, default="-90", help="手指前向水平角(deg), 逗号分隔扫; pad-down 需 yaw=-90(实测 FEAS), +90 为镜像 NOREACH")
parser.add_argument("--ns", type=int, default=8)
parser.add_argument("--step", type=int, default=340)
parser.add_argument("--descend_vel", type=float, default=0.0003)
parser.add_argument("--nozzle_bottom", type=float, default=-0.004)
parser.add_argument("--bdist", type=float, default=0.574, help="panda 基座离瓶距离")
parser.add_argument("--pside", type=str, default="west", choices=["west", "east", "north", "south"],
                    help="panda 钳瓶侧: 默认 west(-x)。presser 掌心下压时 wrist 落在瓶北(y+)柱, "
                         "north 侧 panda 会物理挡 wrist(M3 实测 err~70mm NOREACH), west 侧整列留空")
parser.add_argument("--zgrab", type=float, default=0.515, help="panda 抓瓶高度(世界 z); 需低于 cap 让刀片顶(≈z+0.025)让出 palm 下探柱")
parser.add_argument("--b_fixed", action="store_true", help="回退口径: 不做 panda 钳, panda 停远, palm 直接压 free 瓶")
parser.add_argument("--panda_nocollide", action="store_true", help="debug: 关掉 panda 全部碰撞, 判别下探受阻是接触还是姿态")
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.sensors import ContactSensor, ContactSensorCfg
from isaaclab.sim import SimulationCfg, SimulationContext
from _kuka_shadow_cfg import KUKA_SHADOW_CFG
from _panda_longjaw_cfg import PANDA_LONGJAW_CFG, JAW_TH, JAW_U, JAW_ZC

from isaaclab.sim.utils.stage import get_current_stage as _get_stage
from pxr import PhysxSchema

DEV = "cuda:0"
BX, BY = args_cli.bx, args_cli.by
TAB = args_cli.table
CAP_TOP = TAB + args_cli.cap_z_local
HERE = os.path.dirname(os.path.abspath(__file__))
# 始终用 free 瓶: base-fixed 瓶在 palm 过顶多 seed 收敛时会物理挡住下探路径(jam at ~z0.66,
# err~96mm NOREACH, M3 实测), free 瓶可被让开, palm 过顶达 hover err~6mm(与 probe_palm_press 同)。
BOTTLE_USD = os.path.join(HERE, "..", "assets", "hetero_bottle_chosen_free.usda")
KUKA_USD = os.path.join(HERE, "..", "assets", "kuka_shadow.usda")
# hetero_bottle 弹簧回位口径(tools/author_hetero_bottle.py 默认): prismatic 行程 5mm, 刚度 300 N/m, maxForce 6
NOZZLE_K = 300.0
NOZZLE_TRAVEL = 0.005


def rz_quat(phi):
    return (math.cos(phi / 2), 0.0, 0.0, math.sin(phi / 2))


def mat_to_quat(R):
    R = R.float()
    tr = R.trace()
    if tr > 0:
        S = math.sqrt(float(tr) + 1.0) * 2
        w = 0.25 * S
        x = float(R[2, 1] - R[1, 2]) / S
        y = float(R[0, 2] - R[2, 0]) / S
        z = float(R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = math.sqrt(1.0 + float(R[0, 0]) - float(R[1, 1]) - float(R[2, 2])) * 2
        w = float(R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = float(R[0, 1] + R[1, 0]) / S
        z = float(R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + float(R[1, 1]) - float(R[0, 0]) - float(R[2, 2])) * 2
        w = float(R[0, 2] - R[2, 0]) / S
        x = float(R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = float(R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + float(R[2, 2]) - float(R[0, 0]) - float(R[1, 1])) * 2
        w = float(R[1, 0] - R[0, 1]) / S
        x = float(R[0, 2] + R[2, 0]) / S
        y = float(R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return torch.tensor([w, x, y, z], device=DEV, dtype=torch.float32)


def quat_to_R(q):
    q = q.float() / torch.linalg.norm(q)
    w, x, y, z = q
    return torch.stack([
        torch.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)]),
        torch.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)]),
        torch.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]),
    ])


def quat_apply(q, v):
    qw = q[0]
    t = 2 * torch.linalg.cross(q[1:], v)
    return v + qw * t + torch.linalg.cross(q[1:], t)


def build_q(cfg, jn):
    q = torch.zeros(1, len(jn), device=DEV)
    for pattern, val in cfg.init_state.joint_pos.items():
        rx = re.compile("^" + pattern.replace("(", "(?:").replace(")", ")") + "$")
        for i, n in enumerate(jn):
            if rx.match(n):
                q[0, i] = val
    return q


def main():
    dt = 1 / 60.0
    sim = SimulationContext(SimulationCfg(dt=dt, device=DEV, gravity=(0.0, 0.0, -9.81)))
    gnd_cfg = sim_utils.GroundPlaneCfg()
    gnd_cfg.func("/World/GroundPlane", gnd_cfg)
    light_cfg = sim_utils.DomeLightCfg(intensity=800.0)
    light_cfg.func("/World/Light", light_cfg)
    th = 0.06

    table = RigidObject(RigidObjectCfg(
        prim_path="/World/Table",
        spawn=sim_utils.CuboidCfg(
            size=(0.6, 0.9, th), visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.55, 0.42, 0.28)),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True),
            collision_props=sim_utils.CollisionPropertiesCfg()),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(BX, BY, TAB - th / 2)),
    ))
    bottle = Articulation(ArticulationCfg(
        prim_path="/World/Bottle", spawn=sim_utils.UsdFileCfg(usd_path=BOTTLE_USD),
        init_state=ArticulationCfg.InitialStateCfg(pos=(BX, BY, TAB)), actuators={},
    ))

    # panda 长爪: 混形钳瓶路径才创建。b_fixed 回退不创建: 仅 panda articulation 在场
    # (即便 3m 外) 就会扰动 presser 过顶多 seed 收敛 err 6mm->~76mm(M3 实测), 需完全移除。
    PANDAP = not args_cli.b_fixed
    panda = None
    if PANDAP:
        pcfg = PANDA_LONGJAW_CFG.replace(prim_path="/World/Panda")
        dxc, dyc = {"west": (-1, 0), "east": (1, 0), "north": (0, 1), "south": (0, -1)}[args_cli.pside]
        BAX, BAY = BX + dxc * args_cli.bdist, BY + dyc * args_cli.bdist
        phi_p = math.atan2(BY - BAY, BX - BAX)
        pcfg.init_state.pos = (BAX, BAY, 0.0)
        pcfg.init_state.rot = rz_quat(phi_p)
        panda = Articulation(pcfg)
    else:
        BAX, BAY = BX, BY + 3.0

    # 基座垫高立柱(可选, pz>0 且缩放): 静态 cuboid, 顶面 z=pz, presser 基座落在其上够到原尺寸 cap
    pz = args_cli.pz
    if pz > 0.05:
        RigidObject(RigidObjectCfg(
            prim_path="/World/PresserRiser",
            spawn=sim_utils.CuboidCfg(
                size=(0.24, 0.30, pz), visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.35, 0.38)),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True),
                collision_props=sim_utils.CollisionPropertiesCfg()),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(args_cli.px, args_cli.py, pz / 2)),
        ))

    # presser: 基座放到瓶旁, 绕 z 转 phi 朝瓶; --scale 等比缩放整机(spawn.scale)
    phi = math.atan2(BY - args_cli.py, BX - args_cli.px)
    kcfg = KUKA_SHADOW_CFG.replace(prim_path="/World/Presser")
    kcfg.spawn.usd_path = KUKA_USD
    if args_cli.scale != 1.0:
        kcfg.spawn.scale = (args_cli.scale, args_cli.scale, args_cli.scale)
    kcfg.init_state.pos = (args_cli.px, args_cli.py, pz)
    kcfg.init_state.rot = rz_quat(phi)
    presser = Articulation(kcfg)

    # 相机(video)与 contact sensor 须在 sim.reset() 前建
    cam = None
    vid_every = max(1, args_cli.video_every)
    vid_cnt = [0]
    vid_frames = []
    rec = [False]
    if args_cli.video:
        from isaaclab.sensors import Camera, CameraCfg
        cam = Camera(CameraCfg(
            prim_path="/World/Cam", update_period=0.0, height=540, width=960, data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(focal_length=24.0, focus_distance=400.0,
                                             horizontal_aperture=20.955, clipping_range=(0.1, 1.0e5)),
        ))

    stage = _get_stage()
    palm_paths = [p.GetPath().pathString for p in stage.Traverse() if p.GetName() == "robot0_palm"]
    scope = palm_paths[0].rsplit("/", 1)[0] if palm_paths else "/World/Presser/shadow_mount"
    sensor_bodies = ["robot0_palm"] + [f"robot0_{f}proximal" for f in ["ff", "mf", "rf", "lf"]]
    for b in sensor_bodies:
        pp = [p.GetPath().pathString for p in stage.Traverse() if p.GetName() == b]
        api = stage.GetPrimAtPath(pp[0]).HasAPI(PhysxSchema.PhysxContactReportAPI) if pp else False
        print(f"[diag] {b:14s} contactAPI={api}", flush=True)
    sensor = ContactSensor(ContactSensorCfg(
        prim_path=f"{scope}/(robot0_palm|robot0_(ff|mf|rf|lf)proximal)",
        filter_prim_paths_expr=["/World/Bottle/base", "/World/Bottle/nozzle"],
        update_period=0.0,
    ))

    if args_cli.panda_nocollide:
        from pxr import UsdPhysics
        n_dis = 0
        for p in stage.Traverse():
            if str(p.GetPath()).startswith("/World/Panda"):
                ca = UsdPhysics.CollisionAPI(p)
                if ca and ca.GetCollisionEnabledAttr():
                    ca.GetCollisionEnabledAttr().Set(False)
                    n_dis += 1
        print(f"[panda_nocollide] disabled collision on {n_dis} prims", flush=True)
    sim.reset()
    table.reset(); bottle.reset(); presser.reset()
    if PANDAP:
        panda.reset()

    if PANDAP:
        pjn = list(panda.joint_names)
        pnames = list(panda.body_names)
        arm_ids = [i for i, n in enumerate(pjn) if re.fullmatch(r"panda_joint[1-7]", n)]
        hand_i = pnames.index("panda_hand")
        bL_i = pnames.index("blade_L")
        bR_i = pnames.index("blade_R")
        jaw_ids = [i for i, n in enumerate(pjn) if n.startswith("longjaw_")]
        pqnom = build_q(pcfg, pjn)
        arm_t = torch.tensor(arm_ids, device=DEV)
        jaw_t = torch.tensor(jaw_ids, device=DEV)

    jn = list(presser.joint_names)
    names = list(presser.body_names)
    palm_i = names.index("robot0_palm")
    parm_ids = [i for i, n in enumerate(jn) if n.startswith("iiwa7")]
    finger_ids = [i for i, n in enumerate(jn) if not n.startswith("iiwa7")]
    mcp = {f: names.index("robot0_%sproximal" % f) for f in ["ff", "mf", "rf", "lf"]}
    mid = {f: names.index("robot0_%smiddle" % f) for f in ["ff", "mf", "rf", "lf"]}
    distal = {}
    for f in ["ff", "mf", "rf", "lf"]:
        cand = [n for n in names if n.startswith("robot0_%s" % f) and "distal" in n]
        distal[f] = names.index(cand[0]) if cand else None
    thb = names.index("robot0_thproximal")
    kqnom = build_q(kcfg, jn)
    karm_t = torch.tensor(parm_ids, device=DEV)
    nj = bottle.num_joints

    def step_n(n=1):
        for _ in range(n):
            if PANDAP:
                panda.write_data_to_sim()
            bottle.write_data_to_sim(); presser.write_data_to_sim(); table.write_data_to_sim()
            sim.step()
            if PANDAP:
                panda.update(dt)
            bottle.update(dt); presser.update(dt); table.update(dt)
            sensor.update(dt)
            if rec[0]:
                vid_cnt[0] += 1
                if vid_cnt[0] % vid_every == 0 and cam is not None:
                    cam.update(dt, force_recompute=True)
                    vid_frames.append(np.ascontiguousarray(cam.data.output["rgb"][0].cpu().numpy()))

    def kpose():
        return presser.data.body_pos_w[:, palm_i].float(), presser.data.body_quat_w[:, palm_i].float()

    def set_root_state(x, y, z):
        pose14 = torch.tensor([[x, y, z, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0]], device=DEV, dtype=torch.float32)
        bottle.write_root_state_to_sim(pose14)
        bottle.write_joint_state_to_sim(torch.zeros((1, nj), device=DEV), torch.zeros((1, nj), device=DEV))

    # ---- 初始静置 ----
    if PANDAP:
        panda.write_joint_state_to_sim(pqnom, torch.zeros_like(pqnom))
        panda.set_joint_position_target(pqnom)
    presser.write_joint_state_to_sim(kqnom, torch.zeros_like(kqnom))
    presser.set_joint_position_target(kqnom)
    bottle.write_joint_state_to_sim(torch.zeros((1, nj), device=DEV), torch.zeros((1, nj), device=DEV))
    for _ in range(20):
        step_n(1)

    # palm frame 常数在干净 kqnom 位量一次(手未受 panda 阶段扰动/物理碰撞; 与 probe_palm_press 逐行一致:
    # 拇指提示翻 n_w 符号 + fwd 沿中指轴)。n_local/f_local/c_local 是刚体固有量, 之后整臂 IK 复用。
    def wp(body_idx):
        return presser.data.body_pos_w[0, body_idx].float()

    p0 = wp(palm_i)
    q0 = presser.data.body_quat_w[0, palm_i].float()
    R0 = quat_to_R(q0)
    M = torch.stack([wp(mcp[f]) for f in ["ff", "mf", "rf", "lf"]])
    centroid0 = M.mean(0)
    n_cand = torch.linalg.cross(M[1] - M[0], M[2] - M[3])
    if torch.linalg.norm(n_cand) < 1e-6:
        n_cand = torch.linalg.cross(M[2] - M[0], M[3] - M[1])
    n_w = n_cand / torch.linalg.norm(n_cand)
    thv = wp(thb) - centroid0
    inner_hint = thv - (thv @ n_w) * n_w
    if torch.linalg.norm(inner_hint) > 1e-3 and float(inner_hint @ n_w) < 0:
        n_w = -n_w
    # 拇指提示定的是 MCP 平面法向符号, 但实测该方向是「手背朝外」: 按它对齐世界 -z 得到的是
    # 掌心朝上、手背贴 cap(M3 视频判读)。取反 -> n_w 为掌心(接触面)外法向, 之后对齐 -z 即掌心朝下压。
    n_w = -n_w
    fwd = wp(mid["mf"]) - wp(mcp["mf"])
    fwd = fwd - (fwd @ n_w) * n_w
    fwd = fwd / torch.linalg.norm(fwd)
    n_local = R0.t().matmul(n_w)
    f_local = R0.t().matmul(fwd)
    c_local = R0.t().matmul(centroid0 - p0)
    print(f"[frame] palm0=({float(p0[0]):+.3f},{float(p0[1]):+.3f},{float(p0[2]):+.3f}) "
          f"n·(-z)={float(n_w @ torch.tensor([0.,0.,-1.],device=DEV)):+.3f} "
          f"c_local=({float(c_local[0]):+.3f},{float(c_local[1]):+.3f},{float(c_local[2]):+.3f})", flush=True)

    if cam is not None:
        # panda 现从西(x-)侧钳瓶, presser 从东下压 → 高架东北俯瞰, 同框两臂与瓶
        eye = torch.tensor([[BX + 0.15, BY + 0.95, 1.35]], device=DEV, dtype=torch.float32)
        tgt = torch.tensor([[BX, BY + 0.02, 0.52]], device=DEV, dtype=torch.float32)
        cam.set_world_poses_from_view(eye, tgt)
        print(f"[video] camera eye={eye[0].tolist()} target={tgt[0].tolist()}")

    # =====================================================================
    # 阶段 ① (不录): panda 静态多 seed 把刀片骑跨空座位, 瓶瞬移回 home 进开爪, 闭爪钳住
    # =====================================================================
    if not args_cli.b_fixed:
        def ppose(b_i):
            return panda.data.body_pos_w[:, b_i].float(), panda.data.body_quat_w[:, b_i].float()

        def drift_p():
            rr = bottle.data.root_pos_w[0]
            return math.hypot(float(rr[0]) - bx0, float(rr[1]) - by0) * 1000

        qo = pqnom.clone()
        for i in jaw_ids:
            qo[0, i] = JAW_U
        panda.write_joint_state_to_sim(qo, torch.zeros_like(qo))
        panda.set_joint_position_target(qo)
        step_n(20)
        H0, Q0 = ppose(hand_i)
        R0 = quat_to_R(Q0[0])
        BL, BR = panda.data.body_pos_w[0, bL_i].float(), panda.data.body_pos_w[0, bR_i].float()
        G0 = (BL + BR) / 2
        dH2G_local = R0.t().matmul(G0 - H0[0])
        eo_local = R0.t().matmul((BR - BL) / torch.linalg.norm(BR - BL))
        print(f"[geom] dH2G_local=({float(dH2G_local[0]):+.3f},{float(dH2G_local[1]):+.3f},{float(dH2G_local[2]):+.3f}) "
              f"eo_local=({float(eo_local[0]):+.3f},{float(eo_local[1]):+.3f},{float(eo_local[2]):+.3f})", flush=True)

        base_xy = torch.tensor([BAX, BAY], device=DEV, dtype=torch.float32)
        inward = torch.tensor([BX - base_xy[0], BY - base_xy[1], 0.0], device=DEV, dtype=torch.float32)
        inward = inward / torch.linalg.norm(inward)
        wz = torch.tensor([0.0, 0.0, 1.0], device=DEV)
        straddle = torch.linalg.cross(wz, inward)
        straddle = straddle / torch.linalg.norm(straddle)
        Rtar = torch.stack([straddle, torch.linalg.cross(inward, straddle), inward], dim=1)
        q = mat_to_quat(Rtar)

        RNG = {0: (-2.897, 2.897), 1: (-1.763, 1.763), 2: (-2.897, 2.897), 3: (-3.072, -0.070),
               4: (-2.897, 2.897), 5: (-0.017, 3.752), 6: (-2.897, 2.897)}

        def clampq(s):
            for j, (lo, hi) in RNG.items():
                s[0, arm_ids[j]] = float(min(hi, max(lo, s[0, arm_ids[j]])))
            return s

        seeds = [pqnom.clone()]
        for a2, a4 in [(1.2, -0.5), (1.6, -0.9), (0.6, -1.5), (-0.8, -0.5), (1.4, -2.6), (-1.2, -2.6),
                       (1.7, -0.3), (0.2, -2.0), (1.2, -1.4), (-0.4, -1.0)]:
            s = pqnom.clone(); s[0, arm_ids[1]] = a2; s[0, arm_ids[3]] = a4
            seeds.append(clampq(s))
        for vv in [1.5, -1.5]:
            s = pqnom.clone(); s[0, arm_ids[0]] = vv; s[0, arm_ids[3]] = -1.0
            seeds.append(clampq(s))
        for w6 in [0.3, 1.5, 2.5]:
            s = pqnom.clone(); s[0, arm_ids[5]] = w6; s[0, arm_ids[3]] = -1.2
            seeds.append(clampq(s))

        # park 瓶远处, 刀片静态多 seed 收敛到空座位
        set_root_state(BX, BY - 1.4, 0.0)
        for _ in range(15):
            step_n(1)
        ikp = DifferentialIKController(
            DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls"),
            num_envs=1, device=DEV)
        cmd_seat = torch.cat([(torch.tensor([BX, BY, args_cli.zgrab], device=DEV) - inward * JAW_ZC).unsqueeze(0), q.unsqueeze(0)], dim=1)
        best = (1e9, None)
        for s in seeds:
            panda.write_joint_state_to_sim(s, torch.zeros_like(s))
            panda.set_joint_position_target(s)
            step_n(3)
            for _ in range(520):
                ep, eq = ppose(hand_i)
                jac = panda.root_physx_view.get_jacobians()[:, hand_i, :, arm_t]
                jp = panda.data.joint_pos[:, arm_t].float().clone()
                ikp.set_command(cmd_seat)
                jd = torch.clamp(ikp.compute(ep, eq, jac, jp), min=jp - 0.09, max=jp + 0.09)
                panda.set_joint_position_target(jd, arm_ids)
                step_n(1)
            ep, _ = ppose(hand_i)
            err = float(torch.linalg.norm(ep[0, :3] - cmd_seat[0, :3]))
            if err < best[0]:
                best = (err, panda.data.joint_pos[:, arm_t].float().clone())
        # polish: 从 best 位形继续收敛(补方向/贴靠), 不做新 seed
        if best[1] is not None:
            pre_pol = best[0]
            jp0 = torch.zeros(1, panda.num_joints, device=DEV)
            jp0[:, arm_t] = best[1]
            for i in jaw_ids:
                jp0[0, i] = JAW_U
            panda.write_joint_state_to_sim(jp0, torch.zeros_like(jp0))
            panda.set_joint_position_target(jp0)
            step_n(3)
            for _ in range(600):
                ep, eq = ppose(hand_i)
                jac = panda.root_physx_view.get_jacobians()[:, hand_i, :, arm_t]
                jp = panda.data.joint_pos[:, arm_t].float().clone()
                ikp.set_command(cmd_seat)
                jd = torch.clamp(ikp.compute(ep, eq, jac, jp), min=jp - 0.04, max=jp + 0.04)
                panda.set_joint_position_target(jd, arm_ids)
                step_n(1)
            ep, _ = ppose(hand_i)
            errp = float(torch.linalg.norm(ep[0, :3] - cmd_seat[0, :3]))
            if errp < best[0]:
                best = (errp, panda.data.joint_pos[:, arm_t].float().clone())
            print(f"  [polish] seat_err {pre_pol*1000:.0f}->{errp*1000:.0f}mm", flush=True)
        full = torch.zeros(1, panda.num_joints, device=DEV)
        full[:, arm_t] = best[1]
        for i in jaw_ids:
            full[0, i] = JAW_U
        panda.write_joint_state_to_sim(full, torch.zeros_like(full))
        panda.set_joint_position_target(full)
        step_n(30)
        ep, _ = ppose(hand_i)
        print(f"SEAT_IK err={best[0]*1000:.0f}mm hand=({float(ep[0,0]):+.3f},{float(ep[0,1]):+.3f},"
              f"{float(ep[0,2]):+.3f}) {'REACH' if best[0]*1000 < 15 else 'NOREACH'}", flush=True)
        if best[0] * 1000 >= 15:
            print("ABORT noseat"); print("DONE"); simulation_app.close(); return

        set_root_state(BX, BY, TAB)
        for _ in range(60):
            step_n(1)

        def straddle_state():
            bl = panda.data.body_pos_w[0, bL_i].float()
            br = panda.data.body_pos_w[0, bR_i].float()
            M = (bl + br) / 2
            eo = br - bl
            eo = eo / torch.linalg.norm(eo)
            rr = bottle.data.root_pos_w[0]
            d2 = torch.tensor([rr[0], rr[1]], device=DEV, dtype=torch.float32) - M[:2]
            s = float((d2 * eo[:2]).sum())
            inner_half = float(torch.linalg.norm(br - bl)) / 2 - JAW_TH / 2
            return s, inner_half, inner_half - 0.021 - abs(s)

        s, ih, cl = straddle_state()
        print(f"[straddle] offset s={s*1000:+.1f}mm inner_half={ih*1000:.1f}mm clearance={cl*1000:+.1f}mm", flush=True)
        bx0, by0 = float(bottle.data.root_pos_w[0, 0]), float(bottle.data.root_pos_w[0, 1])
        rec[0] = True  # 开录: 从闭爪钳住开始(不含 seat/瞬移/hover IK 扫掠)

        print("[clamp] closing jaws (U->0) ...", flush=True)
        arm_hold = panda.data.joint_pos[:, arm_t].float().clone()
        pbase = torch.zeros(1, panda.num_joints, device=DEV)
        pbase[:, arm_t] = arm_hold
        for i, jv in enumerate(torch.linspace(JAW_U, 0.0, 300)):
            qc = pbase.clone(); qc[0, jaw_t] = float(jv)
            panda.set_joint_position_target(qc)
            step_n(1)
            if i % 100 == 0:
                print(f"  close {float(jv):+.4f}-> q_read={float(panda.data.joint_pos[0, jaw_t].mean()):+.4f} "
                      f"drift={drift_p():.1f}mm clear={straddle_state()[2]*1000:+.1f}mm", flush=True)
        for _ in range(150):
            panda.set_joint_position_target(pbase)
            step_n(1)
        qread = float(panda.data.joint_pos[0, jaw_t].mean())
        drift_c = drift_p()
        blz = max(float(panda.data.body_pos_w[0, bL_i, 2]), float(panda.data.body_pos_w[0, bR_i, 2]))
        print(f"  blade_top_z={blz:.4f} cap_top={CAP_TOP:.4f} gap={CAP_TOP-blz:+.0f}mm", flush=True)
        # 小钳移(<6mm)不 abort: 瓶被爪钳住后 palm hover 以实测瓶位(bx0c/by0c)为目标, 压准确不受影响。
        print(f"CLAMP_LONGJAW zgrab={args_cli.zgrab} seat_err={best[0]*1000:.0f}mm drift={drift_c:.1f}mm "
              f"q_read={qread:.4f} {'CLAMP_OK' if drift_c < 6.0 else 'CLAMP_DRIFT'}", flush=True)
        if drift_c >= 6.0:
            print("ABORT clamp_drift"); print("DONE"); simulation_app.close(); return
    else:
        bx0, by0 = float(bottle.data.root_pos_w[0, 0]), float(bottle.data.root_pos_w[0, 1])
        print("[b_fixed] bottle base-fixed (no panda clamp)", flush=True)
        pbase = None
        arm_hold = None
    rec[0] = False  # hover IK 多 seed 扫掠不录

    # =====================================================================
    # 阶段 ②+③+④: presser 掌心原点正压 cap (palm ContactSensor 读力), 到底-稳住-抬回看回弹
    # =====================================================================
    bx0c, by0c = float(bottle.data.root_pos_w[0, 0]), float(bottle.data.root_pos_w[0, 1])

    def drift_mm():
        rr = bottle.data.root_pos_w[0]
        return math.hypot(float(rr[0]) - bx0c, float(rr[1]) - by0c) * 1000

    # palm frame 常数已在上方干净 kqnom 位量(见 [frame] 打印); 此处直接复用 n_local/f_local/c_local。

    HOVER = args_cli.hover_mm / 1000.0
    # palm hover 以实测(被爪钳住后的)瓶位为心, 不受小钳移影响
    cap_goal = torch.tensor([bx0c, by0c, CAP_TOP + HOVER], device=DEV, dtype=torch.float32)
    Wv = torch.tensor([0.0, 0.0, -1.0], device=DEV)
    p_bx, p_by = args_cli.px, args_cli.py
    aim_local = torch.zeros(3, device=DEV)  # 掌心原点正压 cap (origin-aim)

    def build_cmd_yaw(yw_deg):
        gy = math.radians(yw_deg)
        dx, dy = bx0c - p_bx, by0c - p_by
        dist = math.hypot(dx, dy)
        ex_w = torch.tensor([(math.cos(gy) * dx - math.sin(gy) * dy) / dist,
                             (math.sin(gy) * dx + math.cos(gy) * dy) / dist, 0.0], device=DEV)
        U = ex_w / torch.linalg.norm(ex_w)
        V = torch.linalg.cross(Wv, U)
        u = f_local / torch.linalg.norm(f_local)
        w = n_local / torch.linalg.norm(n_local)
        v = torch.linalg.cross(w, u)
        Rcol = torch.stack([u, v, w], dim=1)
        R = torch.stack([U, V, Wv], dim=1).matmul(Rcol.t())
        pos = cap_goal - R.matmul(aim_local)
        q = mat_to_quat(R)
        return torch.cat([pos.unsqueeze(0), q.unsqueeze(0)], dim=1), R, pos

    ik = DifferentialIKController(
        DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls"),
        num_envs=1, device=DEV)

    def track(cmd, n, clamp=0.09):
        for _ in range(n):
            ep, eq = kpose()
            jac = presser.root_physx_view.get_jacobians()[:, palm_i, :, karm_t]
            jp = presser.data.joint_pos[:, karm_t].float().clone()
            ik.set_command(cmd)
            jd = torch.clamp(ik.compute(ep, eq, jac, jp), min=jp - clamp, max=jp + clamp)
            presser.set_joint_position_target(jd, parm_ids)
            step_n(1)

    seeds_k = [kqnom.clone()]
    for j1 in [-1.6, -0.8, 0.8, 1.6]:
        s = kqnom.clone(); s[0, parm_ids[0]] = j1; seeds_k.append(s)
    s = kqnom.clone(); s[0, parm_ids[3]] = 0.6; seeds_k.append(s)
    s = kqnom.clone(); s[0, parm_ids[1]] = 0.9; seeds_k.append(s)
    for wj in [4, 5, 6]:
        for v in [0.9, -0.9]:
            s = kqnom.clone(); s[0, parm_ids[wj]] = v; seeds_k.append(s)
    seeds_k = seeds_k[: max(1, args_cli.ns)]

    def best_over_seeds(cmd, n):
        best = (1e9, None)
        for s in seeds_k:
            presser.write_joint_state_to_sim(s, torch.zeros_like(s))
            presser.set_joint_position_target(s)
            step_n(3)
            track(cmd, n)
            ep, _ = kpose()
            err = float(torch.linalg.norm(ep[0, :3] - cmd[0, :3]))
            if err < best[0]:
                best = (err, presser.data.joint_pos[:, karm_t].float().clone())
        presser.write_joint_state_to_sim(kqnom, torch.zeros_like(kqnom))
        return best

    best_all = []
    for yw in [float(x) for x in args_cli.yaw.split(",")]:
        cmd_h, R, pos = build_cmd_yaw(yw)
        err, q_best = best_over_seeds(cmd_h, args_cli.step)
        fullk = torch.zeros(1, presser.num_joints, device=DEV)
        fullk[:, karm_t] = q_best
        fullk[0, finger_ids] = kqnom[0, finger_ids]
        presser.write_joint_state_to_sim(fullk, torch.zeros_like(fullk))
        presser.set_joint_position_target(fullk)
        for _ in range(10):
            step_n(1)
        ep, eq = kpose()
        qp = presser.data.body_quat_w[0, palm_i].float()
        cos_dn = float(quat_apply(qp, n_local) @ Wv)
        best_all.append((err, yw, cos_dn, fullk.clone()))
        print(f"  IK cell yaw={yw:+.0f}: err={err*1000:4.0f}mm cos_down={cos_dn:+.3f} "
              f"{'ok' if err*1000 < 18 and cos_dn > 0.95 else 'x'} "
              f"target=({float(pos[0]):+.3f},{float(pos[1]):+.3f},{float(pos[2]):+.3f})", flush=True)
    best_all.sort(key=lambda x: x[0])
    err0, yw0, cos_dn, fullk0 = best_all[0]
    cmd_hover, R0b, pos0 = build_cmd_yaw(yw0)
    presser.write_joint_state_to_sim(fullk0, torch.zeros_like(fullk0))
    presser.set_joint_position_target(fullk0)
    for _ in range(25):
        step_n(1)
    ep, eq = kpose()
    qp = presser.data.body_quat_w[0, palm_i].float()
    cos_dn = float(quat_apply(qp, n_local) @ Wv)
    reach = err0 * 1000 < 18 and cos_dn > 0.95
    print(f"PALM_IK aim=palm yaw={yw0:+.0f} err={err0*1000:.0f}mm palm=({float(ep[0,0]):+.3f},{float(ep[0,1]):+.3f},"
          f"{float(ep[0,2]):+.3f}) cos_down={cos_dn:+.3f} {'REACH' if reach else 'NOREACH'}", flush=True)
    Mp = torch.stack([presser.data.body_pos_w[0, mcp[f]].float() for f in ["ff", "mf", "rf", "lf"]]).mean(0)
    print(f"  palm_origin=({float(ep[0,0]):+.3f},{float(ep[0,1]):+.3f}) MCP_centroid=({float(Mp[0]):+.3f},"
          f"{float(Mp[1]):+.3f}) bottle=({bx0c},{by0c})", flush=True)
    if not reach:
        # 诊断: dump presser(及 panda, 若在场)近场 link 世界位, 定位 palm-down 下探柱被谁挡
        pnb = list(presser.body_names)
        for i, n in enumerate(pnb):
            if n.startswith("iiwa7") or n.endswith("robot0_palm"):
                p = presser.data.body_pos_w[0, i]
                if float(p[2]) > 0.40:
                    print(f"  PRESS {n.split('/')[-1]:16s} ({float(p[0]):+.3f},{float(p[1]):+.3f},{float(p[2]):+.3f})", flush=True)
        if PANDAP:
            pdb = list(panda.body_names)
            for i, n in enumerate(pdb):
                p = panda.data.body_pos_w[0, i]
                if float(p[2]) > 0.30:
                    print(f"  PANDA {n.split('/')[-1]:16s} ({float(p[0]):+.3f},{float(p[1]):+.3f},{float(p[2]):+.3f})", flush=True)
        print("NO_REACH_PALM_DOWN skip descend"); print("DONE"); simulation_app.close(); return

    # hover 多 seed 扫掠可能把「被爪钳住但摩擦有限」的自由瓶撞偏; 下探前以实测瓶位重建 hover 定心,
    # 这样按压期漂移以「重定心后」为参考(与 b_fixed 口径"压前瞬移回 home"同理, 排除 approach 阶段的撞击)。
    if PANDAP:
        bx0_old, by0_old = float(bx0c), float(by0c)  # 钳住后、hover 扫掠前的参考瓶位
        bx0c, by0c = float(bottle.data.root_pos_w[0, 0]), float(bottle.data.root_pos_w[0, 1])
        knock_mm = math.hypot(bx0c - bx0_old, by0c - by0_old) * 1000
        print(f"[recenter] hover扫掠后瓶漂移={knock_mm:.1f}mm 瓶位=({bx0c:+.4f},{by0c:+.4f}) "
              f"-> 重建 cap_goal/hover 对准实测 cap", flush=True)
        cap_goal[0] = bx0c; cap_goal[1] = by0c
        cmd_hover, R0b, pos0 = build_cmd_yaw(yw0)
        track(cmd_hover, 100, clamp=0.05)
        for _ in range(10):
            step_n(1)
        ep2, _ = kpose()
        print(f"  [recenter] re-aim err={float(torch.linalg.norm(ep2[0,:3]-cmd_hover[0,:3]))*1000:.1f}mm "
              f"palm=({float(ep2[0,0]):+.3f},{float(ep2[0,1]):+.3f}) bottle=({bx0c:.4f},{by0c:.4f})", flush=True)

    bl_names = ["palm"] + [f + "proximal" for f in ["ff", "mf", "rf", "lf"]]
    n_b = len(bl_names)
    force_sum = torch.zeros(n_b, device=DEV)
    contact_bodies = set()
    bottom = False
    first_force = None
    palm_fvec = None
    cmd = cmd_hover.clone()
    n_steps = 0

    def mcp_z():
        return float(torch.stack([presser.data.body_pos_w[0, mcp[f]].float() for f in ["ff", "mf", "rf", "lf"]]).mean(0)[2])

    # 自由瓶: 过顶多 seed 扫掠会把瓶撞离 home(probe 实测压空)。下探前瞬移回 home 归零并静置
    # (掌心已 hover 于名义 cap 位, 30mm 间隙不碰; 混形钳瓶路径 PANDAP 下瓶被爪钳住不需此步)。
    if not PANDAP:
        set_root_state(BX, BY, TAB)
        bx0c, by0c = BX, BY
        for _ in range(40):
            step_n(1)

    rec[0] = True  # 开录: 掌心已在 cap 上 hover, 从下探-到底-稳住-抬回-开爪
    print("[descend] palm origin along -z onto cap ...", flush=True)
    while True:
        cmd[:, 2] = float(cmd[0, 2]) - args_cli.descend_vel
        zc = float(cmd[0, 2])
        track(cmd, 1, clamp=0.04)
        n_steps += 1
        nz = float(bottle.data.joint_pos[0, 0])
        fw = sensor.data.force_matrix_w
        if fw is not None and fw.numel():
            fm = torch.norm(fw[0, :, :, :], dim=-1)
            hit = [b for b in range(n_b) if (fm[b] > 0.05).any()]
            for b in hit:
                contact_bodies.add(b)
            if hit and first_force is None:
                first_force = (n_steps, zc)
                print(f"  >> first contact step {n_steps}: z_cmd={zc:.4f} mcp_z={mcp_z():.4f} "
                      f"nozzle={nz:+.5f} bodies={[bl_names[b] for b in hit]}", flush=True)
            force_sum = torch.maximum(force_sum, fm.sum(-1))
        palm_z = float(presser.data.body_pos_w[0, palm_i, 2])
        if nz <= args_cli.nozzle_bottom:
            bottom = True
            nz_bottom = nz
            trig_palm = float(force_sum[0])
            rr = bottle.data.root_pos_w[0]
            slip = (float(rr[0]) - bx0c, float(rr[1]) - by0c)
            if fw is not None and fw.numel():
                palm_fvec = fw[0, 0].sum(0).clone()
            print(f"  >> nozzle bottom step {n_steps}: z_cmd={zc:.4f} palm_z={palm_z:.4f} "
                  f"nozzle={nz:+.5f} drift={drift_mm():.2f}mm bodies={[bl_names[b] for b in sorted(contact_bodies)]} "
                  f"palm_force_at_trigger={trig_palm:.2f}N slip=({slip[0]*1000:+.1f},{slip[1]*1000:+.1f})mm", flush=True)
            break
        if zc < CAP_TOP - 0.050 and nz > -0.0002:
            print(f"  !! descended w/o nozzle stroke (nozzle={nz:+.5f} palm_z={palm_z:.4f} "
                  f"contact={[bl_names[b] for b in sorted(contact_bodies)]}); abort", flush=True)
            # 定位最低碍物: 低于 palm origin 的 presser 部件 + 近场 panda/bottle
            pnb = list(presser.body_names)
            bl = list(bottle.body_names)
            low = []
            for i, n in enumerate(pnb):
                p = presser.data.body_pos_w[0, i]
                if 0.40 < float(p[2]) < palm_z - 0.005:
                    low.append((float(p[2]), n.split("/")[-1], float(p[0]), float(p[1])))
            low.sort()
            for zz, nm, xx, yy in low[:8]:
                print(f"    LOWPRESS {nm:18s} z={zz:.3f} xy=({xx:+.3f},{yy:+.3f})", flush=True)
            if PANDAP:
                pdb = list(panda.body_names)
                for i, n in enumerate(pdb):
                    p = panda.data.body_pos_w[0, i]
                    if float(p[2]) > 0.45:
                        print(f"    PANDA {n.split('/')[-1]:16s} ({float(p[0]):+.3f},{float(p[1]):+.3f},{float(p[2]):+.3f})", flush=True)
            for i, n in enumerate(bl):
                p = bottle.data.body_pos_w[0, i]
                print(f"    BOTTLE {n.split('/')[-1]:10s} ({float(p[0]):+.3f},{float(p[1]):+.3f},{float(p[2]):+.3f})", flush=True)
            break
        if n_steps > 5000:
            print("  !! too many descend steps"); break
        if n_steps % 300 == 0:
            print(f"  descend {n_steps}: z_cmd={zc:.4f} palm_z={palm_z:.4f} nozzle={nz:+.5f} "
                  f"drift={drift_mm():.2f}mm", flush=True)

    # 稳住到底
    for _ in range(100):
        track(cmd, 1, clamp=0.03)
    nz_held = float(bottle.data.joint_pos[0, 0])
    drift_press = drift_mm()
    palm_norm_force = float(force_sum[0])
    spring_reaction = -NOZZLE_K * nz_held  # q<0 -> 向上的回位弹簧反力(触发所需对抗的力)
    print(f"[held] nozzle={nz_held:+.5f} drift={drift_press:.2f}mm bodies={[bl_names[b] for b in sorted(contact_bodies)]} "
          f"max_force=({', '.join(f'{bl_names[b]}:{float(force_sum[b]):.2f}N' for b in range(n_b))})", flush=True)
    qb = presser.data.body_quat_w[0, palm_i].float()
    cos_now = float(quat_apply(qb, n_local) @ Wv)
    rr = bottle.data.root_pos_w[0]
    fv = palm_fvec
    fv_txt = f"({float(fv[0]):+.2f},{float(fv[1]):+.2f},{float(fv[2]):+.2f})N" if fv is not None else "n/a"
    print(f"[diag_bottom] palm_force_vec={fv_txt} cos_down={cos_now:.3f} "
          f"palm_xy=({float(presser.data.body_pos_w[0,palm_i,0]):+.3f},{float(presser.data.body_pos_w[0,palm_i,1]):+.3f}) "
          f"bottle_xy=({float(rr[0]):+.3f},{float(rr[1]):+.3f})", flush=True)
    for i, n in enumerate(list(presser.body_names)):
        p = presser.data.body_pos_w[0, i]
        if 0.42 < float(p[2]) < float(presser.data.body_pos_w[0, palm_i, 2]) and abs(float(p[0]) - float(rr[0])) < 0.12 and abs(float(p[1]) - float(rr[1])) < 0.12:
            print(f"    LOWNEAR {n.split('/')[-1]:16s} ({float(p[0]):+.3f},{float(p[1]):+.3f},{float(p[2]):+.3f})", flush=True)
    print(f"TRIGGER nozzle_held={nz_held:+.5f} ({-nz_held*1000:.2f}/{NOZZLE_TRAVEL*1000:.0f} mm travel) "
          f"palm_force={palm_norm_force:.2f}N spring_reaction={spring_reaction:.2f}N", flush=True)

    # 抬回 hover -> 看 nozzle 回弹
    for _ in range(250):
        cmd[:, 2] = float(cmd[0, 2]) + args_cli.descend_vel * 3
        track(cmd, 1, clamp=0.05)
    for _ in range(120):
        step_n(1)
    nz_after = float(bottle.data.joint_pos[0, 0])
    rebound_mm = float((nz_after - 0.0)) * 1000
    print(f"[release] nozzle_after={nz_after:+.5f} rebound={rebound_mm:+.3f}mm palm_z={float(presser.data.body_pos_w[0, palm_i, 2]):.4f}", flush=True)

    palm_contact = 0 in contact_bodies or any(b in contact_bodies for b in range(1, n_b))
    has_force = palm_norm_force > 0.5
    rebound_ok = abs(nz_after) < 0.0008
    drift_ok = drift_press < 2.0
    usable = bottom and palm_contact and has_force and rebound_ok and drift_ok
    print(f"PRESS_BOTTOM={bottom} palm/knuckle_contact={palm_contact} palm_force>{palm_norm_force:.2f}N "
          f"drift_press={drift_press:.2f}mm nozzle_held={nz_held:+.5f} rebound={rebound_mm:+.3f}mm "
          f"rebound_ok={rebound_ok} drift_ok={drift_ok}")
    print(f"tactile_verdict={'USABLE' if usable else 'NOT_USABLE'} "
          f"(palm presses cap -> nozzle reach bottom -> spring returns on release -> bottle stays)")

    # 阶段 ⑤: 松开长爪撤回
    if pbase is not None:
        for i, jv in enumerate(torch.linspace(0.0, JAW_U, 200)):
            qc = pbase.clone(); qc[0, jaw_t] = float(jv)
            panda.set_joint_position_target(qc)
            step_n(1)
        for _ in range(60):
            step_n(1)
        print(f"[jaw_open] q_read={float(panda.data.joint_pos[0, jaw_t].mean()):+.4f} drift={drift_mm():.2f}mm", flush=True)

    rec[0] = False
    print("DONE", flush=True)

    if args_cli.video and vid_frames:
        import cv2
        H, W = vid_frames[0].shape[:2]
        fps = int(round(1.0 / dt / vid_every))
        os.makedirs(os.path.dirname(os.path.abspath(args_cli.video)) or ".", exist_ok=True)
        writer = cv2.VideoWriter(args_cli.video, cv2.VideoWriter_fourcc(*"mp4v"), fps, (W, H))
        for fr in vid_frames:
            writer.write(fr)
        writer.release()
        print(f"\n[video] {len(vid_frames)} frames {W}x{H} @ {fps}fps -> {args_cli.video}", flush=True)
        try:
            cv2.imwrite(os.path.splitext(args_cli.video)[0] + "_last.png", vid_frames[-1])
        except Exception as e:
            print("png skip", e)
    simulation_app.close()


if __name__ == "__main__":
    main()
