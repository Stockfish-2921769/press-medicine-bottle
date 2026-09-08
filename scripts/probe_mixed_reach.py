"""混形可达性扫描(M1 gate): Shadow 掌心水平向下压 cap 是否可达 + Panda 平爪侧向够瓶身。

设计要点:
- "掌心平面"用 4 根长指 MCP 近节体(robot0_{ff,mf,rf,lf}proximal)构成的平面近似:
  法线(叉积)+ 掌心中心(MCP 平均)都在 robot0_palm 局部系量一次;目标=把该平面摆成
  世界水平、法线=-z、MCP 质心到 cap 顶上方,再约束手指前向(f_local,由 mf proximal->middle)
  沿世界某水平方向(roll),构造 palm 体 6D 目标,arm-only DLS IK 追踪 robot0_palm。
- 多 seed DLS(项目铁律:残差大≠不可达)。基座用"平移/绕 z 旋转"目标变换等效,机器人本体
  留在原点 yaw0,避免重摆基座;按基座 ring 扫 cap 周围,选半径+方位,rz 使机器人前向朝 cap。
- 判据: MCP 质心水平/垂直残差与 palm 法线·(-z) > cos(8°)。
- Panda 平爪 gate: 从另一侧 base(默认 yaw 朝瓶) IK panda 臂把 EE(panda_hand/link8)送到
  瓶身中段 (bx,by,z0.50) 附近、爪开合轴水平垂直于瓶轴;IK 残差<20mm 记 REACH。

用法(headless):
  ./isaaclab.sh -p probe_mixed_reach.py --headless [--mount default|rx90|ry-90|rx-90]
      [--bx --by --top --table --ns --step --rad "0.5,0.62" --astep 60]
输出逐 cell 行 + 每基座 best + 最终 FEAS/REACH 汇总。日志落 logs/probe_mixed_reach.log。
"""

import argparse
import math
import re
import time

import torch

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--bx", type=float, default=-0.545)
parser.add_argument("--by", type=float, default=-0.026)
parser.add_argument("--table", type=float, default=0.44)
parser.add_argument("--cap_z_local", type=float, default=0.145, help="cap 顶在瓶系 local z")
parser.add_argument("--hover_mm", type=float, default=6.0, help="MCP 质心 hover 高于 cap 顶(mm)")
parser.add_argument("--mount", type=str, default="default")
parser.add_argument("--rad", type=str, default="0.5,0.62", help="基座 ring 半径列表(m)")
parser.add_argument("--astep", type=float, default=60.0, help="ring 方位角步长(度)")
parser.add_argument("--yaw_off", type=str, default="0", help="每 cell 手指 yaw 相对朝向瓶的偏移(度, 逗号列表)")
parser.add_argument("--ns", type=int, default=10, help="每 cell seed 数")
parser.add_argument("--step", type=int, default=220)
parser.add_argument("--max_err_mm", type=float, default=15.0, help="FEAS palm 质心残差上限")
parser.add_argument("--cos_tol", type=float, default=0.99, help="掌面法线·(-z) 下限")
parser.add_argument("--panda", action="store_true", help="同时跑 Panda 平爪侧向 gate")
parser.add_argument("--posonly", action="store_true", help="只测 palm 原点位置可达(不要求掌面向下)")
parser.add_argument("--press_point", type=str, default="origin", choices=["origin", "centroid"],
                    help="掌心接触点: origin=palm 原点(脚跟) / centroid=4 MCP 质心(掌面垫)")
parser.add_argument("--scale", type=float, default=1.0, help="presser 整机等比缩放(spawn.scale), 1.0=不缩")
parser.add_argument("--zlist", type=str, default="0", help="基座世界 z(垫高)候选列表(逗号, m); 扫每个 z 的 ring")
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.sim import SimulationCfg, SimulationContext
from _kuka_shadow_cfg import KUKA_SHADOW_CFG

DEV = "cuda:0"
CAP = torch.tensor([args_cli.bx, args_cli.by, args_cli.table + args_cli.cap_z_local],
                   device=DEV, dtype=torch.float32)
HOVER = args_cli.hover_mm / 1000.0


def mat_to_quat(R):
    R = R.float()
    tr = R.trace()
    if tr > 0:
        S = torch.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = torch.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = torch.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = torch.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    q = torch.stack([w, x, y, z])
    return q / torch.linalg.norm(q)


def quat_to_R(q):
    q = q.float() / torch.linalg.norm(q)
    w, x, y, z = q
    return torch.stack([
        torch.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)]),
        torch.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)]),
        torch.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]),
    ])


def build_q(cfg, jn, dev):
    q = torch.zeros(1, len(jn), device=dev)
    for pattern, val in cfg.init_state.joint_pos.items():
        rx = re.compile("^" + pattern.replace("(", "(?:") .replace(")", ")") + "$")
        for i, n in enumerate(jn):
            if rx.match(n):
                q[0, i] = val
    return q


class RobotIK:
    """单 robot 的 DLS IK 工具(arm joints only, body 作 EE)。robot 固定在原点。"""

    def __init__(self, rob, sim, ee_i, arm_ids, dev=DEV, clamp=0.09):
        self.rob = rob
        self.sim = sim
        self.ee_i = ee_i
        self.arm = arm_ids
        self.arm_t = torch.tensor(arm_ids, device=dev)
        self.dev = dev
        self.clamp = clamp
        self.ik = DifferentialIKController(
            DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls"),
            num_envs=1, device=dev)

    def body_pose(self):
        return (self.rob.data.body_pos_w[:, self.ee_i].float(),
                self.rob.data.body_quat_w[:, self.ee_i].float())

    def track(self, cmd, n):
        rob = self.rob
        for _ in range(n):
            ep, eq = self.body_pose()
            jac = rob.root_physx_view.get_jacobians()[:, self.ee_i, :, self.arm_t]
            jp = rob.data.joint_pos[:, self.arm_t].float().clone()
            self.ik.set_command(cmd)
            jp_des = self.ik.compute(ep, eq, jac, jp)
            jp_des = torch.clamp(jp_des, min=jp - self.clamp, max=jp + self.clamp)
            rob.set_joint_position_target(jp_des, self.arm)
            rob.write_data_to_sim()
            self.sim.step()
            rob.update(1 / 60.0)

    def best_over_seeds(self, qnom, seeds, cmd, n):
        rob = self.rob
        best = (1e9, None)
        for s in seeds:
            rob.write_joint_state_to_sim(s, torch.zeros_like(s))
            rob.set_joint_position_target(s)
            for _ in range(3):
                rob.write_data_to_sim()
                self.sim.step()
                rob.update(1 / 60.0)
            self.track(cmd, n)
            ep, eq = self.body_pose()
            err = float(torch.linalg.norm(ep[0, :3] - cmd[0, :3]))
            if err < best[0]:
                best = (err, rob.data.joint_pos[:, self.arm_t].float().clone())
        rob.write_joint_state_to_sim(qnom, torch.zeros_like(qnom))
        return best


def rz_quat(angle_rad):
    h = angle_rad / 2.0
    return torch.tensor([math.cos(h), 0.0, 0.0, math.sin(h)], device=DEV, dtype=torch.float32)


def quat_apply(q, v):
    """q(wxyz) 旋转 v。"""
    qw, qx, qy, qz = q
    t = 2 * torch.linalg.cross(q[1:], v)
    return v + qw * t + torch.linalg.cross(q[1:], t)


def main():
    dt = 1 / 60.0
    sim = SimulationContext(SimulationCfg(dt=dt, device=DEV, gravity=(0.0, 0.0, 0.0)))

    # ---- Shadow presser: mount 变体 ----
    mount_file = {
        "default": "kuka_shadow.usda",
        "rx90": "kuka_shadow_rx90.usda",
        "rx-90": "kuka_shadow_rx-90.usda",
        "ry-90": "kuka_shadow_ry-90.usda",
    }[args_cli.mount]
    import os
    _here = os.path.dirname(os.path.abspath(__file__))
    usd = os.path.join(_here, "..", "assets", mount_file)
    cfg = KUKA_SHADOW_CFG.replace(prim_path="/World/Presser")
    cfg.spawn.usd_path = usd
    if args_cli.scale != 1.0:
        cfg.spawn.scale = (args_cli.scale, args_cli.scale, args_cli.scale)
    print(f"[scale] s={args_cli.scale} spawn.scale={cfg.spawn.scale}", flush=True)
    presser = Articulation(cfg)
    sim.reset()
    presser.reset()
    jn = list(presser.joint_names)
    names = list(presser.body_names)
    palm_i = names.index("robot0_palm")
    wrist_i = names.index("robot0_wrist")
    arm_ids = [i for i, n in enumerate(jn) if n.startswith("iiwa7")]
    mcp = {f: names.index("robot0_%sproximal" % f) for f in ["ff", "mf", "rf", "lf"]}
    mid = {f: names.index("robot0_%smiddle" % f) for f in ["ff", "mf", "rf", "lf"]}
    thb = names.index("robot0_thproximal")
    qnom = build_q(cfg, jn, DEV)
    presser.write_joint_state_to_sim(qnom, torch.zeros_like(qnom))
    presser.set_joint_position_target(qnom)
    for _ in range(20):
        presser.write_data_to_sim()
        sim.step()
        presser.update(dt)

    def world_pts(body_idx):
        return presser.data.body_pos_w[0, body_idx].float()

    # 名义位量掌平面 frame
    p0 = world_pts(palm_i)
    q0 = presser.data.body_quat_w[0, palm_i].float()
    R0 = quat_to_R(q0)
    M = torch.stack([world_pts(mcp[f]) for f in ["ff", "mf", "rf", "lf"]])  # 4x3
    centroid0 = M.mean(0)
    # 平面法线: 对角叉积(取与 thumb 侧一致? 先用两对角, 符号由 thumb 锚定)
    n_cand = torch.linalg.cross(M[1] - M[0], M[2] - M[3])  # (mf-ff) x (rf-lf)
    if torch.linalg.norm(n_cand) < 1e-6:
        n_cand = torch.linalg.cross(M[2] - M[0], M[3] - M[1])
    n_w = n_cand / torch.linalg.norm(n_cand)
    # thumb 锚定 inner 侧: palm 中心 -> thumb proximal 的平面外残差
    thv = world_pts(thb) - centroid0
    inner_hint = thv - (thv @ n_w) * n_w
    if torch.linalg.norm(inner_hint) > 1e-3 and float(inner_hint @ n_w) < 0:
        n_w = -n_w
    # 拇指提示定的是手背外法向(按它对齐 -z 视频判读掌心朝上), 取反 -> 掌心外法向; 对齐 -z = 掌心朝下压
    n_w = -n_w
    # 前向 = mf proximal -> mf middle(指向指尖), 投影到平面内
    fwd = world_pts(mid["mf"]) - world_pts(mcp["mf"])
    fwd = fwd - (fwd @ n_w) * n_w
    fwd = fwd / torch.linalg.norm(fwd)
    n_local = R0.t().matmul(n_w)
    f_local = R0.t().matmul(fwd)
    c_local = R0.t().matmul(centroid0 - p0)  # MCP 质心相对 palm 原点
    print(f"[frame] mount={args_cli.mount} palm0=({float(p0[0]):+.3f},{float(p0[1]):+.3f},{float(p0[2]):+.3f}) "
          f"n_w=({n_w[0]:+.3f},{n_w[1]:+.3f},{n_w[2]:+.3f}) fwd_w=({fwd[0]:+.3f},{fwd[1]:+.3f},{fwd[2]:+.3f}) "
          f"n_w.dot(-z)={float(n_w @ torch.tensor([0.,0.,-1.],device=DEV)):+.3f}", flush=True)
    print(f"  c_local=({float(c_local[0]):+.3f},{float(c_local[1]):+.3f},{float(c_local[2]):+.3f})", flush=True)

    seeds = []
    base = qnom.clone()
    seeds.append(base.clone())
    for j1 in [-1.6, -0.8, 0.8, 1.6]:
        s = qnom.clone()
        s[0, arm_ids[0]] = j1
        seeds.append(s)
    s = qnom.clone(); s[0, arm_ids[3]] = 0.6; seeds.append(s)   # j4 elbow
    s = qnom.clone(); s[0, arm_ids[1]] = 0.9; seeds.append(s)   # j2
    # 腕部: 决定 palm 姿态的关节必须先多样 seed
    for wj in [4, 5, 6]:  # arm_ids[j5,j6,j7] 索引 4,5,6
        for v in [0.9, -0.9]:
            s = qnom.clone()
            s[0, arm_ids[wj]] = v
            seeds.append(s)
    seeds = seeds[: args_cli.ns]

    c_world0 = centroid0 - p0  # 名义位 palm 原点 -> MCP 质心 的世界偏移(定常)

    def make_target(cap_goal, ex_w):
        """目标 palm 体姿态: n_local->(cap_goal - ... 法线 -z); f_local->ex_w. 返回 (pos_arm, quat_arm)."""
        if args_cli.posonly:
            return cap_goal - c_world0, q0  # 保持名义姿态, 只试 palm 原点能否到位
        W = torch.tensor([0.0, 0.0, -1.0], device=DEV)  # 掌面内法线向下
        U = ex_w - (ex_w @ W) * W
        U = U / torch.linalg.norm(U)
        V = torch.linalg.cross(W, U)
        u = f_local / torch.linalg.norm(f_local)
        w = n_local / torch.linalg.norm(n_local)
        v = torch.linalg.cross(w, u)
        Rcol_local = torch.stack([u, v, w], dim=1)   # local cols in local frame
        R_worldcols = torch.stack([U, V, W], dim=1)  # world cols
        # R maps local->world: world col_i = image of local basis_i. R = R_worldcols * Rcol_local^T
        R = R_worldcols.matmul(Rcol_local.t())
        q = mat_to_quat(R)
        # 接触点选择: 'centroid' = 4 MCP 质心(掌面中心/战术垫近似); 'origin' = palm 原点(腕侧脚跟)。
        if args_cli.press_point == "centroid":
            pos = cap_goal - R.matmul(c_local)  # palm 原点放到能让 MCP 质心落在 cap 上的位置
        else:
            pos = cap_goal.clone()
        return pos, q

    yaw_offs = [float(x) for x in args_cli.yaw_off.split(",")]
    rings = [float(x) for x in args_cli.rad.split(",")]
    print(f"=== mixed reach cap=({CAP[0]:+.2f},{CAP[1]:+.2f},z{float(CAP[2]):.3f}) mount={args_cli.mount} "
          f"hover={args_cli.hover_mm}mm 掌面向下, ns={args_cli.ns} yaw_off={yaw_offs} ===", flush=True)
    best_all = []
    _t0 = time.time()
    cap_goal = CAP.clone()
    cap_goal[2] = float(CAP[2]) + HOVER

    def set_root(bx, by, bz, phi):
        """把 presser 基座放到世界 (bx,by,bz), 绕 z 转 phi(使 robot 局部 x 大致朝 cap)。"""
        pose = torch.cat([torch.tensor([bx, by, bz], device=DEV, dtype=torch.float32), rz_quat(phi)])
        presser.write_root_pose_to_sim(pose.unsqueeze(0))
        for _ in range(3):
            presser.write_data_to_sim(); sim.step(); presser.update(dt)

    def apply_best_arm(q_best):
        presser.write_joint_state_to_sim(qnom, torch.zeros_like(qnom))
        presser.set_joint_position_target(qnom)
        for _ in range(3):
            presser.write_data_to_sim(); sim.step(); presser.update(dt)
        full = torch.zeros(1, presser.num_joints, device=DEV)
        full[:, torch.tensor(arm_ids, device=DEV)] = q_best
        presser.write_joint_state_to_sim(full, torch.zeros_like(full))
        presser.set_joint_position_target(full)
        for _ in range(6):
            presser.write_data_to_sim(); sim.step(); presser.update(dt)

    for bz in [float(x) for x in args_cli.zlist.split(",")]:
        for Rdist in rings:
            n_ang = max(1, int(round(360.0 / max(args_cli.astep, 1e-3))))
            for k in range(n_ang):
                ang = math.radians(k * args_cli.astep)
                bx = CAP[0] + Rdist * math.cos(ang)
                by = CAP[1] + Rdist * math.sin(ang)
                dx, dy = CAP[0] - bx, CAP[1] - by
                dist = math.hypot(dx, dy)
                if dist < 1e-4:
                    continue
                phi = math.atan2(dy, dx)  # robot 基座朝 cap 的方位
                for yw in yaw_offs:
                    gy = math.radians(yw)
                    ex_w = torch.tensor(
                        [(math.cos(gy) * dx - math.sin(gy) * dy) / dist,
                         (math.sin(gy) * dx + math.cos(gy) * dy) / dist, 0.0], device=DEV)
                    pos_a, q_a = make_target(cap_goal, ex_w)
                    cmd = torch.cat([pos_a.unsqueeze(0), q_a.unsqueeze(0)], dim=1)
                    set_root(bx, by, bz, phi)
                    rk = RobotIK(presser, sim, palm_i, arm_ids)
                    best = rk.best_over_seeds(qnom, seeds, cmd, args_cli.step)
                    err_m, q_best = best
                    apply_best_arm(q_best)
                    # 量指标: 掌面朝下 = palm 局部 n_local 映到 -z; 残差直接世界坐标
                    qp = presser.data.body_quat_w[0, palm_i].float()
                    n_ach = quat_apply(qp, n_local)
                    cos_dn = float(n_ach @ torch.tensor([0.0, 0.0, -1.0], device=DEV))
                    Mp = torch.stack([world_pts(mcp[f]) for f in ["ff", "mf", "rf", "lf"]]).mean(0)
                    err_cent = math.hypot(float(Mp[0]) - float(CAP[0]), float(Mp[1]) - float(CAP[1])) * 1000
                    err_z = (float(Mp[2]) - float(CAP[2]) - HOVER) * 1000
                    ww = world_pts(wrist_i)
                    wr_clear = math.hypot(float(ww[0]) - float(CAP[0]), float(ww[1]) - float(CAP[1])) * 1000
                    feas = err_m * 1000 < args_cli.max_err_mm and (args_cli.posonly or cos_dn > args_cli.cos_tol)
                    best_all.append((err_m * 1000, cos_dn, Rdist, math.degrees(ang), yw, bz, feas, q_best.clone()))
                    print(f"bz={bz:+.3f} r={Rdist:.2f} ang={int(math.degrees(ang)):3d} yaw={yw:+.0f} | "
                          f"palm_err={err_m * 1000:5.0f}mm cent_xy={err_cent:5.1f} z={err_z:+5.1f} "
                          f"cos_down={cos_dn:+.3f} wrist_clear={wr_clear:5.0f}mm w_z={float(ww[2]):.3f} "
                          f"{'FEAS' if feas else '....'} "
                          f"base=({bx:+.2f},{by:+.2f}) [{time.time()-_t0:5.0f}s]", flush=True)
    best_all.sort()
    feas = [b for b in best_all if b[6]]
    print("\nSUMMARY_PRESSER mount=%s scale=%s" % (args_cli.mount, args_cli.scale), flush=True)
    if feas:
        for e, c, R, a, yw, bz, f, qb in feas[:6]:
            print(f"  FEAS err={e:.0f}mm cos={c:+.3f} r={R:.2f} ang={a:.0f}deg yaw={yw:+.0f} bz={bz:+.3f}", flush=True)
        best_f = feas[0]
        print(f"VERDICT_SCALE s={args_cli.scale} FEAS err={best_f[0]:.0f}mm cos={best_f[1]:+.3f} "
              f"r={best_f[2]:.2f} ang={best_f[3]:.0f}deg yaw={best_f[4]:+.0f} bz={best_f[5]:+.3f}", flush=True)
    else:
        print("  NO_PRESSER_FEAS  (best cells:)", flush=True)
        for e, c, R, a, yw, bz, f, qb in best_all[:6]:
            print(f"    err={e:.0f}mm cos={c:+.3f} r={R:.2f} ang={a:.0f}deg yaw={yw:+.0f} bz={bz:+.3f}", flush=True)
        print(f"VERDICT_SCALE s={args_cli.scale} NO_FEAS", flush=True)

    # ---- Panda 平爪 gate(可选) ----
    if args_cli.panda:
        from isaaclab_assets import FRANKA_PANDA_CFG
        pcfg = FRANKA_PANDA_CFG.replace(prim_path="/World/ClampFar")
        pcfg.spawn.rigid_props.disable_gravity = True
        panda = Articulation(pcfg)
        sim.reset(); panda.reset()
        # 放远避免与 presser 重叠
        # (Articulation 位置由 cfg.init_state.pos 控制; 已远置? 未设则原点重叠, 我们 park 远)
        jnp = list(panda.joint_names)
        namesp = list(panda.body_names)
        arm_p = [i for i, n in enumerate(jnp) if re.fullmatch(r"panda_joint[0-9]+", n)]
        ee_name = next((b for b in ["panda_hand", "panda_link8"] if b in namesp), namesp[-1])
        ee_p = namesp.index(ee_name)
        qp = build_q(pcfg, jnp, DEV)
        panda.write_joint_state_to_sim(qp, torch.zeros_like(qp))
        panda.set_joint_position_target(qp)
        for _ in range(15):
            panda.write_data_to_sim(); sim.step(); panda.update(dt)
        # target: EE 中心到瓶身边上一侧, 爪开合轴水平垂直瓶轴
        # 简化 kinematic: EE 到 (bx,by, table+0.06) 上方附近的瓶身中点取 z=table+0.06+? body 中段 0.50
        zg = args_cli.table + 0.06  # 瓶身中段 z 目标(世界)
        # 用与 presser 互补的一侧: +y 侧
        py0, px0 = CAP[0] + 0.0, CAP[1] + 0.30
        # panda 从 +y 侧来, EE z 轴指向 -y(朝瓶)  => 开合轴水平 = ±x
        tgt_p = torch.tensor([[px0, py0, zg]], device=DEV, dtype=torch.float32)
        # 姿态: 让 EE 局部 z 指向瓶(-y)
        from isaaclab.utils import math as m_math
        ez = torch.zeros((1, 3), device=DEV); ez[:, 2] = 1.0
        qcur = panda.data.body_quat_w[:, ee_p].float()
        v = m_math.quat_apply(qcur, ez)
        wv = torch.tensor([[0.0, -1.0, 0.0]], device=DEV)
        cr = torch.linalg.cross(v, wv); c = torch.clamp((v * wv).sum(-1), -1, 1)
        qrot = m_math.quat_from_angle_axis(torch.acos(c), cr)
        qt = m_math.quat_mul(qrot, qcur); qt = qt / torch.linalg.norm(qt)
        cmd_p = torch.cat([tgt_p, qt], dim=1)
        rkp = RobotIK(panda, ee_p, arm_p, clamp=0.1)
        best_p = rkp.best_over_seeds(qp, [qp.clone()], cmd_p, 300)
        # 恢复
        full = torch.zeros(1, panda.num_joints, device=DEV)
        full[:, torch.tensor(arm_p, device=DEV)] = best_p[1]
        panda.write_joint_state_to_sim(full, torch.zeros_like(full))
        panda.set_joint_position_target(full)
        for _ in range(6):
            panda.write_data_to_sim(); sim.step(); panda.update(dt)
        ep, eq = rkp.body_pose()
        print(f"PANDA_EE body={ee_name} target=({px0:+.3f},{py0:+.3f},{zg:+.3f}) err={best_p[0] * 1000:.0f}mm "
              f"ee=({float(ep[0,0]):+.3f},{float(ep[0,1]):+.3f},{float(ep[0,2]):+.3f}) "
              f"{'REACH' if best_p[0] * 1000 < 20 else 'NOREACH'}")

    print("DONE")
    simulation_app.close()


if __name__ == "__main__":
    main()
