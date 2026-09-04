"""Reach 探测:固定名义指形,只用 7 个 iiwa 关节做 DLS IK,测 index_link_3 在若干候选点
能否(a)仅定位到中心,(b)定位且转到竖直朝下(-Z)。一个进程顺序测,免多次启动。

输出:每候选点 pos_only / align 的收敛误差与 zaxis 指向,用于选定药瓶摆放位。
"""

import argparse

import torch

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.utils import math as m_math
from isaaclab_assets import KUKA_ALLEGRO_CFG

# 候选瓶位(表高 table_top,喷嘴顶 = table_top+0.145;EE 探测高度 z_ee 取喷嘴顶上方 ~0.04)
TABLE_TOP = 0.44
NOZZLE_TOP = TABLE_TOP + 0.145
Z_EE = NOZZLE_TOP + 0.045  # EE 原点离喷嘴顶 4.5cm,指尖(原点下方 ~1.5cm)仍在上方
CANDIDATES = [
    (-0.500, 0.000),
    (-0.500, -0.040),
    (-0.450, 0.000),
    (-0.450, -0.040),
    (-0.420, -0.020),
    (-0.400, 0.000),
    (-0.550, -0.040),
]


def main():
    dt = 1 / 120.0
    sim = SimulationContext(SimulationCfg(dt=dt, device="cuda:0", gravity=(0.0, 0.0, -9.81)))
    gnd_cfg = sim_utils.GroundPlaneCfg()
    gnd_cfg.func("/World/GroundPlane", gnd_cfg)
    robot = Articulation(KUKA_ALLEGRO_CFG.replace(prim_path="/World/Robot"))
    sim.reset()
    robot.reset()

    dev = "cuda:0"
    jn = list(robot.joint_names)
    qpos = torch.zeros(1, robot.num_joints, device=dev)
    for pattern, val in KUKA_ALLEGRO_CFG.init_state.joint_pos.items():
        import re as _re
        rx = _re.compile("^" + pattern.replace("(", "(?:") .replace(")", ")") + "$")
        for i, n in enumerate(jn):
            if rx.match(n):
                qpos[0, i] = val
    qvel0 = torch.zeros_like(qpos)
    robot.write_joint_state_to_sim(qpos, qvel0)
    robot.set_joint_position_target(qpos)
    for _ in range(10):
        robot.write_data_to_sim()
        sim.step()
        robot.update(dt)

    names = list(robot.body_names)
    tip_i = names.index("index_link_3")
    arm_ids = [i for i, n in enumerate(jn) if n.startswith("iiwa7")]
    arm_ids_t = torch.tensor(arm_ids, device=dev)

    ez = torch.zeros((1, 3), device=dev)
    ez[:, 2] = 1.0

    def body_pose(idx):
        return robot.data.body_pos_w[:, idx].float(), robot.data.body_quat_w[:, idx].float()

    def zaxis_w(idx):
        return m_math.quat_apply(robot.data.body_quat_w[:, idx].float(), ez)

    q_cur0 = robot.data.body_quat_w[:, tip_i].float()
    v0 = m_math.quat_apply(q_cur0, ez)
    w = -ez
    cross = torch.linalg.cross(v0, w)
    c = torch.clamp((v0 * w).sum(-1), -1, 1)
    q_tgt = m_math.quat_from_angle_axis(torch.acos(c), cross)
    q_tgt = m_math.quat_mul(q_tgt, q_cur0)
    q_tgt = q_tgt / torch.linalg.norm(q_tgt, dim=-1, keepdim=True)
    nominal_ee, _ = body_pose(tip_i)

    ik = DifferentialIKController(DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls"), num_envs=1, device=dev)

    def track(command, n=1):
        for _ in range(n):
            ee_pos, ee_quat = body_pose(tip_i)
            jac = robot.root_physx_view.get_jacobians()[:, tip_i, :, arm_ids_t]
            jp = robot.data.joint_pos[:, arm_ids_t].float().clone()
            ik.set_command(command)
            jp_des = torch.clamp(ik.compute(ee_pos, ee_quat, jac, jp), min=jp - 0.08, max=jp + 0.08)
            robot.set_joint_position_target(jp_des, arm_ids)
            robot.write_data_to_sim()
            sim.step()
            robot.update(dt)

    def reset_nominal():
        robot.write_joint_state_to_sim(qpos, qvel0)
        robot.set_joint_position_target(qpos)
        for _ in range(8):
            robot.write_data_to_sim()
            sim.step()
            robot.update(dt)

    def run_case(pos_tgt, orient, n_settle=700):
        """settle at pose command; returns (pos_err, ang_from_down)."""
        command = torch.cat([pos_tgt, orient], dim=1)
        # 直接发目标,one-shot IK 会平滑趋近
        track(command, n_settle)
        ee_pos, _ = body_pose(tip_i)
        za = zaxis_w(tip_i)[0]
        pos_err = float(torch.linalg.norm(ee_pos - pos_tgt))
        ang_from_down = float(torch.acos(torch.clamp(-za[2], -1, 1)))
        return pos_err, ang_from_down, ee_pos, za

    print(f"\nnominal EE = ({float(nominal_ee[0,0]):+.4f},{float(nominal_ee[0,1]):+.4f},{float(nominal_ee[0,2]):+.4f})")
    print(f"q_tgt -> straight down. Z_EE={Z_EE}  (bottle nozzle top at z={NOZZLE_TOP})")

    # 0) 先在当前名义位置原地测:能否仅转直(不移动)?
    print("\n[orient-in-place @ nominal]")
    reset_nominal()
    pe, ad, eep, za = run_case(nominal_ee, q_tgt)
    print(f"  pos_err={pe:.4f} ang_down={ad:.3f} rad  ee=({eep[0,0]:+.3f},{eep[0,1]:+.3f},{eep[0,2]:+.3f}) zaxis=({za[0]:+.2f},{za[1]:+.2f},{za[2]:+.2f})")

    print("\n[candidate sweep]  (pos_only: 定位不转 / align: 定位+竖直)")
    for (cx, cy) in CANDIDATES:
        pos_tgt = torch.tensor([[cx, cy, Z_EE]], device=dev, dtype=torch.float32)
        line = f"  ({cx:+.2f},{cy:+.2f}): "
        # pos_only -> keep current orientation (nominal)
        reset_nominal()
        pe, _, eep, za = run_case(pos_tgt, q_cur0.clone())
        line += f"pos_only err={pe:.4f}(ee=({eep[0,0]:+.3f},{eep[0,1]:+.3f},{eep[0,2]:+.3f}))"
        # align
        reset_nominal()
        pe, ad, eep, za = run_case(pos_tgt, q_tgt)
        line += (f" | align err={pe:.4f} ang_down={ad:.3f}"
                 f" ee=({eep[0,0]:+.3f},{eep[0,1]:+.3f},{eep[0,2]:+.3f}) zaxis=({za[0]:+.2f},{za[1]:+.2f},{za[2]:+.2f})")
        print(line)

    print("\nDONE")
    simulation_app.close()


if __name__ == "__main__":
    main()
