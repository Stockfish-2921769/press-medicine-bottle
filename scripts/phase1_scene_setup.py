"""Phase 1 场景装配 + 稳定性检查：工作台(kinematic) + 药瓶(弹簧喷嘴,固定底座) + Kuka/Allegro。

检查:
  1) 三类实体都正确加载(数量/关节/刚体/碰撞)
  2) 药瓶底座不漂移(固定关节 anchor 生效)、喷嘴在弹簧下稳定于 ~0
  3) 机器人底座是否固定(is_fixed_base);若自由且 disable_gravity,按压会顶起自身,后续需接地
"""

import argparse
import os

import torch

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--table_top", type=float, default=0.44, help="工作台台面高度 z")
parser.add_argument("--bottle_x", type=float, default=-0.5, help="药瓶 x")
parser.add_argument("--bottle_y", type=float, default=0.0, help="药瓶 y")
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab_assets import KUKA_ALLEGRO_CFG

BOTTLE_USD = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets", "medicine_bottle.usda")


def main():
    dt = 1 / 120.0
    sim = SimulationContext(
        SimulationCfg(dt=dt, device="cuda:0", gravity=(0.0, 0.0, -9.81), use_fabric=False)
    )

    table_top = args_cli.table_top
    bx, by = args_cli.bottle_x, args_cli.bottle_y

    # 地面
    gnd_cfg = sim_utils.GroundPlaneCfg()
    gnd_cfg.func("/World/GroundPlane", gnd_cfg)
    light_cfg = sim_utils.DomeLightCfg(intensity=800.0)
    light_cfg.func("/World/Light", light_cfg)

    # 工作台（kinematic 刚体）
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

    # 药瓶（底座固定到世界；顶在台面 table_top）
    bottle = Articulation(
        ArticulationCfg(
            prim_path="/World/Bottle",
            spawn=sim_utils.UsdFileCfg(usd_path=BOTTLE_USD),
            init_state=ArticulationCfg.InitialStateCfg(pos=(bx, by, table_top)),
            actuators={},
        )
    )

    # 机械臂 + 灵巧手
    robot = Articulation(KUKA_ALLEGRO_CFG.replace(prim_path="/World/Robot"))

    sim.reset()
    table.reset()
    bottle.reset()
    robot.reset()

    print("\n================== 场景加载信息 ==================")
    print(f"[table] num_bodies={table.num_bodies}")
    print(f"[bottle] num_joints={bottle.num_joints} joint_names={bottle.joint_names} body_names={bottle.body_names}")
    print(f"[robot] num_joints={robot.num_joints} is_fixed_base={robot.is_fixed_base}")
    print(f"[robot] arm joints = {[n for n in robot.joint_names if n.startswith('iiwa7')]}")

    # ---- 应用名义 arm+hand joint pose（先不按，只验证静止稳定）----
    jn = list(robot.joint_names)
    qpos = torch.zeros(1, robot.num_joints, device=robot.device)
    for pattern, val in KUKA_ALLEGRO_CFG.init_state.joint_pos.items():
        import re as _re
        rx = _re.compile("^" + pattern.replace("(", "(?:") .replace(")", ")") + "$")
        for i, n in enumerate(jn):
            if rx.match(n):
                qpos[0, i] = val
    robot.set_joint_position_target(qpos)
    robot.write_data_to_sim()

    bn = list(bottle.joint_names)
    nozzle_dof = bn.index("nozzle_joint")
    bb = list(bottle.body_names)
    base_bi = bb.index("base")

    table.write_data_to_sim()
    bottle.write_data_to_sim()
    robot.write_data_to_sim()

    print("\n================== 步进稳定性 ==================")
    for i in range(400):
        sim.step()
        table.update(dt)
        bottle.update(dt)
        robot.update(dt)
        if i < 60:
            # 前 60 步持续写目标让臂到名义位形
            robot.write_data_to_sim()
            bottle.write_data_to_sim()
            table.write_data_to_sim()
        if i in [59, 100, 200, 399]:
            rp = robot.data.root_pos_w[0]
            bp = bottle.data.body_pos_w[0, base_bi]
            np_ = float(bottle.data.joint_pos[0, nozzle_dof])
            print(
                f"  step {i+1:4d}: robot_root=({float(rp[0]):+.3f},{float(rp[1]):+.3f},{float(rp[2]):+.3f})"
                f"  bottle_base=({float(bp[0]):+.3f},{float(bp[1]):+.3f},{float(bp[2]):+.3f})"
                f"  nozzle_pos={np_:+.5f}"
            )

    print("\nDONE")
    simulation_app.close()


if __name__ == "__main__":
    main()
