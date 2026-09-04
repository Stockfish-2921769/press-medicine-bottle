"""验证 medicine_bottle.usda 能被 Isaac Lab 解析为 articulation，且喷嘴可被压到底并靠弹簧回弹。"""

import argparse
import os

import torch

from isaaclab.app import AppLauncher

# launch app
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# after app
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.utils import configclass

USD = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets", "medicine_bottle.usda")


@configclass
class BottleCfg(ArticulationCfg):
    pass


def main():
    dt = 1 / 120.0
    sim_cfg = SimulationCfg(dt=dt, device="cuda:0", gravity=(0.0, 0.0, -9.81))
    sim = SimulationContext(sim_cfg)

    bottle = Articulation(
        ArticulationCfg(
            prim_path="/World/Bottle",
            spawn=sim_utils.UsdFileCfg(usd_path=USD),
            init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.25)),
            actuators={},
        )
    )

    sim.reset()
    bottle.reset()

    print("num_joints:", bottle.num_joints)
    print("joint_names:", bottle.joint_names)
    print("body_names:", bottle.body_names)
    print("num_dofs:", bottle.num_joints)

    joint_names = list(bottle.joint_names)
    assert "nozzle_joint" in joint_names, f"nozzle_joint missing in {joint_names}"
    nozzle_dof = joint_names.index("nozzle_joint")
    body_names = list(bottle.body_names)
    nozzle_body = body_names.index("nozzle") if "nozzle" in body_names else None

    def read_pos():
        return float(bottle.data.joint_pos[0, nozzle_dof].cpu())

    print("\n[1] initial nozzle joint pos:", round(read_pos(), 5))

    # ---- press: set nozzle to bottom of travel (-0.004) then let spring act ----
    print("\n[2] writing nozzle to -0.004 (fully pressed), then releasing (expect spring back to ~0):")
    pos = torch.tensor([-0.004], device=bottle.device).unsqueeze(0)
    vel = torch.zeros(1, 1, device=bottle.device)
    bottle.write_joint_state_to_sim(pos, vel, joint_ids=[nozzle_dof])
    # settle a few steps while held at bottom via re-writing each step to simulate finger holding
    for i in range(10):
        bottle.write_joint_state_to_sim(pos, vel, joint_ids=[nozzle_dof])
        sim.step()
        bottle.update(dt)
    print(f"   held at bottom: pos = {read_pos():+.5f}")

    # ---- release: stop forcing it down -> spring pulls back to 0 ----
    print("\n[3] released, spring rebound (expect pos -> ~0):")
    for i in range(200):
        sim.step()
        bottle.update(dt)
        if i % 25 == 24 or i == 199:
            print(f"   step {i+1:3d}: pos = {read_pos():+.5f}")

    print("\nDONE")

    simulation_app.close()


if __name__ == "__main__":
    main()
