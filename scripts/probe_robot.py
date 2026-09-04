"""探测 Kuka+Allegro 在 dexsuite 默认位形下手掌/指尖的位置与朝向，用于决定药瓶放置。"""

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
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab_assets import KUKA_ALLEGRO_CFG


def quat_to_mat(q):
    w, x, y, z = q
    return torch.tensor(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def main():
    dt = 1 / 60.0
    sim = SimulationContext(SimulationCfg(dt=dt, device="cuda:0", gravity=(0, 0, -9.81)))

    robot = Articulation(KUKA_ALLEGRO_CFG.replace(prim_path="/World/Robot"))
    sim.reset()
    robot.reset()

    # 应用 KUKA_ALLEGRO_CFG 预置的名义臂姿（非零 iiwa + 手指），步进若干帧使其到位
    jnames = list(robot.joint_names)
    target = torch.zeros(1, len(jnames), device=robot.device)
    for pattern, val in KUKA_ALLEGRO_CFG.init_state.joint_pos.items():
        import re as _re
        rx = _re.compile("^" + pattern.replace("(", "(?:") .replace(")", ")") + "$")
        for i, n in enumerate(jnames):
            if rx.match(n):
                target[0, i] = val
    robot.set_joint_position_target(target)
    robot.write_data_to_sim()
    for _ in range(60):
        robot.write_data_to_sim()
        sim.step()
        robot.update(dt)

    names = list(robot.body_names)
    print("num bodies:", len(names))
    for key in ["palm", "ee_link", "index_link_3", "middle_link_3", "index_link_0", "index_link_2"]:
        idxs = [i for i, n in enumerate(names) if key in n]
        if idxs:
            print(f"  bodies matching '{key}':", [names[i] for i in idxs])

    # positions / orientations of a few key bodies
    keys = []
    for k in ["palm_link", "ee_link", "index_link_3", "middle_link_3", "ring_link_3", "thumb_link_3"]:
        if k in names:
            keys.append(names.index(k))
    pos = robot.data.body_pos_w[0]
    quat = robot.data.body_quat_w[0]
    for i in keys:
        p = pos[i]
        q = quat[i]
        R = quat_to_mat(q)
        z_axis = R[:, 2]
        x_axis = R[:, 0]
        print(f"\n[{names[i]}]")
        print(f"  pos      = ({float(p[0]):+.4f}, {float(p[1]):+.4f}, {float(p[2]):+.4f})")
        print(f"  z_axis   = ({float(z_axis[0]):+.3f}, {float(z_axis[1]):+.3f}, {float(z_axis[2]):+.3f})")
        print(f"  x_axis   = ({float(x_axis[0]):+.3f}, {float(x_axis[1]):+.3f}, {float(x_axis[2]):+.3f})")

    print("\njoint_pos init (arm):")
    jn = list(robot.joint_names)
    arm = [i for i, n in enumerate(jn) if n.startswith("iiwa7")]
    for i in arm:
        print(f"   {jn[i]:16s} = {float(robot.data.joint_pos[0, i]):+.3f}")

    print("DONE")
    simulation_app.close()


if __name__ == "__main__":
    main()
