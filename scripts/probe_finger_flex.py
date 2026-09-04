"""从 dexsuite 名义位形出发,直驱食指关节 1-3 屈曲,记录指尖(index_link_3)运动轨迹。

用 write_joint_state_to_sim 直接(运动学)设定关节角,绕开低刚度 PD,纯看几何。
输出: 名义位形 + 每个屈曲角度下的指尖位置/位移,用于决定药瓶喷嘴的摆放。
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

    jnames = list(robot.joint_names)
    ndof = len(jnames)
    qpos = torch.zeros(1, ndof, device=robot.device)
    for pattern, val in KUKA_ALLEGRO_CFG.init_state.joint_pos.items():
        import re as _re
        rx = _re.compile("^" + pattern.replace("(", "(?:") .replace(")", ")") + "$")
        for i, n in enumerate(jnames):
            if rx.match(n):
                qpos[0, i] = val
    qvel = torch.zeros(1, ndof, device=robot.device)

    # 关节限位
    lo = robot.data.joint_pos_limits[0, :, 0]
    hi = robot.data.joint_pos_limits[0, :, 1]
    for i, n in enumerate(jnames):
        if n.startswith("index"):
            print(f"  {n:16s} limit [{float(lo[i]):+.3f}, {float(hi[i]):+.3f}]")

    def set_and_settle(q):
        robot.write_joint_state_to_sim(q, qvel)
        for _ in range(5):
            robot.write_joint_state_to_sim(q, qvel)
            sim.step()
            robot.update(dt)

    set_and_settle(qpos)
    names = list(robot.body_names)
    tip_i = names.index("index_link_3")
    palm_i = names.index("palm_link")

    def report(label):
        p = robot.data.body_pos_w[0]
        qt = robot.data.body_quat_w[0]
        tip = p[tip_i]
        print(f"\n[{label}]")
        print(f"  index_link_3 = ({float(tip[0]):+.4f}, {float(tip[1]):+.4f}, {float(tip[2]):+.4f})")
        for tag, i in [("palm", palm_i)]:
            pp = p[i]
            print(f"  {tag}_link    = ({float(pp[0]):+.4f}, {float(pp[1]):+.4f}, {float(pp[2]):+.4f})")
        # 屈曲参考面: index_link_2 -> index_link_3 方向
        R = quat_to_mat(qt[tip_i])
        z_axis = R[:, 2]
        print(f"  tip z_axis  = ({float(z_axis[0]):+.3f}, {float(z_axis[1]):+.3f}, {float(z_axis[2]):+.3f})")
        return tip

    start_tip = report("nominal (curl=0.3)")
    # 记录关节名/编号
    idx_1 = jnames.index("index_joint_1")
    idx_2 = jnames.index("index_joint_2")
    idx_3 = jnames.index("index_joint_3")
    print("\nflexing index_joint_(1|2|3) together:")
    for curl in [0.0, 0.6, 1.0, 1.4, 1.8]:
        if curl > float(hi[idx_1]) + 1e-3:
            print(f"  curl {curl} exceeds upper limit {float(hi[idx_1]):.3f}; clamp")
            curl = float(hi[idx_1])
        qpos[0, idx_1] = curl
        qpos[0, idx_2] = curl
        qpos[0, idx_3] = curl
        set_and_settle(qpos)
        tip = report(f"curl={curl}")
        disp = tip - start_tip
        print(f"  disp from nominal = ({float(disp[0]):+.4f}, {float(disp[1]):+.4f}, {float(disp[2]):+.4f})")

    print("\nDONE")
    simulation_app.close()


if __name__ == "__main__":
    main()
