"""合并 Kuka iiwa7 + Shadow Hand 成单条 articulation 的 USD 生成器。

事实依据(probe_usd_structure): kuka.usd /kuka 下并列 iiwa7_link_* + 整棵 Allegro(ee_link),
ee_link/allegro_mount 由 /kuka/iiwa7_link_7/allegro_mount_joint(→allegro_mount)与
ee_link/allegro_mount/allegro_mount_joint(→palm_link)两固定关节焊到 iiwa7_link_7;
ArticulationRoot 在 /kuka/root_joint(接地固定)。shadow_hand_instanceable.usd 为
/shadow_hand(ArticulationRoot) + robot0_hand_mount 由 /shadow_hand/joints/rootJoint 接世界,
手沿 -y 伸到 0.46m。

做法: 引用 kuka </kuka> → 停用 ee_link 及两个 allegro 焊点 → 在 iiwa7_link_7 下引用
shadow 整棵 → 移除 shadow 的 ArticulationRoot 与 rootJoint → 加固定关节焊 robot0_hand_mount
到 iiwa7_link_7(--mx/--my/--mz 为 link_7 系内挂载偏移; --rot_deg 绕 link_7 x 旋转)。
输出 assets/kuka_shadow.usda(内部引用 https 原始资产, 由 cache 解析)。"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--mx", type=float, default=0.0)
parser.add_argument("--my", type=float, default=0.0)
parser.add_argument("--mz", type=float, default=0.0)
parser.add_argument("--rot_deg", type=float, default=0.0, help="绕 link_7 局部 x 轴额外旋转(度, 向后兼容)")
parser.add_argument("--rotx", type=float, default=0.0, help="挂载额外旋转: 绕 link_7 局部 x 轴(度)")
parser.add_argument("--roty", type=float, default=0.0, help="绕 link_7 局部 y 轴(度)")
parser.add_argument("--rotz", type=float, default=0.0, help="绕 link_7 局部 z 轴(度)")
parser.add_argument("--out", type=str, default="/home/ubuntu/press_demo/isaac_demo/assets/kuka_shadow.usda")
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

KUKA_URL = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/"
            "Isaac/IsaacLab/Robots/KukaAllegro/kuka.usd")
SHADOW_URL = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/"
              "Isaac/Robots/ShadowRobot/ShadowHand/shadow_hand.usd")


def strip_articulation_root(stage, path):
    p = stage.GetPrimAtPath(path)
    if not p.IsValid():
        return "no-prim"
    out = "unknown"
    try:
        out = f"RemoveAppliedSchema(cls)={p.RemoveAppliedSchema(UsdPhysics.ArticulationRootAPI)}"
    except Exception as e:
        try:
            out = f"RemoveAPI(cls)={p.RemoveAPI(UsdPhysics.ArticulationRootAPI)}"
        except Exception as e2:
            out = f"err1={e} | err2={e2}"
    still = False
    for probe in ("GetAppliedSchemas", "HasAPI"):
        try:
            fn = getattr(p, probe)
            if probe == "GetAppliedSchemas":
                schemas = fn()
                still = "ArticulationRootAPI" in str(schemas)
            else:
                still = bool(fn(UsdPhysics.ArticulationRootAPI))
            break
        except Exception:
            continue
    return f"{out} (still_present={still})"


def main():
    import math
    stage = Usd.Stage.CreateNew(args_cli.out)
    # 与 kuka 资产一致的米/Z 上轴
    info = Usd.Stage.Open(KUKA_URL)
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.GetStageMetersPerUnit(info))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.GetStageUpAxis(info))

    root = UsdGeom.Xform.Define(stage, "/KukaShadow")
    stage.GetRootLayer().defaultPrim = "KukaShadow"
    root.GetPrim().GetReferences().AddReference(KUKA_URL, Sdf.Path("/kuka"))
    rp = stage.GetPrimAtPath("/KukaShadow")
    if not rp or not rp.IsValid():
        raise RuntimeError("failed to compose kuka reference")

    # 1) 停用 Allegro 整棵 + 两个焊点
    for dead in ["/KukaShadow/ee_link", "/KukaShadow/iiwa7_link_7/allegro_mount_joint"]:
        pr = stage.GetPrimAtPath(dead)
        if pr.IsValid():
            pr.SetActive(False)
            print("deactivated", dead)
        else:
            print("WARN not found", dead)

    # 2) 引用 shadow 整棵到 /KukaShadow 下(与 iiwa7_link_* 平级; PhysX 禁刚体层级嵌套)
    mount = UsdGeom.Xform.Define(stage, "/KukaShadow/shadow_mount")
    mount.GetPrim().GetReferences().AddReference(SHADOW_URL)  # defaultPrim /shadow_hand
    sh_root_path = "/KukaShadow/shadow_mount"
    print("strip articulation root:", sh_root_path,
          "->", strip_articulation_root(stage, sh_root_path))

    # 3) 停用 shadow 自己的接世界 rootJoint
    rj = "/KukaShadow/shadow_mount/joints/rootJoint"
    prj = stage.GetPrimAtPath(rj)
    if prj.IsValid():
        prj.SetActive(False)
        print("deactivated", rj)
    else:
        print("WARN rootJoint not found", rj)

    # 4) 焊 link_7 -> robot0_hand_mount(固定), local pos0=挂载偏移
    import math as m
    weld = UsdPhysics.FixedJoint.Define(stage, "/KukaShadow/shadow_weld_joint")
    weld.CreateBody0Rel().SetTargets([Sdf.Path("/KukaShadow/iiwa7_link_7")])
    weld.CreateBody1Rel().SetTargets(
        [Sdf.Path("/KukaShadow/shadow_mount/robot0_hand_mount")])
    weld.CreateLocalPos0Attr().Set((args_cli.mx, args_cli.my, args_cli.mz))
    # 组合挂载旋转: rot_deg(旧, 绕x) 若给出则并入; rotx/roty/rotz 绕 link_7 局部固定轴 XYZ 组
    rx = args_cli.rot_deg + args_cli.rotx
    q = [1.0, 0.0, 0.0, 0.0]
    for axis, deg in (("x", rx), ("y", args_cli.roty), ("z", args_cli.rotz)):
        if abs(deg) <= 1e-6:
            continue
        a = m.radians(deg) / 2.0
        d = {"x": (m.cos(a), m.sin(a), 0.0, 0.0),
             "y": (m.cos(a), 0.0, m.sin(a), 0.0),
             "z": (m.cos(a), 0.0, 0.0, m.sin(a))}[axis]
        # q = q * d (先局部 x 后 y 后 z 作用于刚体)
        w, x, y, z = q
        dw, dx, dy, dz = d
        q = [w * dw - x * dx - y * dy - z * dz,
             w * dx + x * dw + y * dz - z * dy,
             w * dy - x * dz + y * dw + z * dx,
             w * dz + x * dy - y * dx + z * dw]
    if any(abs(v) > 1e-6 for v in q[1:]):
        weld.CreateLocalRot0Attr().Set(Gf.Quatf(*q))
    weld.CreateLocalPos1Attr().Set((0.0, 0.0, 0.0))
    print("weld joint created link_7 -> robot0_hand_mount offset=(",
          args_cli.mx, args_cli.my, args_cli.mz, ") rot_quat=",
          tuple(round(v, 4) for v in q))

    # 5) 关节驱动(默认 id)与碰撞沿用; 输出前校验影子体/关节已进入
    stage.GetRootLayer().Save()
    print("saved", args_cli.out)
    # quick sanity: count shadow rigid bodies present
    n = 0
    for pr in Usd.PrimRange(stage.GetPseudoRoot()):
        if "robot0_" in pr.GetPath().pathString and pr.HasAPI(UsdPhysics.RigidBodyAPI):
            n += 1
    print("shadow rigid bodies under merged:", n)
    print("DONE")


if __name__ == "__main__":
    main()
