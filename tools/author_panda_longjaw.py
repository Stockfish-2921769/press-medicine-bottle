"""给 Panda(panda_instanceable)焊一个 ALOHA 式长指平行爪, 输出长指爪合并 asset。

动机: 原 Panda 平爪净口 ~40mm、franka.usd 的 Robotiq 2F_85 指节只前伸 ~10mm —— 两者都
钳不住 Ø42 高竖瓶。本工具: 引用 panda_instanceable /panda, 停用 panda_hand 整棵(含原手指),
在 panda_link7 上按原 panda_hand 固定关节位姿焊回同名 jaw(RIGID, 让 IK-EE 与旧代码兼容),
jaw 下挂两块竖直长刀片 blade_L/blade_R, 各由 PrismaticJoint(axis X, 镜像)开合:
q=0 内面 |X|=off-thick/2(~0.020 可压 Ø42), q=U 内面 |X|=off-thick/2+u(~0.040, mouth80 可套过)。

几何在 jaw(=panda_hand)系: +Z 前向(接近瓶), +X 开合轴, +Y 竖直(刀片高向)。
刀片 = 长扁盒: 竖直 Y 高 tall、横向 X 厚 thick、前向 Z 长 blen, 盒心前伸 z=zc。
参数默认经推导: off .024 thick .008 tall .050 blen .100 zc .085 u .020。
用法: cd IsaacLab && ./isaaclab.sh -p ../isaac_demo/tools/author_panda_longjaw.py --headless [opts]
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--off", type=float, default=0.024)
parser.add_argument("--thick", type=float, default=0.008)
parser.add_argument("--tall", type=float, default=0.050)
parser.add_argument("--blen", type=float, default=0.100)
parser.add_argument("--zc", type=float, default=0.085)
parser.add_argument("--u", type=float, default=0.020)
parser.add_argument("--out", type=str, default="/home/ubuntu/press_demo/isaac_demo/assets/panda_longjaw.usda")
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

PANDA_URL = f"{ISAACLAB_NUCLEUS_DIR}/Robots/FrankaEmika/panda_instanceable.usd"
OFF, TH, TALL, BLEN, ZC, U = args_cli.off, args_cli.thick, args_cli.tall, args_cli.blen, args_cli.zc, args_cli.u


def appi(p, schema):
    """apply schema by name if not already."""
    if not p.HasAPI(schema):
        p.ApplyAPI(schema)


def add_body(stage, path, mass, inertia):
    p = UsdGeom.Xform.Define(stage, path).GetPrim()
    for s in ("PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"):
        appi(p, s)
    p.GetAttribute("physics:mass").Set(mass)
    p.GetAttribute("physics:diagonalInertia").Set(Gf.Vec3f(*inertia))
    return p


def add_box(stage, path, half, color=None, collision=True):
    """在 path 造一个全尺寸 = 2*half 的盒子(居中于该 xform 原点)。单 translate/scale 嵌套避免顺序坑。"""
    geo = UsdGeom.Xform.Define(stage, path)
    dim = UsdGeom.Xform.Define(stage, path + "/dim")
    dim.AddScaleOp().Set(Gf.Vec3f(half[0] * 2, half[1] * 2, half[2] * 2))
    c = UsdGeom.Cube.Define(stage, path + "/dim/box")
    c.CreateSizeAttr(1.0)
    if collision:
        cp = c.GetPrim()
        appi(cp, "PhysicsCollisionAPI")
        appi(cp, "PhysxCollisionAPI")
        cp.GetAttribute("physics:collisionEnabled").Set(True)
    return geo.GetPrim()


def make_prismatic(stage, path, body0, body1, lp0, lr0, lower, upper):
    """prismatic 开合关节, 显式 Apply PhysicsDriveAPI:linear —— PhysX 才把该 DOF 注册成受驱关节。

    不 Apply schema 而只裸写 drive:linear:physics:* 属性, PhysX 不会建驱动(IsaacLab ImplicitActuator
    绑不上), blade 变被动仅靠碰撞静止 —— 这正是 author_panda_longjaw v1 叶片失控的根因。
    """
    jp = UsdPhysics.PrismaticJoint.Define(stage, path)
    jp.CreateBody0Rel().SetTargets([body0])
    jp.CreateBody1Rel().SetTargets([body1])
    jp.GetLocalPos0Attr().Set(Gf.Vec3f(*lp0))
    jp.GetLocalRot0Attr().Set(Gf.Quatf(*lr0))
    jp.GetLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    jp.GetLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    jp.CreateAxisAttr().Set("X")
    jp.CreateLowerLimitAttr().Set(lower)
    jp.CreateUpperLimitAttr().Set(upper)
    # 关键: multi-instance 驱动 schema 必须先 Apply(带 instance token "linear")
    da = UsdPhysics.DriveAPI.Apply(jp.GetPrim(), "linear")
    da.CreateTypeAttr().Set("force")
    da.CreateStiffnessAttr().Set(200000.0)
    da.CreateDampingAttr().Set(300.0)
    da.CreateMaxForceAttr().Set(500.0)
    da.CreateTargetPositionAttr().Set(0.0)
    # 初始 DOF 位形(带 state schema 才被 PhysX 采用; 无则 isaaclab reset 用 init_state 覆写)
    p = jp.GetPrim()
    p.CreateAttribute("state:linear:physics:position", Sdf.ValueTypeNames.Float).Set(lower)
    p.CreateAttribute("state:linear:physics:velocity", Sdf.ValueTypeNames.Float).Set(0.0)
    return jp


def main():
    stage = Usd.Stage.CreateNew(args_cli.out)
    info = Usd.Stage.Open(PANDA_URL)
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.GetStageMetersPerUnit(info))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.GetStageUpAxis(info))
    P = "/PandaLongJaw"
    root = UsdGeom.Xform.Define(stage, P)
    stage.GetRootLayer().defaultPrim = "PandaLongJaw"
    root.GetPrim().GetReferences().AddReference(PANDA_URL, Sdf.Path("/panda"))
    hand = stage.GetPrimAtPath(P + "/panda_hand")
    if not hand.IsValid():
        raise RuntimeError("panda_hand not found")
    # 保留 panda_hand(RIGID, 已是 jaw 挂载系); 停用其旧碰撞/视觉与两根原指(体+关节)
    for dead in (P + "/panda_hand/visuals", P + "/panda_hand/collisions",
                 P + "/panda_hand/panda_finger_joint1", P + "/panda_hand/panda_finger_joint2",
                 P + "/panda_leftfinger", P + "/panda_rightfinger"):
        d = stage.GetPrimAtPath(dead)
        if d.IsValid():
            d.SetActive(False)
            print("deactivated", dead)
        else:
            print("WARN not found", dead)
    # panda_hand 质量源(原碰撞/视觉几何)已被停用 → 显式给质量/惯量, 否则 PhysX 球近似(两刀片 joint 的 body0)
    hprim = stage.GetPrimAtPath(P + "/panda_hand")
    if not hprim.HasAPI(UsdPhysics.MassAPI):
        hprim.ApplyAPI(UsdPhysics.MassAPI)
    ma = UsdPhysics.MassAPI(hprim)
    ma.CreateMassAttr().Set(0.55)
    ma.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(5e-4, 8e-4, 5e-4))
    print("panda_hand mass set to 0.55 (collision child scope deactivated)")
    add_box(stage, P + "/panda_hand/jaw_vis", (0.024, 0.011, 0.020), collision=False)

    # 刀片: 在 jaw 系 +X 为开合向。 blade_L 体+立方在 (off,0,zc) 附近; prismatic 平动体沿 X。
    # 用"整体 xform 定位 + 局域盒子"简化: 刀片 body 原点即关节 attach; 盒心放在 body 系 (0,0,zc)。
    for tag, sx, lr in (("L", +1.0, (1.0, 0.0, 0.0, 0.0)),
                        ("R", -1.0, (0.0, 0.0, 0.0, 1.0))):
        b = P + f"/blade_{tag}"
        add_body(stage, b, 0.10, (5e-5, 2e-3, 2e-3))
        # 盒心: body 系 (0,0,zc); 但因 blade_R 的 joint frame 绕 Z 转 180, 盒仍沿 body +Z。
        t = UsdGeom.Xform.Define(stage, b + "/geo")
        t.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, ZC))
        add_box(stage, b + "/geo/col", (TH / 2, TALL / 2, BLEN / 2), collision=True)
        add_box(stage, b + "/geo/vis", (TH / 2, TALL / 2, BLEN / 2), collision=False)
        # prismatic body0=jaw(panda_hand) body1=blade; 关节基在 jaw 系 (sx*off,0,0)
        make_prismatic(stage, P + f"/longjaw_{tag.lower()}", Sdf.Path(P + "/panda_hand"),
                       Sdf.Path(b), (sx * OFF, 0.0, 0.0), lr, 0.0, U)
    stage.GetRootLayer().Save()
    print("saved", args_cli.out)
    print("params: off=%.4f thick=%.4f tall=%.4f blen=%.4f zc=%.4f u=%.4f" % (OFF, TH, TALL, BLEN, ZC, U))
    print("  close inner|X|=%.4f mouth=%.3f ; open inner|X|=%.4f mouth=%.3f"
          % (OFF - TH / 2, 2 * (OFF - TH / 2), OFF + U - TH / 2, 2 * (OFF + U - TH / 2)))
    print("DONE")


if __name__ == "__main__":
    main()
