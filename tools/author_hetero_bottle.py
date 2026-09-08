"""生成"异形药瓶" usda：矮/平面可握瓶身 + 偏心顶部按压喷嘴(puck 沿 Z 滑台)。

动机：原直立 Ø44 药瓶"一次握持内环抱瓶身 + 拇指压顶"几何不可行，根因是喷嘴在
瓶身顶端中心、比瓶口高 ~45mm，逼手抬高。本工具把喷嘴做成瓶身顶面上的一个偏心、
低位的小按压 puck(仍竖直下压、顶面朝上)，瓶身几何可变(圆柱/扁盒/带脚)，并放开
按压口径(弹簧 K/行程/maxForce 可调)。本工具为纯文本 usda 拼串(模板=medicine_bottle.usda，
其 articulation/drive schema 已验证正确)，无需 Isaac 环境，运行即产出可被 Isaac Lab
解析的 PhysX articulation。

坐标系：默认Prim "Bottle" 原点在 spawn 位置（scripts 里 spawn pos=(bx,by,table_top)，
即瓶底落在桌面）。故本文件内 z=0 = 桌面；瓶身几何从 z=0 向上到 body_top。
  base —— 瓶身刚体(可含 body/foot/neck_visual)；默认 PhysicsFixedJoint base_to_world 固定
  nozzle—— 按压 puck 刚体，prismatic(Z)，轴心在 (noz_x,noz_y,body_top)，home 时 puck
            底部悬在 body_top 上方 rest_gap，向下压 travel 到达下极限；弹簧回位。
单位一律米。
"""

import argparse
import os

TPL_HEAD = """#usda 1.0
(
    defaultPrim = "Bottle"
    metersPerUnit = 1
    upAxis = "Z"
)

# 异形药瓶按压 articulation(本文件由 tools/author_hetero_bottle.py 生成)：
#   base  —— 瓶身(几何参数化; 默认 base_to_world FixedJoint 固定) prismatic nozzle(Z)
#   nozzle—— 按压 puck，行程 lower=-{travel}，弹簧 stiffness={K} damping={D} maxForce={MF}
# z=0 = spawn 瓶底接触面(桌面)。几何参数：{shape_descr}
"""


def _rigid_header(name, mass, inertia):
    return (
        f'    def Xform "{name}" (\n'
        f'        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", '
        f'"PhysicsMassAPI", "PhysxRigidBodyAPI"]\n'
        f"    )\n"
        f"    {{\n"
        f"        uniform token[] xformOpOrder = []\n"
        f"        bool physics:rigidBodyEnabled = 1\n"
        f"        bool physics:kinematicEnabled = 0\n"
        f"        float physics:mass = {mass}\n"
        f"        float3 physics:diagonalInertia = ({inertia[0]:.7g}, {inertia[1]:.7g}, {inertia[2]:.7g})\n"
        f"        float physics:angularDamping = 0.1\n"
        f"        float physics:linearDamping = 0.1\n"
        f"        bool physxRigidBody:disableGravity = 0\n"
    )


def _collision_on_body(def_lines, name, radius, height, zc, axis="Z"):
    """圆柱碰撞体(瓶身用)。返回缩进 prim 文本块。"""
    return (
        f'        def Cylinder "{name}" (\n'
        f'            prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI"]\n'
        f"        )\n"
        f"        {{\n"
        f"            uniform token axis = \"{axis}\"\n"
        f"            double height = {height:.7g}\n"
        f"            double radius = {radius:.7g}\n"
        f"            float3 xformOp:translate = (0, 0, {zc:.7g})\n"
        f'            uniform token[] xformOpOrder = ["xformOp:translate"]\n'
        f"            bool physics:collisionEnabled = 1\n"
        f"        }}\n"
    )


def _visual_body(def_lines, name, radius, height, zc, color=None):
    return (
        f'        def Cylinder "{name}"\n'
        f"        {{\n"
        f"            uniform token axis = \"Z\"\n"
        f"            double height = {height:.7g}\n"
        f"            double radius = {radius:.7g}\n"
        f"            float3 xformOp:translate = (0, 0, {zc:.7g})\n"
        f'            uniform token[] xformOpOrder = ["xformOp:translate"]\n'
        f"        }}\n"
    )


def _cyl_body(radius, body_top):
    return _collision_on_body(None, "body", radius, body_top, body_top / 2.0)


def _box_body(hx, hy, body_top):
    """扁盒身体(水平截面半宽 hx,半厚 hy)，中心在瓶身顶部 body_top 之下 body_h=body_top。"""
    # UsdGeomCube 是 [-0.5,0.5]^3，用 scale 得到全尺寸 (2hx, 2hy, body_top)
    return (
        f'        def Cube "body" (\n'
        f'            prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI"]\n'
        f"        )\n"
        f"        {{\n"
        f"            float3 xformOp:scale = ({2*hx:.7g}, {2*hy:.7g}, {body_top:.7g})\n"
        f"            float3 xformOp:translate = (0, 0, {body_top/2:.7g})\n"
        f'            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]\n'
        f"            bool physics:collisionEnabled = 1\n"
        f"        }}\n"
    )


def _neck_visual(radius, noz_x, noz_y, body_top, pad_top):
    """碰撞关闭的视觉瓶颈(让 puck 看着像插在瓶口里的喷嘴)。"""
    h = max(pad_top - body_top + 0.004, 0.002)
    zc = body_top + h / 2 - 0.001
    return (
        f'        def Cylinder "neck_visual"\n'
        f"        {{\n"
        f"            uniform token axis = \"Z\"\n"
        f"            double height = {h:.7g}\n"
        f"            double radius = {radius:.7g}\n"
        f"            float3 xformOp:translate = ({noz_x:.7g}, {noz_y:.7g}, {zc:.7g})\n"
        f'            uniform token[] xformOpOrder = ["xformOp:translate"]\n'
        f"        }}\n"
    )


def build_usda(args):
    body_top = args.body_h
    # ---------- base geometry ----------
    base_geo = []
    if args.body_shape == "cyl":
        base_geo.append(_cyl_body(args.body_r, body_top))
    else:
        base_geo.append(_box_body(args.body_w / 2.0, args.body_t / 2.0, body_top))
    if args.foot_h > 0:
        fr = max(args.body_r if args.body_shape == "cyl" else args.body_w / 2.0, args.foot_r)
        base_geo.append(_collision_on_body(None, "foot", fr, args.foot_h, args.foot_h / 2.0))

    # nozzle puck 局部于 nozzle 原点(noz_x,noz_y,body_top)
    travel = args.travel
    noz_h = args.noz_h
    noz_r = args.noz_r
    # puck 底部(按下到底后)距 body_top = rest_gap_after，必须 >0 避免压穿
    home_bottom = travel + args.bottom_gap
    puck_center = home_bottom + noz_h / 2.0
    pad_top = home_bottom + noz_h  # 相对 body_top

    # 视觉脖颈(仅美观，碰撞关)
    base_geo.append(_neck_visual(noz_r + 0.0015, args.noz_x, args.noz_y, body_top, pad_top))

    # base inertia 按主几何近似(圆柱/盒取包围圆柱)
    bbox_rad = max(args.body_r if args.body_shape == "cyl" else args.body_w / 2.0, args.body_t / 2.0,
                   args.foot_r if args.foot_h > 0 else 0.0)
    m = args.base_mass
    r2 = bbox_rad ** 2
    Izz = 0.5 * m * r2
    Ixx = Iyy = (1.0 / 12.0) * m * (3.0 * r2 + body_top ** 2)
    base_inertia = (Ixx, Iyy, Izz)

    shape_descr = (
        f"body={args.body_shape} body_h={body_top}"
        + (f" r={args.body_r}" if args.body_shape == "cyl" else f" w={args.body_w} t={args.body_t}")
        + (f" foot_r={args.foot_r} foot_h={args.foot_h}" if args.foot_h > 0 else "")
        + f" nozzle@({args.noz_x},{args.noz_y}) pad_top+{pad_top:.4f} travel={travel}"
    )

    L = []
    L.append(TPL_HEAD.format(travel=travel, K=args.K, D=args.D, MF=args.max_force, shape_descr=shape_descr))
    L.append(
        f'\ndef Xform "Bottle" (\n'
        f'    prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]\n'
        f")\n{{\n"
        f"    uniform token[] xformOpOrder = []\n"
        f"    bool physxArticulation:articulationEnabled = 1\n"
    )
    # base
    L.append(_rigid_header("base", m, base_inertia))
    L.extend(base_geo)
    L.append("    }\n")
    # nozzle
    noz_mass = args.noz_mass
    L.append(_rigid_header("nozzle", noz_mass, (1e-7, 1e-7, 1e-7)))
    L.append(
        f'        def Cylinder "puck" (\n'
        f'            prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI"]\n'
        f"        )\n"
        f"        {{\n"
        f'            uniform token axis = "Z"\n'
        f"            double height = {noz_h:.7g}\n"
        f"            double radius = {noz_r:.7g}\n"
        f"            float3 xformOp:translate = (0, 0, {puck_center:.7g})\n"
        f'            uniform token[] xformOpOrder = ["xformOp:translate"]\n'
        f"            bool physics:collisionEnabled = 1\n"
        f"        }}\n"
    )
    L.append("    }\n")
    # joints
    if not args.free:
        L.append(
            f'    def PhysicsFixedJoint "base_to_world"\n'
            f"    {{\n"
            f"        rel physics:body0 = None\n"
            f"        rel physics:body1 = </Bottle/base>\n"
            f"        point3f physics:localPos0 = (0, 0, 0)\n"
            f"        point3f physics:localPos1 = (0, 0, 0)\n"
            f"        quatf physics:localRot0 = (1, 0, 0, 0)\n"
            f"        quatf physics:localRot1 = (1, 0, 0, 0)\n"
            f"    }}\n"
        )
    L.append(
        f'    def PhysicsPrismaticJoint "nozzle_joint" (\n'
        f'        prepend apiSchemas = ["PhysicsDriveAPI:linear", "PhysicsJointStateAPI:linear"]\n'
        f"    )\n"
        f"    {{\n"
        f'        uniform token physics:axis = "Z"\n'
        f"        rel physics:body0 = </Bottle/base>\n"
        f"        rel physics:body1 = </Bottle/nozzle>\n"
        f"        point3f physics:localPos0 = ({args.noz_x:.7g}, {args.noz_y:.7g}, {body_top:.7g})\n"
        f"        point3f physics:localPos1 = (0, 0, 0)\n"
        f"        quatf physics:localRot0 = (1, 0, 0, 0)\n"
        f"        quatf physics:localRot1 = (1, 0, 0, 0)\n"
        f"        float physics:lowerLimit = {-travel:.7g}\n"
        f"        float physics:upperLimit = {args.upper:.7g}\n"
        f"        # 复位弹簧(放开按压口径：K 低则轻压即到底)\n"
        f"        float drive:linear:physics:stiffness = {args.K}\n"
        f"        float drive:linear:physics:damping = {args.D}\n"
        f"        float drive:linear:physics:targetPosition = 0.0\n"
        f"        float drive:linear:physics:maxForce = {args.max_force}\n"
        f'        uniform token drive:linear:physics:type = "force"\n'
        f"        float state:linear:physics:position = 0.0\n"
        f"        float state:linear:physics:velocity = 0.0\n"
        f"    }}\n"
    )
    L.append("}\n")
    return "\n".join(L)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--tag", default="hetero", help="输出文件名异形 tag -> hetero_bottle_<tag>.usda")
    p.add_argument("--out", default=None, help="输出目录(默认 assets/)")
    # 身体
    p.add_argument("--body_shape", choices=["cyl", "box"], default="box")
    p.add_argument("--body_r", type=float, default=0.018, help="cyl 半径")
    p.add_argument("--body_w", type=float, default=0.050, help="box 全宽(x)")
    p.add_argument("--body_t", type=float, default=0.030, help="box 全厚(y)")
    p.add_argument("--body_h", type=float, default=0.095, help="瓶身高度(z) = body_top")
    p.add_argument("--foot_r", type=float, default=0.0, help="底部加宽半径/半宽(0=不要)")
    p.add_argument("--foot_h", type=float, default=0.0, help="底部加宽高度(0=不要)")
    p.add_argument("--base_mass", type=float, default=0.25)
    # 喷嘴
    p.add_argument("--noz_x", type=float, default=0.0, help="喷嘴原点 x 偏移(瓶身顶面)")
    p.add_argument("--noz_y", type=float, default=0.008, help="喷嘴原点 y 偏移")
    p.add_argument("--noz_r", type=float, default=0.011, help="puck 半径")
    p.add_argument("--noz_h", type=float, default=0.010, help="puck 高度")
    p.add_argument("--noz_mass", type=float, default=0.010)
    p.add_argument("--travel", type=float, default=0.005, help="下行程")
    p.add_argument("--upper", type=float, default=0.001, help="上极限")
    p.add_argument("--bottom_gap", type=float, default=0.002, help="压到底时 puck 底距 body_top 的间隙")
    p.add_argument("--K", type=float, default=300.0, help="弹簧刚度 N/m")
    p.add_argument("--D", type=float, default=2.0, help="弹簧阻尼")
    p.add_argument("--max_force", type=float, default=6.0, help="maxForce")
    p.add_argument("--free", action="store_true", help="去掉 base_to_world 固定(自由站)")
    args = p.parse_args()

    usda = build_usda(args)

    base_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.normpath(args.out) if args.out else os.path.join(base_dir, "..", "assets")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"hetero_bottle_{args.tag}.usda")
    with open(out_path, "w") as f:
        f.write(usda)
    print(f"wrote {out_path}")
    print(f"  body_top={args.body_h} pad_top_local(相对瓶顶)={args.travel + args.bottom_gap + args.noz_h:.4f}")
    print(f"  fixed={not args.free}  travel={args.travel}  K={args.K} maxForce={args.max_force}")


if __name__ == "__main__":
    main()
