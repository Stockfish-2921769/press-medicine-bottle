"""panda_longjaw.usda 的 ArticulationCfg: panda 臂(HIGH_PD, IK 用) + 两长指 prismatic 驱动。

几何(jaw=panda_hand 系): +Z 前向、+X 开合、+Y 刀片高向。close 目标 q→0 (mouth~40mm 压 Ø42),
open 目标 q→U (mouth~80mm 套过)。EE 用 panda_hand; 夹持心 = hand + ZC(=夹持 offset) 沿 +Z。
"""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

_HERE = os.path.dirname(os.path.abspath(__file__))
LONGJAW_USD = os.path.join(_HERE, "..", "assets", "panda_longjaw.usda")

# author 默认参数(与 tools/author_panda_longjaw.py 保持一致)
JAW_OFF = 0.024
JAW_TH = 0.008
JAW_TALL = 0.050
JAW_BLEN = 0.100
JAW_ZC = 0.085
JAW_U = 0.020

PANDA_LONGJAW_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=LONGJAW_USD,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True, max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "panda_joint1": 0.0,
            "panda_joint2": -0.569,
            "panda_joint3": 0.0,
            "panda_joint4": -2.810,
            "panda_joint5": 0.0,
            "panda_joint6": 3.037,
            "panda_joint7": 0.741,
            "longjaw_.*": JAW_U,  # 默认张开
        },
    ),
    actuators={
        "panda_shoulder": ImplicitActuatorCfg(
            joint_names_expr=["panda_joint[1-4]"],
            effort_limit_sim=87.0, stiffness=400.0, damping=80.0,
        ),
        "panda_forearm": ImplicitActuatorCfg(
            joint_names_expr=["panda_joint[5-7]"],
            effort_limit_sim=12.0, stiffness=400.0, damping=80.0,
        ),
        "longjaw": ImplicitActuatorCfg(
            joint_names_expr=["longjaw_.*"],
            effort_limit_sim=500.0, stiffness=200000.0, damping=300.0,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
