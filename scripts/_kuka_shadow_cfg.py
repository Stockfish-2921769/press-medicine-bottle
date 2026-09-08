"""合并 Kuka iiwa7 + Shadow Hand 单条 articulation 的 ArticulationCfg。
资产: ../assets/kuka_shadow.usda(引用 kuka + shadow, 摘除 Allegro/影子 ArticulationRoot,
robot0_hand_mount 焊到 iiwa7_link_7)。驱动 iiwa(arm) + shadow(fingers) 两套。"""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

_HERE = os.path.dirname(os.path.abspath(__file__))
_ASSET = os.path.join(_HERE, "..", "assets", "kuka_shadow.usda")

KUKA_SHADOW_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=_ASSET,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            retain_accelerations=True,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1000.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=32,
            solver_velocity_iteration_count=1,
            sleep_threshold=0.005,
            stabilization_threshold=0.0005,
        ),
        joint_drive_props=sim_utils.JointDrivePropertiesCfg(drive_type="force"),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos={
            "iiwa7_joint_(1|2|7)": 0.0,
            "iiwa7_joint_3": 0.7854,
            "iiwa7_joint_4": 1.5708,
            "iiwa7_joint_(5|6)": -1.5708,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["iiwa7_joint_(1|2|3|4|5|6|7)"],
            effort_limit_sim={"iiwa7_joint_(1|2|3|4|5|6|7)": 300.0},
            stiffness={
                "iiwa7_joint_(1|2|3|4)": 300.0,
                "iiwa7_joint_5": 100.0,
                "iiwa7_joint_6": 50.0,
                "iiwa7_joint_7": 25.0,
            },
            damping={
                "iiwa7_joint_(1|2|3|4)": 45.0,
                "iiwa7_joint_5": 20.0,
                "iiwa7_joint_6": 15.0,
                "iiwa7_joint_7": 15.0,
            },
            friction={"iiwa7_joint_(1|2|3|4|5|6|7)": 1.0},
        ),
        "fingers": ImplicitActuatorCfg(
            joint_names_expr=["robot0_WR.*", "robot0_(FF|MF|RF|LF|TH)J(3|2|1)",
                              "robot0_(LF|TH)J4", "robot0_THJ0"],
            effort_limit_sim={
                "robot0_WRJ1": 4.785,
                "robot0_WRJ0": 2.175,
                "robot0_(FF|MF|RF|LF)J1": 0.7245,
                "robot0_FFJ(3|2)": 0.9,
                "robot0_MFJ(3|2)": 0.9,
                "robot0_RFJ(3|2)": 0.9,
                "robot0_LFJ(4|3|2)": 0.9,
                "robot0_THJ4": 2.3722,
                "robot0_THJ3": 1.45,
                "robot0_THJ(2|1)": 0.99,
                "robot0_THJ0": 0.81,
            },
            stiffness={
                "robot0_WRJ.*": 5.0,
                "robot0_(FF|MF|RF|LF|TH)J(3|2|1)": 1.0,
                "robot0_(LF|TH)J4": 1.0,
                "robot0_THJ0": 1.0,
            },
            damping={
                "robot0_WRJ.*": 0.5,
                "robot0_(FF|MF|RF|LF|TH)J(3|2|1)": 0.1,
                "robot0_(LF|TH)J4": 0.1,
                "robot0_THJ0": 0.1,
            },
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
