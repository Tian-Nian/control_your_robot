"""在一组停稳的姿态上，用关节力矩辨识法兰负载的质量和质心。"""

from __future__ import annotations

import numpy as np

from robot.controller.payload_gravity import fit_payload

# 姿态要拉开，尤其是腕的朝向。只在一个锥里采样，质心的某个方向会解不出来。
DEMO_POSES = [
    [0.0, 1.3, -1.0, 0.0, 0.5, 0.0],
    [0.5, 1.2, -0.8, 0.6, 0.7, 0.4],
    [-0.4, 1.0, -1.2, -0.7, 0.4, -0.5],
    [0.2, 1.5, -0.6, 1.0, 0.9, 0.3],
    [-0.3, 0.9, -1.3, 0.8, 0.2, 0.8],
    [0.6, 1.4, -0.9, -0.5, 1.0, -0.3],
    [0.1, 1.1, -1.4, 1.2, 0.3, -0.6],
    [-0.6, 1.5, -0.7, -1.0, 0.8, 0.5],
]
DEMO_MASS = 0.35
DEMO_COM = [0.02, -0.01, 0.06]


def samples_from_arm(arm, poses) -> list[dict]:
    """每个关节目标先下发并到位，再读托住它的力矩。"""
    arm.set_control_mode("position")
    samples = []
    for pose in poses:
        arm.set_joint(np.asarray(pose, dtype=float).reshape(6))
        flange = arm.flange_pose()
        torque = np.asarray(arm.holding_torque(), dtype=float).reshape(6)
        link = np.asarray(arm.link_gravity(), dtype=float).reshape(6)
        samples.append(
            {
                "joint": np.asarray(arm.get_state()["joint"], dtype=float).reshape(6).copy(),
                "jacobian": np.asarray(arm.flange_jacobian(), dtype=float).reshape(6, 6).copy(),
                "rotation": np.asarray(flange[:3, :3], dtype=float).copy(),
                "torque": torque,
                "link_gravity": link,
                "residual": torque - link,
            }
        )
    return samples


def identify_payload(arm, poses) -> dict:
    fit = fit_payload(samples_from_arm(arm, poses))
    arm.set_identified_payload(fit["mass_kg"], fit["com_m"])
    return fit
