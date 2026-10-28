"""Robot-only alternatives for a stalled visual policy, never an automatic escape."""
import numpy as np
from scipy.spatial.transform import Rotation

from robot.harness.policy import DIRECTIONS, TOOL_DIRECTIONS
from robot.harness.vision import LOWER, UPPER, validate_action


def recovery_options(joint, kinematics, fk):
    """Show feasible 1 cm directions and predicted effects of bounded joint steps.

    Range/FK checks are NOT collision clearance or permission to open a held bowl.
    Every selected action still passes normal fresh-feedback validation/execution.
    """
    q = np.asarray(joint, dtype=float)
    pose = np.asarray(fk["tool"], dtype=float)
    rotation = Rotation.from_quat(pose[[4, 5, 6, 3]])
    result = {"source": "current measured joints + FK/IK; no object/contact data or collision check",
              "feasible_base_moves": [], "feasible_tool_moves": [], "joint_steps": [],
              "joint_limit_margins_rad": {"lower": (q - LOWER).tolist(), "upper": (UPPER - q).tolist()}}
    for key, directions, field, action_name in (
            ("one_cm_translation_options", DIRECTIONS, "feasible_base_moves", "move"),
            ("tool_translation_options", TOOL_DIRECTIONS, "feasible_tool_moves", "tool_move")):
        for item in fk.get(key, []):
            if item.get("ik_joint_feasible") is not True:
                continue
            for name, vector in directions.items():
                if np.allclose(item["delta_xyz"], vector, atol=1e-9, rtol=0):
                    result[field].append({"action": action_name, "direction": name, "step_cm": 1})
                    break
    for index in range(6):
        for sign in (-1, 1):
            target = q.copy(); target[index] += .04 * sign
            try:
                validate_action({"action": "move_joint_absolute", "joint": target.tolist()}, q)
            except ValueError:
                continue
            after = np.asarray(kinematics.pose(target, "tool"), dtype=float)
            r_after = Rotation.from_quat(after[[4, 5, 6, 3]])
            tilt = np.rad2deg(np.arccos(np.clip((r_after.as_matrix()[:, 2] @ [0, 0, -1]), -1, 1)))
            away = ((q[index] - LOWER[index] < .08 and sign > 0)
                    or (UPPER[index] - q[index] < .08 and sign < 0))
            result["joint_steps"].append({"action": {"action": "joint_step", "joint_index": index + 1, "sign": sign},
                "delta_xyz_base_m": (after[:3] - pose[:3]).tolist(), "tool_z_after_m": float(after[2]),
                "tool_down_tilt_after_deg": float(tilt), "relieves_near_joint_limit": bool(away),
                "rotation_change_rad": float((r_after * rotation.inv()).magnitude())})
    return result
