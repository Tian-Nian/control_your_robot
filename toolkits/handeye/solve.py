"""OpenCV calibrateHandEye。眼在手外时先把法兰位姿取逆。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from pose import invert

SETUPS = ("eye_in_hand", "eye_to_hand")


def solve_handeye(gripper_poses, target_poses, setup: str, cv2=None) -> dict:
    if setup not in SETUPS:
        raise ValueError(f"setup 只能是 {SETUPS}，得到 {setup!r}")
    if len(gripper_poses) < 3 or len(gripper_poses) != len(target_poses):
        raise ValueError("至少 3 对位姿，且末端和板子的数量要一样")
    if cv2 is None:
        import cv2 as cv2
    bases = [invert(np.asarray(pose, dtype=float)) for pose in gripper_poses] if setup == "eye_to_hand" else gripper_poses
    out = _handeye(bases, target_poses, cv2)
    meaning = "camera_to_gripper" if setup == "eye_in_hand" else "camera_to_base"
    return {"setup": setup, "meaning": meaning, "transform": out}


def _handeye(gripper_poses, target_poses, cv2) -> np.ndarray:
    rotations_g = [np.asarray(pose, dtype=float)[:3, :3] for pose in gripper_poses]
    translations_g = [np.asarray(pose, dtype=float)[:3, 3].reshape(3, 1) for pose in gripper_poses]
    rotations_t = [np.asarray(pose, dtype=float)[:3, :3] for pose in target_poses]
    translations_t = [np.asarray(pose, dtype=float)[:3, 3].reshape(3, 1) for pose in target_poses]
    if hasattr(cv2, "calibrateHandEye"):
        rotation, translation = cv2.calibrateHandEye(
            rotations_g,
            translations_g,
            rotations_t,
            translations_t,
            method=cv2.CALIB_HAND_EYE_TSAI,
        )
        out = np.eye(4)
        out[:3, :3] = np.asarray(rotation, dtype=float)
        out[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
        return out
    return _park(gripper_poses, target_poses)


def _park(gripper_poses, target_poses) -> np.ndarray:
    """AX = XB。A 是相邻两次法兰运动，B 是板子在相机里的相对运动。"""
    motions_a = []
    motions_b = []
    for index in range(len(gripper_poses) - 1):
        motions_a.append(invert(np.asarray(gripper_poses[index], dtype=float)) @ np.asarray(gripper_poses[index + 1], dtype=float))
        motions_b.append(np.asarray(target_poses[index], dtype=float) @ invert(np.asarray(target_poses[index + 1], dtype=float)))
    moment = np.zeros((3, 3))
    for motion_a, motion_b in zip(motions_a, motions_b):
        moment += np.outer(_so3_log(motion_b[:3, :3]), _so3_log(motion_a[:3, :3]))
    eigvals, eigvecs = np.linalg.eigh(moment.T @ moment)
    inv_sqrt = eigvecs @ np.diag(1.0 / np.sqrt(np.maximum(eigvals, 1e-12))) @ eigvecs.T
    rotation = inv_sqrt @ moment.T
    left, _singular, right = np.linalg.svd(rotation)
    rotation = left @ np.diag([1.0, 1.0, np.linalg.det(left @ right)]) @ right
    rows = []
    values = []
    for motion_a, motion_b in zip(motions_a, motions_b):
        rows.append(motion_a[:3, :3] - np.eye(3))
        values.append(rotation @ motion_b[:3, 3] - motion_a[:3, 3])
    translation, *_rest = np.linalg.lstsq(np.vstack(rows), np.concatenate(values), rcond=None)
    out = np.eye(4)
    out[:3, :3] = rotation
    out[:3, 3] = translation
    return out


def _so3_log(rotation: np.ndarray) -> np.ndarray:
    cosine = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    angle = float(np.arccos(cosine))
    if angle < 1e-8:
        return np.zeros(3)
    axis = np.array(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ],
        dtype=float,
    )
    return axis * (angle / (2.0 * np.sin(angle)))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="用采集到的 npz 解手眼")
    parser.add_argument("samples", type=Path)
    parser.add_argument("--setup", default=None)
    args = parser.parse_args(argv)
    data = np.load(args.samples)
    setup = args.setup or str(data["setup"])
    result = solve_handeye(data["gripper"], data["target"], setup)
    print(json.dumps({"setup": result["setup"], "meaning": result["meaning"], "transform": result["transform"].tolist()}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    raise SystemExit(main())
