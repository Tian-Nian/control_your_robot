"""从 Session 读 eef 和 color，检测 Charuco，存成 solve.py 用的 npz。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
HERE = Path(__file__).resolve().parent
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from board import board_pose, detect_charuco, load_config  # noqa: E402
from check import from_session  # noqa: E402
from pose import matrix_from_rvec_tvec, matrix_from_xyz_wxyz  # noqa: E402


def grab(session, config, cv2):
    arm = config["arm"]
    camera = config["camera"]
    arm_raw = session.read_raw(arm) or {}
    cam_raw = session.read_raw(camera) or {}
    arm_data = arm_raw.get("data") if isinstance(arm_raw, dict) else None
    cam_data = cam_raw.get("data") if isinstance(cam_raw, dict) else None
    if not isinstance(arm_data, dict) or "eef" not in arm_data:
        raise RuntimeError(f"{arm} 没有 eef")
    if not isinstance(cam_data, dict) or "color" not in cam_data:
        raise RuntimeError(f"{camera} 没有 color")
    image = np.asarray(cam_data["color"])
    corners, ids, board = detect_charuco(image, config, cv2)
    minimum = int(config.get("capture", {}).get("min_corners", 8))
    count = 0 if ids is None else int(len(ids))
    if count < minimum:
        raise RuntimeError(f"这张只看到 {count} 个 Charuco 角点，至少要 {minimum} 个。把板子放进画面再拍。")
    posed = board_pose(corners, ids, board, config["camera_matrix"], config.get("dist_coeffs") or [0, 0, 0, 0, 0], cv2)
    if posed is None:
        raise RuntimeError("角点不够解板子的位姿。")
    rvec, tvec = posed
    return matrix_from_xyz_wxyz(arm_data["eef"]), matrix_from_rvec_tvec(rvec, tvec, cv2)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="采一张手眼样本，或把多张存进 npz")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--service", default=None)
    args = parser.parse_args(argv)
    import cv2
    from robot.runtime.session import Session

    config = load_config(args.config)
    try:
        session = Session.attach(args.service)
    except RuntimeError as exc:
        print(json.dumps({"ok": False, "questions": [f"后台没有服务（{exc}）。"]}, ensure_ascii=False))
        return 2
    try:
        report = from_session(session, config, confirm_board=True)
        if not report["ok"]:
            print(json.dumps(report, ensure_ascii=False))
            return 2
        gripper, target = grab(session, config, cv2)
    finally:
        session.close()
    grippers, targets = _append(args.output, gripper, target, str(config["setup"]))
    print(json.dumps({"ok": True, "samples": len(grippers), "output": str(args.output)}, ensure_ascii=False))
    minimum = int(config.get("capture", {}).get("min_samples", 12))
    if len(grippers) < minimum:
        arm = config["arm"]
        print(
            json.dumps(
                {
                    "questions": [
                        f"还要继续采，凑满 {minimum} 张再运行 solve.py。"
                        f"换姿态用 cyr call {arm} 下发关节目标，走到停稳后再拍下一张。没有重力补偿时搬不动臂。"
                    ]
                },
                ensure_ascii=False,
            )
        )
    return 0


def _append(path: Path, gripper, target, setup: str):
    grippers = [gripper]
    targets = [target]
    if path.is_file():
        previous = np.load(path)
        grippers = list(previous["gripper"]) + grippers
        targets = list(previous["target"]) + targets
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(path, gripper=np.stack(grippers), target=np.stack(targets), setup=np.array(setup))
    return grippers, targets


if __name__ == "__main__":
    raise SystemExit(main())
