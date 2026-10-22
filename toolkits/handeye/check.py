"""手眼条件。缺相机、板子或内参时打印问题，退出码 2。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
HERE = Path(__file__).resolve().parent
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from board import board_question, load_config  # noqa: E402


def opencv_status() -> tuple[bool, str]:
    try:
        import cv2
    except ImportError as exc:
        return False, str(exc)
    if not hasattr(cv2, "aruco"):
        return False, "cv2 没有 aruco，检测不了 Charuco"
    return True, ""


def evaluate(components, states, config, opencv_ok, opencv_error="", confirm_board=False) -> dict:
    questions = []
    if not opencv_ok:
        questions.append(f"当前环境没有可用的 OpenCV（{opencv_error}）。装上带 calib3d 的 opencv 后再解。")
    arms = [item["name"] for item in components if "eef" in list(item.get("publish") or [])]
    cameras = [item["name"] for item in components if "color" in list(item.get("publish") or [])]
    if not arms:
        questions.append("没有机械臂在 publish 里提供 eef（xyz + wxyz）。")
    if not cameras:
        questions.append("当前服务没有发布 color 的相机。PiperSim 看不见标定板。相机组件名是什么？")
    setup = config.get("setup")
    if setup not in ("eye_in_hand", "eye_to_hand"):
        questions.append("眼在手上还是眼在手外？回答 eye_in_hand 或 eye_to_hand，并写进 configs/default.yml 的 setup。")
    camera_name = config.get("camera")
    if cameras and camera_name not in cameras:
        questions.append(f"configs/default.yml 的 camera 要写成其中一台：{cameras}。")
    arm_name = config.get("arm")
    if arms and arm_name not in arms:
        questions.append(f"configs/default.yml 的 arm 要写成其中一条：{arms}。")
    matrix = config.get("camera_matrix")
    if not _matrix_ok(matrix):
        questions.append("请给出相机内参，填进 configs/default.yml 的 camera_matrix（3×3，像素）。")
    if not confirm_board:
        questions.append(board_question(config))
    return {
        "ok": not questions,
        "questions": questions,
        "arms": arms,
        "cameras": cameras,
        "arm_has_eef": {name: _eef_ok((states or {}).get(name)) for name in arms},
    }


def _matrix_ok(matrix) -> bool:
    if matrix is None:
        return False
    try:
        import numpy as np

        values = np.asarray(matrix, dtype=float)
    except (TypeError, ValueError):
        return False
    return values.shape == (3, 3)


def _eef_ok(state) -> bool:
    if not isinstance(state, dict) or "eef" not in state:
        return False
    try:
        import numpy as np

        return np.asarray(state["eef"]).size >= 7
    except (TypeError, ValueError):
        return False


def from_session(session, config, confirm_board=False) -> dict:
    described = session.describe()
    states = {}
    for item in described:
        raw = session.read_raw(item["name"])
        data = None if raw is None else raw.get("data")
        states[item["name"]] = data if isinstance(data, dict) else {}
    ok, error = opencv_status()
    report = evaluate(described, states, config, ok, error, confirm_board=confirm_board)
    report["service"] = session._config.name
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="检查手眼条件")
    parser.add_argument("--service", default=None)
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--confirm-board", action="store_true")
    args = parser.parse_args(argv)
    config = load_config(args.config)
    from robot.runtime.session import Session

    try:
        session = Session.attach(args.service)
    except RuntimeError as exc:
        ok, error = opencv_status()
        report = evaluate([], {}, config, ok, error, confirm_board=args.confirm_board)
        report["questions"].insert(0, f"后台没有服务（{exc}）。先把带机械臂和彩色相机的配置跑起来。")
        report["ok"] = False
    else:
        try:
            report = from_session(session, config, confirm_board=args.confirm_board)
        finally:
            session.close()
    print(json.dumps(report, ensure_ascii=False))
    return 0 if report["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
