"""Charuco 板。参数与 roma 那份 default.yml 一致。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

CONFIG = Path(__file__).resolve().parent / "configs" / "default.yml"


def load_config(path: Path | None = None) -> dict:
    with Path(path or CONFIG).open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    if not isinstance(raw, dict):
        raise ValueError(f"{path or CONFIG} 不是映射")
    return raw


def board_question(config: dict) -> str:
    board = config["board"]
    return (
        "你有一块和配置一致的 Charuco 吗？"
        f"{board['squares_x']} 列 × {board['squares_y']} 行，格 {board['square_length_m'] * 1000:.0f} mm，"
        f"标记 {board['marker_length_m'] * 1000:.0f} mm，{board['dictionary']}。"
        "没有的话改 toolkits/handeye/configs/default.yml，不要用另一块板硬解。"
    )


def make_board(config: dict, cv2):
    board_cfg = config["board"]
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, board_cfg["dictionary"]))
    squares = (int(board_cfg["squares_x"]), int(board_cfg["squares_y"]))
    square = float(board_cfg["square_length_m"])
    marker = float(board_cfg["marker_length_m"])
    board = None
    if hasattr(cv2.aruco, "CharucoBoard"):
        try:
            board = cv2.aruco.CharucoBoard(squares, square, marker, dictionary)
        except TypeError:
            board = None
    if board is None:
        board = cv2.aruco.CharucoBoard_create(squares[0], squares[1], square, marker, dictionary)
    if hasattr(board, "setLegacyPattern"):
        board.setLegacyPattern(bool(board_cfg.get("legacy_pattern", False)))
    return board, dictionary


def detect_charuco(image: np.ndarray, config: dict, cv2):
    board, dictionary = make_board(config, cv2)
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if hasattr(cv2.aruco, "CharucoDetector"):
        detector = cv2.aruco.CharucoDetector(board)
        corners, ids, _marker_corners, _marker_ids = detector.detectBoard(gray)
        return corners, ids, board
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters) if hasattr(cv2.aruco, "ArucoDetector") else None
    if detector is not None:
        marker_corners, marker_ids, _rejected = detector.detectMarkers(gray)
    else:
        marker_corners, marker_ids, _rejected = cv2.aruco.detectMarkers(gray, dictionary)
    if marker_ids is None or len(marker_ids) == 0:
        return None, None, board
    _count, corners, ids = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, gray, board)
    return corners, ids, board


def board_pose(corners, ids, board, camera_matrix, dist_coeffs, cv2):
    if corners is None or ids is None or len(ids) < 4:
        return None
    matrix = np.asarray(camera_matrix, dtype=float).reshape(3, 3)
    dist = np.asarray(dist_coeffs, dtype=float).reshape(-1)
    if hasattr(board, "matchImagePoints"):
        obj, img = board.matchImagePoints(corners, ids)
        if obj is None or len(obj) < 4:
            return None
        ok, rvec, tvec = cv2.solvePnP(obj, img, matrix, dist)
        if not ok:
            return None
        return rvec.reshape(3), tvec.reshape(3)
    ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(corners, ids, board, matrix, dist, None, None)
    if not ok:
        return None
    return rvec.reshape(3), tvec.reshape(3)
