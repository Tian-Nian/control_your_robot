"""eef 的 xyz + wxyz，和 OpenCV 手眼用的 4×4。"""

from __future__ import annotations

import numpy as np


def matrix_from_xyz_wxyz(pose) -> np.ndarray:
    values = np.asarray(pose, dtype=float).reshape(-1)
    if values.size < 7:
        raise ValueError("eef 需要 xyz + wxyz")
    translation = values[:3]
    quat = values[3:7]
    norm = float(np.linalg.norm(quat))
    if norm < 1e-8:
        raise ValueError("eef 的四元数是 0")
    w, x, y, z = quat / norm
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )
    out = np.eye(4)
    out[:3, :3] = rotation
    out[:3, 3] = translation
    return out


def matrix_from_rvec_tvec(rvec, tvec, cv2) -> np.ndarray:
    rotation, _jacobian = cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3))
    out = np.eye(4)
    out[:3, :3] = rotation
    out[:3, 3] = np.asarray(tvec, dtype=float).reshape(3)
    return out


def invert(pose: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    rotation = pose[:3, :3]
    out[:3, :3] = rotation.T
    out[:3, 3] = -rotation.T @ pose[:3, 3]
    return out
