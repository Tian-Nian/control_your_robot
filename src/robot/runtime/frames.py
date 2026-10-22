"""末端增量。位置加在工具系，姿态用四元数乘法。"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def compose_delta(pose, delta) -> np.ndarray:
    """p' = p + R Δp。Δq 右乘，表示绕工具系转。逆解只使用返回值的 xyz。"""
    current = _as_wxyz(pose)
    raw = np.asarray(delta, dtype=float).reshape(-1)
    if raw.size < 3:
        raise ValueError("末端增量至少要有 xyz")
    rotation = Rotation.from_quat(_xyzw(current[3:7]))
    shift = np.zeros(3, dtype=float)
    shift[: min(3, raw.size)] = raw[:3]
    out = current.copy()
    out[:3] = current[:3] + rotation.as_matrix() @ shift
    if raw.size >= 7:
        out[3:7] = _mul_wxyz(current[3:7], _normalize(raw[3:7]))
    original = np.asarray(pose, dtype=float).reshape(-1)
    if original.size == 6:
        rpy = Rotation.from_quat(_xyzw(out[3:7])).as_euler("xyz")
        return np.concatenate([out[:3], rpy])
    return out


def _as_wxyz(pose) -> np.ndarray:
    values = np.asarray(pose, dtype=float).reshape(-1)
    if values.size >= 7:
        return values[:7].astype(float, copy=True)
    if values.size == 6:
        quat = Rotation.from_euler("xyz", values[3:]).as_quat()
        return np.array([values[0], values[1], values[2], quat[3], quat[0], quat[1], quat[2]], dtype=float)
    raise ValueError("当前末端需要 xyz+wxyz 或 xyz+rpy")


def _xyzw(wxyz) -> np.ndarray:
    return np.array([wxyz[1], wxyz[2], wxyz[3], wxyz[0]], dtype=float)


def _normalize(wxyz) -> np.ndarray:
    quat = np.asarray(wxyz, dtype=float).reshape(4)
    norm = float(np.linalg.norm(quat))
    if norm <= 0:
        raise ValueError("姿态增量不能是零四元数")
    return quat / norm


def _mul_wxyz(left, right) -> np.ndarray:
    lw, lx, ly, lz = (float(item) for item in left)
    rw, rx, ry, rz = (float(item) for item in right)
    return np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ],
        dtype=float,
    )
