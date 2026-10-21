"""把一组关节 MIT 参数收成同样长度的向量。

τ = kp (q_d - q) + kd (qd_d - qd) + τ_ff 由驱动计算。这里只检查个数，不下发。
"""

from __future__ import annotations

import numpy as np


def pack_joint_mit(q_des, kp, kd, qd_des=None, tau_ff=None, dofs=6) -> dict:
    count = int(dofs)
    packed = {
        "q_des": _column(q_des, count, "q_des"),
        "kp": _column(kp, count, "kp"),
        "kd": _column(kd, count, "kd"),
        "qd_des": np.zeros(count) if qd_des is None else _column(qd_des, count, "qd_des"),
        "feedforward": np.zeros(count) if tau_ff is None else _column(tau_ff, count, "tau_ff"),
    }
    return packed


def _column(value, dofs: int, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.size != dofs:
        raise ValueError(f"{name} 需要 {dofs} 个数，得到 {array.size}")
    return array
