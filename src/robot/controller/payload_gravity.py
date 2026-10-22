"""法兰负载的静态重力。质量和质心对关节力矩是线性的，静止采样就能最小二乘。"""

from __future__ import annotations

import numpy as np

GRAVITY = np.array([0.0, 0.0, -9.81])


def payload_torque(jacobian, rotation, mass, com, gravity=GRAVITY) -> np.ndarray:
    """τ = J^T [m g; (R c) × (m g)]。c 在法兰系，J 是世界系下的法兰雅可比。"""
    wrench = payload_wrench(rotation, mass, com, gravity)
    return np.asarray(jacobian, dtype=float).reshape(6, 6).T @ wrench


def payload_wrench(rotation, mass, com, gravity=GRAVITY) -> np.ndarray:
    force = float(mass) * np.asarray(gravity, dtype=float).reshape(3)
    lever = np.asarray(rotation, dtype=float).reshape(3, 3) @ np.asarray(com, dtype=float).reshape(3)
    moment = np.cross(lever, force)
    return np.concatenate([force, moment])


def payload_regressor(jacobian, rotation, gravity=GRAVITY) -> np.ndarray:
    """6×4。列是 m、h_x、h_y、h_z，h = m c，c 在法兰系。"""
    gravity = np.asarray(gravity, dtype=float).reshape(3)
    rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
    skew = np.array(
        [
            [0.0, -gravity[2], gravity[1]],
            [gravity[2], 0.0, -gravity[0]],
            [-gravity[1], gravity[0], 0.0],
        ],
        dtype=float,
    )
    wrench = np.zeros((6, 4))
    wrench[:3, 0] = gravity
    wrench[3:, 1:] = -skew @ rotation
    return np.asarray(jacobian, dtype=float).reshape(6, 6).T @ wrench


def fit_payload(samples) -> dict:
    """samples 里每项有 jacobian、rotation，以及 residual（测到的力矩减去连杆 g(q)）。"""
    if len(samples) < 4:
        raise ValueError(f"至少 4 个姿态差得开的采样，现在是 {len(samples)}")
    rows = []
    measured = []
    for sample in samples:
        rows.append(payload_regressor(sample["jacobian"], sample["rotation"]))
        measured.append(np.asarray(sample["residual"], dtype=float).reshape(6))
    matrix = np.vstack(rows)
    target = np.concatenate(measured)
    theta, _residuals, rank, _singular = np.linalg.lstsq(matrix, target, rcond=None)
    mass = float(theta[0])
    first_moment = np.asarray(theta[1:], dtype=float).reshape(3)
    com = np.zeros(3) if abs(mass) < 1e-6 else first_moment / mass
    predicted = matrix @ theta
    rms = float(np.sqrt(np.mean((predicted - target) ** 2)))
    return {
        "mass_kg": mass,
        "com_m": com,
        "first_moment": first_moment,
        "rms": rms,
        "rank": int(rank),
    }
