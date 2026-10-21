"""下发前的关节检查。拒绝只说明原因，不表示设备故障。"""

from __future__ import annotations

import numpy as np

from robot.harness.motion import path_samples


class CommandRejected(Exception):
    """这条命令不能下发。组件保持当前位置，服务继续跑。"""


def _as_vector(value, size: int | None = None) -> np.ndarray:
    vector = np.asarray(value, dtype=float).reshape(-1)
    if size is not None and vector.size != size:
        raise CommandRejected(f"关节长度 {vector.size}，需要 {size}")
    return vector


def validate_joints(
    current,
    target,
    *,
    lower,
    upper,
    max_step_rad: float,
    min_clearance_m: float = 0.0,
    clearance=None,
    eef=None,
    flange=None,
    max_fk_error_m: float = 0.008,
    trajectory: bool = False,
    path_step_rad: float | None = None,
) -> np.ndarray:
    """检查限位、单步或整段轨迹的间隙，以及末端残差。通过时返回目标。"""
    goal = _as_vector(target)
    now = _as_vector(current, goal.size)
    low = _as_vector(lower, goal.size)
    high = _as_vector(upper, goal.size)
    if np.any(goal < low - 1e-4) or np.any(goal > high + 1e-4):
        raise CommandRejected("关节超出限位")
    samples = [goal]
    if trajectory:
        step = path_step_rad if path_step_rad is not None else max_step_rad
        samples = path_samples(now, goal, step)
        for sample in samples:
            if np.any(sample < low - 1e-4) or np.any(sample > high + 1e-4):
                raise CommandRejected("轨迹超出限位")
    else:
        step = float(np.max(np.abs(goal - now)))
        if step > float(max_step_rad) + 1e-6:
            raise CommandRejected(f"单步 {step:.3f} rad 超过 {max_step_rad:.3f}")
    if clearance is not None and min_clearance_m > 0.0:
        for sample in samples:
            gap = float(clearance(sample))
            if gap < float(min_clearance_m):
                raise CommandRejected(
                    f"立柱间隙 {gap * 1000:.1f} mm 低于 {float(min_clearance_m) * 1000:.0f} mm"
                )
    if eef is not None and flange is not None:
        pose = np.asarray(flange(goal), dtype=float).reshape(-1)
        wanted = np.asarray(eef, dtype=float).reshape(-1)
        if pose.size < 3 or wanted.size < 3:
            raise CommandRejected("末端位姿至少要有 xyz")
        error = float(np.linalg.norm(pose[:3] - wanted[:3]))
        if error > float(max_fk_error_m):
            raise CommandRejected(f"FK 误差 {error * 1000:.1f} mm")
    return goal
