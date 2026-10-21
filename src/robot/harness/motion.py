"""关节目标按控制周期限速前进一步。"""

from __future__ import annotations

import time

import numpy as np


class JointMotion:
    """每拍把指令往目标靠近，最多 `max_step_rad`。到位还要等反馈进入容差。"""

    def __init__(
        self,
        start,
        goal,
        *,
        max_step_rad: float,
        tol_rad: float,
        timeout_s: float,
        seq: int,
    ):
        self.command = np.asarray(start, dtype=float).reshape(-1).copy()
        self.goal = np.asarray(goal, dtype=float).reshape(-1).copy()
        if self.command.shape != self.goal.shape:
            raise ValueError(f"关节长度 {self.goal.size}，当前是 {self.command.size}")
        self.max_step = max(float(max_step_rad), 1e-6)
        self.tol = max(float(tol_rad), 0.0)
        self.deadline = time.monotonic() + max(float(timeout_s), 0.05)
        self.seq = int(seq)

    def advance(self) -> np.ndarray:
        delta = self.goal - self.command
        self.command = self.command + np.clip(delta, -self.max_step, self.max_step)
        return self.command.copy()

    def command_reached(self) -> bool:
        return float(np.max(np.abs(self.goal - self.command))) <= 1e-8

    def error(self, measured) -> float:
        values = np.asarray(measured, dtype=float).reshape(-1)
        if values.shape != self.goal.shape:
            return float("inf")
        return float(np.max(np.abs(self.goal - values)))

    def tracked(self, measured) -> bool:
        return self.command_reached() and self.error(measured) <= self.tol

    def timed_out(self) -> bool:
        return time.monotonic() >= self.deadline


def path_samples(start, goal, max_step_rad: float) -> list[np.ndarray]:
    """包含终点的线性采样，相邻两点不超过 `max_step_rad`。"""
    begin = np.asarray(start, dtype=float).reshape(-1)
    end = np.asarray(goal, dtype=float).reshape(-1)
    delta = float(np.max(np.abs(end - begin)))
    steps = max(1, int(np.ceil(delta / max(float(max_step_rad), 1e-6))))
    return [begin + (index / steps) * (end - begin) for index in range(1, steps + 1)]
