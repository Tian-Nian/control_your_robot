"""把各组件最新帧收成一帧，并标出它们相差多少毫秒。"""

from __future__ import annotations

import time

import numpy as np


def read_observation(session, *, names: list[str] | None = None, max_skew_ms: float = 80.0, body=None) -> dict:
    """读原始观测。图像留在数组里，不经过 snapshot 的摘要。"""
    selected = list(names) if names is not None else [item["name"] for item in session.describe()]
    now_ns = time.time_ns()
    components = {}
    stamps = []
    for name in selected:
        state = session.read_raw(name)
        if state is None:
            components[name] = None
            continue
        stamps.append(int(state["timestamp_ns"]))
        components[name] = {
            "seq": int(state["seq"]),
            "timestamp_ns": int(state["timestamp_ns"]),
            "age_ms": _age_ms(int(state["timestamp_ns"]), now_ns),
            "data": state.get("data"),
        }
    skew_ms = 0.0 if len(stamps) < 2 else (max(stamps) - min(stamps)) / 1e6
    missing = [name for name, value in components.items() if value is None]
    observation = {
        "timestamp_ns": now_ns,
        "skew_ms": float(skew_ms),
        "within_skew": not missing and skew_ms <= float(max_skew_ms),
        "missing": missing,
        "components": components,
    }
    if body is not None:
        observation["palm"] = _palms(body, components)
        observation["flange"] = _flanges(body, components)
    return observation


def _age_ms(timestamp_ns: int, now_ns: int) -> float | None:
    if timestamp_ns <= 0:
        return None
    return max(0.0, (now_ns - timestamp_ns) / 1e6)


def _joints(components: dict, arm: str):
    state = components.get(f"{arm}_arm")
    if not state or not isinstance(state.get("data"), dict):
        return None
    joint = state["data"].get("joint")
    if joint is None:
        return None
    return np.asarray(joint, dtype=float).reshape(-1)


def _palms(body, components: dict) -> dict:
    poses = {}
    for arm in body.arm_names():
        joints = _joints(components, arm)
        if joints is None:
            continue
        poses[arm] = body.palm_pose(arm, joints)
    return poses


def _flanges(body, components: dict) -> dict:
    poses = {}
    for arm in body.arm_names():
        joints = _joints(components, arm)
        if joints is None:
            continue
        poses[arm] = body.flange_pose(arm, joints)
    return poses
