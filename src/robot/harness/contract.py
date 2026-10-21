"""和训练侧对齐的动作。臂是关节或 xyz+wxyz，手是 20 维模型顺序。"""

from __future__ import annotations

from typing import Mapping

import numpy as np

HAND_DOF = 20
EEF_DIM = 7
# 拇指、食指、中指、无名指、小指，每指 flex、abd、pip、dip。和手控制器读回的顺序一致。
HAND_JOINT_ORDER = "finger_major"


def components_from_action(action: Mapping) -> dict[str, dict]:
    """把组件命令或训练字段收成 `{组件名: 命令}`。"""
    if not isinstance(action, Mapping):
        raise TypeError("动作必须是映射")
    commands: dict[str, dict] = {}
    for name in ("left_arm", "right_arm", "left_hand", "right_hand"):
        payload = action.get(name)
        if isinstance(payload, Mapping) and payload:
            commands[name] = _component_payload(name, payload)
    _fill_arm(commands, action, "left")
    _fill_arm(commands, action, "right")
    _fill_hand(commands, action, "left")
    _fill_hand(commands, action, "right")
    if not commands:
        raise ValueError("动作里没有臂或手的目标")
    return commands


class ActionContract:
    """`joint` 只接受关节。`eef` 把臂目标交给身体模型求解。"""

    def __init__(self, mode: str = "joint", control_hz: float = 15.0):
        if mode not in ("joint", "eef"):
            raise ValueError("mode 只能是 joint 或 eef")
        self.mode = mode
        self.control_hz = float(control_hz)
        if self.control_hz <= 0:
            raise ValueError("control_hz 必须为正")

    @property
    def period_s(self) -> float:
        return 1.0 / self.control_hz

    def commands(self, action: Mapping) -> dict[str, dict]:
        commands = components_from_action(action)
        if self.mode == "joint":
            for name, payload in commands.items():
                if name.endswith("_arm") and "eef" in payload and "joint" not in payload:
                    raise ValueError(f"{name} 当前合同是关节模式")
        return commands

    def take_action(self, session, action: Mapping, *, timeout: float = 2.0) -> dict:
        results = {}
        for name, payload in self.commands(action).items():
            results[name] = session.call(name, payload, timeout=timeout)
        return results


def _component_payload(name: str, payload: Mapping) -> dict:
    command = {}
    if "joint" in payload:
        command["joint"] = _vector(payload["joint"], HAND_DOF if name.endswith("_hand") else None)
    if "eef" in payload:
        command["eef"] = _vector(payload["eef"], EEF_DIM)
    if "control_mode" in payload:
        command["control_mode"] = payload["control_mode"]
    if not command:
        raise ValueError(f"{name} 没有 joint 或 eef")
    return command


def _fill_arm(commands: dict[str, dict], action: Mapping, side: str) -> None:
    name = f"{side}_arm"
    if name in commands:
        return
    joint = action.get(f"{side}_arm_joint")
    pose = action.get(f"{side}_ee_pose")
    if joint is None and pose is None:
        return
    command = {}
    if joint is not None:
        command["joint"] = _vector(joint)
    if pose is not None:
        command["eef"] = _vector(pose, EEF_DIM)
    commands[name] = command


def _fill_hand(commands: dict[str, dict], action: Mapping, side: str) -> None:
    name = f"{side}_hand"
    if name in commands:
        return
    for key in (f"{side}_ee_joint_state", f"{side}_hand_joint"):
        if key in action and action[key] is not None:
            commands[name] = {"joint": _vector(action[key], HAND_DOF)}
            return


def _vector(value, size: int | None = None) -> list[float]:
    vector = np.asarray(value, dtype=float).reshape(-1)
    if size is not None and vector.size != size:
        raise ValueError(f"长度应为 {size}，得到 {vector.size}")
    return [float(item) for item in vector]
