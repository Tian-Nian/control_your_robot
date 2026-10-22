"""把原始 HDF5 收成训练看到的状态，并把 action[t] 定义成 state[t+1]。"""

from __future__ import annotations

from pathlib import Path

import h5py
import numpy as np

_ARM_FIELDS = {
    "joint": "arm_joint_state",
    "eef": "ee_pose",
    "gripper": "ee_joint_state",
}


def to_xspark(source, dest, *, instruction: str, frequency: float) -> Path:
    source = Path(source)
    dest = Path(dest)
    dest.parent.mkdir(parents=True, exist_ok=True)
    state = {}
    with h5py.File(source, "r") as handle:
        for name, group in handle.items():
            if not isinstance(group, h5py.Group):
                continue
            side = _side(name)
            if side is None:
                continue
            if name.endswith("_hand") and "joint" in group:
                state[f"{side}_ee_joint_state"] = np.asarray(group["joint"])
                continue
            if not name.endswith("_arm"):
                continue
            for field, suffix in _ARM_FIELDS.items():
                if field not in group:
                    continue
                key = f"{side}_{suffix}"
                if key in state and field == "gripper":
                    continue
                state[key] = np.asarray(group[field])
    if not state:
        raise ValueError(f"{source} 里没有臂或手的状态")
    length = min(int(np.asarray(value).shape[0]) for value in state.values())
    if length < 2:
        raise ValueError("至少要有两帧才能把下一帧定义成动作")
    with h5py.File(dest, "w") as handle:
        handle.attrs["instruction"] = str(instruction)
        handle.attrs["frequency"] = float(frequency)
        handle.attrs["data_format_version"] = "v1.0"
        state_group = handle.create_group("state")
        action_group = handle.create_group("action")
        for key, value in state.items():
            series = np.asarray(value)[:length]
            state_group.create_dataset(key, data=series[:-1])
            action_group.create_dataset(key, data=series[1:])
    return dest


def load_actions(path) -> list[dict]:
    """回放用的动作。优先读 xspark 的 action，否则用原始关节。"""
    path = Path(path)
    with h5py.File(path, "r") as handle:
        if "action" in handle:
            keys = list(handle["action"].keys())
            count = int(np.asarray(handle["action"][keys[0]]).shape[0]) if keys else 0
            actions = []
            for index in range(count):
                action = {}
                for key in keys:
                    row = np.asarray(handle["action"][key][index])
                    if key.endswith("_arm_joint_state"):
                        action[key[: -len("_state")]] = row.tolist()
                    elif key.endswith("_ee_pose"):
                        action[key] = row.tolist()
                    elif key.endswith("_ee_joint_state"):
                        action[key] = row.tolist()
                    else:
                        action[key] = row.tolist()
                actions.append(action)
            return actions
        actions = []
        arms = [name for name in handle.keys() if name.endswith("_arm")]
        if not arms:
            raise ValueError(f"{path} 里没有可回放的动作")
        count = int(np.asarray(handle[arms[0]]["joint"]).shape[0])
        for index in range(count):
            action = {}
            for name in arms:
                side = _side(name)
                action[f"{side}_arm_joint"] = np.asarray(handle[name]["joint"][index]).tolist()
                if "gripper" in handle[name]:
                    action[f"{side}_ee_joint_state"] = np.asarray(handle[name]["gripper"][index]).tolist()
            actions.append(action)
        return actions


def _side(name: str) -> str | None:
    if name.startswith("left"):
        return "left"
    if name.startswith("right"):
        return "right"
    return None
