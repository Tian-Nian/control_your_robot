"""XPolicyLab 看到的四个方法。写入走最新目标，节拍在这里睡。"""

from __future__ import annotations

import importlib
import time

import numpy as np

from robot.harness.contract import ActionContract
from robot.harness.harness import RobotHarness
from robot.runtime.lease import ControlLease


class RuntimeEnv:
    def __init__(
        self,
        session,
        *,
        mode: str = "joint",
        control_hz: float = 25.0,
        instruction: str = "",
        max_steps: int = 0,
        lease_s: float = 2.0,
        role: str = "model",
    ):
        self.session = session
        self.harness = RobotHarness(session, mode=mode, control_hz=control_hz)
        self.contract = ActionContract(mode=mode, control_hz=control_hz)
        self.control_hz = float(control_hz)
        self.instruction = str(instruction)
        self.max_steps = int(max_steps)
        self.steps = 0
        self._stop = False
        self.lease = ControlLease(session, role, lease_s)

    def get_obs(self) -> dict:
        raw = self.harness.get_obs()
        return {
            "state": _state(raw),
            "vision": _vision(raw),
            "instruction": self.instruction,
            "additional_info": {"frequency": self.control_hz},
            "data_format_version": "v1.0",
            "skew_ms": raw.get("skew_ms"),
            "within_skew": raw.get("within_skew"),
        }

    def take_action(self, action) -> dict:
        if self.is_episode_end():
            raise RuntimeError("episode 已结束，不再接受动作")
        if not self.lease.alive:
            raise RuntimeError("部署控制锁已过期")
        commands = self.contract.commands(_contract_action(action))
        results = {}
        for name, payload in commands.items():
            results[name] = self.session.write_target(name, payload)
            if not results[name]["accepted"]:
                raise RuntimeError(results[name]["error"] or f"{name} 没有接受目标")
        self.steps += 1
        time.sleep(self.contract.period_s)
        return results

    def is_episode_end(self) -> bool:
        if self._stop:
            return True
        return self.max_steps > 0 and self.steps >= self.max_steps

    def stop(self) -> None:
        self._stop = True

    def reset(self) -> dict:
        """策略循环不会调用这里。进入 eval_one_episode 之前由部署客户端复位。"""
        results = {}
        for item in self.session.describe():
            if not item["accepts_commands"] or not str(item["name"]).endswith("_arm"):
                continue
            results[item["name"]] = self.session.reset(item["name"])
        self.steps = 0
        self._stop = False
        return results

    def close(self) -> None:
        self.lease.close()


def run_policy(env: RuntimeEnv, model_client, policy_name: str):
    env.reset()
    module = importlib.import_module(f"XPolicyLab.policy.{policy_name}.deploy")
    return module.eval_one_episode(env, model_client)


def replay(env: RuntimeEnv, actions: list[dict]) -> int:
    count = 0
    for action in actions:
        if env.is_episode_end():
            break
        env.take_action(action)
        count += 1
    return count


def _contract_action(action) -> dict:
    """标量末端关节是夹爪。20 维留给手，不在这里改。"""
    if not isinstance(action, dict):
        raise TypeError("动作必须是映射")
    out = dict(action)
    for side in ("left", "right"):
        joint_key = f"{side}_arm_joint"
        if f"{side}_arm_joint_state" in action and joint_key not in out:
            out[joint_key] = action[f"{side}_arm_joint_state"]
        grip = action.get(f"{side}_ee_joint_state")
        if grip is None:
            continue
        vector = np.asarray(grip, dtype=float).reshape(-1)
        if vector.size != 1:
            continue
        out.pop(f"{side}_ee_joint_state", None)
        gripper = float(vector[0])
        arm_name = f"{side}_arm"
        current = out.get(arm_name)
        arm = dict(current) if isinstance(current, dict) else {}
        arm["gripper"] = gripper
        if joint_key in out and "joint" not in arm:
            arm["joint"] = out[joint_key]
        pose_key = f"{side}_ee_pose"
        if pose_key in out and "eef" not in arm:
            arm["eef"] = out[pose_key]
        out[arm_name] = arm
    return out


def _state(obs: dict) -> dict:
    state = {}
    components = obs.get("components") or {}
    for side in ("left", "right"):
        arm = components.get(f"{side}_arm")
        data = None if not arm else arm.get("data")
        if isinstance(data, dict):
            if data.get("joint") is not None:
                state[f"{side}_arm_joint_state"] = _plain(data["joint"])
            if data.get("eef") is not None:
                state[f"{side}_ee_pose"] = _plain(data["eef"])
            if data.get("gripper") is not None:
                state[f"{side}_ee_joint_state"] = _plain(data["gripper"])
        hand = components.get(f"{side}_hand")
        hand_data = None if not hand else hand.get("data")
        if isinstance(hand_data, dict) and hand_data.get("joint") is not None:
            state[f"{side}_ee_joint_state"] = _plain(hand_data["joint"])
    return state


def _vision(obs: dict) -> dict:
    vision = {}
    for name, component in (obs.get("components") or {}).items():
        data = None if not component else component.get("data")
        if not isinstance(data, dict) or "color" not in data:
            continue
        vision[name] = {"color": _jpeg(data["color"]), "timestamp_ns": component.get("timestamp_ns")}
    return vision


def _jpeg(color) -> bytes:
    from robot.data.rgb_jpeg import encode_rgb_jpeg

    return encode_rgb_jpeg(color)


def _plain(value):
    if hasattr(value, "tolist"):
        return value.tolist()
    return value
