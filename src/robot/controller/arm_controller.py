
from abc import abstractmethod
from typing import Dict, Any
import numpy as np

from robot.controller.controller import Controller

_ARM_MODE_ALIASES = {
    "pos": "position",
    "mit_control": "mit",
    "gravity": "gravity_compensation",
    "imp": "impedance",
    "imp_joint": "impedance",
    "teleop_imp": "impedance",
}


def normalize_arm_mode(mode: str) -> str:
    name = str(mode).strip().lower()
    return _ARM_MODE_ALIASES.get(name, name)


def reject_unsupported_mode(name: str, mode: str, supported) -> str:
    target = normalize_arm_mode(mode)
    allowed = tuple(supported)
    if target not in allowed:
        raise ValueError(f"{name} 不支持 {mode!r}，只有 {list(allowed)}")
    return target


class ArmController(Controller):
    supported_modes = ("position",)

    def __init__(self):
        super().__init__()
        self.name = "arm_controller"
        self.controller = None
        self.controller_type = "robotic_arm"
        self.control_mode = "position"

    def get_information(self) -> Dict[str, Any]:
        arm_info = {}
        state = self.get_state()
        if "joint" in self.collect_info:
            arm_info["joint"] = state.get("joint")
        if "eef" in self.collect_info:
            arm_info["eef"] = state.get("eef")
        if "gripper" in self.collect_info:
            arm_info["gripper"] = state.get("gripper")
        if "action" in self.collect_info:
            arm_info["action"] = state.get("action")
        if "velocity" in self.collect_info:
            arm_info["velocity"] = state.get("velocity")
        if "force" in self.collect_info:
            arm_info["force"] = state.get("force")
        if "control_mode" in self.collect_info:
            arm_info["control_mode"] = state.get("control_mode", self.control_mode)
        if "servo_fault" in self.collect_info:
            arm_info["servo_fault"] = state.get("servo_fault", "")
        return arm_info

    def move_controller(self, move_data: Dict[str, Any], is_delta=False):
        payload = dict(move_data)
        mode = payload.pop("control_mode", None)
        if mode is not None:
            self.set_control_mode(mode)
        if is_delta:
            now_state = self.get_state()
            for key, value in payload.items():
                if key == "joint":
                    self.set_joint(np.array(now_state["joint"] + value))
                elif key == "eef":
                    self.set_position(np.array(now_state["eef"] + value))
        else:
            for key, value in payload.items():
                if key == "joint":
                    self.set_joint(np.array(value))
                elif key == "eef":
                    self.set_position(np.array(value))

        # For action and gripper, use absolute values instead of deltas
        for key, value in payload.items():
            if key == "teleop_eef":
                self.set_position_teleop(np.array(value))
            if key == "action":
                self.set_action(np.array(value))
            if key == "gripper":
                self.set_gripper(np.array(value))
            if key == "velocity":
                self.set_velocity(np.array(value))
            if key == "force":
                self.set_force(np.array(value))

    def set_control_mode(self, mode: str) -> str:
        target = reject_unsupported_mode(self.name, mode, self.supported_modes)
        self.control_mode = target
        return target

    @abstractmethod
    def get_state(self) -> Dict[str, Any]:
        """Get the current state of the arm (joint, eef, etc.)."""
        pass

    def set_joint(self, joint: np.ndarray):
        pass

    def set_position(self, position: np.ndarray):
        pass

    def set_gripper(self, gripper: np.ndarray):
        pass

    def set_action(self, action: np.ndarray):
        pass

    def set_velocity(self, velocity: np.ndarray):
        pass

    def set_force(self, force: np.ndarray):
        pass

    def set_position_teleop(self, position: np.ndarray):
        pass

    def apply_reset(self, start_state):
        """用 set_joint 走一段 smootherstep。先离开阻抗和重力，避免单次关节指令被吞掉。"""
        if "position" in getattr(self, "supported_modes", ()) and getattr(self, "control_mode", "position") != "position":
            self.set_control_mode("position")
        self.interpolate_set_joint(self.get_state()["joint"], start_state)

    def set_up(self):
        pass

    def __repr__(self):
        if self.controller is not None:
            return f"{self.name}: \n \
                    controller: {self.controller}"
        else:
            return super().__repr__()
