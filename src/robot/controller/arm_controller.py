

from abc import abstractmethod
from typing import Dict, Any
import numpy as np

from robot.controller.controller import Controller
from robot.utils.base.data_handler import debug_print

class ArmController(Controller):
    def __init__(self):
        super().__init__()
        self.name = "arm_controller"
        self.controller = None
        self.controller_type = "robotic_arm"

    def get_information(self) -> Dict[str, Any]:
        arm_info = {}
        state = self.get_state()
        if "joint" in self.collect_info:
            arm_info["joint"] = state.get("joint")
        if "qpos" in self.collect_info:
            arm_info["qpos"] = state.get("qpos")
        if "gripper" in self.collect_info:
            arm_info["gripper"] = state.get("gripper")
        if "action" in self.collect_info:
            arm_info["action"] = state.get("action")
        if "velocity" in self.collect_info:
            arm_info["velocity"] = state.get("velocity")
        if "force" in self.collect_info:
            arm_info["force"] = state.get("force")
        return arm_info
    
    def move_controller(self, move_data: Dict[str, Any], is_delta=False):
        if is_delta:
            now_state = self.get_state()
            for key, value in move_data.items():
                if key == "joint":
                    self.set_joint(np.array(now_state["joint"] + value))
                elif key == "qpos":
                    self.set_position(np.array(now_state["qpos"] + value))
        else:
            for key, value in move_data.items():
                if key == "joint":
                    self.set_joint(np.array(value))
                elif key == "qpos":
                    self.set_position(np.array(value))
        
        # For action and gripper, use absolute values instead of deltas
        for key, value in move_data.items():
            if key == "teleop_qpos":
                self.set_position_teleop(np.array(value))
            if key == "action":
                self.set_action(np.array(value))
            if key == "gripper":
                self.set_gripper(np.array(value))
            if key == "velocity":
                self.set_velocity(np.array(value))
            if key == "force":
                self.set_force(np.array(value))

    @abstractmethod
    def get_state(self) -> Dict[str, Any]:
        """Get the current state of the arm (joint, qpos, etc.)."""
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

    def set_up(self):
        pass

    def __repr__(self):
        if self.controller is not None:
            return f"{self.name}: \n \
                    controller: {self.controller}"
        else:
            return super().__repr__()