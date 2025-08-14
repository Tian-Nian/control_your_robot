import sys
sys.path.append("./")

import numpy as np

from controller.controller import Controller
from utils.data_handler import debug_print
from typing import Dict, Any

class ArmController(Controller):
    def __init__(self):
        super().__init__()
        self.name = "arm_controller"
        self.controller = None
        self.controller_type = "robotic_arm"

    def get_information(self):
        arm_info = {}
        state = self.get_state().copy()
        if "joint" in self.collect_info:
            arm_info["joint"] = state["joint"]
        if "qpos" in self.collect_info:
            arm_info["qpos"] = state["qpos"]
        if "gripper" in self.collect_info:
            arm_info["gripper"] = state["gripper"]
        if "action" in self.collect_info:
            arm_info["action"] = state["action"]
        if "velocity" in self.collect_info:
            arm_info["velocity"] = state["velocity"]
        if "force" in self.collect_info:
            arm_info["force"] = state["force"]
        return arm_info.copy()
    
    def move_controller(self, move_data:Dict[str, Any], is_delta=False):
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

    def __repr__(self):
        if self.controller is not None:
            return f"{self.name}: \n \
                    controller: {self.controller}"
        else:
            return super().__repr__()