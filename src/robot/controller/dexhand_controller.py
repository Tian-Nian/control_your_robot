

from robot.controller.controller import Controller
from typing import Dict, Any
import numpy as np

class DexHandController(Controller):
    def __init__(self):
        super().__init__()
        self.controller_type = "robotic_hand"
        self.is_set_up = False
        self.controller = None
    
    def get_information(self):
        hand_info = {}
        if "joint" in self.collect_info:
            hand_info["joint"] = self.get_joint()
        if "action" in self.collect_info:
            hand_info["action"] = self.get_action()
        if "velocity" in self.collect_info:
            hand_info["velocity"] = self.get_velocity()
        if "force" in self.collect_info:
            hand_info["force"] = self.get_force()
        return hand_info
    
    def move(self, move_data:Dict[str, Any],is_delta=False):
        if is_delta:
            now_state = self.get_state()
            for key, value in move_data.items():
                if key == "joint":
                    self.set_joint(now_state["joint"] + value)
                if key == "action":
                    self.set_action(now_state["action"] + value)
        else:
            for key, value in move_data.items():
                if key == "joint":
                    self.set_joint(value)
                if key == "action":
                    self.set_action(value)

    def set_joint(self, joint):
        pass

    def apply_reset(self, start_state):
        current = self._reset_joint()
        if current is None:
            target = np.zeros(6) if start_state is None else np.asarray(start_state, dtype=float)
            self.set_joint(target)
            return
        self.interpolate_set_joint(current, start_state)

    def _reset_joint(self):
        state_fn = getattr(self, "get_state", None)
        if callable(state_fn):
            state = state_fn()
            if isinstance(state, dict) and state.get("joint") is not None:
                return state["joint"]
        joint_fn = getattr(self, "get_joint", None)
        if callable(joint_fn):
            try:
                return joint_fn()
            except Exception:
                return None
        return None

    def __repr__(self):
        if self.controller is not None:
            return f"{self.name}: \n \
                    controller: {self.controller}"
        else:
            return super().__repr__()
    
    
        

