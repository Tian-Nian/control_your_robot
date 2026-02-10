

from robot.controller.controller import Controller
from typing import Dict, Any

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
        
    def __repr__(self):
        if self.controller is not None:
            return f"{self.name}: \n \
                    controller: {self.controller}"
        else:
            return super().__repr__()
    
    
        

