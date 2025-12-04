from controller.controller import Controller
from typing import Dict, Any
import numpy as np

class MobileController(Controller):
    def __init__(self):
        super().__init__()
        self.controller_type = "robotic_mobile"
        self.controller = None
    '''
    对于底盘移动,不进行is_delta的判断, 直接进行移动
    '''
    def move_controller(self, move_data:Dict[str, Any], is_delta=False):
        # moving by setting velocity for every joint
        if "move_velocity" in move_data.keys():
            self.set_move_velocity(np.array(move_data["move_velocity"]))
        # moving by set position
        if "move_to" in move_data.keys():
            self.set_move_to(np.array(move_data["move_to"]))
        
    def get_information(self):
        mobile_info = {}
        if "move_velocity" in self.collect_info:
            mobile_info["move_velocity"] = self.get_move_velocity()
        if "position" in self.collect_info:
            mobile_info["position"] = self.get_position() 
        return mobile_info

    def __repr__(self):
        if self.controller is not None:
            return f"{self.name}: \n \
                    controller: {self.controller}"
        else:
            return super().__repr__()
