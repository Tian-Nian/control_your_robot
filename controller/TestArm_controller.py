import sys
sys.path.append("./")

from controller.arm_controller import ArmController

import numpy as np
import time

from utils.data_handler import debug_print

class TestArmController(ArmController):
    def __init__(self, name, DoFs=6,INFO="DEBUG"):
        super().__init__()
        self.name = name
        self.controller_type = "user_controller"
        self.controller = None
        self.INFO = INFO
        self.DoFs = DoFs
    
    def set_up(self, input=None):
        debug_print(self.name, f"setup success",self.INFO)

    def reset(self, start_state):
        if start_state.shape[0] == self.DoFs:
            debug_print(self.name, f"reset success, to start state \n {start_state}",self.INFO)
        else:
            debug_print(self.name, f"reset() input should be joint controll which dim is {self.DoFs}","ERROR")

    def get_state(self):
        state = {}
        # randly return a vaild value  
        state["joint"] = np.random.rand(self.DoFs) * 3.1515926
        state["qpos"] = np.random.rand(6)
        state["gripper"] = np.random.rand(1)
        debug_print(self.name, f"get state to \n {state}", self.INFO)
        return state

    def set_position(self, position):
        if position.shape[0] == 6:
            debug_print(self.name, f"using EULER set position to \n {position}", self.INFO)
        elif position.shape[0] == 7:
            debug_print(self.name, f"using QUATERNION set position to \n {position}", self.INFO)
        else:
            debug_print(self.name, f"set_position input size should be 6 -> EULER or 7 -> QUATERNION","ERROR")
    
    def set_joint(self, joint):
        if joint.shape[0] != self.DoFs:
            debug_print(self.name, f"set_joint() input size should be {self.DoFs}","ERROR")   
        else: 
            debug_print(self.name, f"set joint to \n {joint}", self.INFO)

    # The input gripper value is in the range [0, 1], representing the degree of opening.
    def set_gripper(self, gripper):
        if isinstance(gripper, (int, float, complex,np.ndarray)) and not isinstance(gripper, bool):
            if 1> gripper > 0:
                debug_print(self.name, f"set gripper to {gripper}", self.INFO)
            else:
                debug_print(self.name, f"gripper better be 0~1, but get number {gripper}","WARNING")
        else:
            debug_print(self.name, f"gripper should be a number 0~1, but get type {type(gripper)}","ERROR")
    
    def __del__(self):
        try:
            if hasattr(self, 'controller'):
                # Add any necessary cleanup for the arm controller
                pass
        except:
            pass

if __name__=="__main__":
    import os
    os.environ["INFO_LEVEL"] = "DEBUG"
    
    controller = TestArmController("test_arm",DoFs=6,INFO="DEBUG")

    controller.set_collect_info(["joint","qpos","gripper"])

    controller.set_up()

    controller.get_state()

    controller.set_gripper(0.2)

    controller.set_joint(np.array([0.1,0.1,-0.2,0.3,-0.2,0.5]))
    time.sleep(0.1)

    controller.set_position(np.array([0.057, 0.0, 0.260, 0.0, 0.085, 0.0]))
    time.sleep(0.1)

    controller.reset(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

    controller.get()

    move_data = {
        "joint":np.random.rand(6) * 3.1515926,
        "gripper":0.2
    }
    controller.move(move_data)