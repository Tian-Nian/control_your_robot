import sys
sys.path.append("src/")

import numpy as np
import json
import time

from utils.data_handler import debug_print

class TestModel:
    def __init__(self,model_path, task_name, DoFs=6, is_dual=True, INFO="DEBUG"):
        self.model_path = model_path
        self.task_name = task_name
        self.INFO = INFO

        self.is_dual = is_dual

        self.DoFs = DoFs

        debug_print("model", "loading model success", self.INFO)

        self.img_size = (224,224)
        self.observation_window = None
        self.random_set_language()

    # set img_size
    def set_img_size(self,img_size):
        self.img_size = img_size
    
    # set language randomly
    def random_set_language(self):
        json_Path =f"task_instructions/{self.task_name}.json"
        with open(json_Path, 'r') as f_instr:
            instruction_dict = json.load(f_instr)
        instructions = instruction_dict['instructions']
        instruction = np.random.choice(instructions)
        self.instruction = instruction
        debug_print("model",f"successfully set instruction:{instruction}",self.INFO)
    
    # Update the observation window buffer
    def update_observation_window(self, img_arr, state):
        img_front, img_right, img_left, _ = img_arr[0], img_arr[1], img_arr[2], state
        img_front = np.transpose(img_front, (2, 0, 1))
        img_right = np.transpose(img_right, (2, 0, 1))
        img_left = np.transpose(img_left, (2, 0, 1))

        self.observation_window = {
            "state": state,
            "images": {
                "cam_high": img_front,
                "cam_left_wrist": img_left,
                "cam_right_wrist": img_right,
            },
            "prompt": self.instruction,
        }

        if self.is_dual:
            if state.shape[0] != 2 * (self.DoFs + 1):
                debug_print("model",f"dual arm infer model iput dim should be 2*(DoFs + 1),now DoFs={self.DoFs}, but got dim {state.shape[0]}","ERROR")
        else:
            if state.shape[0] != (self.DoFs + 1):
                debug_print("model",f"single arm infer model iput dim should be (DoFs + 1),now DoFs={self.DoFs}, but got dim {state.shape[0]}","ERROR")
        debug_print("model",f"update observation windows success", self.INFO)
        
    def get_action(self):
        horizon = 3
        assert (self.observation_window is not None), "update observation_window first!"
        if self.is_dual:
            action = np.concatenate([
                np.random.rand(horizon, self.DoFs) * 3.1515926,  # 第一条手臂的关节
                np.random.rand(horizon, 1),                      # 第一条手臂的夹爪
                np.random.rand(horizon, self.DoFs) * 3.1515926,  # 第二条手臂的关节
                np.random.rand(horizon, 1)                       # 第二条手臂的夹爪
            ], axis=1)
        else:
            action = np.concatenate([
                np.random.rand(horizon, self.DoFs) * 3.1515926,  # 第一条手臂的关节
                np.random.rand(horizon, 1)])
            
        time.sleep(np.random.rand()/10)

        debug_print("model",f"infer action success", self.INFO)
        return action

    def reset_obsrvationwindows(self):
        self.instruction = None
        self.observation_window = None
        debug_print("model",f"successfully unset obs and language intruction",self.INFO)

if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "INFO"
    
    DoFs = 14
    model = TestModel("test",DoFs=DoFs, is_dual=True)
    height = 480
    width = 640
    img_arr = [np.random.randint(0, 256, size=(height, width, 3), \
                                 dtype=np.uint8), np.random.randint(0, 256, size=(height, width, 3), \
                                dtype=np.uint8), np.random.randint(0, 256, size=(height, width, 3), dtype=np.uint8)]
    state = np.random.rand(DoFs) * 3.1515926
    model.update_observation_window(img_arr, state)
    model.get_action()
    model.reset_obsrvationwindows()
