import sys
sys.path.append("./")

import numpy as np
import json
import time

from utils.data_handler import debug_print

from policy.ACT.act_policy import ACT
            
import yaml

from argparse import Namespace

class MYACT:
    def __init__(self,model_path, task_name,INFO="DEBUG"):
        with open("policy/ACT/deploy_policy.yml", "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)
        self.args = config  
        # override
        self.args["ckpt_dir"] = model_path
        self.args["task_name"] = task_name
        self.args["left_arm_dim"] = 6
        self.args["right_arm_dim"] = 6

        self.model_path = model_path
        self.task_name = task_name
        self.INFO = INFO
        
        self.model = ACT(self.args, Namespace(**self.args))

        debug_print("model", "loading model success", self.INFO)

        self.img_size = (640,480)
        self.observation_window = None
        self.random_set_language()

    # set img_size
    def set_img_size(self,img_size):
        self.img_size = img_size
    
    # set language randomly
    def random_set_language(self):
        self.observation_window = None
        return
    
    # Update the observation window buffer
    def update_observation_window(self, img_arr, state):
        head_cam = img_arr[0]
        left_cam = img_arr[1]
        right_cam = img_arr[2]
        head_cam = np.moveaxis(head_cam, -1, 0) / 255.0
        left_cam = np.moveaxis(left_cam, -1, 0) / 255.0
        right_cam = np.moveaxis(right_cam, -1, 0) / 255.0
        qpos = state
        self.observation_window = {
                "head_cam": head_cam,
                "left_cam": left_cam,
                "right_cam": right_cam,
                "qpos": qpos,
            }
        
    def get_action(self):
        action = self.model.get_action(self.observation_window)
        # print(action)
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
    model = MYACT("/home/agilex/project/control_your_robot/policy/ACT/act_ckpt/act-Make_a_beef_sandwich/Make_a_beef_sandwich-50","act-Make_a_beef_sandwich")
    height = 480
    width = 640
    img_arr = [np.random.randint(0, 256, size=(height, width, 3), \
                                 dtype=np.uint8), np.random.randint(0, 256, size=(height, width, 3), \
                                dtype=np.uint8), np.random.randint(0, 256, size=(height, width, 3), dtype=np.uint8)]
    state = np.random.rand(DoFs) * 3.1515926
    model.update_observation_window(img_arr, state)
    model.get_action()
    model.reset_obsrvationwindows()
