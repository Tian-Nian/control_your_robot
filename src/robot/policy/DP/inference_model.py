

import numpy as np
import json
import time
import math
import yaml
from robot.utils.base.data_handler import debug_print
from robot.policy.DP.dp_model import DP

class MYDP:
    def __init__(self, model_path, task_name, INFO="DEBUG"):
        ckpt_file = model_path
        #action_dim = 6+ 6+ 2 # dual arm+2 gripper
        action_dim = 6+6+2 # single arm+1 gripper

        load_config_path = f'./policy/DP/diffusion_policy/config/robot_dp_{action_dim}.yaml'
        with open(load_config_path, "r", encoding="utf-8") as f:
            model_training_config = yaml.safe_load(f)
        
        n_obs_steps = model_training_config['n_obs_steps']
        n_action_steps = model_training_config['n_action_steps']
        self.model = DP(ckpt_file, n_obs_steps, n_action_steps)
        self.INFO = INFO
        debug_print("model",f"load model success", INFO)
        
        self.img_size = (640,480)
        self.observation_window = None
        self.random_set_language()
    # Update the observation window buffer
    def update_observation_window(self, img_arr, state):
        head_cam = img_arr[0]
        # left_cam = img_arr[1]
        # right_cam = img_arr[2]
        head_cam = np.moveaxis(head_cam, -1, 0) / 255.0
        # left_cam = np.moveaxis(left_cam, -1, 0) / 255.0
        # right_cam = np.moveaxis(right_cam, -1, 0) / 255.0
        qpos = state
        self.observation_window =dict(
            head_cam=head_cam,
            # left_cam=left_cam,
            # right_cam=right_cam,
        )
        self.observation_window["agent_pos"] = qpos
    # set language randomly
    def random_set_language(self):
        self.observation_window = None
        return
    def get_action(self, obs):
        action_array = self.model.get_action(obs)
        debug_print("model",f"infer action success", self.INFO)
        return action_array
    
    def reset_obsrvationwindows(self):
        self.instruction = None
        self.observation_window = None
        self.model.reset_obs()
        debug_print("model",f"successfully unset obs and language intruction",self.INFO)
if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "DEBUG"
    DoFs =14 
    height = 480
    width = 640
    img_arr = [np.random.randint(0, 256, size=(height, width, 3), \
                                 dtype=np.uint8), np.random.randint(0, 256, size=(height, width, 3), \
                                dtype=np.uint8), np.random.randint(0, 256, size=(height, width, 3), dtype=np.uint8)]
    state = np.random.rand(DoFs) * 3.1515926

    model = MYDP(model_path="policy/DP/checkpoints/feed_test-100-0/300.ckpt", task_name="feed_test", INFO="DEBUG")

    model.update_observation_window(img_arr, state)
    action = model.get_action(model.observation_window)
    print(action)
    model.reset_obsrvationwindows()