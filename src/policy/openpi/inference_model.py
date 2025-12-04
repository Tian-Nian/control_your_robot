
#!/home/lin/software/miniconda3/envs/aloha/bin/python
# -- coding: UTF-8
"""
#!/usr/bin/python3
"""
from pathlib import Path

# get current workspace
current_file = Path(__file__)

import sys
parent_dir = current_file.parent
sys.path.append(str(parent_dir))

import json
import sys
import jax
import numpy as np
from openpi.models import model as _model
from openpi.policies import aloha_policy
from openpi.policies import policy_config as _policy_config
from openpi.shared import download
from openpi.training import config as _config
from openpi.training import data_loader as _data_loader
import os
import cv2
from PIL import Image

from openpi.models import model as _model
from openpi.policies import policy_config as _policy_config
from openpi.shared import download
from openpi.training import config as _config
from openpi.training import data_loader as _data_loader

class PI0_DUAL:
    # def __init__(self, task_name,train_config_name,model_name,checkpoint_id):
    def __init__(self, model_path, task_name):
        self.task_name = task_name

        train_config_name = "pi0_base_aloha_robotwin_lora"
        config = _config.get_config(train_config_name)
        print("get config success!")
        self.policy = _policy_config.create_trained_policy(config, model_path)
        print("loading model success!")
        self.img_size = (224,224)
        self.observation_window = None
        self.random_set_language()

    # set img_size
    def set_img_size(self,img_size):
        self.img_size = img_size
    
    # set language randomly
    def random_set_language(self):
        # json_Path =f"datasets/instructions/{self.task_name}.json"
        root_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        json_Path = os.path.join(root_dir, "task_instructions", f"{self.task_name}.json")
        with open(json_Path, 'r') as f_instr:
            instruction_dict = json.load(f_instr)
        instructions = instruction_dict['instructions']
        instruction = np.random.choice(instructions)
        self.instruction = instruction
        print(f"successfully set instruction:{instruction}")
    
    # Update the observation window buffer
    def update_observation_window(self, img_arr, state):
        img_front, img_right, img_left, puppet_arm = img_arr[0], img_arr[1], img_arr[2], state
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

    def get_action(self):
        assert (self.observation_window is not None), "update observation_window first!"
        return self.policy.infer(self.observation_window)["actions"]

    def reset_obsrvationwindows(self):
        self.instruction = None
        self.observation_window = None
        print("successfully unset obs and language intruction")

class PI0_SINGLE:
    def __init__(self, task_name,train_config_name,model_name,checkpoint_id):
        self.train_config_name = train_config_name
        self.task_name = task_name
        self.model_name = model_name
        self.checkpoint_id = checkpoint_id

        config = _config.get_config(self.train_config_name)
        self.policy = _policy_config.create_trained_policy(config, f"policy/openpi/checkpoints/{self.train_config_name}/{self.model_name}/{self.checkpoint_id}")
        print("loading model success!")
        self.img_size = (224,224)
        self.observation_window = None
        self.random_set_language()

    # set img_size
    def set_img_size(self,img_size):
        self.img_size = img_size
    
    # set language randomly
    def random_set_language(self):
        json_Path =f"datasets/instructions/{self.task_name}.json"
        with open(json_Path, 'r') as f_instr:
            instruction_dict = json.load(f_instr)
        instructions = instruction_dict['instructions']
        instruction = np.random.choice(instructions)
        self.instruction = instruction
        print(f"successfully set instruction:{instruction}")
    
    # Update the observation window buffer
    def update_observation_window(self, img_arr, state):
        img_front, img_right = img_arr[0], img_arr[1]
        # (480,640,3) -> (3,480,640)
        img_front = np.transpose(img_front, (2, 0, 1))
        img_right = np.transpose(img_right, (2, 0, 1))
        img_left = np.zeros_like(img_front)
        state = np.pad(state, (0, 8), mode='constant', constant_values=0)
        self.observation_window = {
            "state": state,
            "images": {
                "cam_high": img_front,
                "cam_left_wrist": img_left,
                "cam_right_wrist": img_right,
            },
            "prompt": self.instruction,
        }

    def get_action(self):
        assert (self.observation_window is not None), "update observation_window first!"
        return self.policy.infer(self.observation_window)["actions"][:,:8]

    def reset_obsrvationwindows(self):
        self.instruction = None
        self.observation_window = None
        print("successfully unset obs and language intruction")