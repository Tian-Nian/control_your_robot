
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
import numpy as np


import cv2
from PIL import Image

import pickle
from experiments.robot.libero.run_libero_eval import GenerateConfig
from experiments.robot.openvla_utils import get_action_head, get_processor, get_proprio_projector, get_vla, get_vla_action
from prismatic.vla.constants import NUM_ACTIONS_CHUNK, PROPRIO_DIM

class OPENVLA_OFT_ALOHA:
    def __init__(self, model_path, task_name):
        self.task_name = task_name

        cfg = GenerateConfig(
            pretrained_checkpoint = f"policy/openvla-oft/checkpoints/{model_path}",
            use_l1_regression = True,
            use_diffusion = False,
            use_film = False,
            num_images_in_input = 3,
            use_proprio = True,
            load_in_8bit = False,
            load_in_4bit = False,
            center_crop = True,
            num_open_loop_steps = NUM_ACTIONS_CHUNK,
            unnorm_key = "libero_spatial_no_noops",
        )
        self.vla = get_vla(cfg)
        self.processor = get_processor(cfg) 
        
        # Load MLP action head to generate continuous actions (via L1 regression)
        self.action_head = get_action_head(cfg, llm_dim=vla.llm_dim)

        # Load proprio projector to map proprio to language embedding space
        self.proprio_projector = get_proprio_projector(cfg, llm_dim=vla.llm_dim, proprio_dim=PROPRIO_DIM)

        self.img_size = (256,256)
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
        # print(state)

    def get_action(self):
        assert (self.observation_window is not None), "update observation_window first!"
        return self.policy.infer(self.observation_window)["actions"]

    def reset_obsrvationwindows(self):
        self.instruction = None
        self.observation_window = None
        print("successfully unset obs and language intruction")

class OPENVLA_OFT_LIBERO:
    def __init__(self, model_path, task_name):
        self.task_name = task_name

        cfg = GenerateConfig(
            pretrained_checkpoint = f"policy/openvla-oft/checkpoints/{model_path}",
            use_l1_regression = True,
            use_diffusion = False,
            use_film = False,
            num_images_in_input = 2,
            use_proprio = True,
            load_in_8bit = False,
            load_in_4bit = False,
            center_crop = True,
            num_open_loop_steps = NUM_ACTIONS_CHUNK,
            unnorm_key = "libero_spatial_no_noops",
        )
        self.vla = get_vla(cfg)
        self.processor = get_processor(cfg) 
        
        # Load MLP action head to generate continuous actions (via L1 regression)
        self.action_head = get_action_head(cfg, llm_dim=vla.llm_dim)

        # Load proprio projector to map proprio to language embedding space
        self.proprio_projector = get_proprio_projector(cfg, llm_dim=vla.llm_dim, proprio_dim=PROPRIO_DIM)

        self.img_size = (256,256)
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
        img_front, img_wrist, puppet_arm = img_arr[0], img_arr[1], state
        img_front = np.transpose(img_front, (2, 0, 1))
        img_wrist = np.transpose(img_wrist, (2, 0, 1))

        self.observation_window = {
            "state": state,
            "images": {
                "cam_high": img_front,
                "cam_lwrist": img_wrist,
            },
            "prompt": self.instruction,
        }
        # print(state)

    def get_action(self):
        assert (self.observation_window is not None), "update observation_window first!"
        return self.policy.infer(self.observation_window)["actions"]

    def reset_obsrvationwindows(self):
        self.instruction = None
        self.observation_window = None
        print("successfully unset obs and language intruction")

