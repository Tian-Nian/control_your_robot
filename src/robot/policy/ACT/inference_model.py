

import numpy as np
import json
import time
import math
# 修改导入路径，使用正确的相对路径
from robot.utils.base.data_handler import debug_print

from robot.policy.ACT.act_policy import ACT
            
import yaml

from argparse import Namespace
from my_robot.agilex_piper_single_base import PiperSingle

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
        # right_cam = img_arr[2]
        head_cam = np.moveaxis(head_cam, -1, 0) / 255.0
        left_cam = np.moveaxis(left_cam, -1, 0) / 255.0
        # right_cam = np.moveaxis(right_cam, -1, 0) / 255.0
        qpos = state
        self.observation_window = {
                "cam_high": head_cam,
                "cam_wrist": left_cam,
                # "right_cam": right_cam,
                "qpos": qpos,
            }
        
    def get_action(self):
        action = self.model.get_action(self.observation_window)
        debug_print("model",f"infer action success", self.INFO)
        return action

    def reset_obsrvationwindows(self):
        self.instruction = None
        self.observation_window = None
        debug_print("model",f"successfully unset obs and language intruction",self.INFO)

def input_transform(data):
    
    # 检测机械臂维度
    has_left_arm = "left_arm" in data[0]
    has_right_arm = "right_arm" in data[0]
    
    # 如果只有left_arm，补充right_arm数据（全为0.0）
    if has_left_arm and not has_right_arm:
        # 获取left_arm的关节数和夹爪数
        left_joint_dim = len(data[0]["left_arm"]["joint"])
        left_gripper_dim = 1
        
        # 补充right_arm数据
        data[0]["right_arm"] = {
            "joint": [0.0] * left_joint_dim,
            "gripper": [0.0] * left_gripper_dim
        }
        has_right_arm = True
    
    # 如果只有right_arm，补充left_arm数据（全为0.0）
    elif has_right_arm and not has_left_arm:
        # 获取right_arm的关节数和夹爪数
        right_joint_dim = len(data[0]["right_arm"]["joint"])
        right_gripper_dim = 1
        
        # 补充left_arm数据
        data[0]["left_arm"] = {
            "joint": [0.0] * right_joint_dim,
            "gripper": [0.0] * right_gripper_dim
        }
        has_left_arm = True
    
    # 如果都没有，创建默认的双臂数据
    elif not has_left_arm and not has_right_arm:
        # 默认6关节 + 1夹爪
        default_joint_dim = 6
        default_gripper_dim = 1
        
        data[0]["left_arm"] = {
            "joint": [0.0] * default_joint_dim,
            "gripper": 0.0
        }
        data[0]["right_arm"] = {
            "joint": [0.0] * default_joint_dim,
            "gripper": 0.0
        }
        has_left_arm = True
        has_right_arm = True
    
    state = np.concatenate([
        np.array(data[0]["left_arm"]["joint"]).reshape(-1),
        np.array(data[0]["left_arm"]["gripper"]).reshape(-1),
        np.array(data[0]["right_arm"]["joint"]).reshape(-1),
        np.array(data[0]["right_arm"]["gripper"]).reshape(-1)
    ])
    img_arr = data[1]["cam_head"]["color"], data[1]["cam_wrist"]["color"]
    return img_arr, state

def output_transform(data):
    joint_limits_rad = [
        (math.radians(-150), math.radians(150)),   # joint1
        (math.radians(0), math.radians(180)),    # joint2
        (math.radians(-170), math.radians(0)),   # joint3
        (math.radians(-100), math.radians(100)),   # joint4
        (math.radians(-70), math.radians(70)),   # joint5
        (math.radians(-120), math.radians(120))    # joint6
        ]
    def clamp(value, min_val, max_val):
        """将值限制在[min_val, max_val]范围内"""
        return max(min_val, min(value, max_val))
    left_joints = [
        clamp(data[i], joint_limits_rad[i][0], joint_limits_rad[i][1])
        for i in range(6)
    ]
    left_gripper = data[6]
    
    move_data = {
        "left_arm": {
            "joint": left_joints,
            "gripper": left_gripper
        }
    }
    return move_data
if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "DEBUG"
    robot = PiperSingle()
    robot.set_up()
    robot.reset()
    data = robot.get()
    img_arr, state = input_transform(data)
    
    model = MYACT("/home/usst/kwj/GitCode/control_your_robot_jie/policy/ACT/act_ckpt/act-pick_place_cup/50","act-pick_place_cup")
    
    model.update_observation_window(img_arr, state)
    model.get_action()
    model.reset_obsrvationwindows()
