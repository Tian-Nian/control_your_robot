import sys
sys.path.append("src/")

import os
from data.collect_any import CollectAny
from data.generate_lerobot import MyLerobotDataset
import h5py
from utils.data_handler import *

'''
Single-arm lerobot, simulated in libero format. The default robot arm has 6 degrees of freedom plus 1 gripper degree of freedom. 
If your robot arm has a different number of degrees of freedom, please modify accordingly.

features={
    "image": {
        "dtype": "image",
        "shape": (3, 480, 640),
        "names": [
            "channels",
            "height",
            "width",
        ],
    },
    "wrist_image": {
        "dtype": "image",
        "shape": (3, 480, 640),
        "names": [
            "channels",
            "height",
            "width",
        ],
    },
    "state": {
        "dtype": "float32",
        "shape": (7,),
        "names": ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","gripper"],
    },
    "actions": {
        "dtype": "float32",
        "shape": (7,),
        "names": ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","gripper"],
    },
}

Dual-arm lerobot

features={
    "observation.images.cam_high": {
        "dtype": "image",
        "shape": (3, 480, 640),
        "names": [
        "channels",
        "height",
        "width",
    ],
    },
    "observation.images.cam_left_wrist": {
        "dtype": "image",
        "shape": (3, 480, 640),
        "names": [
        "channels",
        "height",
        "width",
    ],
    },
    "observation.images.cam_right_wrist": {
        "dtype": "image",
        "shape": (3, 480, 640),
        "names": [
        "channels",
        "height",
        "width",
    ],
    },
    "observation.state": { # 这里的state使用joint, 因为openpi是用joint
        "dtype": "float32",
        "shape": (14,),
        "names": ["l1,l2,l3,l4,l5,l6,gl,r1,r2,r3,r4,r5,r6,gr"],
    },
    "action": {
        "dtype": "float32",
        "shape": (14,),
        "names": ["l1,l2,l3,l4,l5,l6,gl,r1,r2,r3,r4,r5,r6,gr"],
    },
}
'''

if __name__== '__main__':
    features={
        "image": {
            "dtype": "image",
            "shape": (3, 480, 640),
            "names": [
                "channels",
                "height",
                "width",
            ],
        },
        "wrist_image": {
            "dtype": "image",
            "shape": (3, 480, 640),
            "names": [
                "channels",
                "height",
                "width",
            ],
        },
        "state": {
            "dtype": "float64",
            "shape": (7,),
            "names": ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","gripper"],
        },
        "actions": {
            "dtype": "float64",
            "shape": (7,),
            "names": ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","gripper"],
        },
    }

    feature_map = {
        "image": "cam_head.color",
        "wrist_image": "cam_wrist.color",
        "state": ["left_arm.joint","left_arm.gripper"],
        "actions": ["left_arm.joint","left_arm.gripper"],
    }
    
    import argparse
    import json
    parser = argparse.ArgumentParser(description='Transform datasets typr to HDF5.')
    parser.add_argument('data_path', type=str,
                        help="raw data path")
    parser.add_argument('repo_id', type=str,
                        help='repo_id should be a string, lerobotdataset default be aved at ~/.huggingface/lerobot/')
    parser.add_argument('multi', typr=bool,default=False,
                        help="if you are converting a multi-task dataset, please set this to true and set data_path to the root directory of the multi-task dataset.")
    args = parser.parse_args()
    data_path = args.data_path
    repo_id = args.repo_id
    multi = args.multi
    hdf5_paths = get_files(data_path, "*.hdf5")
    
    if not multi:
        data_config = json.load(os.path.join(data_path, "config.json"))
        inst_path = f"./task_instructions/{data_config["task_name"]}.json"
    else:
        inst_path = None
    lerobot = MyLerobotDataset(repo_id, "piper", 10 ,features, feature_map, inst_path)

    for hdf5_path in hdf5_paths:
        data = hdf5_groups_to_dict(hdf5_path)
        if multi:
            # for every episode, reset instruction
            data_config = json.load(os.path.join(hdf5_path, "../config.json"))
            inst_path = f"./task_instructions/{data_config["task_name"]}.json"
            lerobot.write(data, inst_path)
        else:
            lerobot.write(data)