import sys
sys.path.append("./")

import h5py
import numpy as np
import os
from tqdm import tqdm

from utils.data_handler import hdf5_groups_to_dict, get_files, get_item

import cv2
map = {
    "cam_high": "cam_head.color",
    "cam_left_wrist": "cam_left_wrist.color",
    "cam_right_wrist": "cam_right_wrist.color",
    "qpos": ["left_arm.joint","left_arm.gripper","right_arm.joint","right_arm.gripper"],
    "action": ["left_arm.joint","left_arm.gripper","right_arm.joint","right_arm.gripper"],
}
hdf5_path = "save/Make_a_beef_sandwichv3/0.hdf5"

data = hdf5_groups_to_dict(hdf5_path)

data.pop("cam_head", None)
data.pop("cam_left_wrist", None)
data.pop("cam_right_wrist", None)

with h5py.File("./tarj.hdf5", "w") as f:
    obs = f
    for controller_name in data.keys():
        controller_group = obs.create_group(controller_name)
        for item in data[controller_name].keys():
            print(f"{controller_name}, {item}")
            now_data = get_item(data, [f"{controller_name}.{item}"])
            controller_group.create_dataset(item, data=now_data)
