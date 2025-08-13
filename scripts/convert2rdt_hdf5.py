import sys
sys.path.append("./")

import h5py
import numpy as np
import os
from tqdm import tqdm

from utils.data_handler import hdf5_groups_to_dict, get_files, get_item

import cv2

'''
dual-arm:

map = {
    "cam_high": "cam_head.color",
    "cam_left_wrist": "cam_left_wrist.color",
    "cam_right_wrist": "cam_right_wrist.color",
    "qpos": ["left_arm.joint","left_arm.gripper","right_arm.joint","right_arm.gripper"],
    "action": ["left_arm.joint","left_arm.gripper","right_arm.joint","right_arm.gripper"],
}

single-arm:

map = {
    # "cam_high": "cam_head.color",
    "cam_wrist": "cam_wrist.color",
    "qpos": ["left_arm.joint","left_arm.gripper"],
    "action": ["left_arm.joint","left_arm.gripper"],
}
'''

map = {
    "cam_high": "cam_head.color",
    "cam_left_wrist": "cam_left_wrist.color",
    "cam_right_wrist": "cam_right_wrist.color",
    "qpos": ["left_arm.joint","left_arm.gripper","right_arm.joint","right_arm.gripper"],
    "action": ["left_arm.joint","left_arm.gripper","right_arm.joint","right_arm.gripper"],
}

def images_encoding(imgs):
    encode_data = []
    padded_data = []
    max_len = 0
    for i in range(len(imgs)):
        success, encoded_image = cv2.imencode('.jpg', imgs[i])
        jpeg_data = encoded_image.tobytes()
        encode_data.append(jpeg_data)
        max_len = max(max_len, len(jpeg_data))
    # padding
    for i in range(len(imgs)):
        padded_data.append(encode_data[i].ljust(max_len, b'\0'))
    return encode_data, max_len

def convert(hdf5_paths, output_path, start_index=0):
    if not os.path.exists(output_path):
        os.makedirs(output_path)
    
    index = start_index
    for hdf5_path in hdf5_paths:
        data = hdf5_groups_to_dict(hdf5_path)
        
        hdf5_output_path = os.path.join(output_path, f"episode_{index}.hdf5")
        index += 1
        print(data.keys())
        with h5py.File(hdf5_output_path, "w") as f:
            obs = f.create_group("observations")
            '''
            Basic robot arm parameters: if you’re using joint values, 
            you can rename them to avoid confusion instead of calling them qpos, 
            but remember to update the corresponding model’s data loading phase accordingly.
            '''
            qpos = np.array(get_item(data, map["qpos"])).astype(np.float32)
            action = np.array(get_item(data, map["action"])).astype(np.float32)

            obs.create_dataset('qpos', data=np.array(qpos), dtype="float32")
            f.create_dataset('action', data=np.array(action), dtype="float32")

            images = obs.create_group("images")
            
            # Retrieve data based on your camera/view names, then encode and compress it for storage.

            cam_high = get_item(data, map["cam_high"])
            # cam_wrist = get_item(data, map["cam_wrist"])
            cam_left_wrist = get_item(data, map["cam_left_wrist"])
            cam_right_wrist = get_item(data, map["cam_right_wrist"])
            
            head_enc, head_len = images_encoding(cam_high)
            # wrist_enc, wrist_len = images_encoding(cam_wrist)
            left_enc, left_len = images_encoding(cam_left_wrist)
            right_enc, right_len = images_encoding(cam_right_wrist)

            images.create_dataset('cam_high', data=head_enc, dtype=f'S{head_len}')
            # images.create_dataset('cam_wrist', data=wrist_enc, dtype=f'S{wrist_len}')
            images.create_dataset('cam_left_wrist', data=left_enc, dtype=f'S{left_len}')
            images.create_dataset('cam_right_wrist', data=right_enc, dtype=f'S{right_len}')
        
        print(f"convert {hdf5_path} to rdt data format")

if __name__ == "__main__":
    import argparse
    import json
    parser = argparse.ArgumentParser(description='Transform datasets typr to HDF5.')
    parser.add_argument('data_path', type=str,
                        help="your data dir like: datasets/task/")
    parser.add_argument('outout_path', type=str,default=None,
                        help='output path commanded like datasets/RDT/...')
    
    args = parser.parse_args()
    data_path = args.data_path
    output_path = args.outout_path
    if output_path is None:
        data_config = json.load(os.path.join(data_path, "config.json"))
        output_path = f"./datasets/RDT/{data_config['task_name']}"
    
    hdf5_paths = get_files(data_path, "*.hdf5")
    print("hdf5 files:\n",hdf5_paths)
    convert(hdf5_paths, output_path)

