

import h5py
import numpy as np
import os
from tqdm import tqdm

from robot.utils.base.data_handler import hdf5_groups_to_dict, get_files, get_item

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
            # 降采样
            input_data = {}

            for key in map.keys():
                input_data[key] = get_item(data, map[key])[:]

            qpos = np.array(input_data["qpos"]).astype(np.float32)
            
            actions = []

            for i in range(len(qpos) - 1):
                actions.append(qpos[i+1])
            
            last_action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            
            # 最后一帧结束无动作
            actions.append(last_action)

            actions = np.array(actions)
            f.create_dataset('action', data=np.array(actions), dtype="float32")

            obs = f.create_group("observations")
            '''
            Basic robot arm parameters: if you’re using joint values, 
            you can rename them to avoid confusion instead of calling them qpos, 
            but remember to update the corresponding model’s data loading phase accordingly.
            '''

            obs.create_dataset('qpos', data=np.array(qpos), dtype="float32")
            obs.create_dataset("left_arm_dim", data=np.array(6))
            obs.create_dataset("right_arm_dim", data=np.array(6))

            images = obs.create_group("images")
            
            # Retrieve data based on your camera/view names, then encode and compress it for storage.
            def decode(imgs):
                if isinstance(imgs, np.ndarray) and imgs.ndim == 4:
                    return imgs

                imgs_array = []

                for data in imgs:
                    if isinstance(data, (bytes, bytearray)):
                        data = np.frombuffer(data, dtype=np.uint8)

                    img = cv2.imdecode(data, cv2.IMREAD_COLOR)
                    if img is None:
                        raise ValueError("Failed to decode JPEG image")

                    # imgs_array.append(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

                return np.stack(imgs_array, axis=0)


            cam_high = decode(input_data["cam_high"])
            cam_left_wrist = decode(input_data["cam_left_wrist"])
            cam_right_wrist = decode(input_data["cam_right_wrist"])
            
            images.create_dataset("cam_high", data=np.stack(cam_high), dtype=np.uint8)
            images.create_dataset("cam_right_wrist", data=np.stack(cam_right_wrist), dtype=np.uint8)
            images.create_dataset("cam_left_wrist", data=np.stack(cam_left_wrist), dtype=np.uint8)

        print(f"convert {hdf5_path} to rdt data format at {hdf5_output_path}")

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
