

import h5py
import numpy as np
import os
from tqdm import tqdm
import queue
import threading

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

def process_one(hdf5_path, hdf5_output_path):
    data = hdf5_groups_to_dict(hdf5_path)
    
    with h5py.File(hdf5_output_path, "w") as f:
        # 提取数据
        input_data = {}
        for key in map.keys():
            input_data[key] = get_item(data, map[key])[:]

        qpos = np.array(input_data["qpos"]).astype(np.float32)
        
        actions = []
        for i in range(len(qpos) - 1):
            actions.append(qpos[i+1])
        
        # 最后一帧结束无动作，填充补零 (长度匹配当前 qpos 维度)
        qpos_dim = qpos.shape[1]
        last_action = np.zeros(qpos_dim, dtype=np.float32)
        actions.append(last_action)

        actions = np.array(actions)
        f.create_dataset('action', data=np.array(actions), dtype="float32")

        obs = f.create_group("observations")
        obs.create_dataset('qpos', data=np.array(qpos), dtype="float32")
        # 这里的 dim 可能需要根据实际情况调整，目前保留原样
        obs.create_dataset("left_arm_dim", data=np.array(6))
        obs.create_dataset("right_arm_dim", data=np.array(6))

        images = obs.create_group("images")
        
        def decode(imgs):
            if isinstance(imgs, np.ndarray) and imgs.ndim == 4:
                return imgs

            imgs_array = []
            for data_item in imgs:
                if isinstance(data_item, (bytes, bytearray)):
                    data_item = np.frombuffer(data_item, dtype=np.uint8)

                img = cv2.imdecode(data_item, cv2.IMREAD_COLOR)
                if img is None:
                    raise ValueError("Failed to decode JPEG image")
                # 恢复 append 逻辑
                imgs_array.append(img)

            return np.stack(imgs_array, axis=0)

        if "cam_high" in input_data:
            cam_high = decode(input_data["cam_high"])
            images.create_dataset("cam_high", data=cam_high, dtype=np.uint8)
        
        if "cam_left_wrist" in input_data:
            cam_left_wrist = decode(input_data["cam_left_wrist"])
            images.create_dataset("cam_left_wrist", data=cam_left_wrist, dtype=np.uint8)
            
        if "cam_right_wrist" in input_data:
            cam_right_wrist = decode(input_data["cam_right_wrist"])
            images.create_dataset("cam_right_wrist", data=cam_right_wrist, dtype=np.uint8)
            
        if "cam_wrist" in input_data: # 单臂情况
            cam_wrist = decode(input_data["cam_wrist"])
            images.create_dataset("cam_wrist", data=cam_wrist, dtype=np.uint8)

def worker(q, pbar):
    while True:
        try:
            task = q.get(block=False)
            if task is None:
                break
            hdf5_path, hdf5_output_path = task
            process_one(hdf5_path, hdf5_output_path)
            pbar.update(1)
            q.task_done()
        except queue.Empty:
            break
        except Exception as e:
            print(f"\nError processing {task[0]}: {e}")
            q.task_done()

def convert(hdf5_paths, output_path, start_index=0, num_workers=8):
    if not os.path.exists(output_path):
        os.makedirs(output_path)
    
    q = queue.Queue()
    for i, hdf5_path in enumerate(hdf5_paths):
        hdf5_output_path = os.path.join(output_path, f"episode_{i + start_index}.hdf5")
        q.put((hdf5_path, hdf5_output_path))
    
    print(f"Total files: {len(hdf5_paths)}. Using {num_workers} threads...")
    with tqdm(total=len(hdf5_paths)) as pbar:
        threads = []
        for _ in range(num_workers):
            t = threading.Thread(target=worker, args=(q, pbar))
            t.start()
            threads.append(t)
        
        for t in threads:
            t.join()

if __name__ == "__main__":
    import argparse
    import json
    parser = argparse.ArgumentParser(description='Transform datasets type to HDF5.')
    parser.add_argument('data_path', type=str,
                        help="your data dir like: datasets/task/")
    parser.add_argument('output_path', type=str, default=None,
                        help='output path commanded like datasets/RDT/...')
    parser.add_argument('--num_workers', type=int, default=8,
                        help='number of threads for parallel processing')
    
    args = parser.parse_args()
    data_path = args.data_path
    output_path = args.output_path

    if output_path is None:
        config_file = os.path.join(data_path, "config.json")
        with open(config_file, 'r') as f:
            data_config = json.load(f)
        output_path = f"./datasets/RDT/{data_config['task_name']}"
    
    hdf5_paths = sorted(get_files(data_path, "*.hdf5"))
    print(f"Found {len(hdf5_paths)} hdf5 files.")
    convert(hdf5_paths, output_path, num_workers=args.num_workers)
