import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

import os
import h5py
import numpy as np
import pickle
import cv2
import argparse
import yaml
import json
from tqdm import tqdm

from robot.utils.base.data_handler import (
    get_files,
    get_item,
    hdf5_groups_to_dict,
)


def images_encoding(imgs):
    """图像编码函数，与目标格式保持一致"""
    encode_data = []
    padded_data = []
    max_len = 0
    for i in range(len(imgs)):
        success, encoded_image = cv2.imencode(".jpg", imgs[i])
        jpeg_data = encoded_image.tobytes()
        encode_data.append(jpeg_data)
        max_len = max(max_len, len(jpeg_data))
    # padding
    for i in range(len(imgs)):
        padded_data.append(encode_data[i].ljust(max_len, b"\0"))
    return encode_data, max_len


def load_source_hdf5(hdf5_path):
    """
    从源格式hdf5文件中读取数据
    基于convert2act_hdf5.py的数据读取方式
    """
    try:
        data = hdf5_groups_to_dict(hdf5_path)
    except Exception as e:
        print(f"Error reading {hdf5_path}: {e}")
        return None
    
    # collect data from convert2act_hdf5.py
    source_map = {
        "cam_high": "slave_cam_head.color",
        "cam_wrist": "slave_cam_wrist.color",
        "left_arm_joint": "slave_left_arm.joint",
        "left_arm_gripper": "slave_left_arm.gripper", 
    }
    #test data
    # source_map = {
    #     "cam_high": "cam_head.color",
    #     "cam_wrist": "cam_wrist.color",
    #     "left_arm_joint": "left_arm.joint",
    #     "left_arm_gripper": "left_arm.gripper", 
    # }
    result = {}
    
    # 读取图像数据
    try:
        result["cam_high"] = get_item(data, source_map["cam_high"])
        result["cam_wrist"] = get_item(data, source_map["cam_wrist"])
    except Exception as e:
        print(f"Error reading camera data: {e}")
        return None
    
    # 读取关节数据
    def try_get(name):
        try:
            return get_item(data, name)
        except Exception:
            return None
    
    result["left_arm_joint"] = try_get(source_map["left_arm_joint"])
    result["left_arm_gripper"] = try_get(source_map["left_arm_gripper"])
    
    return result


def convert_to_target_format(source_data, episode_idx, save_path, instructions_file_path=None):
    """
    将源数据转换为目标格式
    """
    # 创建episode目录
    episode_dir = os.path.join(save_path, f"episode_{episode_idx}")
    os.makedirs(episode_dir, exist_ok=True)
    
    # 复制instructions.json文件到episode目录
    if instructions_file_path and os.path.exists(instructions_file_path):
        import shutil
        target_instructions_path = os.path.join(episode_dir, "instructions.json")
        shutil.copy2(instructions_file_path, target_instructions_path)
        # print(f"Copied instructions.json to {target_instructions_path}")
    else:
        # 如果没有找到instructions文件，创建默认的
        default_instructions = {"instructions": f"Complete the task for episode {episode_idx}"}
        with open(os.path.join(episode_dir, "instructions.json"), "w") as f:
            json.dump(default_instructions, f, indent=2)
    
    # 处理关节数据（仅单臂）
    left_arm_joint = source_data.get("left_arm_joint")
    left_arm_gripper = source_data.get("left_arm_gripper")
    
    # 确定时间步长
    T = 0
    for arr in [left_arm_joint, left_arm_gripper]:
        if arr is not None:
            T = len(arr)
            break
    
    if T == 0:
        print(f"No valid arm data found for episode {episode_idx}")
        return False
    
    # 构建qpos和actions
    qpos = []
    actions = []
    cam_high = []
    cam_wrist = []  # 单臂机器人只需要一个手腕相机
    left_arm_dim = []
    
    for j in range(T):
        # 获取当前时刻的关节状态（仅单臂）
        left_joint = left_arm_joint[j] if left_arm_joint is not None else np.zeros(6)
        left_gripper = left_arm_gripper[j] if left_arm_gripper is not None else 0.0
        
        # 确保数据格式正确
        if np.isscalar(left_gripper):
            left_gripper = [left_gripper]
            
        # 构建状态向量（仅单臂：6个关节 + 1个夹爪）
        state = np.concatenate([
            np.array(left_joint).flatten(),
            np.array(left_gripper).flatten()
        ]).astype(np.float32)
        
        # qpos不包含最后一帧
        if j != T - 1:
            qpos.append(state)
            
            # 处理图像数据
            if source_data.get("cam_high") is not None:
                img_high = source_data["cam_high"][j]
                if isinstance(img_high, np.ndarray) and img_high.shape[-1] == 3:
                    img_high_resized = cv2.resize(img_high, (640, 480))
                    cam_high.append(img_high_resized)
            
            if source_data.get("cam_wrist") is not None:
                img_wrist = source_data["cam_wrist"][j]
                if isinstance(img_wrist, np.ndarray) and img_wrist.shape[-1] == 3:
                    img_wrist_resized = cv2.resize(img_wrist, (640, 480))
                    # 单臂机器人只保存手腕相机数据
                    cam_wrist.append(img_wrist_resized)
        
        # actions不包含第一帧
        if j != 0:
            actions.append(state)
            left_arm_dim.append(len(left_joint))
    
    # 保存HDF5文件
    hdf5_path = os.path.join(episode_dir, f"episode_{episode_idx}.hdf5")
    
    with h5py.File(hdf5_path, "w") as f:
        f.create_dataset("action", data=np.array(actions))
        
        obs = f.create_group("observations")
        obs.create_dataset("qpos", data=np.array(qpos))
        obs.create_dataset("left_arm_dim", data=np.array(left_arm_dim))
        
        images = obs.create_group("images")
        
        # 编码并保存图像
        if cam_high:
            cam_high_enc, len_high = images_encoding(cam_high)
            images.create_dataset("image", data=cam_high_enc, dtype=f"S{len_high}")
        
        if cam_wrist:
            cam_wrist_enc, len_wrist = images_encoding(cam_wrist)
            images.create_dataset("wrist_image", data=cam_wrist_enc, dtype=f"S{len_wrist}")
    
    return True


def load_instructions_from_source(source_path, episode_num=None, desc_type="seen"):
    """
    从源数据目录读取instructions，支持两种格式：
    1. 统一的instructions.json文件（原格式）
    2. 每个episode单独的instruction文件（参考例程格式）
    """
    # 方法1: 尝试读取统一的instructions.json文件
    unified_instructions_path = os.path.join(source_path, "instructions.json")
    if os.path.exists(unified_instructions_path):
        try:
            with open(unified_instructions_path, "r", encoding="utf-8") as f:
                instructions_data = json.load(f)
            
            if "instructions" in instructions_data:
                instructions_list = instructions_data["instructions"]
                print(f"Loaded {len(instructions_list)} instructions from unified file {unified_instructions_path}")
                return instructions_list
        except Exception as e:
            print(f"Error reading unified instructions from {unified_instructions_path}: {e}")
    
    # 方法2: 尝试读取每个episode单独的instruction文件（参考例程格式）
    instructions_dir = os.path.join(source_path, "instructions")
    if os.path.exists(instructions_dir):
        instructions_list = []
        
        # 如果没有指定episode_num，尝试自动检测
        if episode_num is None:
            # 查找所有episode文件来确定数量
            episode_files = []
            i = 0
            while True:
                episode_file = os.path.join(instructions_dir, f"episode{i}.json")
                if os.path.exists(episode_file):
                    episode_files.append(episode_file)
                    i += 1
                else:
                    break
            episode_num = len(episode_files)
        
        if episode_num > 0:
            print(f"Found instructions directory, attempting to load {episode_num} episode instructions...")
            
            for i in range(episode_num):
                episode_instruction_path = os.path.join(instructions_dir, f"episode{i}.json")
                
                if os.path.exists(episode_instruction_path):
                    try:
                        with open(episode_instruction_path, "r", encoding="utf-8") as f:
                            instruction_dict = json.load(f)
                        
                        # 提取指定类型的instruction
                        if desc_type in instruction_dict:
                            instruction = instruction_dict[desc_type]
                            instructions_list.append(instruction)
                        else:
                            print(f"Warning: desc_type '{desc_type}' not found in {episode_instruction_path}")
                            instructions_list.append(f"Complete the task for episode {i}")
                            
                    except Exception as e:
                        print(f"Error reading instruction from {episode_instruction_path}: {e}")
                        instructions_list.append(f"Complete the task for episode {i}")
                else:
                    print(f"Warning: instruction file not found: {episode_instruction_path}")
                    instructions_list.append(f"Complete the task for episode {i}")
            
            if instructions_list:
                print(f"Loaded {len(instructions_list)} instructions from episode-specific files")
                return instructions_list
    
    print(f"No instruction files found in {source_path}")
    return None


def data_transform(source_path, save_path, episode_num=None):
    """
    主转换函数
    """
    # 获取所有源hdf5文件
    hdf5_files = sorted(
        get_files(source_path, "*.hdf5"),
        key=lambda path: (
            0,
            int(os.path.splitext(os.path.basename(path))[0]),
        )
        if os.path.splitext(os.path.basename(path))[0].isdigit()
        else (1, path),
    )
    
    if episode_num is not None:
        hdf5_files = hdf5_files[:episode_num]
    
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    
    # 自动在源目录中寻找instructions.json文件
    instructions_file_path = os.path.join(source_path, "instructions.json")
    if os.path.exists(instructions_file_path):
        print(f"Found instructions file: {instructions_file_path}")
    else:
        print("No instructions.json file found in source directory")
        instructions_file_path = None
    
    success_count = 0
    
    for i, hdf5_path in enumerate(tqdm(hdf5_files, desc="Converting episodes")):
        # print(f"Processing {hdf5_path}")
        
        # 读取源数据
        source_data = load_source_hdf5(hdf5_path)
        if source_data is None:
            print(f"Skipping {hdf5_path} due to read error")
            continue
        
        # 转换为目标格式，传递instructions文件路径
        if convert_to_target_format(source_data, i, save_path, instructions_file_path):
            success_count += 1
            # print(f"Successfully converted episode {i}")
        else:
            
            print(f"Failed to convert episode {i}")
    
    # print(f"Conversion completed: {success_count}/{len(hdf5_files)} episodes converted successfully")
    return success_count


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert source HDF5 to target format")
    parser.add_argument("source_path", type=str, help="Path to source HDF5 files directory")
    parser.add_argument("save_path", type=str, help="Path to save converted data")
    parser.add_argument("--episode_num", type=int, default=None, help="Number of episodes to process")
    args = parser.parse_args()
    
    # 示例用法
    # source_path = "save/feed_rice/"  # 源数据路径
    # save_path = "processed_data/feed_rice_converted/"  # 目标保存路径
    
    source_path = args.source_path
    save_path = args.save_path
    episode_num = args.episode_num
    
    print(f"Converting data from {source_path} to {save_path}")
    if episode_num:
        print(f"Processing {episode_num} episodes")
    
    # 自动在源数据文件夹中寻找instructions.json
    success_count = data_transform(source_path, save_path, episode_num)
    
    print(f"\nConversion summary:")
    print(f"- Source path: {source_path}")
    print(f"- Target path: {save_path}")
    print(f"- Successfully converted: {success_count} episodes")
