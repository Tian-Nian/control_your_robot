#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
sys.path.append("../../")

import h5py
import numpy as np
import zarr
import shutil
import argparse
import cv2
import gc
import psutil
from tqdm import tqdm
from typing import Generator, Tuple, Optional

from utils.data_handler import hdf5_groups_to_dict, get_files, get_item

'''
    usage: python process_data.py <source_dir> <output_dir> <num_episodes>
    example: python process_data.py data/test_data/ processed_data/test_data-100.zarr/ 100
'''
# Data mapping configuration
map_config = {
    "cam_high": "slave_cam_head.color",
    "cam_wrist": "slave_cam_wrist.color",
    # qpos target dimension is 14 (dual-arm: 6+1+6+1), fill with 0 if right arm doesn't exist
    "qpos": ["slave_left_arm.joint", "slave_left_arm.gripper", "slave_right_arm.joint", "slave_right_arm.gripper"],
    "action": ["slave_left_arm.joint", "slave_left_arm.gripper", "slave_right_arm.joint", "slave_right_arm.gripper"],
}

# Redundancy operation configuration
REDUNDANCY_CONFIG = {
    "target_qpos_dim": 14,  # Target qpos dimension (left arm 6 + left gripper 1 + right arm 6 + right gripper 1)
    "left_joint_dim": 6,    # Left arm joint dimension
    "left_gripper_dim": 1,  # Left arm gripper dimension
    "right_joint_dim": 6,   # Right arm joint dimension
    "right_gripper_dim": 1, # Right arm gripper dimension
    "validate_data": True,  # Whether to validate data validity (NaN/Inf check)
    "verbose_logging": False # Whether to output verbose logs (reduce memory usage)
}

# Data sampling configuration
SAMPLING_CONFIG = {
    "downsample_factor": 1,  # Downsampling factor: 1 means no downsampling, keep original sampling rate
    "image_dtype": "uint8",  # Image data type
}


def load_hdf5_data(hdf5_path):
    """
    Load data from HDF5 file
    
    Args:
        hdf5_path: HDF5 file path
        
    Returns:
        tuple: (qpos_data, action_data, image_data)
    """
    try:
        data = hdf5_groups_to_dict(hdf5_path)
    except Exception as e:
        print(f"Skip {hdf5_path} due to read error: {e}")
        return None, None, None
    
    # Read camera data
    image_data = {}
    try:
        image_data["cam_high"] = get_item(data, map_config["cam_high"])
    except Exception as e:
        print(f"Warning: Failed to get cam_high data: {e}")
        image_data["cam_high"] = None
    
    try:
        image_data["cam_wrist"] = get_item(data, map_config["cam_wrist"])
    except Exception as e:
        print(f"Warning: Failed to get cam_wrist data: {e}")
        image_data["cam_wrist"] = None
    
    # Construct 14-dim qpos: left_arm(6)+left_gripper(1)+right_arm(6)+right_gripper(1)
    def try_get(name):
        try:
            return get_item(data, name)
        except Exception:
            return None

    left_joint = try_get("slave_left_arm.joint")
    left_gripper = try_get("slave_left_arm.gripper")
    right_joint = try_get("slave_right_arm.joint")
    right_gripper = try_get("slave_right_arm.gripper")

    # Detect arm configuration and add redundancy operations
    has_left_arm = left_joint is not None or left_gripper is not None
    has_right_arm = right_joint is not None or right_gripper is not None
    
    
    # Redundancy operation: if only single arm data exists, ensure the other arm's data structure is complete
    if has_left_arm and not has_right_arm:
        # Create zero data for right arm to ensure consistent time steps
        if left_joint is not None:
            T_ref = len(left_joint)
        elif left_gripper is not None:
            T_ref = len(left_gripper)
        else:
            T_ref = 0
        
        if T_ref > 0:
            right_joint = np.zeros((T_ref, 6), dtype=np.float32)
            right_gripper = np.zeros((T_ref, 1), dtype=np.float32)
            # print(f"    Created zero-padded right arm data with {T_ref} timesteps")
            
    elif has_right_arm and not has_left_arm:
        # print("  - Single right arm detected, will pad left arm with zeros")
        # Create zero data for left arm to ensure consistent time steps
        if right_joint is not None:
            T_ref = len(right_joint)
        elif right_gripper is not None:
            T_ref = len(right_gripper)
        else:
            T_ref = 0
            
        if T_ref > 0:
            left_joint = np.zeros((T_ref, 6), dtype=np.float32)
            left_gripper = np.zeros((T_ref, 1), dtype=np.float32)
            # print(f"    Created zero-padded left arm data with {T_ref} timesteps")
            
    elif has_left_arm and has_right_arm:
        print("  - Dual arm configuration detected")
    else:
        print(f"  - No arm data found, skipping file")
        return None, None, None

    # Infer time steps T (prioritize left arm, then right arm)
    T = None
    for arr in (left_joint, left_gripper, right_joint, right_gripper):
        if arr is not None:
            T = len(arr)
            break
    
    if T is None:
        print(f"Skip {hdf5_path}, no arm data found")
        return None, None, None
    
    # print(f"  - Time steps: {T}")

    def ensure_shape(arr, T, D):
        """Ensure array shape is (T, D)"""
        if arr is None:
            out = np.zeros((T, D), dtype=np.float32)
        else:
            arr = np.asarray(arr)
            if arr.ndim == 1:
                arr = arr.reshape(-1, 1)
            out = arr.astype(np.float32)
            # Crop/pad columns
            if out.shape[1] < D:
                pad = np.zeros((out.shape[0], D - out.shape[1]), dtype=np.float32)
                out = np.concatenate([out, pad], axis=1)
            elif out.shape[1] > D:
                out = out[:, :D]
        return out

    # Unify time length and apply redundant padding
    left_joint = ensure_shape(left_joint, T, REDUNDANCY_CONFIG["left_joint_dim"])
    left_gripper = ensure_shape(left_gripper, T, REDUNDANCY_CONFIG["left_gripper_dim"])
    right_joint = ensure_shape(right_joint, T, REDUNDANCY_CONFIG["right_joint_dim"])
    right_gripper = ensure_shape(right_gripper, T, REDUNDANCY_CONFIG["right_gripper_dim"])
    
    # Additional redundancy check: ensure all data is valid
    def validate_data(data, name):
        if not REDUNDANCY_CONFIG["validate_data"]:
            return data
        if np.any(np.isnan(data)):
            # print(f"  - Warning: NaN values found in {name}, replacing with zeros")
            data = np.nan_to_num(data, nan=0.0)
        if np.any(np.isinf(data)):
            # print(f"  - Warning: Inf values found in {name}, replacing with zeros")
            data = np.nan_to_num(data, posinf=0.0, neginf=0.0)
        return data
    
    left_joint = validate_data(left_joint, "left_joint")
    left_gripper = validate_data(left_gripper, "left_gripper")
    right_joint = validate_data(right_joint, "right_joint")
    right_gripper = validate_data(right_gripper, "right_gripper")

    # Construct qpos (T, target_dim)
    qpos = np.concatenate([left_joint, left_gripper, right_joint, right_gripper], axis=1)
    target_dim = REDUNDANCY_CONFIG["target_qpos_dim"]
    
    # Final redundancy validation: ensure qpos dimensions are correct
    if qpos.shape[1] != target_dim:
        if qpos.shape[1] < target_dim:
            # If dimension is insufficient, pad with zeros
            pad_width = target_dim - qpos.shape[1]
            qpos = np.pad(qpos, ((0, 0), (0, pad_width)), mode='constant', constant_values=0)
        else:
            # If dimension is excessive, truncate to target dimension
            qpos = qpos[:, :target_dim]
            
    # Downsample qpos data (30Hz -> 15Hz)
    downsample_factor = SAMPLING_CONFIG["downsample_factor"]
    qpos_downsampled = downsample_data(qpos, downsample_factor)
    
    # Construct action: next timestep qpos (based on downsampled data)
    actions = []
    for i in range(len(qpos_downsampled) - 1):
        actions.append(qpos_downsampled[i+1])
    
    # Last frame action is zero vector
    last_action = np.zeros(target_dim, dtype=np.float32)
    actions.append(last_action)
    actions = np.array(actions)
    
    # Update qpos to downsampled data
    qpos = qpos_downsampled
    
    # Redundancy validation: ensure action data is correct
    if actions.shape[1] != target_dim:
        # print(f"  - Error: action dimension is {actions.shape[1]}, expected {target_dim}. Padding/truncating to {target_dim}.")
        if actions.shape[1] < target_dim:
            pad_width = target_dim - actions.shape[1]
            actions = np.pad(actions, ((0, 0), (0, pad_width)), mode='constant', constant_values=0)
        else:
            actions = actions[:, :target_dim]
        # print(f"  - Corrected action shape: {actions.shape}")
    
    # Validate action data validity
    actions = validate_data(actions, "actions")
    
    
    # print(f"  - Successfully processed episode with {len(qpos)} timesteps (after downsampling from {T})")
    
    return qpos, actions, image_data


def downsample_data(data, downsample_factor):
    """
    Downsample data
    
    Args:
        data: Input data array
        downsample_factor: Downsampling factor
        
    Returns:
        np.ndarray: Downsampled data
    """
    if data is None:
        return None
    return data[::downsample_factor]


def process_images(image_arrays, downsample_factor=1):
    """
    Process image data to ensure correct format
    
    Args:
        image_arrays: List of image arrays
        downsample_factor: Downsampling factor
        
    Returns:
        np.ndarray: Processed image array (N, C, H, W) in uint8 format
    """
    if image_arrays is None or len(image_arrays) == 0:
        return None
    
    # Downsample first
    if downsample_factor > 1:
        image_arrays = downsample_data(image_arrays, downsample_factor)
    
    processed_images = []
    for img in image_arrays:
        if isinstance(img, bytes):
            # If encoded image data, decode it
            img_array = cv2.imdecode(np.frombuffer(img, np.uint8), cv2.IMREAD_COLOR)
        else:
            img_array = img
        
        if img_array is not None:
            # Ensure image data is in uint8 format
            if img_array.dtype != np.uint8:
                # If float type, convert value range from [0,1] or [-1,1] to [0,255]
                if img_array.dtype in [np.float32, np.float64]:
                    if img_array.max() <= 1.0 and img_array.min() >= 0.0:
                        img_array = (img_array * 255).astype(np.uint8)
                    elif img_array.max() <= 1.0 and img_array.min() >= -1.0:
                        img_array = ((img_array + 1.0) * 127.5).astype(np.uint8)
                    else:
                        img_array = np.clip(img_array, 0, 255).astype(np.uint8)
                else:
                    img_array = img_array.astype(np.uint8)
            processed_images.append(img_array)
    
    if len(processed_images) == 0:
        return None
    
    # Convert to numpy array and adjust dimensions NHWC -> NCHW
    image_stack = np.array(processed_images, dtype=np.uint8)
    if image_stack.ndim == 4:  # (N, H, W, C)
        image_stack = np.moveaxis(image_stack, -1, 1)  # -> (N, C, H, W)
    
    return image_stack


def convert_to_zarr(source_dir, output_dir, num_episodes):
    """
    Convert HDF5 data to zarr format using streaming approach to reduce memory usage
    
    Args:
        source_dir: Source data directory
        output_dir: Output directory
        num_episodes: Number of episodes to process
    """
    # Get all HDF5 files
    hdf5_paths = get_files(source_dir, "*.hdf5")
    
    if len(hdf5_paths) == 0:
        print(f"No HDF5 files found in {source_dir}")
        return
    
    # Limit number of files to process
    if num_episodes > 0:
        hdf5_paths = hdf5_paths[:num_episodes]
    
    # print(f"Found {len(hdf5_paths)} HDF5 files")
    
    # If output directory exists, remove it
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    
    # Create zarr root directory
    zarr_root = zarr.group(output_dir)
    zarr_data = zarr_root.create_group("data")
    zarr_meta = zarr_root.create_group("meta")
    
    # Set up compressor
    compressor = zarr.Blosc(cname="zstd", clevel=3, shuffle=1)
    
    # Initialize zarr datasets and tracking variables
    state_dataset = None
    action_dataset = None
    head_camera_dataset = None
    episode_ends = []
    total_count = 0
    valid_episodes = 0
    
    # First pass: determine data shapes and total size
    # print("First pass: analyzing data dimensions...")
    total_steps = 0
    state_dim = None
    action_dim = None
    image_shape = None
    
    for i, hdf5_path in enumerate(tqdm(hdf5_paths, desc="Analyzing episodes")):
        qpos, actions, image_data = load_hdf5_data(hdf5_path)
        
        if qpos is None:
            continue
        
        # Get dimensions from first valid episode
        if state_dim is None:
            state_dim = qpos.shape[1]
            action_dim = actions.shape[1]
            
        # Get image dimensions from first valid episode with images
        if image_shape is None and image_data["cam_high"] is not None:
            downsample_factor = SAMPLING_CONFIG["downsample_factor"]
            # Downsample first, then get shape from first frame (consistent with actual processing logic)
            temp_downsampled = downsample_data(image_data["cam_high"], downsample_factor)
            processed_images = process_images(temp_downsampled[:1], downsample_factor=1)  # Already downsampled, no repeat
            if processed_images is not None:
                image_shape = processed_images.shape[1:]  # (C, H, W)
        
        # Note: qpos has already been downsampled in load_hdf5_data(), no need to downsample again here
        episode_steps = len(qpos) - 1  # Exclude last frame
        total_steps += episode_steps
        valid_episodes += 1
        
        # Clear memory immediately
        del qpos, actions, image_data
    
    if valid_episodes == 0:
        print("No valid episodes found")
        return

    # Create zarr datasets with known total size
    state_chunk_size = (min(100, total_steps), state_dim)
    action_chunk_size = (min(100, total_steps), action_dim)
    
    state_dataset = zarr_data.create_dataset(
        "state",
        shape=(total_steps, state_dim),
        chunks=state_chunk_size,
        dtype="float32",
        compressor=compressor,
    )
    
    action_dataset = zarr_data.create_dataset(
        "action",
        shape=(total_steps, action_dim),
        chunks=action_chunk_size,
        dtype="float32",
        compressor=compressor,
    )
    
    if image_shape is not None:
        head_camera_chunk_size = (min(100, total_steps), *image_shape)
        head_camera_dataset = zarr_data.create_dataset(
            "head_camera",
            shape=(total_steps, *image_shape),
            chunks=head_camera_chunk_size,
            dtype=SAMPLING_CONFIG["image_dtype"],  # Use uint8 type
            compressor=compressor,
        )
    
    # Second pass: stream data directly to zarr datasets
    print("Second pass: streaming data to zarr...")
    current_idx = 0
    
    for i, hdf5_path in enumerate(tqdm(hdf5_paths, desc="Streaming episodes")):
        qpos, actions, image_data = load_hdf5_data(hdf5_path)
        
        if qpos is None:
            continue
        
        # Process state data (exclude last frame)
        states = qpos[:-1]  # Remove last frame
        episode_actions = actions[:-1]  # Remove last frame action
        episode_steps = len(states)
        
        # Verify dimension consistency
        if len(states) != len(episode_actions):
            print(f"Warning: Episode {i} - states length {len(states)} != actions length {len(episode_actions)}")
        
        # Write state and action data directly to zarr
        end_idx = current_idx + episode_steps
        state_dataset[current_idx:end_idx] = states.astype(np.float32)
        action_dataset[current_idx:end_idx] = episode_actions.astype(np.float32)
        
        # Process and write image data if available
        if head_camera_dataset is not None and image_data["cam_high"] is not None:
            downsample_factor = SAMPLING_CONFIG["downsample_factor"]
            # Downsample first, then remove last frame (consistent with qpos processing order)
            head_camera_downsampled = downsample_data(image_data["cam_high"], downsample_factor)
            head_camera_processed = process_images(head_camera_downsampled[:-1], downsample_factor=1)  # Already downsampled, no repeat
            if head_camera_processed is not None:
                # Verify image data dimensions match state dimensions
                if len(head_camera_processed) != episode_steps:
                    print(f"Warning: Episode {i} - image length {len(head_camera_processed)} != episode_steps {episode_steps}")
                head_camera_dataset[current_idx:end_idx] = head_camera_processed
        
        # Update tracking variables
        current_idx = end_idx
        total_count += episode_steps
        episode_ends.append(total_count)
        
        # print(f"Streamed episode {i+1}/{len(hdf5_paths)}, steps: {episode_steps}, total: {total_count}")
        
        # Clear memory immediately after processing each episode
        del qpos, actions, image_data, states, episode_actions
        if 'head_camera_downsampled' in locals():
            del head_camera_downsampled
        if 'head_camera_processed' in locals():
            del head_camera_processed
    
    # Save episode ends metadata
    episode_ends_arrays = np.array(episode_ends, dtype=np.int64)
    zarr_meta.create_dataset(
        "episode_ends",
        data=episode_ends_arrays,
        dtype="int64",
        compressor=compressor,
    )
    
    print(f"Data conversion completed!")
    print(f"Output directory: {output_dir}")
    # print(f"Total steps: {total_count}")
    # print(f"Episodes: {len(episode_ends_arrays)}")
    # print(f"State data shape: {state_dataset.shape}")
    # print(f"Action data shape: {action_dataset.shape}")
    if head_camera_dataset is not None:
        print(f"Head camera data shape: {head_camera_dataset.shape}")


def main():
    parser = argparse.ArgumentParser(description="Convert HDF5 data to zarr format for Diffusion Policy training")
    parser.add_argument(
        "source_dir",
        type=str,
        help="Source data directory path (containing HDF5 files)"
    )
    parser.add_argument(
        "output_dir", 
        type=str,
        help="Output data directory path (zarr format)"
    )
    parser.add_argument(
        "num_episodes",
        type=int,
        help="Number of episodes to process (0 means process all)"
    )
    
    args = parser.parse_args()
    
    # Check if source directory exists
    if not os.path.exists(args.source_dir):
        print(f"Error: Source directory does not exist: {args.source_dir}")
        return
    
    # Create parent directory of output directory
    os.makedirs(os.path.dirname(args.output_dir), exist_ok=True)
    # Execute conversion
    convert_to_zarr(args.source_dir, args.output_dir, args.num_episodes)


if __name__ == "__main__":
    main()
