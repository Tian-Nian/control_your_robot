"""
使用Rerun进行HDF5机器人数据可视化（通用版本）

安装依赖:
    pip install rerun-sdk h5py numpy tqdm
    pip install opencv-python  # 可选，用于更好的颜色映射

使用示例:
    # 可视化单个文件
    python visual_hdf5_rerun.py /path/to/file.hdf5
    
    # 可视化文件夹中的所有文件
    python visual_hdf5_rerun.py /path/to/folder/
    
    # 保存为.rrd文件供后续查看
    python visual_hdf5_rerun.py /path/to/file.hdf5 --save output.rrd
    
    # 连接到远程查看器
    python visual_hdf5_rerun.py /path/to/file.hdf5 --connect
"""

import h5py
import numpy as np
import os
import json
from tqdm import tqdm
import argparse
from pathlib import Path
import base64
from io import BytesIO
import sys
# 添加项目根目录到Python路径
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot.utils.base.data_handler import debug_print

try:
    import rerun as rr
except ImportError:
    debug_print("RERUN", "未安装rerun-sdk，请运行: pip install rerun-sdk", "ERROR")
    sys.exit(1)

try:
    from PIL import Image
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


def detect_hdf5_format(f):
    """
    自动检测HDF5文件的格式类型
    
    Returns:
        str: 格式类型 ('act', 'openpi', 'rdt', 'custom')
    """
    keys = list(f.keys())
    
    # 检测ACT格式（有left_arm, right_arm, cam_*等）
    has_left_arm = any(k in keys for k in ['left_arm', 'slave_left_arm', 'master_left_arm'])
    has_right_arm = any(k in keys for k in ['right_arm', 'slave_right_arm', 'master_right_arm'])
    has_cam = any(k.startswith(('cam_', 'slave_cam_', 'master_cam_')) for k in keys)
    
    if has_left_arm or has_right_arm or has_cam:
        return 'act'
    
    # 检测OpenPI格式（有observations, action等）
    if 'observations' in keys or 'action' in keys:
        return 'openpi'
    
    # 检测RDT格式
    if 'data' in keys and isinstance(f['data'], h5py.Group):
        return 'rdt'
    
    return 'custom'


def decode_image_from_bytes(img_bytes):
    """
    从字节数据解码图像
    支持多种编码格式：jpg、png、base64等
    """
    if not HAS_PIL:
        return None
    
    try:
        # 尝试直接解码
        img = Image.open(BytesIO(img_bytes))
        return np.array(img)
    except:
        try:
            # 尝试base64解码
            img_data = base64.b64decode(img_bytes)
            img = Image.open(BytesIO(img_data))
            return np.array(img)
        except:
            return None


def extract_images_from_dataset(dataset, frame_idx):
    """
    从数据集中提取图像
    处理各种可能的图像存储格式
    """
    if frame_idx >= len(dataset):
        return None
    
    data = dataset[frame_idx]
    
    # 情况1: 直接是numpy数组图像
    if isinstance(data, np.ndarray):
        if len(data.shape) >= 2:  # 已经是图像
            return data
    
    # 情况2: 字节串（压缩的图像）
    if isinstance(data, (bytes, np.bytes_)):
        img = decode_image_from_bytes(data)
        if img is not None:
            return img
    
    # 情况3: 字符串类型的numpy标量
    if hasattr(data, 'tobytes'):
        img = decode_image_from_bytes(data.tobytes())
        if img is not None:
            return img
    
    return None


def is_tactile_image_data(data, frame_idx=0):
    """
    检测触觉数据是否为图像格式
    触觉图像通常是2D的压力/接触图
    
    支持两种格式:
    1. (n_frames, height, width) - 最常见
    2. (height, width) - 单帧
    """
    if data is None:
        return False
    
    # 检查数据集的形状
    if hasattr(data, 'shape'):
        shape = data.shape
        # 格式1: (n_frames, h, w) - 多帧触觉图像
        if len(shape) == 3:
            n_frames, h, w = shape
            if 4 <= h <= 256 and 4 <= w <= 256:
                return True
        # 格式2: (h, w) - 单帧触觉图像
        elif len(shape) == 2:
            h, w = shape
            if 4 <= h <= 256 and 4 <= w <= 256:
                return True
    
    # 如果有frame_idx，检查单帧数据
    if frame_idx < len(data):
        try:
            frame_data = data[frame_idx]
            if isinstance(frame_data, np.ndarray) and len(frame_data.shape) == 2:
                h, w = frame_data.shape
                if 4 <= h <= 256 and 4 <= w <= 256:
                    return True
        except:
            pass
    
    return False


def apply_tactile_colormap(tactile_data):
    """
    为触觉数据应用颜色映射
    将触觉压力数据转换为彩色热力图
    """
    # 归一化到0-255
    if tactile_data.dtype != np.uint8:
        if tactile_data.max() > tactile_data.min():
            normalized = (tactile_data - tactile_data.min()) / (tactile_data.max() - tactile_data.min())
            normalized = (normalized * 255).astype(np.uint8)
        else:
            normalized = np.zeros_like(tactile_data, dtype=np.uint8)
    else:
        normalized = tactile_data
    
    # 使用VIRIDIS颜色映射（类似OpenCV的COLORMAP_VIRIDIS）
    # Rerun需要RGB格式，我们手动创建一个类似viridis的映射
    # 为了更好的显示效果，我们创建一个RGB版本
    if HAS_CV2:
        # 如果有OpenCV，使用VIRIDIS colormap
        colored = cv2.applyColorMap(normalized, cv2.COLORMAP_VIRIDIS)
        # OpenCV返回BGR，转换为RGB
        colored = cv2.cvtColor(colored, cv2.COLOR_BGR2RGB)
        return colored
    
    # 如果没有OpenCV，创建一个简单的蓝-绿-黄-红映射
    h, w = normalized.shape
    colored = np.zeros((h, w, 3), dtype=np.uint8)
    
    # 简单的hot colormap: 蓝->绿->黄->红
    for i in range(h):
        for j in range(w):
            val = normalized[i, j] / 255.0
            if val < 0.25:
                # 蓝到青
                colored[i, j] = [0, int(val * 4 * 255), 255]
            elif val < 0.5:
                # 青到绿
                t = (val - 0.25) * 4
                colored[i, j] = [0, 255, int((1-t) * 255)]
            elif val < 0.75:
                # 绿到黄
                t = (val - 0.5) * 4
                colored[i, j] = [int(t * 255), 255, 0]
            else:
                # 黄到红
                t = (val - 0.75) * 4
                colored[i, j] = [255, int((1-t) * 255), 0]
    
    return colored


def log_timeseries_data(entity_path, data, frame_idx, name_prefix="value"):
    """
    记录时间序列数据到Rerun
    自动处理标量、向量和多维数据
    """
    # 如果数据为None或空，直接返回，不记录任何内容
    if data is None or len(data) == 0 or frame_idx >= len(data):
        return
    
    frame_data = data[frame_idx]
    
    # 标量
    if np.isscalar(frame_data) or (isinstance(frame_data, np.ndarray) and frame_data.size == 1):
        value = float(frame_data) if not isinstance(frame_data, np.ndarray) else float(frame_data.item())
        rr.log(entity_path, rr.Scalars([value]))
    
    # 向量
    elif isinstance(frame_data, np.ndarray):
        if len(frame_data.shape) == 1:
            # 1D向量 - 记录每个元素
            for i, val in enumerate(frame_data):
                rr.log(f"{entity_path}/{name_prefix}_{i+1}", rr.Scalars([float(val)]))
            # 也记录整个向量
            rr.log(f"{entity_path}_vector", rr.Tensor(frame_data, dim_names=["dim"]))
        else:
            # 多维数据 - 作为张量
            rr.log(entity_path, rr.Tensor(frame_data))


def visualize_act_format(f, verbose=False):
    """处理ACT格式的HDF5文件"""
    if verbose:
        debug_print("ACT_FORMAT", "检测到ACT格式数据", "DEBUG")
    
    # 读取左臂数据
    left_arm_data = {'joints': None, 'gripper': None, 'eefort': None}
    left_arm_keys = ['left_arm', 'slave_left_arm', 'master_left_arm']
    for key in left_arm_keys:
        if key in f:
            left_arm_group = f[key]
            if 'joint' in left_arm_group and len(left_arm_group['joint']) > 0:
                left_arm_data['joints'] = left_arm_group['joint'][:]
            if 'gripper' in left_arm_group and len(left_arm_group['gripper']) > 0:
                left_arm_data['gripper'] = left_arm_group['gripper'][:]
            if 'eefort' in left_arm_group and len(left_arm_group['eefort']) > 0:
                left_arm_data['eefort'] = left_arm_group['eefort'][:]
            break
    
    # 读取右臂数据
    right_arm_data = {'joints': None, 'gripper': None, 'eefort': None}
    right_arm_keys = ['right_arm', 'slave_right_arm', 'master_right_arm']
    for key in right_arm_keys:
        if key in f:
            right_arm_group = f[key]
            if 'joint' in right_arm_group and len(right_arm_group['joint']) > 0:
                right_arm_data['joints'] = right_arm_group['joint'][:]
            if 'gripper' in right_arm_group and len(right_arm_group['gripper']) > 0:
                right_arm_data['gripper'] = right_arm_group['gripper'][:]
            if 'eefort' in right_arm_group and len(right_arm_group['eefort']) > 0:
                right_arm_data['eefort'] = right_arm_group['eefort'][:]
            break
    
    # 读取相机数据
    camera_data = {}
    for key in f.keys():
        if (key.startswith('cam_') or key.startswith('camera_') or 
            key.startswith('slave_cam_') or key.startswith('master_cam_')):
            if key in f:
                cam_dataset = None
                if 'color' in f[key]:
                    cam_dataset = f[key]['color']
                elif 'rgb' in f[key]:
                    cam_dataset = f[key]['rgb']
                elif 'image' in f[key]:
                    cam_dataset = f[key]['image']
                
                # 只添加非空的相机数据集
                if cam_dataset is not None and len(cam_dataset) > 0:
                    camera_data[key] = cam_dataset
    
    # 读取触觉数据
    tactile_data = {}
    tactile_keywords = ['tactile', 'force', 'pressure', 'touch', 'tac', 'vitac', 'contact', 
                        'haptic', 'sensor_force', 'torque_sensor']
    
    def search_tactile_data(group, prefix=""):
        """递归搜索触觉数据"""
        for key in group.keys():
            full_path = f"{prefix}/{key}" if prefix else key
            item = group[key]
            
            # 检查是否匹配触觉关键词
            if any(keyword in key.lower() for keyword in tactile_keywords):
                if isinstance(item, h5py.Dataset):
                    # 直接是Dataset，只添加非空数据集
                    if len(item) > 0:
                        tactile_data[full_path] = item
                elif isinstance(item, h5py.Group):
                    # 是Group，继续递归搜索其子项
                    search_tactile_data(item, full_path)
            elif isinstance(item, h5py.Group):
                # 即使名字不匹配，也搜索Group内部（可能包含tactile子项）
                # 特别是处理像 slave_right_arm_tac/tactile 这样的结构
                search_tactile_data(item, full_path)
    
    search_tactile_data(f)
    
    # 调试：打印找到的触觉数据
    if verbose:
        if tactile_data:
            debug_print("ACT_FORMAT", f"找到 {len(tactile_data)} 个触觉数据集:", "DEBUG")
            for tac_name, tac_dataset in tactile_data.items():
                debug_print("ACT_FORMAT", f"  - {tac_name}: 长度={len(tac_dataset)}", "DEBUG")
        else:
            debug_print("ACT_FORMAT", "未找到触觉数据", "DEBUG")
    
    # 确定最大帧数
    max_frames = 0
    if camera_data:
        max_frames = max(len(cam) for cam in camera_data.values())
    if left_arm_data['joints'] is not None:
        max_frames = max(max_frames, len(left_arm_data['joints']))
    if right_arm_data['joints'] is not None:
        max_frames = max(max_frames, len(right_arm_data['joints']))
    
    return left_arm_data, right_arm_data, camera_data, tactile_data, max_frames


def visualize_openpi_format(f, verbose=False):
    """处理OpenPI格式的HDF5文件"""
    if verbose:
        debug_print("OPENPI_FORMAT", "检测到OpenPI格式数据", "DEBUG")
    
    robot_data = {}
    camera_data = {}
    max_frames = 0
    
    # 处理observations
    if 'observations' in f:
        obs_group = f['observations']
        
        # 处理eef（关节位置）
        if 'eef' in obs_group and len(obs_group['eef']) > 0:
            robot_data['eef'] = obs_group['eef'][:]
            max_frames = max(max_frames, len(robot_data['eef']))
        
        # 处理qvel（关节速度）
        if 'qvel' in obs_group and len(obs_group['qvel']) > 0:
            robot_data['qvel'] = obs_group['qvel'][:]
            max_frames = max(max_frames, len(robot_data['qvel']))
        
        # 处理effort（关节力矩）
        if 'effort' in obs_group and len(obs_group['effort']) > 0:
            robot_data['effort'] = obs_group['effort'][:]
            max_frames = max(max_frames, len(robot_data['effort']))
        
        # 处理图像数据
        if 'images' in obs_group:
            img_group = obs_group['images']
            for img_key in img_group.keys():
                # 只添加非空的图像数据集
                if len(img_group[img_key]) > 0:
                    camera_data[img_key] = img_group[img_key]
                    max_frames = max(max_frames, len(img_group[img_key]))
    
    # 处理action
    if 'action' in f and len(f['action']) > 0:
        robot_data['action'] = f['action'][:]
        max_frames = max(max_frames, len(robot_data['action']))
    
    return robot_data, camera_data, max_frames


def visualize_custom_format(f, verbose=False):
    """处理自定义格式的HDF5文件 - 自动探测所有数据"""
    if verbose:
        debug_print("CUSTOM_FORMAT", "检测到自定义格式数据，自动探测结构...", "DEBUG")
    
    datasets = {}
    images = {}
    tactile_images = {}  # 单独存储触觉图像
    max_frames = 0
    
    # 触觉数据关键词
    tactile_keywords = ['tactile', 'force', 'pressure', 'touch', 'tac', 'vitac', 'contact', 
                        'haptic', 'sensor_force', 'torque_sensor']
    
    def explore_group(group, prefix=""):
        """递归探索HDF5组"""
        nonlocal max_frames
        
        for key in group.keys():
            full_path = f"{prefix}/{key}" if prefix else key
            item = group[key]
            
            if isinstance(item, h5py.Dataset):
                data = item
                
                # 跳过空数据集
                if len(data) == 0:
                    continue
                
                max_frames = max(max_frames, len(data))
                
                # 判断是否是图像数据
                # 检查第一个元素
                first_elem = data[0]
                is_image = False
                is_tactile = any(keyword in full_path.lower() for keyword in tactile_keywords)
                
                # 字节串可能是压缩的图像
                if isinstance(first_elem, (bytes, np.bytes_)) or (
                    isinstance(first_elem, np.ndarray) and first_elem.dtype.kind in ['S', 'O']
                ):
                    is_image = True
                    if is_tactile:
                        tactile_images[full_path] = data
                    else:
                        images[full_path] = data
                # numpy数组且形状像图像
                elif isinstance(first_elem, np.ndarray) and len(first_elem.shape) in [2, 3]:
                    if len(first_elem.shape) == 3 and first_elem.shape[2] in [1, 3, 4]:
                        is_image = True
                        if is_tactile:
                            tactile_images[full_path] = data
                        else:
                            images[full_path] = data
                    elif len(first_elem.shape) == 2:
                        # 2D数据可能是触觉图像或普通图像
                        h, w = first_elem.shape
                        # 触觉传感器通常是小尺寸的方阵
                        if is_tactile or (4 <= h <= 256 and 4 <= w <= 256 and abs(h - w) <= max(h, w) * 0.5):
                            is_image = True
                            tactile_images[full_path] = data
                        elif min(first_elem.shape) > 10:
                            is_image = True
                            images[full_path] = data
                
                if not is_image:
                    # 只存储非空数据
                    data_array = data[:]
                    if data_array is not None and len(data_array) > 0:
                        datasets[full_path] = data_array
                    
            elif isinstance(item, h5py.Group):
                explore_group(item, full_path)
    
    explore_group(f)
    
    # 调试：打印找到的数据
    if verbose:
        if tactile_images:
            debug_print("CUSTOM_FORMAT", f"找到 {len(tactile_images)} 个触觉图像数据集:", "DEBUG")
            for tac_name in tactile_images.keys():
                debug_print("CUSTOM_FORMAT", f"  - {tac_name}", "DEBUG")
        else:
            debug_print("CUSTOM_FORMAT", "未找到触觉图像数据", "DEBUG")
    
    return datasets, images, tactile_images, max_frames


def visualize_hdf5_with_rerun(hdf5_path, verbose=False):
    """
    使用Rerun可视化HDF5文件内容（通用版本）
    自动检测文件格式并适配
    
    Parameters:
        hdf5_path: HDF5文件路径
        verbose: 是否显示详细信息
    """
    # 打开HDF5文件
    with h5py.File(hdf5_path, 'r') as f:
        if verbose:
            debug_print("VISUALIZE", f"处理文件: {os.path.basename(hdf5_path)}", "INFO")
        
        # 自动检测格式
        data_format = detect_hdf5_format(f)
        if verbose:
            debug_print("VISUALIZE", f"数据格式: {data_format.upper()}", "DEBUG")
        
        # 根据格式处理数据
        max_frames = 0
        
        if data_format == 'act':
            left_arm_data, right_arm_data, camera_data, tactile_data, max_frames = visualize_act_format(f, verbose)
            
            if max_frames == 0:
                debug_print("VISUALIZE", f"文件 {hdf5_path} 中没有找到有效数据", "WARNING")
                return
            
            if verbose:
                debug_print("DATA_STATS", f"总帧数: {max_frames}", "DEBUG")
                if camera_data:
                    debug_print("DATA_STATS", f"相机数量: {len(camera_data)}", "DEBUG")
                    for cam_name in camera_data.keys():
                        debug_print("DATA_STATS", f"  - {cam_name}", "DEBUG")
                if tactile_data:
                    debug_print("DATA_STATS", f"触觉传感器: {len(tactile_data)}", "DEBUG")
                    for tac_name, tac_dataset in tactile_data.items():
                        debug_print("DATA_STATS", f"  - {tac_name}: 长度={len(tac_dataset)}", "DEBUG")
                else:
                    debug_print("DATA_STATS", "触觉传感器: 0 (无触觉数据)", "DEBUG")
                if left_arm_data['joints'] is not None:
                    debug_print("DATA_STATS", f"左臂关节: {left_arm_data['joints'].shape}", "DEBUG")
                if right_arm_data['joints'] is not None:
                    debug_print("DATA_STATS", f"右臂关节: {right_arm_data['joints'].shape}", "DEBUG")
            
            # 记录ACT格式数据
            debug_print("VISUALIZE", "正在记录数据到Rerun...", "INFO")
            if verbose:
                debug_print("VISUALIZE", f"将要记录的数据类型:", "DEBUG")
                debug_print("VISUALIZE", f"  - 左臂: {left_arm_data['joints'] is not None}", "DEBUG")
                debug_print("VISUALIZE", f"  - 右臂: {right_arm_data['joints'] is not None}", "DEBUG")
                debug_print("VISUALIZE", f"  - 相机: {len(camera_data) if camera_data else 0}", "DEBUG")
                debug_print("VISUALIZE", f"  - 触觉: {len(tactile_data) if tactile_data else 0}", "DEBUG")
            
            for frame_idx in tqdm(range(max_frames), desc="记录帧数据", disable=not verbose):
                rr.set_time("frame", sequence=frame_idx)
                
                # 左臂数据 - 只在数据存在时记录
                if left_arm_data['joints'] is not None and len(left_arm_data['joints']) > 0:
                    log_timeseries_data("robot/left_arm/joints", left_arm_data['joints'], frame_idx, "joint")
                if left_arm_data['gripper'] is not None and len(left_arm_data['gripper']) > 0:
                    log_timeseries_data("robot/left_arm/gripper", left_arm_data['gripper'], frame_idx, "gripper")
                if left_arm_data['eefort'] is not None and len(left_arm_data['eefort']) > 0:
                    log_timeseries_data("robot/left_arm/eefort", left_arm_data['eefort'], frame_idx, "force")
                
                # 右臂数据 - 只在数据存在时记录
                if right_arm_data['joints'] is not None and len(right_arm_data['joints']) > 0:
                    log_timeseries_data("robot/right_arm/joints", right_arm_data['joints'], frame_idx, "joint")
                if right_arm_data['gripper'] is not None and len(right_arm_data['gripper']) > 0:
                    log_timeseries_data("robot/right_arm/gripper", right_arm_data['gripper'], frame_idx, "gripper")
                if right_arm_data['eefort'] is not None and len(right_arm_data['eefort']) > 0:
                    log_timeseries_data("robot/right_arm/eefort", right_arm_data['eefort'], frame_idx, "force")
                
                # 相机图像 - 只在有相机数据时记录
                if camera_data:
                    for camera_name, cam_dataset in camera_data.items():
                        if frame_idx < len(cam_dataset):
                            image = extract_images_from_dataset(cam_dataset, frame_idx)
                            if image is not None:
                                # 确保图像格式正确
                                if image.dtype != np.uint8:
                                    if image.max() > 0:
                                        image = (image - image.min()) / (image.max() - image.min()) * 255
                                    image = image.astype(np.uint8)
                                
                                # Rerun期望RGB格式
                                if len(image.shape) == 2:
                                    image = np.stack([image, image, image], axis=-1)
                                elif len(image.shape) == 3 and image.shape[2] == 4:
                                    image = image[:, :, :3]
                                
                                rr.log(f"cameras/{camera_name}", rr.Image(image))
                
                # 触觉数据 - 只在有触觉数据时记录
                if tactile_data:
                    # 在第一帧时记录调试信息
                    if frame_idx == 0 and verbose:
                        debug_print("VISUALIZE", f"开始记录 {len(tactile_data)} 个触觉数据集", "DEBUG")
                    
                    for tactile_name, tactile_dataset in tactile_data.items():
                        if frame_idx < len(tactile_dataset):
                            # 检测是否为触觉图像数据
                            if is_tactile_image_data(tactile_dataset, frame_idx):
                                tactile_frame = tactile_dataset[frame_idx]
                                # 应用热力图颜色映射
                                tactile_colored = apply_tactile_colormap(tactile_frame)
                                rr.log(f"tactile/{tactile_name}_heatmap", rr.Image(tactile_colored))
                                # 同时记录原始数据的张量表示
                                rr.log(f"tactile/{tactile_name}_raw", rr.Tensor(tactile_frame))
                            else:
                                # 非图像格式的触觉数据，使用时间序列显示
                                log_timeseries_data(f"tactile/{tactile_name}", tactile_dataset, frame_idx)
        
        elif data_format == 'openpi':
            robot_data, camera_data, max_frames = visualize_openpi_format(f, verbose)
            
            if max_frames == 0:
                debug_print("VISUALIZE", f"文件 {hdf5_path} 中没有找到有效数据", "WARNING")
                return
            
            if verbose:
                debug_print("DATA_STATS", f"总帧数: {max_frames}", "DEBUG")
                if robot_data:
                    debug_print("DATA_STATS", f"机器人数据: {list(robot_data.keys())}", "DEBUG")
                if camera_data:
                    debug_print("DATA_STATS", f"相机数量: {len(camera_data)}", "DEBUG")
            
            # 记录OpenPI格式数据
            debug_print("VISUALIZE", "正在记录数据到Rerun...", "INFO")
            for frame_idx in tqdm(range(max_frames), desc="记录帧数据", disable=not verbose):
                rr.set_time("frame", sequence=frame_idx)
                
                # 机器人数据 - 只在有数据时记录
                if robot_data:
                    for data_name, data_array in robot_data.items():
                        if data_array is not None and len(data_array) > 0:
                            log_timeseries_data(f"robot/{data_name}", data_array, frame_idx, "dim")
                
                # 相机图像 - 只在有相机数据时记录
                if camera_data:
                    for camera_name, cam_dataset in camera_data.items():
                        if frame_idx < len(cam_dataset):
                            image = extract_images_from_dataset(cam_dataset, frame_idx)
                            if image is not None:
                                # 确保图像格式正确
                                if image.dtype != np.uint8:
                                    if image.max() > 0:
                                        image = (image - image.min()) / (image.max() - image.min()) * 255
                                    image = image.astype(np.uint8)
                                
                                if len(image.shape) == 2:
                                    image = np.stack([image, image, image], axis=-1)
                                elif len(image.shape) == 3 and image.shape[2] == 4:
                                    image = image[:, :, :3]
                                
                                rr.log(f"cameras/{camera_name}", rr.Image(image))
        
        else:  # custom format
            datasets, images, tactile_images, max_frames = visualize_custom_format(f, verbose)
            
            if max_frames == 0:
                debug_print("VISUALIZE", f"文件 {hdf5_path} 中没有找到有效数据", "WARNING")
                return
            
            if verbose:
                debug_print("DATA_STATS", f"总帧数: {max_frames}", "DEBUG")
                if datasets:
                    debug_print("DATA_STATS", f"数据集数量: {len(datasets)}", "DEBUG")
                    debug_print("DATA_STATS", "数据集:", "DEBUG")
                    for name, data in list(datasets.items())[:10]:  # 只显示前10个
                        debug_print("DATA_STATS", f"  - {name}: {data.shape}", "DEBUG")
                if images:
                    debug_print("DATA_STATS", f"图像数量: {len(images)}", "DEBUG")
                    debug_print("DATA_STATS", "图像:", "DEBUG")
                    for name in list(images.keys())[:10]:
                        debug_print("DATA_STATS", f"  - {name}", "DEBUG")
                if tactile_images:
                    debug_print("DATA_STATS", f"触觉图像数量: {len(tactile_images)}", "DEBUG")
                    debug_print("DATA_STATS", "触觉图像:", "DEBUG")
                    for name in list(tactile_images.keys())[:10]:
                        debug_print("DATA_STATS", f"  - {name}", "DEBUG")
            
            # 记录自定义格式数据
            debug_print("VISUALIZE", "正在记录数据到Rerun...", "INFO")
            if verbose:
                debug_print("VISUALIZE", f"将要记录的数据类型:", "DEBUG")
                debug_print("VISUALIZE", f"  - 数值数据: {len(datasets) if datasets else 0}", "DEBUG")
                debug_print("VISUALIZE", f"  - 普通图像: {len(images) if images else 0}", "DEBUG")
                debug_print("VISUALIZE", f"  - 触觉图像: {len(tactile_images) if tactile_images else 0}", "DEBUG")
            
            for frame_idx in tqdm(range(max_frames), desc="记录帧数据", disable=not verbose):
                rr.set_time("frame", sequence=frame_idx)
                
                # 数值数据 - 只在有数据时记录
                if datasets:
                    for data_name, data_array in datasets.items():
                        if data_array is not None and len(data_array) > 0:
                            log_timeseries_data(f"data/{data_name}", data_array, frame_idx, "value")
                
                # 普通图像数据 - 只在有图像数据时记录
                if images:
                    for img_name, img_dataset in images.items():
                        if frame_idx < len(img_dataset):
                            image = extract_images_from_dataset(img_dataset, frame_idx)
                            if image is not None:
                                # 确保图像格式正确
                                if image.dtype != np.uint8:
                                    if image.max() > 0:
                                        image = (image - image.min()) / (image.max() - image.min()) * 255
                                    image = image.astype(np.uint8)
                                
                                if len(image.shape) == 2:
                                    image = np.stack([image, image, image], axis=-1)
                                elif len(image.shape) == 3 and image.shape[2] == 4:
                                    image = image[:, :, :3]
                                
                                rr.log(f"images/{img_name}", rr.Image(image))
                
                # 触觉图像数据（用热力图显示） - 只在有触觉数据时记录
                if tactile_images:
                    # 在第一帧时记录调试信息
                    if frame_idx == 0 and verbose:
                        debug_print("VISUALIZE", f"开始记录 {len(tactile_images)} 个触觉图像数据集", "DEBUG")
                    
                    for tactile_name, tactile_dataset in tactile_images.items():
                        if frame_idx < len(tactile_dataset):
                            tactile_frame = extract_images_from_dataset(tactile_dataset, frame_idx)
                            if tactile_frame is not None:
                                # 如果是2D数据，应用热力图
                                if len(tactile_frame.shape) == 2:
                                    tactile_colored = apply_tactile_colormap(tactile_frame)
                                    rr.log(f"tactile/{tactile_name}_heatmap", rr.Image(tactile_colored))
                                    # 同时记录原始数据
                                    rr.log(f"tactile/{tactile_name}_raw", rr.Tensor(tactile_frame))
                                else:
                                    # 如果已经是彩色图像，直接显示
                                    if tactile_frame.dtype != np.uint8:
                                        if tactile_frame.max() > 0:
                                            tactile_frame = (tactile_frame - tactile_frame.min()) / (tactile_frame.max() - tactile_frame.min()) * 255
                                        tactile_frame = tactile_frame.astype(np.uint8)
                                    rr.log(f"tactile/{tactile_name}", rr.Image(tactile_frame))
        
        if verbose:
            debug_print("VISUALIZE", f"完成记录 {max_frames} 帧数据", "INFO")


def visualize_folder_with_rerun(folder_path, verbose=False):
    """
    使用Rerun可视化文件夹中的所有HDF5文件（递归搜索）
    
    Parameters:
        folder_path: 文件夹路径
        verbose: 是否显示详细信息
    """
    if not os.path.exists(folder_path):
        debug_print("FOLDER", f"文件夹不存在: {folder_path}", "ERROR")
        return
    
    # 递归查找所有HDF5文件
    hdf5_files = []
    for root, dirs, files in os.walk(folder_path):
        for file in files:
            if file.endswith('.hdf5') or file.endswith('.h5'):
                hdf5_files.append(os.path.join(root, file))
    
    if not hdf5_files:
        debug_print("FOLDER", f"在文件夹 {folder_path} 中未找到HDF5文件", "ERROR")
        return
    
    debug_print("FOLDER", f"找到 {len(hdf5_files)} 个HDF5文件", "INFO")
    
    # 为每个文件创建一个独立的recording
    for i, hdf5_file in enumerate(hdf5_files):
        file_name = os.path.basename(hdf5_file)
        rel_path = os.path.relpath(hdf5_file, folder_path)
        
        # 为每个文件创建独立的应用ID
        rr.init(f"hdf5_visualization/{rel_path}", spawn=False)
        
        if verbose:
            debug_print("FOLDER", f"[{i+1}/{len(hdf5_files)}] 处理文件: {rel_path}", "INFO")
        
        try:
            visualize_hdf5_with_rerun(hdf5_file, verbose=verbose)
        except Exception as e:
            debug_print("FOLDER", f"处理文件 {file_name} 时出错: {e}", "ERROR")
            if verbose:
                import traceback
                traceback.print_exc()


def explore_hdf5_structure(hdf5_path):
    """
    探索并打印HDF5文件结构
    
    Parameters:
        hdf5_path: HDF5文件路径
    """
    debug_print("EXPLORE", f"HDF5文件结构: {os.path.basename(hdf5_path)}", "INFO")
    with h5py.File(hdf5_path, 'r') as f:
        def print_structure(name, obj, indent=0):
            prefix = "  " * indent
            if isinstance(obj, h5py.Dataset):
                debug_print("EXPLORE", f"{prefix}📊 数据集: {name}", "DEBUG")
                debug_print("EXPLORE", f"{prefix}   形状: {obj.shape}, 类型: {obj.dtype}", "DEBUG")
            elif isinstance(obj, h5py.Group):
                debug_print("EXPLORE", f"{prefix}📁 组: {name}", "DEBUG")
                for key in obj.keys():
                    print_structure(f"{name}/{key}", obj[key], indent + 1)
        
        for key in f.keys():
            print_structure(key, f[key])


def main():
    os.environ["INFO_LEVEL"] = "INFO"
    parser = argparse.ArgumentParser(
        description='使用Rerun进行HDF5机器人数据可视化',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 可视化单个文件（在浏览器中打开）
  python visual_hdf5_rerun.py data.hdf5
  
  # 可视化文件夹中的所有文件
  python visual_hdf5_rerun.py /path/to/folder/
  
  # 保存为.rrd文件供后续查看
  python visual_hdf5_rerun.py data.hdf5 --save output.rrd
  
  # 连接到远程Rerun查看器
  python visual_hdf5_rerun.py data.hdf5 --connect
  
  # 查看文件结构
  python visual_hdf5_rerun.py data.hdf5 --explore
        """
    )
    
    parser.add_argument('input_path', help='输入HDF5文件或包含HDF5文件的文件夹路径')
    parser.add_argument('-v', '--verbose', action='store_true', help='启用详细输出')
    parser.add_argument('-s', '--save', type=str, help='保存为.rrd文件路径')
    parser.add_argument('-c', '--connect', action='store_true', 
                       help='连接到远程Rerun查看器 (需要先运行 `rerun`)')
    parser.add_argument('--explore', action='store_true', help='仅探索文件结构，不进行可视化')
    parser.add_argument('--addr', type=str, default='127.0.0.1:9876',
                       help='远程Rerun查看器地址 (默认: 127.0.0.1:9876)')
    
    args = parser.parse_args()
    
    # 检查输入路径
    if not os.path.exists(args.input_path):
        debug_print("MAIN", f"路径不存在: {args.input_path}", "ERROR")
        sys.exit(1)
    
    # 如果只是探索结构
    if args.explore:
        if os.path.isfile(args.input_path):
            explore_hdf5_structure(args.input_path)
        else:
            hdf5_files = [f for f in os.listdir(args.input_path) 
                         if f.endswith('.hdf5') or f.endswith('.h5')]
            if not hdf5_files:
                debug_print("MAIN", f"在文件夹 {args.input_path} 中未找到HDF5文件", "ERROR")
                sys.exit(1)
            for hdf5_file in hdf5_files:
                explore_hdf5_structure(os.path.join(args.input_path, hdf5_file))
        return
    
    # 初始化Rerun
    app_id = f"hdf5_visualization/{Path(args.input_path).stem}"
    
    if args.save:
        # 保存模式
        debug_print("MAIN", f"将数据保存到: {args.save}", "INFO")
        rr.init(app_id, spawn=False)
        rr.save(args.save)
    elif args.connect:
        # 连接到远程查看器
        debug_print("MAIN", f"连接到Rerun查看器: {args.addr}", "INFO")
        debug_print("MAIN", "请确保已运行: rerun", "INFO")
        rr.init(app_id, spawn=False)
        rr.connect(args.addr)
    else:
        # 默认：在浏览器中打开
        debug_print("MAIN", "启动Rerun查看器...", "INFO")
        rr.init(app_id, spawn=True)
    
    # 处理输入
    if os.path.isfile(args.input_path):
        # 单个文件
        debug_print("MAIN", f"可视化文件: {args.input_path}", "INFO")
        try:
            visualize_hdf5_with_rerun(args.input_path, verbose=args.verbose)
            debug_print("MAIN", "可视化完成!", "INFO")
            debug_print("MAIN", "提示: 在Rerun查看器中可以:", "INFO")
            debug_print("MAIN", "  - 使用时间轴滑块回放数据", "INFO")
            debug_print("MAIN", "  - 点击左侧面板展开/折叠数据项", "INFO")
            debug_print("MAIN", "  - 使用鼠标缩放和平移图像", "INFO")
            debug_print("MAIN", "  - 同时查看多个数据流", "INFO")
        except Exception as e:
            debug_print("MAIN", f"处理失败: {e}", "ERROR")
            if args.verbose:
                import traceback
                traceback.print_exc()
            sys.exit(1)
    else:
        # 文件夹
        debug_print("MAIN", f"可视化文件夹: {args.input_path}", "INFO")
        visualize_folder_with_rerun(args.input_path, verbose=args.verbose)
        debug_print("MAIN", "批量可视化完成!", "INFO")
    
    # 如果是保存模式，不需要等待
    if not args.save:
        debug_print("MAIN", "按 Ctrl+C 退出", "INFO")
        try:
            # 保持程序运行，以便查看器保持打开
            import time
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            debug_print("MAIN", "退出程序", "INFO")


if __name__ == "__main__":
    main()

