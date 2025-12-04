import sys

sys.path.append("src/")
import os
import numpy as np
import torch
import math
from my_robot.agilex_piper_single_base import PiperSingle
from policy.DP.inference_model import MYDP
from utils.data_handler import is_enter_pressed
from data.collect_any import CollectAny
import time
import pdb
condition = {
    "save_path": "./test/", 
    "task_name": "feed_test_dp", 
    "save_format": "hdf5", 
    "save_freq": 15,
    "collect_type": "teleop",
}
def smooth_action_transition(prev_action_chunk, new_action_chunk, overlap_size=3, strategy="skip", 
                            temporal_ensemble=False, alpha=0.3):
    """
    平滑处理相邻动作块之间的过渡
    
    参数:
        prev_action_chunk: 上一个动作块 (numpy array, shape: [T1, action_dim])
        new_action_chunk: 新生成的动作块 (numpy array, shape: [T2, action_dim])
        overlap_size: 重叠的动作数量，默认为3
        strategy: 平滑策略
            - "skip": 直接跳过新动作块的前overlap_size个动作（默认）
            - "blend": 对重叠部分进行线性混合
            - "weighted": 使用加权平均，越靠近新动作块权重越大
            - "exponential": 指数移动平均 (EMA)，最平滑
            - "temporal_ensemble": 时序集成，综合考虑历史动作
        temporal_ensemble: 是否使用时序集成（对整个序列进行平滑）
        alpha: EMA 平滑系数 (0-1)，越小越平滑，推荐 0.2-0.4
    
    返回:
        smoothed_chunk: 平滑处理后的动作块
    """
    if prev_action_chunk is None:
        # 第一个动作块，无需平滑
        return new_action_chunk
    
    if len(new_action_chunk) <= overlap_size:
        # 新动作块太短，直接返回
        return new_action_chunk
    
    if strategy == "skip":
        # 策略1：直接跳过新动作块的前overlap_size个动作
        # 因为它们与上一个动作块的后overlap_size个动作一致
        smoothed_chunk = new_action_chunk[overlap_size:]
        
    elif strategy == "blend":
        # 策略2：线性混合重叠部分
        # 取上一个动作块的后overlap_size个和新动作块的前overlap_size个进行混合
        if len(prev_action_chunk) >= overlap_size:
            prev_tail = prev_action_chunk[-overlap_size:]
            new_head = new_action_chunk[:overlap_size]
            
            # 线性插值：从prev逐渐过渡到new
            blended = []
            for i in range(overlap_size):
                alpha = (i + 1) / (overlap_size + 1)  # 权重从0逐渐增加到1
                blended_action = (1 - alpha) * prev_tail[i] + alpha * new_head[i]
                blended.append(blended_action)
            
            # 组合：混合后的重叠部分 + 新动作块的剩余部分
            smoothed_chunk = np.vstack([
                np.array(blended),
                new_action_chunk[overlap_size:]
            ])
        else:
            smoothed_chunk = new_action_chunk
            
    elif strategy == "weighted":
        # 策略3：加权平均，使用余弦权重
        if len(prev_action_chunk) >= overlap_size:
            prev_tail = prev_action_chunk[-overlap_size:]
            new_head = new_action_chunk[:overlap_size]
            
            # 使用余弦函数生成平滑的权重
            blended = []
            for i in range(overlap_size):
                # 权重从1平滑过渡到0（对于prev）
                weight_prev = 0.5 * (1 + np.cos(np.pi * (i + 1) / (overlap_size + 1)))
                weight_new = 1 - weight_prev
                
                blended_action = weight_prev * prev_tail[i] + weight_new * new_head[i]
                blended.append(blended_action)
            
            # 组合：混合后的重叠部分 + 新动作块的剩余部分
            smoothed_chunk = np.vstack([
                np.array(blended),
                new_action_chunk[overlap_size:]
            ])
        else:
            smoothed_chunk = new_action_chunk
    
    elif strategy == "exponential":
        # 策略4：指数移动平均 (EMA) - 最适合解决"开倒车"问题
        # 对整个新动作块应用EMA平滑
        if len(prev_action_chunk) > 0:
            smoothed_chunk = []
            # 使用上一个动作块的最后一个动作作为起点
            prev_action = prev_action_chunk[-1]
            
            for i, new_action in enumerate(new_action_chunk):
                # 动态调整alpha：越往后alpha越大（越接近新动作）
                # 这样可以在保持平滑的同时，避免过度滞后
                dynamic_alpha = alpha * (1.0 + i / len(new_action_chunk) * 0.5)
                dynamic_alpha = min(dynamic_alpha, 0.8)  # 限制最大值
                
                # EMA: smoothed = alpha * new + (1 - alpha) * prev
                smoothed_action = dynamic_alpha * new_action + (1 - dynamic_alpha) * prev_action
                smoothed_chunk.append(smoothed_action)
                prev_action = smoothed_action  # 更新为当前平滑后的动作
            
            smoothed_chunk = np.array(smoothed_chunk)
            # 跳过前几个动作以避免重复
            smoothed_chunk = smoothed_chunk[overlap_size:]
        else:
            smoothed_chunk = new_action_chunk
    
    elif strategy == "temporal_ensemble":
        # 策略5：时序集成 - 结合历史信息，最强平滑
        if len(prev_action_chunk) >= overlap_size:
            # 计算重叠区域的差异
            prev_tail = prev_action_chunk[-overlap_size:]
            new_head = new_action_chunk[:overlap_size]
            
            # 检测是否有大的跳变（"开倒车"的标志）
            diff = np.abs(new_head - prev_tail).mean(axis=1)
            max_diff = diff.max()
            
            if max_diff > 0.1:  # 如果检测到大的跳变（阈值可调）
                # 使用更强的平滑
                blended = []
                for i in range(overlap_size):
                    # 使用三次多项式插值，更平滑
                    t = (i + 1) / (overlap_size + 1)
                    # 平滑曲线：3t^2 - 2t^3
                    alpha = 3 * t**2 - 2 * t**3
                    blended_action = (1 - alpha) * prev_tail[i] + alpha * new_head[i]
                    blended.append(blended_action)
                
                # 对剩余部分也应用轻微平滑
                remaining = new_action_chunk[overlap_size:]
                if len(remaining) > 0:
                    smoothed_remaining = []
                    prev_action = blended[-1]
                    for action in remaining:
                        smoothed_action = 0.7 * action + 0.3 * prev_action
                        smoothed_remaining.append(smoothed_action)
                        prev_action = smoothed_action
                    
                    smoothed_chunk = np.vstack([
                        np.array(blended),
                        np.array(smoothed_remaining)
                    ])
                else:
                    smoothed_chunk = np.array(blended)
            else:
                # 跳变不大，使用普通blend
                blended = []
                for i in range(overlap_size):
                    alpha = (i + 1) / (overlap_size + 1)
                    blended_action = (1 - alpha) * prev_tail[i] + alpha * new_head[i]
                    blended.append(blended_action)
                
                smoothed_chunk = np.vstack([
                    np.array(blended),
                    new_action_chunk[overlap_size:]
                ])
        else:
            smoothed_chunk = new_action_chunk
    
    else:
        raise ValueError(f"未知的平滑策略: {strategy}，请选择 'skip', 'blend', 'weighted', 'exponential', 或 'temporal_ensemble'")
    
    return smoothed_chunk

def apply_velocity_limit(action_chunk, prev_action, max_velocity=0.15):
    """
    应用速度限制，防止机械臂突然的大幅度移动
    
    参数:
        action_chunk: 动作序列 (numpy array, shape: [T, action_dim])
        prev_action: 上一个执行的动作 (numpy array, shape: [action_dim])
        max_velocity: 最大速度限制（单位：弧度/步），默认0.15
    
    返回:
        limited_chunk: 应用速度限制后的动作序列
    """
    if prev_action is None:
        return action_chunk
    
    limited_chunk = []
    current_action = prev_action
    
    for action in action_chunk:
        # 计算动作差异
        delta = action - current_action
        
        # 限制每个关节的变化幅度
        delta_limited = np.clip(delta, -max_velocity, max_velocity)
        
        # 应用限制后的动作
        next_action = current_action + delta_limited
        limited_chunk.append(next_action)
        current_action = next_action
    
    return np.array(limited_chunk)

def input_transform(data):
    has_left_arm = "left_arm" in data[0]
    has_right_arm = "right_arm" in data[0]
    
    if has_left_arm and not has_right_arm:
        left_joint_dim = len(data[0]["left_arm"]["joint"])
        left_gripper_dim = 1
        
        data[0]["right_arm"] = {
            "joint": [0.0] * left_joint_dim,
            "gripper": 0.0
        }
        has_right_arm = True
    
    elif has_right_arm and not has_left_arm:
        right_joint_dim = len(data[0]["right_arm"]["joint"])
        right_gripper_dim = 1
        
        # fill left_arm data
        data[0]["left_arm"] = {
            "joint": [0.0] * right_joint_dim,
            "gripper": 0.0
        }
        has_left_arm = True
    
    elif not has_left_arm and not has_right_arm:
        default_joint_dim = 6
        
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
    
    # 处理图像数据 - 支持不同的相机配置
    if "cam_left_wrist" in data[1] and "cam_right_wrist" in data[1]:
        # 双臂配置：三相机
        img_arr = (
            data[1]["cam_head"]["color"], 
            data[1]["cam_left_wrist"]["color"],
            data[1]["cam_right_wrist"]["color"]
        )
    elif "cam_wrist" in data[1]:
        # 单臂配置：两相机
        img_arr = (
            data[1]["cam_head"]["color"], 
            data[1]["cam_wrist"]["color"]
        )
    else:
        # 只有头部相机
        img_arr = (data[1]["cam_head"]["color"],)
    
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
    if data[6] < 0.05:
        data[6] = 0.0
    left_gripper = data[6]
    
    move_data = {
        "left_arm":{
            "joint": left_joints,
            "gripper": left_gripper,
        }
    }
    return move_data

if __name__ == "__main__":
    os.environ["INFO_LEVEL"] = "INFO"
    robot = PiperSingle()
    robot.set_up()
    collection=CollectAny(condition=condition,start_episode=0,move_check=True,resume=True)
    #load model
    model = MYDP(model_path="policy/DP/checkpoints/feed_test_30-100-0/300.ckpt", task_name="feed_test_30", INFO="DEBUG")
    max_step = 1000
    num_episode = 1
    
    # ==================================
    
    for i in range(num_episode):
        step = 0
        prev_action_chunk = None  # 保存上一个动作块
        prev_executed_action = None  # 保存上一个实际执行的动作（用于速度限制）
        
        # 重置所有信息
        robot.reset()
        model.reset_obsrvationwindows()
        model.random_set_language()
        
        # 等待允许执行推理指令, 按enter开始
        is_start = False
        while not is_start:
            if is_enter_pressed():
                is_start = True
                print("start to inference...")
            else:
                print("waiting for start command...")
                time.sleep(1)

        # 开始逐条推理运行
        while step < max_step:
            data = robot.get()
            img_arr, state = input_transform(data)
            model.update_observation_window(img_arr, state)
            action_chunk = model.get_action(model.observation_window)
            
            # pdb.set_trace()
            for action in action_chunk:
                # 将action数据转换为collect_any期望的格式
                move_data = output_transform(action)
                robot.move({"arm": 
                            move_data
                        })
                
                # 更新上一个执行的动作（用于下次速度限制）
                prev_executed_action = action.copy()
                
                step += 1
                data = robot.get()
                collection.collect(data[0],None)
                
                time.sleep(1/robot.condition["save_freq"])
                print(f"Episode {i}, Step {step}/{max_step} completed.")
        time.sleep(1)
        robot.reset()
        collection.write()
        print("finish episode", i)
    robot.reset()


