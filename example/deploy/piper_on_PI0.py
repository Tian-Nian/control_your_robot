import sys
sys.path.append("./")

from my_robot.agilex_piper_dual import PiperDual

import time
import numpy as np
import math
from policy.openpi.inference_model import PI0_DUAL

from utils.data_handler import is_enter_pressed

joint_limits_rad = [
        (math.radians(-150), math.radians(150)),   # joint1
        (math.radians(0), math.radians(180)),    # joint2
        (math.radians(-170), math.radians(0)),   # joint3
        (math.radians(-100), math.radians(100)),   # joint4
        (math.radians(-70), math.radians(70)),   # joint5
        (math.radians(-120), math.radians(120))    # joint6
    ]
gripper_limit=[(0.00,0.07)]
def input_transform(data):
    state = np.concatenate([
        np.array(data[0]["left_arm"]["joint"]).reshape(-1),
        np.array(data[0]["left_arm"]["gripper"]).reshape(-1),
        np.array(data[0]["right_arm"]["joint"]).reshape(-1),
        np.array(data[0]["right_arm"]["gripper"]).reshape(-1)
    ])


    img_arr = data[1]["cam_head"]["color"], data[1]["cam_right_wrist"]["color"], data[1]["cam_left_wrist"]["color"]
    return img_arr, state

def output_transform(data):
    # 2. 安全限位处理函数
    def clamp(value, min_val, max_val):
        """将值限制在[min_val, max_val]范围内"""
        return max(min_val, min(value, max_val))
    left_joints = [
        clamp(data[i], joint_limits_rad[i][0], joint_limits_rad[i][1])
        for i in range(6)
    ]
    left_gripper = data[6]
    
    # 4. 处理右臂数据
    right_joints = [
        clamp(data[i+7], joint_limits_rad[i][0], joint_limits_rad[i][1])
        for i in range(6)
    ]
    right_gripper = data[13]
    
    # 5. 构建输出结构
    move_data = {
        "left_arm": {
            "joint": left_joints,
            "gripper": left_gripper
        },
        "right_arm": {
            "joint": right_joints,
            "gripper": right_gripper
        }
    }
    return move_data

if __name__ == "__main__":
    robot = PiperDual()
    robot.set_up()
    # load model
    model = PI0_DUAL("your_model_path", "task_instruction")
    max_step = 1000
    num_episode = 10
    
    for i in range(num_episode):
        step = 0
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
            action_chunk = model.get_action()
            action_chunk = action_chunk[:30]
            for action in action_chunk:
                move_data = output_transform(action)
                robot.move(move_data)
                step += 1
                time.sleep(1/30)

        print("finish episode", i)