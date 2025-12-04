import sys
sys.path.append("src/")

import select

from my_robot.test_robot import TestRobot
from my_robot.agilex_piper_single_base import PiperSingle
from controller.drAloha_controller import DrAlohaController
import time
import math
from utils.data_handler import is_enter_pressed,debug_print
from typing import Dict, Any

condition = {
    "save_path": "./save/", 
    "task_name": "pick_place_cup", 
    "save_format": "hdf5", 
    "save_freq": 30,
    "collect_type": "teleop",
}
def action_transform(move_data:Dict[str, Any]):
        """ Transform the action from master arm to the slave arm."""
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
        # 直接使用 NumPy 数组，无需转换
        joints = move_data["joint"].copy()
        
        joints[1] = joints[1] - math.radians(90)   # 关节2校准：减去90°
        joints[2] = joints[2] + math.radians(175)  # 关节3校准：增加175°
        
        for i in [1, 2, 4]:  # 关节2、3、5需反转方向
            joints[i] = -joints[i]
        left_joints = [
            clamp(joints[i], joint_limits_rad[i][0], joint_limits_rad[i][1])
        for i in range(6)
        ]
        left_joints = [float(joint) for joint in left_joints]
        # gripper_data = move_data["gripper"]
        gripper = move_data["gripper"]
        
        action = {
                "joint": left_joints,
                "gripper": gripper,
        }
        return action
if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "DEBUG" # DEBUG , INFO, ERROR
    controller = DrAlohaController(name="controller")
    controller.set_collect_info(["joint","gripper"])
    gravity_update_interval = 0.1  # 10Hz 更新频率
    controller.set_up(com="/dev/ttyACM1")
    robot = PiperSingle()
    robot.set_up()
    num_episode = 5
    robot.condition["task_name"] = "my_test"

    # 初始化上次重力补偿更新时间（使用单调时钟，避免系统时间变化影响）
    last_gravity_update = time.monotonic()

    for _ in range(num_episode):
        controller.apply_calibration()
        robot.reset()
        debug_print("main", "Press Enter to start...", "INFO")
        while not robot.is_start() or not is_enter_pressed():
            time.sleep(1/robot.condition["save_freq"])
        controller.zero_gravity()
        debug_print("main", "Press Enter to finish...", "INFO")

        avg_collect_time = 0.0
        collect_num = 0
        while True:
            last_time = time.monotonic()
            
            data = controller.get()
            action = action_transform(data)
            # 更新重力补偿（使用单调时钟）
            current_time = time.monotonic()
            if current_time - last_gravity_update >= gravity_update_interval:
                controller.update_gravity()
                last_gravity_update = current_time
            
            robot.move({"arm": 
                                {
                                    "left_arm": action
                                }
                            })
            data = robot.get()
            robot.collect(data)
            
            if is_enter_pressed():
                robot.finish()
                break
                
            collect_num += 1
            while True:
                now = time.monotonic()
                if now -last_time > 1/robot.condition["save_freq"]:
                    avg_collect_time += now -last_time
                    break
                else:
                    time.sleep(0.001)
        extra_info = {}
        avg_collect_time = avg_collect_time / collect_num
        extra_info["avg_time_interval"] = avg_collect_time
        robot.collection.add_extra_condition_info(extra_info)