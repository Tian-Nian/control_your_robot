
import numpy as np

from controller.Piper_controller import PiperController
from sensor.Realsense_sensor import RealsenseSensor
# from sensor.Vitac3D import Vitac3D
from data.collect_any import CollectAny
from controller.drAloha_controller import DrAlohaController
from my_robot.base_robot import Robot
from typing import Dict, Any
import math
import time
# 组装你的控制器
CAMERA_SERIALS = {
    'head': '420122070816',  # Replace with actual serial number
    # 'left_wrist': '948122073452',   # Replace with actual serial number
    'right_wrist': '338622074268',   # Replace with actual serial number
}

# Define start position (in degrees)
START_POSITION_ANGLE_LEFT_ARM = [
    0,   # Joint 1
    0,    # Joint 2
    0,  # Joint 3
    0,   # Joint 4
    0,  # Joint 5
    0,    # Joint 6
]

# Define start position (in degrees)
START_POSITION_ANGLE_RIGHT_ARM = [
    0,   # Joint 1
    0,    # Joint 2
    0,  # Joint 3
    0,   # Joint 4
    0,  # Joint 5
    0,    # Joint 6
]

condition = {
    "save_path": "./save/", # 保存路径
    "task_name": "test", # 任务名称
    "save_format": "hdf5", # 保存格式
    "save_interval": 30, # 保存频率
}

class dr_Single(Robot):
    def __init__(self, condition=condition, move_check=True, start_episode=0):        
        self.controllers = {
            "arm": {
                "right_arm": DrAlohaController("right_arm"),
            },
        }
        self.sensors = {
            "image": {
                "cam_head": RealsenseSensor("cam_head"),
                # "cam_left_wrist": RealsenseSensor("cam_left_wrist"),
                "cam_right_wrist": RealsenseSensor("cam_right_wrist"),
            },
        }
        
        super().__init__(condition=condition, move_check=move_check, start_episode=start_episode)

        
    #============== 初始化相关 ==============
    def reset(self):
        self.controllers["arm"]["right_arm"].reset(START_POSITION_ANGLE_RIGHT_ARM)

    def set_up(self,is_master=False):
        super().set_up()
        self.controllers["arm"]["right_arm"].set_up("/dev/ttyACM1")
        if is_master :
            self.set_collect_type({"arm": ["joint","gripper"]})
            
        else:
            self.sensors["image"]["cam_head"].set_up(CAMERA_SERIALS["head"], is_depth=False)
            self.sensors["image"]["cam_right_wrist"].set_up(CAMERA_SERIALS["right_wrist"], is_depth=False)
            self.set_collect_type({"arm": ["joint","gripper"],
                               "image": ["color"]
                               })
        print("set up success!")

    def action_transform(self, move_data:Dict[str, Any]):
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
         # 关键修改1：提取关节数据并转换为列表
        # 直接使用 NumPy 数组，无需转换
        joints = move_data["joint"].copy()
        
        # 关键修改2：关节角度校准（度→弧度转换后调整）
        joints[1] = joints[1] - math.radians(90)   # 关节2校准：减去90°
        joints[2] = joints[2] + math.radians(175)  # 关节3校准：增加175°
        
        # 关键修改3：关节方向反转（特定关节取负）
        for i in [1, 2, 4]:  # 关节2、3、5需反转方向
            joints[i] = -joints[i]
        left_joints = [
            clamp(joints[i], joint_limits_rad[i][0], joint_limits_rad[i][1])
        for i in range(6)
        ]
        # gripper_data = move_data["gripper"]
        gripper = move_data["gripper"]
        
        # 5. 构建输出结构
        action = {
            "arm": {
                "joint": left_joints,
                "gripper": gripper
            }
        }
        return action
    def teleoperation_setp(self,is_record=False,force_feedback=False):
        
        master_action=self.controllers["arm"]["right_arm"].get_state()
        action=self.action_transform(master_action)
        return action
if __name__ == "__main__":
    robot = dr_Single()
    robot.set_up()
    # robot.controllers["arm"]["right"].zero_gravity()
    # 采集测试
    data_list = []
    for i in range(100):
        print(i)
        data = robot.get()
        robot.collect(data)
        time.sleep(0.1)
    robot.finish()