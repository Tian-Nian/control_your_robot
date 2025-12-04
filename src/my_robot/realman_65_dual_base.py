import sys
sys.path.append("src/")

from my_robot.base_robot import Robot

from controller.Realman_controller import RealmanController
from sensor.Realsense_sensor import RealsenseSensor

from data.collect_any import CollectAny

from Robotic_Arm.rm_robot_interface import rm_thread_mode_e

import numpy as np

# 组装你的控制器
CAMERA_SERIALS = {
    'head': '111',  # Replace with actual serial number
    'left_wrist': '111',   # Replace with actual serial number
    'right_wrist': '111',   # Replace with actual serial number
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

# 记录统一的数据操作信息, 相关配置信息由CollectAny补充并保存
condition = {
    "save_path": "./save/",
    "task_name": "test", 
    "save_freq": 10,
}

class MyRobot:
    def __init__(self, condition=condition, move_check=True, start_episode=0):
        super().__init__(condition=condition, move_check=move_check, start_episode=start_episode)

        self.controllers = {
            "arm":{
                "left_arm": RealmanController("left_arm"),
                "right_arm": RealmanController("right_arm"),
            }
        }
        self.sensors = {
            "image":{
                "cam_head": RealsenseSensor("cam_head"),
                "cam_left_wrist": RealsenseSensor("cam_left_wrist"),
                "cam_right_wrist": RealsenseSensor("cam_right_wrist"),
            }
        }

    def set_up(self):
        super().set_up()

        self.controllers["arm"]["left_arm"].set_up("192.168.80.18", rm_thread_mode_e.RM_TRIPLE_MODE_E)
        self.controllers["arm"]["right_arm"].set_up("192.168.80.19", rm_thread_mode_e.RM_TRIPLE_MODE_E)

        self.sensors["image"]["cam_head"].set_up(CAMERA_SERIALS['head'], is_depth=False)
        self.sensors["image"]["cam_left_wrist"].set_up(CAMERA_SERIALS['left_wrist'], is_depth=False)
        self.sensors["image"]["cam_right_wrist"].set_up(CAMERA_SERIALS['right_wrist'], is_depth=False)
        
        self.set_collect_type({"arm": ["joint","qpos","gripper"],
                               "iamge": ["color"]
                               })
        print("set up success!")
        
    def reset(self):
        self.controllers["arm"]["left_arm"].reset(START_POSITION_ANGLE_LEFT_ARM)
        self.controllers["arm"]["right_arm"].reset(START_POSITION_ANGLE_RIGHT_ARM)

    def is_start(self):
        if max(abs(self.controllers["arm"]["left_arm"].get_state()["joint"] - START_POSITION_ANGLE_LEFT_ARM), abs(self.controllers["arm"]["right_arm"].get_state()["joint"] - START_POSITION_ANGLE_RIGHT_ARM)) > 0.01:
            return True
        else:
            return False

if __name__ == "__main__":
    robot = MyRobot()

    robot.reset()
    