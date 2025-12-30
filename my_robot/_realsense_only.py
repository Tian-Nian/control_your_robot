import sys
sys.path.append("./")

import numpy as np

from robot.robot.base_robot import Robot

from robot.controller.Piper_controller import PiperController
from robot.sensor.Realsense_sensor import RealsenseSensor

from robot.data.collect_any import CollectAny

# 组装你的控制器
CAMERA_SERIALS = {
    # 'head': '419522072373',  # Replace with actual serial number
    'wrist': '419522071856',   # Replace with actual serial number
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
    "robot":"piper_single",
    "save_path": "./datasets/", # 保存路径
    "task_name": "test", # 任务名称
    "save_format": "hdf5", # 保存格式
    "save_interval": 10, # 保存频率
}


class Camera(Robot):
    def __init__(self, start_episode=0):
        super().__init__(start_episode)

        self.condition = condition
        self.sensors = {
            "image":{
                # "cam_head": RealsenseSensor("cam_head"),
                "cam_wrist": RealsenseSensor("cam_wrist"),
            }
        }
    # ============== 初始化相关 ==============

    def reset(self):
        return
    
    def set_up(self):
        # self.sensors["image"]["cam_head"].set_up(CAMERA_SERIALS["head"])
        self.sensors["image"]["cam_wrist"].set_up(CAMERA_SERIALS["wrist"])

        self.set_collect_type({"image": ["color"]
                               })

        print("set up success!")

if __name__=="__main__":
    import time
    robot = Camera()
    robot.set_up()
    # 采集测试
    data_list = []
    for i in range(100):
        print(i)
        data = robot.get()
        robot.collect(data)
        time.sleep(0.1)
    robot.finish()
