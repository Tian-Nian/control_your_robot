import sys
sys.path.append("./")

import numpy as np

from controller.Piper_controller import PiperController
from sensor.Realsense_sensor import RealsenseSensor
from data.collect_any import CollectAny

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


class Camera:
    def __init__(self, start_episode=0):
        self.condition = condition
        self.image_sensors = {
            # "cam_head": RealsenseSensor("cam_head"),
            "cam_wrist": RealsenseSensor("cam_wrist"),
        }
        self.collection = CollectAny(condition, start_episode=0)

    # ============== 初始化相关 ==============

    def reset(self):
        return
    
    def set_up(self):
        # self.image_sensors["cam_head"].set_up(CAMERA_SERIALS["head"])
        self.image_sensors["cam_wrist"].set_up(CAMERA_SERIALS["wrist"])

        self.set_collect_type(["color"])
        print("set up success!")

    def set_collect_type(self,IMG_INFO_NAME):
        for sensor in self.image_sensors.values():
            sensor.set_collect_info(IMG_INFO_NAME)

    # ============== 机械臂判定相关 ============== 
    def is_start(self):
        return True
        if abs(self.arm_controllers["left_arm"].get_state()["joint"] - np.array(START_POSITION_ANGLE_LEFT_ARM)) > 0.01:
            return True
        else:
            return False
        
    # ============== 数据操作相关 ==============
    def get(self):
        controller_data = {}
        sensor_data = {}
        if self.image_sensors is not None:  
            for sensor_name, sensor in self.image_sensors.items():
                sensor_data[sensor_name] = sensor.get()
        return [controller_data, sensor_data]
    
    def collect(self, data):
        self.collection.collect(data[0], data[1])
    
    def finish(self):
        self.collection.write()
    
    # ============== 运动操作相关 ==============
    def set_action(self, action):
        pass
    
    def move(self, move_data):  
        self.arm_controllers["left_arm"].move(move_data["left_arm"],is_delta=False)

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
