import sys
sys.path.append("./")

import numpy as np

from controller.Piper_controller import PiperController
from sensor.Realsense_sensor import RealsenseSensor
from data.collect_any import CollectAny

CAMERA_SERIALS = {
    'head': '111',  # Replace with actual serial number
    'wrist': '111',   # Replace with actual serial number
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
    "save_path": "./datasets/", 
    "task_name": "test", 
    "save_format": "hdf5", 
    "save_interval": 10, 
}


class PiperSingle:
    def __init__(self, start_episode=0):
        self.condition = condition
        self.arm_controllers = {
            "left_arm": PiperController("left_arm"),
        }
        self.image_sensors = {
            "cam_head": RealsenseSensor("cam_head"),
            "cam_wrist": RealsenseSensor("cam_wrist"),
        }
        self.collection = CollectAny(condition, start_episode=start_episode)

    # ============== init ==============
    def reset(self):
        self.arm_controllers["left_arm"].reset(START_POSITION_ANGLE_LEFT_ARM)

    def set_up(self):
        self.arm_controllers["left_arm"].set_up("can0")

        self.image_sensors["cam_head"].set_up(CAMERA_SERIALS["head"])
        self.image_sensors["cam_wrist"].set_up(CAMERA_SERIALS["wrist"])

        self.set_collect_type(["joint","qpos","gripper"],["color"])
        print("set up success!")

    def set_collect_type(self,ARM_INFO_NAME,IMG_INFO_NAME):
        for controller in self.arm_controllers.values():
            controller.set_collect_info(ARM_INFO_NAME)
        for sensor in self.image_sensors.values():
            sensor.set_collect_info(IMG_INFO_NAME)

    def is_start(self):
        return True
        if abs(self.arm_controllers["left_arm"].get_state()["joint"] - np.array(START_POSITION_ANGLE_LEFT_ARM)) > 0.01:
            return True
        else:
            return False
        
    # ============== arm info ==============
    def get(self):
        controller_data = {}
        if self.arm_controllers is not None:    
            for controller_name, controller in self.arm_controllers.items():
                controller_data[controller_name] = controller.get()
        sensor_data = {}
        if self.image_sensors is not None:  
            for sensor_name, sensor in self.image_sensors.items():
                sensor_data[sensor_name] = sensor.get()
        return [controller_data, sensor_data]
    
    def collect(self, data):
        self.collection.collect(data[0], data[1])
    
    def finish(self):
        self.collection.write()
    
    # ============== arm control ==============
    def set_action(self, action):
        pass
    
    def move(self, move_data):  
        self.arm_controllers["left_arm"].move(move_data["left_arm"],is_delta=False)

if __name__=="__main__":
    import time
    robot = PiperSingle()
    robot.set_up()
    # collection test
    data_list = []
    for i in range(100):
        print(i)
        data = robot.get()
        robot.collect(data)
        time.sleep(0.1)
    robot.finish()
    
    # moving test
    move_data = {
        "left_arm":{
        "qpos":[0.057, 0.0, 0.216, 0.0, 0.085, 0.0],
        "gripper":0.2,
        },
    }
    
    move_data = {
        "left_arm":{
        "qpos":[0.060, 0.0, 0.260, 0.0, 0.085, 0.0],
        "gripper":0.2,
        },
    }