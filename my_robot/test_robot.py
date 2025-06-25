import sys
sys.path.append("./")

import numpy as np

from controller.TestArm_controller import TestArmController
from sensor.TestVision_sensor import TestVisonSensor
from utils.data_handler import debug_print
from data.collect_any import CollectAny

condition = {
    "save_path": "./save/", 
    "task_name": "test", 
    "save_format": "hdf5", 
    "save_interval": 10,
}

class TestRobot:
    def __init__(self, DoFs=6,INFO="DEBUG",start_episode=0):
        self.INFO = INFO
        self.DoFs = DoFs
        self.arm_controllers = {
            "left_arm": TestArmController("left_arm",DoFs=self.DoFs,INFO=self.INFO),
            "right_arm": TestArmController("right_arm",DoFs=self.DoFs,INFO=self.INFO),
        }
        self.image_sensors = {
            "cam_head": TestVisonSensor("cam_head",INFO=self.INFO),
            "cam_left_wrist": TestVisonSensor("cam_left_wrist",INFO=self.INFO),
            "cam_right_wrist": TestVisonSensor("cam_right_wrist",INFO=self.INFO),
        }
        self.condition = condition
        self.collection = CollectAny(condition, start_episode=start_episode)
    
    def reset(self):
        self.arm_controllers["left_arm"].reset(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        self.arm_controllers["right_arm"].reset(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    
    def set_up(self):
        self.arm_controllers["left_arm"].set_up()
        self.arm_controllers["right_arm"].set_up()
        self.image_sensors["cam_head"].set_up(is_depth=False)
        self.image_sensors["cam_left_wrist"].set_up(is_depth=False)
        self.image_sensors["cam_right_wrist"].set_up(is_depth=False)
        self.set_collect_type(["joint","qpos","gripper"],["color"])

    def set_collect_type(self,ARM_INFO_NAME,IMG_INFO_NAME):
        for controller in self.arm_controllers.values():
            controller.set_collect_info(ARM_INFO_NAME)
        for sensor in self.image_sensors.values():
            sensor.set_collect_info(IMG_INFO_NAME)
    
    def is_start(self):
        return True
    
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
    
    def set_action(self, action):
        self.arm_controllers["left_arm"].set_action(action)
        self.arm_controllers["right_arm"].set_action(action)
    
    def move(self, move_data):
        self.arm_controllers["left_arm"].move(move_data["left_arm"],is_delta=False)
        self.arm_controllers["right_arm"].move(move_data["right_arm"],is_delta=False)

if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "DEBUG" # DEBUG , INFO, ERROR
    
    robot = TestRobot()

    robot.set_up()

    robot.get()

    move_data = {
        "left_arm":{
            "joint":np.random.rand(6) * 3.1515926
        },
        "right_arm":{
            "joint":np.random.rand(6) * 3.1515926
        }
    }
    robot.move(move_data)

    