

import numpy as np

from robot.robot.base_robot import Robot

from robot.controller.Piper_controller import PiperController
from robot.sensor.Realsense_sensor import RealsenseSensor

from robot.data.collect_any import CollectAny

CAMERA_SERIALS = {
    'head': '111',  # Replace with actual serial number
    'wrist': '111',   # Replace with actual serial number
}

# Define start position (in degrees)
START_POSITION_ANGLE_LEFT_ARM = [
    0.0,   # Joint 1
    0.85220935,    # Joint 2
    -0.68542569,  # Joint 3
    0.,   # Joint 4
    0.78588684,  # Joint 5
    -0.05256932,    # Joint 6
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
    "save_freq": 10, 
}


class PiperSingle(Robot):
    def __init__(self, condition=condition, move_check=True, start_episode=0):
        super().__init__(condition=condition, move_check=move_check, start_episode=start_episode)

        self.condition = condition
        self.controllers = {
            "arm":{
                "left_arm": PiperController("left_arm"),
            },
        }
        self.sensors = {
            "image":{
                "cam_head": RealsenseSensor("cam_head"),
                "cam_wrist": RealsenseSensor("cam_wrist"),
            },
        }

    # ============== init ==============
    def reset(self):
        self.controllers["arm"]["left_arm"].reset(np.array(START_POSITION_ANGLE_LEFT_ARM))

    def set_up(self):
        super().set_up()

        self.controllers["arm"]["left_arm"].set_up("can0")
        self.sensors["image"]["cam_head"].set_up(CAMERA_SERIALS["head"])
        self.sensors["image"]["cam_wrist"].set_up(CAMERA_SERIALS["wrist"])

        self.set_collect_type({"arm": ["joint","qpos","gripper"],
                               "image": ["color"]
                               })
        
        print("set up success!")
    
if __name__=="__main__":
    import time
    robot = PiperSingle()
    robot.set_up()
    # collection test
    robot.reset()
    data_list = []
    for i in range(100):
        print(i)
        data = robot.get()
        robot.collect(data)
        time.sleep(0.1)
    robot.finish()
    
    # moving test
    move_data = {
        "arm":{
            "left_arm":{
            "qpos":[0.057, 0.0, 0.216, 0.0, 0.085, 0.0],
            "gripper":0.2,
            },
        },
    }
    robot.move(move_data)
    time.sleep(1)
    move_data = {
        "arm":{
            "left_arm":{
            "joint":[0.00, 0.0, 0.0, 0.0, 0.0, 0.0],
            "gripper":0.2,
            },
        },
    }
    robot.move(move_data)