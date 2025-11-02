import sys
sys.path.append("./")

import numpy as np

from my_robot.base_robot import Robot

from controller.TestArm_controller import TestArmController
from controller.TestMobile_controller import TestMobileController
from sensor.TestVision_sensor import TestVisonSensor
from utils.data_handler import debug_print
from data.collect_any import CollectAny

from utils.data_transofrm_pipeline import image_rgb_encode_pipeline, general_hdf5_rdt_format_pipeline

condition = {
    "save_path": "./save/", 
    "task_name": "test_robot", 
    "save_format": "hdf5", 
    "save_freq": 30,
}

class TestRobot(Robot):
    def __init__(self, condition=condition, move_check=True, start_episode=0, DoFs=6,INFO="DEBUG"):
        super().__init__(condition=condition, move_check=move_check, start_episode=start_episode)  
        
        self.INFO = INFO
        self.DoFs = DoFs
        self.controllers = {
            "arm": {
                "left_arm": TestArmController("left_arm",DoFs=self.DoFs,INFO=self.INFO),
                "right_arm": TestArmController("right_arm",DoFs=self.DoFs,INFO=self.INFO),
            },
            "mobile": {
                "test_mobile": TestMobileController("test_mobile",INFO=self.INFO),
            }
        }
        self.sensors = {
            "image": {
                "cam_head": TestVisonSensor("cam_head",INFO=self.INFO),
                "cam_left_wrist": TestVisonSensor("cam_left_wrist",INFO=self.INFO),
                "cam_right_wrist": TestVisonSensor("cam_right_wrist",INFO=self.INFO),
            }, 
        }

        # self.collection._add_data_transform_pipeline(image_rgb_encode_pipeline)
        # self.collection._add_data_transform_pipeline(general_hdf5_rdt_format_pipeline)

    def reset(self):
        self.controllers["arm"]["left_arm"].reset(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        self.controllers["arm"]["right_arm"].reset(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    
    def set_up(self):
        super().set_up()

        self.controllers["arm"]["left_arm"].set_up()
        self.controllers["arm"]["right_arm"].set_up()
        self.controllers["mobile"]["test_mobile"].set_up()
        self.sensors["image"]["cam_head"].set_up(is_depth=False)
        self.sensors["image"]["cam_left_wrist"].set_up(is_depth=False)
        self.sensors["image"]["cam_right_wrist"].set_up(is_depth=False)
        self.set_collect_type({"arm": ["joint","qpos","gripper"],
                               "mobile": ["move_velocity", "position"],
                               "image": ["color"],
                               })
    
    def is_start(self):
        return True

if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "DEBUG" # DEBUG , INFO, ERROR
    
    robot = TestRobot()

    robot.set_up()

    robot.get()

    for i in range(10):
        data = robot.get()
        robot.collect(data)
    robot.finish()

    data_path = os.path.join(condition["save_path"], condition["task_name"], "0.hdf5")
    robot.replay(data_path, key_banned=["qpos"], is_collect=True, episode_id=100)

    move_data = {
        "arm":{
            "left_arm":{
                "joint":np.random.rand(6) * 3.1515926
            },
            "right_arm":{
                "joint":np.random.rand(6) * 3.1515926
            }
        }
    }
    robot.move(move_data)

    move_data = {
        "mobile":{
            "test_mobile":{
                "move_to":np.random.rand(6) * 3.1515926
            },
        }
    }
