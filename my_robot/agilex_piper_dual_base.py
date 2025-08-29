import sys
sys.path.append("./")

import numpy as np

from my_robot.base_robot import Robot

from controller.Piper_controller import PiperController
from sensor.Realsense_sensor import RealsenseSensor
from sensor.VisionROS_sensor import VisionROSensor

from data.collect_any import CollectAny

from utils.data_handler import is_enter_pressed

import time, os

# setting your realsense serial
CAMERA_SERIALS = {
    'head': '342622301553',  # Replace with actual serial number
    'left_wrist': '242522071124',   # Replace with actual serial number
    'right_wrist': '244622071566',   # Replace with actual serial number
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
    "save_path": "./save/", 
    "task_name": "Make_a_beef_sandwichv3",  # Make_a_beef_sandwichv2
    "save_format": "hdf5", 
    "save_freq": 30,
}

class PiperDual(Robot):
    def __init__(self, condition=condition, move_check=True, start_episode=0):
        super().__init__(condition=condition, move_check=move_check, start_episode=start_episode)

        self.controllers = {
            "arm":{
                "left_arm": PiperController("left_arm"),
                "right_arm": PiperController("right_arm"),
            }
        }
        self.sensors = {
            "image": {
                "cam_head": RealsenseSensor("cam_head"),
                "cam_left_wrist": RealsenseSensor("cam_left_wrist"),
                "cam_right_wrist": RealsenseSensor("cam_right_wrist"),
            },
        }

    def set_up(self):
        self.controllers["arm"]["left_arm"].set_up("can_left")
        self.controllers["arm"]["right_arm"].set_up("can_right")

        # self.sensors["image"]["cam_head"].set_up(CAMERA_SERIALS['head'], is_depth=False)
        # self.sensors["image"]["cam_left_wrist"].set_up(CAMERA_SERIALS["left_wrist"], is_depth=False)
        # self.sensors["image"]["cam_right_wrist"].set_up(CAMERA_SERIALS["right_wrist"], is_depth=False)

        self.set_collect_type({"arm": ["joint","qpos","gripper"],
                               "image": ["color"],
                               })
                               
        
        print("set up success!")
    
    def reset(self):
        move_data = {
            "arm":{
                "left_arm":{
                    "joint": np.array([0.0, 0.0, 0.0, 0.0 ,0.0, 0.0]),
                    "gripper": 1.0 / 0.7,
                },
                "right_arm":{
                    "joint": np.array([0.0, 0.0, 0.0, 0.0 ,0.0, 0.0]),
                    "gripper": 1.0 / 0.7,
                }
            }
        }

        self.move(move_data)
        time.sleep(1)


if __name__ == "__main__":
    os.environ["INFO_LEVEL"] = "INFO"

    # import rospy
    # rospy.init_node('ros_subscriber_node', anonymous=True)
    start = 48
    episode_num = 50
    robot = PiperDual(condition=condition, move_check=True)
    
    robot.set_up()
    
    # replay data
    # 回到零位
    move_d = {
        "arm":{
            "left_arm":{
                "joint": np.array([0.0, 0.0, 0.0, 0.0 ,0.0, 0.0]),
                "gripper": 0.0,
            },
            "right_arm":{
                "joint": np.array([0.0, 0.0, 0.0, 0.0 ,0.0, 0.0]),
                "gripper": 1.0,
            }
        }
    }
    robot.move(move_d)

    time.sleep(1)
    # data = robot.get()
    # print(data)
    # exit()
    replay_id = 10
    # robot.show_pic(f"./save/Make_a_beef_sandwichv3/{replay_id}.hdf5", "cam_head")
    # robot.show_pic(f"./save/base/{replay_id}.hdf5", "cam_left_wrist")
    # robot.show_pic(f"./save/base/{replay_id}.hdf5", "cam_right_wrist")
    
    robot.replay(f"./save/Make_a_beef_sandwichv3/10.hdf5", key_banned=["qpos"])
    # robot.replay(f"./save/Make_a_beef_sandwich_dataset/{replay_id}.hdf5", key_banned=["qpos"]) #None
    
    exit()
    # '''
    for i in range(start, start + episode_num):
        time.sleep(3)

        # collection test
        data_list = []

        s = time.time()
        last_time = time.monotonic()
        now = time.monotonic()
        
        while True:
            if is_enter_pressed():
                    break
        
            data = robot.get()
            robot.collect(data)
            
            last_time = time.monotonic()

            while True:
                now = time.monotonic()
                if now - last_time >= 1 / condition["save_freq"]:
                    break
                else:
                    time.sleep(0.001)

            time.sleep(1/condition["save_freq"])
            
        robot.finish(i)
        
        print("collect finish!")
        e = time.time()

        print(f"time {e-s}s")
    #'''