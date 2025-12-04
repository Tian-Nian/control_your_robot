import sys
sys.path.append("src/")

import numpy as np

from my_robot.base_robot import Robot

from controller.Piper_controller import PiperController
from sensor.Realsense_sensor import RealsenseSensor
from control_your_robot.sensor.PikaRos_sensor import PikaRosSensor

from data.collect_any import CollectAny

from utils.data_handler import debug_print, matrix_to_xyz_rpy, apply_local_delta_pose 


CAMERA_SERIALS = {
    'head': '1111',  # Replace with actual serial number
    'left_wrist': '1111',   # Replace with actual serial number
    'right_wrist': '1111',   # Replace with actual serial number
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
    "task_name": "test", 
    "save_format": "hdf5",
    "save_freq": 10,
}

class PikaPiper(Robot):
    def __init__(self, condition=condition, move_check=True, start_episode=0):
        super().__init__(condition=condition, move_check=move_check, start_episode=start_episode)

        self.controllers = {
            "arm": {
                "left_arm": PiperController("left_arm"),
                "right_arm": PiperController("right_arm"),
            },
        }
        self.sensors = {
            "image":{
                "cam_head": RealsenseSensor("cam_head"),
                "cam_left_wrist": RealsenseSensor("cam_left_wrist"),
                "cam_right_wrist": RealsenseSensor("cam_right_wrist"),
            },
            "teleop": {
                "pika_left": PikaRosSensor("left_pika"),
                "pika_right": PikaRosSensor("right_pika"),
            },
        }

    def reset(self):
        self.controllers["arm"]["left_arm"].reset(START_POSITION_ANGLE_LEFT_ARM)
        self.controllers["arm"]["right_arm"].reset(START_POSITION_ANGLE_RIGHT_ARM)

    def set_up(self):
        super().set_up()
        self.controllers["arm"]["left_arm"].set_up("can0")
        self.controllers["arm"]["right_arm"].set_up("can1")

        self.sensors["iamge"]["cam_head"].set_up(CAMERA_SERIALS['head'], is_depth=False)
        self.sensors["iamge"]["cam_left_wrist"].set_up(CAMERA_SERIALS['left_wrist'], is_depth=False)
        self.sensors["iamge"]["cam_right_wrist"].set_up(CAMERA_SERIALS['right_wrist'], is_depth=False)

        self.sensors["teleop"]["pika_left"].set_up("/pika_pose_l","/gripper_l/joint_states")
        self.sensors["teleop"]["pika_right"].set_up("/pika_pose_r","/gripper_r/joint_states")

        self.set_collect_type({"arm":["joint","qpos"],
                              "image":["color"], 
                              "teleop":["end_pose"],
                              })
        debug_print("robot", "set up success!", "INFO")

if __name__ == "__main__":
    import time
    import rospy
    rospy.init_node("rm_controller_node", anonymous=True)

    robot = PikaPiper()
    robot.set_up()

    robot.reset()
    time.sleep(3)
    # 等待数据稳定
    while True:
        data = robot.get()
        if data[1]["pika_left"]["end_pose"] is not None and data[1]["pika_right"]["end_pose"] is not None and\
            data[0]["left_arm"]["qpos"] is not None and data[0]["left_arm"]["qpos"] is not None:
            break
        else:
            time.sleep(0.1)
    
    print("start teleop")

    time.sleep(3)

    left_base_pose = data[0]["left_arm"]["qpos"]
    right_base_pose = data[0]["right_arm"]["qpos"]
    
    # 遥操
    while True:
        try:
            data = robot.get()

            left_delta_pose = matrix_to_xyz_rpy(data[1]["pika_left"]["end_pose"])
            right_delta_pose = matrix_to_xyz_rpy(data[1]["pika_right"]["end_pose"])

            # print("left:", left_pose)
            # print("right:", right_pose)

            left_wrist_mat = apply_local_delta_pose(left_base_pose, left_delta_pose)
            right_wrist_mat = apply_local_delta_pose(right_base_pose, right_delta_pose)

            l_data = matrix_to_xyz_rpy(left_wrist_mat)
            r_data = matrix_to_xyz_rpy(right_wrist_mat)

            print("left:", l_data.tolist())
            print("right:", r_data.tolist())

            move_data = {
                "arm":{
                    "left_arm": {
                        "qpos":l_data},
                    "right_arm": {
                        "qpos":r_data},
                }
            }

            robot.move(move_data)
            time.sleep(0.02)
        except:
            print("data is none")
            time.sleep(0.1)
            
    robot.reset()    