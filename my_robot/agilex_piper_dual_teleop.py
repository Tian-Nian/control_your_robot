import sys
sys.path.append("./")

import numpy as np

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
    "save_interval": 10,
}

class PikaPiper:
    def __init__(self, start_episode=0):
        self.arm_controllers = {
            "left_arm": PiperController("left_arm"),
            "right_arm": PiperController("right_arm"),
        }
        self.image_sensors = {
            "cam_head": RealsenseSensor("cam_head"),
            "cam_left_wrist": RealsenseSensor("cam_left_wrist"),
            "cam_right_wrist": RealsenseSensor("cam_right_wrist"),
        }
        self.pika_sensors = {
            "pika_left": PikaRosSensor("left_pika"),
            "pika_right": PikaRosSensor("right_pika"),
        }

        self.condition = condition
        self.collection = CollectAny(condition, start_episode=start_episode)

    def reset(self):
        return True
        self.arm_controllers["left_arm"].reset(START_POSITION_ANGLE_LEFT_ARM)
        self.arm_controllers["right_arm"].reset(START_POSITION_ANGLE_RIGHT_ARM)

    def set_up(self):
        self.arm_controllers["left_arm"].set_up("can0")
        self.arm_controllers["right_arm"].set_up("can1")

        self.image_sensors["cam_head"].set_up(CAMERA_SERIALS['head'], is_depth=False)
        self.image_sensors["cam_left_wrist"].set_up(CAMERA_SERIALS['left_wrist'], is_depth=False)
        self.image_sensors["cam_right_wrist"].set_up(CAMERA_SERIALS['right_wrist'], is_depth=False)

        self.pika_sensors["pika_left"].set_up("/pika_pose_l","/gripper_l/joint_states")
        self.pika_sensors["pika_right"].set_up("/pika_pose_r","/gripper_r/joint_states")

        self.set_collect_type(["joint","qpos"],["color"], ["end_pose"])
        debug_print("robot", "set up success!", "INFO")

    def set_collect_type(self,ARM_INFO_NAME,IMG_INFO_NAME, PIKA_INFO_NAME):
        for controller in self.arm_controllers.values():
            controller.set_collect_info(ARM_INFO_NAME)
        
        for sensor in self.image_sensors.values():
            sensor.set_collect_info(IMG_INFO_NAME)

        for sensor in self.pika_sensors.values():
            sensor.set_collect_info(PIKA_INFO_NAME)

    def is_start(self):
        return True
    
    def get(self):
        controller_data = {}
        sensor_data = {}

        if self.arm_controllers is not None:    
            for controller_name, controller in self.arm_controllers.items():
                controller_data[controller_name] = controller.get()
        
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
        self.arm_controllers["left_arm"].move(move_data["left_arm"],is_delta=True)
        self.arm_controllers["right_arm"].move(move_data["right_arm"],is_delta=True)


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
                "left_arm": {
                    "qpos":l_data},
                "right_arm": {
                    "qpos":r_data},
            }

            robot.move(move_data)
            time.sleep(0.02)
        except:
            print("data is none")
            time.sleep(0.1)
            
    robot.reset()    