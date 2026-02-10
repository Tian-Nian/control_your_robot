

from robot.robot.base_robot import Robot

from robot.controller.RealmanRos_controller import RealmanRosController
from robot.sensor.Realsense_sensor import RealsenseSensor

from robot.data.collect_any import CollectAny

import numpy as np

# 组装你的控制器
CAMERA_SERIALS = {
    'head': '111',  # Replace with actual serial number
    'left_wrist': '111',   # Replace with actual serial number
    'right_wrist': '336222070133',   # Replace with actual serial number
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

class MyRobot(Robot):
    def __init__(self, condition=condition, move_check=True, start_episode=0):   
        super().__init__(condition=condition, move_check=move_check, start_episode=start_episode)

        self.controllers = {
            "arm":{
                "left_arm": RealmanRosController("left_arm"),
                "right_arm": RealmanRosController("right_arm"),
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

        self.controllers["arm"]["left_arm"].set_up("rm_left")
        self.controllers["arm"]["right_arm"].set_up("rm_right")

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
        if max(abs(self.controllers["left_arm"].get_state()["joint"] - START_POSITION_ANGLE_LEFT_ARM), abs(self.controllers["right_arm"].get_state()["joint"] - START_POSITION_ANGLE_RIGHT_ARM)) > 0.01:
            return True
        else:
            return False

if __name__ == "__main__":
    import time
    import rospy
    rospy.init_node("rm_controller_node", anonymous=True)

    robot = MyRobot()

    robot.set_up()
    time.sleep(3)
    # get state
    for i in range(10):
        print(robot.get()[0])
        time.sleep(0.1)
    
    # qpos
    move_data = {
        "arm": {
            "left_arm": {
                # "qpos":np.array([0.46, 0.156, 0.3, -2., 0.3 ,-1.87]),
                "joint": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                },
            "right_arm": {
                # "qpos":np.array([0.40, -0.40, 0.11, -2.6, 1.0, 2.72]),
                "joint": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                },
        },
    }

    robot.move(move_data)
    
    time.sleep(0.01)

    move_data = {
        "arm": {
            "left_arm": {
                # "qpos":np.array([0.46, 0.156, 0.3, -2., 0.3 ,-1.87]),
                "joint": np.array([0.0, 0.0, 0.10, 0.10, 0.0, 0.0])
                },
            "right_arm": {
                # "qpos":np.array([0.40, -0.40, 0.11, -2.6, 1.0, 2.72]),
                "joint": np.array([0.0, 0.0, 0.10, 0.10, 0.0, 0.0])
                },
        }
    }

    robot.move(move_data)

    time.sleep(5)
    