from robot import ROBOT_PATH
import sys
sys.path.append(ROBOT_PATH)

from robot.utils.task.task import YmlTask, Tasks, ShareSpace
from my_robot.test_robot import TestRobot
import numpy as np
import os

if __name__ == "__main__":
    # os.environ["INFO_LEVEL"] = "DEBUG" # DEBUG , INFO, ERROR
    robot = TestRobot()
    robot.set_up()
    sp = ShareSpace()
    my_task = Tasks.build_top({
        "type": "Serial",
        "subtasks": [
            YmlTask("./config/robot_1_move_mobile_1.yml", share_space=sp, robot=robot),
            YmlTask("./config/robot_1_model_infer.yml",share_space=sp, robot=robot),
            YmlTask("./config/robot_1_move_mobile_2.yml", share_space=sp, robot=robot),
        ],
    })
    while not my_task.is_success():
        my_task.run()
        my_task.update()