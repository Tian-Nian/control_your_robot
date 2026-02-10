from robot import ROBOT_PATH
import sys
sys.path.append(ROBOT_PATH)

from robot.utils.task.task import YmlTask, Tasks, ShareSpace
from my_robot.test_robot import TestRobot
import numpy as np
import os



if __name__ == "__main__":
    # os.environ["INFO_LEVEL"] = "DEBUG" # DEBUG , INFO, ERROR
    robot_1 = TestRobot()
    robot_2 = TestRobot()

    robot_1.set_up()
    robot_2.set_up()

    sp = ShareSpace()

    my_task = Tasks.build_top({
        "type": "Serial",
        "subtasks": [
            {"type": "Parallel",
             "subtasks": [
                 YmlTask("./config/robot_1_move_mobile_1.yml", share_space=sp, robot=robot_1),
                 YmlTask("./config/robot_2_move_mobile_1.yml", share_space=sp, robot=robot_2),
             ]},
            {"type": "Parallel",
             "subtasks": [
                YmlTask("./config/robot_1_model_infer.yml", share_space=sp, robot=robot_1),
                YmlTask("./config/robot_2_model_infer.yml", share_space=sp, robot=robot_2),
             ]},
             {"type": "Parallel",
             "subtasks": [
                 YmlTask("./config/robot_1_move_mobile_2.yml", share_space=sp, robot=robot_1),
                 YmlTask("./config/robot_2_move_mobile_2.yml", share_space=sp, robot=robot_2),
             ]},
        ],
    })
    while not my_task.is_success():
        my_task.run()
        my_task.update()