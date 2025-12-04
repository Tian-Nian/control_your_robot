import sys
sys.path.append("src/")

from utils.task import YmlTask, Tasks, Success

import numpy as np
import os

if __name__ == "__main__":
    # os.environ["INFO_LEVEL"] = "DEBUG" # DEBUG , INFO, ERROR

    my_task = Tasks.build_top({
        "type": "Serial",
        "subtasks": [
            YmlTask("./config/robot_1_move_mobile_1.yml", is_block=False),
            YmlTask("./config/robot_1_model_infer.yml", is_block=False),
            YmlTask("./config/robot_1_move_mobile_2.yml", is_block=True),
        ],
    })
    while not my_task.is_success():
        my_task.run()
        my_task.update()