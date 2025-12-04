import sys
sys.path.append("src/")

from utils.task import YmlTask, Tasks

import numpy as np
import time
def success(task, threshold):
    if np.random.random() > threshold:
        return False
    return True
def move_mobile_to(task, target):
    move_data = {
            "mobile":{
                "test_mobile": {
                    "move_to": target,
                }
            }
        }
    task.robot.move(move_data)
    time.sleep(0.1)

def infer_once(task):
    def input_transform(data):
        state = np.concatenate([
            np.array(data[0]["left_arm"]["joint"]).reshape(-1),
            np.array(data[0]["left_arm"]["gripper"]).reshape(-1),
            np.array(data[0]["right_arm"]["joint"]).reshape(-1),
            np.array(data[0]["right_arm"]["gripper"]).reshape(-1)
        ])

        img_arr = data[1]["cam_head"]["color"], data[1]["cam_right_wrist"]["color"], data[1]["cam_left_wrist"]["color"]
        return img_arr, state

    def output_transform(data):
        move_data = {
            "arm":{
                "left_arm":{
                    "joint":data[:6],
                    "gripper":data[6]
                },
                "right_arm":{
                    "joint":data[7:13],
                    "gripper":data[13]
                }
            }
        }
        return move_data
    
    img_arr, state = input_transform(task.robot.get())
    task.extras["model"].update_observation_window(img_arr, state)
    actions = task.extras["model"].get_action()
    for action in actions:
        move_data = output_transform(action)

        task.robot.move(move_data)
        time.sleep(0.1)


