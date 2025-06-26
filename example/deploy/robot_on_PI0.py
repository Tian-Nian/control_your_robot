import sys
sys.path.append("./")

from my_robot.test_robot import TestRobot

import time
import numpy as np

from policy.openpi.inference_model import PI0_DUAL

from utils.data_handler import is_enter_pressed, debug_print

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
        "left_arm":{
            "joint":data[:6],
            "gripper":data[6]
        },
        "right_arm":{
            "joint":data[7:13],
            "gripper":data[13]
        }
    }
    return move_data

if __name__ == "__main__":
    robot = TestRobot(DoFs=6)
    robot.set_up()
    # load model
    model = PI0_DUAL("model_path", "task_name")
    max_step = 1000
    num_episode = 10

    for i in range(num_episode):
        step = 0
        # 重置所有信息
        robot.reset()
        model.reset_obsrvationwindows()
        model.random_set_language()

        # 等待允许执行推理指令, 按enter开始
        is_start = False
        while not is_start:
            if is_enter_pressed():
                is_start = True
                print("start to inference...")
            else:
                print("waiting for start command...")
                time.sleep(1)

        # 开始逐条推理运行
        while step < max_step:
            data = robot.get()
            img_arr, state = input_transform(data)
            model.update_observation_window(img_arr, state)
            action_chunk = model.get_action()
            for action in action_chunk:
                if is_enter_pressed():
                    debug_print("main", "Moving interrupted!", "WARNING")
                    is_start = False
                    break
                move_data = output_transform(action)
                robot.move(move_data)
                step += 1
                time.sleep(1/robot.condition["save_interval"])

        print("finish episode", i)