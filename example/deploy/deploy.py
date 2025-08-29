import sys
sys.path.append('./')

import os
import importlib
import argparse
import numpy as np
import time

from utils.data_handler import debug_print, is_enter_pressed

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

def get_class(import_name, class_name):
    try:
        class_module = importlib.import_module(import_name)
        debug_print("function", f"Module loaded: {class_module}", "DEBUG")
    except ModuleNotFoundError as e:
        raise SystemExit(f"ModuleNotFoundError: {e}")

    try:
        return_class = getattr(class_module, class_name)
        debug_print("function", f"Class found: {return_class}", "DEBUG")

    except AttributeError as e:
        raise SystemExit(f"AttributeError: {e}")
    except Exception as e:
        raise SystemExit(f"Unexpected error instantiating model: {e}")
    return return_class

def init():
    parser = argparse.ArgumentParser()
    
    parser.add_argument("--model_name", type=str, required=True, help="Name of the task") 
    parser.add_argument("--model_class", type=str, required=True, help="Name of the model class")
    parser.add_argument("--model_path", type=str, required=True, help="model path, e.g., policy/RDT/checkpoints/checkpoint-10000")
    parser.add_argument("--task_name", type=str, required=True, help="task name, read intructions from task_instuctions/{task_name}.json")
    parser.add_argument("--robot_name", type=str, required=True, help="robot name, read my_robot/{robot_name}.py")
    parser.add_argument("--robot_class", type=str, required=True, help="robot class, get class from my_robot/{robot_name}.py")
    parser.add_argument("--episode_num", type=int, required=False,default=10, help="how many episode you want to deploy")
    parser.add_argument("--max_step", type=int, required=False,default=100000, help="the maxinum step for each episode")
    
    args = parser.parse_args()
    model_name = args.model_name
    model_class = args.model_class
    model_path = args.model_path
    task_name = args.task_name
    robot_name = args.robot_name
    robot_class = args.robot_class
    episode_num = args.episode_num
    max_step = args.max_step


    model_class = get_class(f"policy.{model_name}.inference_model", model_class)
    model = model_class(model_path, task_name)

    robot_class = get_class(f"my_robot.{robot_name}", robot_class)
    condition = get_class(f"my_robot.{robot_name}", "condition")
    robot = robot_class(condition=condition)

    return model, robot, episode_num, max_step

if __name__ == "__main__":
    os.environ["INFO_LEVEL"] = "INFO" # DEBUG , INFO, ERROR
    
    model, robot, episode_num, max_step = init()
    robot.set_up()

    robot.reset()

    for i in range(episode_num):
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
                print("start to inference, press ENTER to end...")
            else:
                print("waiting for start command, press ENTER to star...")
                time.sleep(1)

        # 开始逐条推理运行
        while step < max_step and is_start:
            # while True:
            #     if is_enter_pressed():
            #         break
            #     else:
            #         time.sleep(0.01)
            # time.sleep(0.05)
            data = robot.get()
            img_arr, state = input_transform(data)
            model.update_observation_window(img_arr, state)
            action_chunk = model.get_action()
            for action in action_chunk:
                if step % 10 == 0:
                    debug_print("main", f"step: {step}/{max_step}", "INFO")
                move_data = output_transform(action)
                robot.move(move_data)
                step += 1
                # time.sleep(1/robot.condition["save_freq"])
                now = time.monotonic()
                while True:
                    last = time.monotonic()
                    if last - now >= 1/robot.condition["save_freq"]:
                        print(f"{last - now}s")
                        break
                    else:
                        time.sleep(0.0001) 

                if step >= max_step or is_enter_pressed():
                    debug_print("main", "enter pressed, the episode end", "INFO")
                    is_start = False
                    break
        
        debug_print("main",f"finish episode {i}, running steps {step}","INFO")