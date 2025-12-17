import sys
sys.path.append('./')

import os
import importlib
import argparse
import numpy as np
import time

from utils.data_handler import debug_print, is_enter_pressed
from utils.action_queue import ActionQueue
from utils.latency_tracker import LatencyTracker

from threading import Event, Lock, Thread

video_path="save/videos/"
fps = 30

def images_encoding(imgs):
    encode_data = []
    padded_data = []
    max_len = 0
    for i in range(len(imgs)):
        success, encoded_image = cv2.imencode('.jpg', imgs[i])
        jpeg_data = encoded_image.tobytes()
        encode_data.append(jpeg_data)
        max_len = max(max_len, len(jpeg_data))
    # padding
    for i in range(len(imgs)):
        padded_data.append(encode_data[i].ljust(max_len, b'\0'))
    return encode_data, max_len

def input_transform(data, size=256):
    # ====== 处理 state ======
    state = np.concatenate([
        np.array(data[0]["left_arm"]["joint"]).reshape(-1),
        np.array(data[0]["left_arm"]["gripper"]).reshape(-1),
        np.array(data[0]["right_arm"]["joint"]).reshape(-1),
        np.array(data[0]["right_arm"]["gripper"]).reshape(-1),
    ])

    # ====== 处理图像 ======
    img_arr = [
        data[1]["cam_head"]["color"],
        data[1]["cam_right_wrist"]["color"],
        data[1]["cam_left_wrist"]["color"],
    ]
    
    img_enc, img_enc_len = images_encoding(img_arr)

    return img_enc, state

def output_transform(data):
    if data[6] < 0.1:
        data[6] = 0
    
    if data[13] < 0.1:
        data[13] = 0

    move_data = {
        "arm":{
            "left_arm":{
                "joint":data[:6],
                "gripper":data[6]
            },  
            "right_arm":{ # left_arm
                    "joint":data[7:13],
                    "gripper":data[13]
                },
        }
    }
    return move_data

def policy_infer(
    policy,
    robot,
    input_processor,
    action_queue,
    shutdown_event,
    rtc_cfg=None,
):
    if rtc_cfg is None:
        get_actions_threshold = 0
    else:
        get_actions_threshold = cfg.action_queue_size_to_get_new_actions

    latency_tracker = LatencyTracker()  # Track latency of action chunks
    fps = cfg.fps
    time_per_chunk = 1.0 / fps
    
    


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
    
    parser.add_argument("--model_name", type=str, required=True, help="Name of the task")  # 如果你用得上
    parser.add_argument("--model_class", type=str, required=True, help="Name of the model class")
    parser.add_argument("--model_path", type=str, required=True, help="model path, e.g., policy/RDT/checkpoints/checkpoint-10000")
    parser.add_argument("--task_name", type=str, required=True, help="task name, read intructions from task_instuctions/{task_name}.json")
    parser.add_argument("--robot_name", type=str, required=True, help="robot name, read my_robot/{robot_name}.py")
    parser.add_argument("--robot_class", type=str, required=True, help="robot class, get class from my_robot/{robot_name}.py")
    parser.add_argument("--episode_num", type=int, required=False,default=10, help="how many episode you want to deploy")
    parser.add_argument("--max_step", type=int, required=False,default=1000000, help="the maxinum step for each episode")
    parser.add_argument("--video", type=str, required=False, default=None, help="Recording the video if set, should set to cam_name like cam_head.")
    
    args = parser.parse_args()
    model_name = args.model_name
    model_class = args.model_class
    model_path = args.model_path
    task_name = args.task_name
    robot_name = args.robot_name
    robot_class = args.robot_class
    episode_num = args.episode_num
    max_step = args.max_step
    is_video = args.video


    model_class = get_class(f"policy.{model_name}.inference_model", model_class)
    model = model_class(model_path, task_name)

    robot_class = get_class(f"my_robot.{robot_name}", robot_class)
    robot = robot_class()

    return model, robot, episode_num, max_step, is_video

if __name__ == "__main__":
    # os.environ["INFO_LEVEL"] = "DEBUG" # DEBUG , INFO, ERROR
    
    model, robot, episode_num, max_step, video_cam_name = init()
    robot.set_up()

    for i in range(episode_num):
        step = 0
        # 重置所有信息
        robot.reset()
        model.reset_obsrvationwindows()
        model.random_set_language()

        writer = None
        if video_cam_name is not None:
            import cv2
            first_frame = robot.get()[1][video_cam_name]["color"][:,:,::-1]
            height, width, channels = first_frame.shape
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # 或 'XVID'
            video_dir = video_path + f"/{i}/"
            os.makedirs(video_dir, exist_ok=True)
            writer = cv2.VideoWriter(os.path.join(video_dir, f"{video_cam_name}.mp4"), fourcc, fps, (width, height))
            print(f"Video saving enabled: {video_path}, fps={fps}, size=({width},{height})")

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
            data = robot.get()
            img_arr, state = input_transform(data)
            model.update_observation_window(img_arr, state)
            action_chunk = model.get_action()
            for action in action_chunk:
                if video_cam_name is not None:
                    frame = robot.get()[1][video_cam_name]["color"][:,:,::-1]
                    writer.write(frame)
                
                if step % 10 == 0:
                    debug_print("main", f"step: {step}/{max_step}", "INFO")
                move_data = output_transform(action)
                robot.move(move_data)
                step += 1
                # time.sleep(1/robot.condition["save_freq"])
                time.sleep(1 / 20)
                if step >= max_step or is_enter_pressed():
                    debug_print("main", "enter pressed, the episode end", "INFO")
                    is_start = False
                    break
                    
        if writer is not None:
            writer.release()
        debug_print("main",f"finish episode {i}, running steps {step}","INFO")
