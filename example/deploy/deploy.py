import sys
sys.path.append('./')

import os
import importlib
import argparse
import numpy as np
import time
import yaml

from utils.data_handler import debug_print, is_enter_pressed

# START ================ you could modify to your format ================ 
video_path="save/videos/"
fps = 30

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

# END ================ you could modify to your format ================ 
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


def parse_args_and_config():
    parser = argparse.ArgumentParser()
    
    parser.add_argument("--model_name", type=str, required=True, help="Name of the task")
    parser.add_argument("--model_class", type=str, required=True, help="Name of the model class")
    parser.add_argument("--model_path", type=str, required=True, help="model path, e.g., policy/RDT/checkpoints/checkpoint-10000. If using RoboTwin pipeline, this should be set as checkpoint_id")
    parser.add_argument("--task_name", type=str, required=True, help="task name, read intructions from task_instuctions/{task_name}.json")
    parser.add_argument("--robot_name", type=str, required=True, help="robot name, read my_robot/{robot_name}.py")
    parser.add_argument("--robot_class", type=str, required=True, help="robot class, get class from my_robot/{robot_name}.py")
    parser.add_argument("--episode_num", type=int, default=10, help="how many episodes you want to deploy")
    parser.add_argument("--max_step", type=int, default=1000000, help="the maximum step for each episode")
    parser.add_argument("--robotwin", action="store_true", help="If using RoboTwin pipeline, you should set it.")
    parser.add_argument("--video", type=str, default=None, help="Recording the video if set, should set to cam_name like cam_head.")
    parser.add_argument("--overrides", nargs=argparse.REMAINDER)

    args = parser.parse_args()

    args_dict = vars(args)

    # ---------- 读取 YAML 配置 ----------
    def load_yaml_safe(path):
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
                if isinstance(data, dict):
                    return data
        return {}

    # 分别读取两个配置文件
    robotwin_path = "config/RoboTwin_setting.yml"
    model_path = f"policy/{args.model_name}/deploy_policy.yml"

    robotwin_setting = load_yaml_safe(robotwin_path)
    model_setting = load_yaml_safe(model_path)

    # ---------- 合并配置 ----------
    # 优先级顺序：
    # 命令行参数 < robotwin_setting < model_setting < overrides
    merged = {}
    merged.update(args_dict)
    merged.update(robotwin_setting)
    merged.update(model_setting)

    # ---------- 解析 overrides ----------
    def parse_override_pairs(pairs):
        override_dict = {}
        for i in range(0, len(pairs), 2):
            key = pairs[i].lstrip("--")
            value = pairs[i + 1]
            try:
                value = eval(value)
            except Exception:
                pass
            override_dict[key] = value
        return override_dict

    if args.overrides:
        overrides = parse_override_pairs(args.overrides)
        merged.update(overrides)

    # 返回合并后的结果（dict）
    return merged

# ROboTwin eval
class TASK_ENV:
    def __init__(self, task_name):
        self.task_name = task_name
    
    def get_instruction(self):
        json_Path = os.path.join( "task_instructions", f"{self.task_name}.json")
        with open(json_Path, 'r') as f_instr:
            instruction_dict = json.load(f_instr)
        instructions = instruction_dict['instructions']
        instruction = np.random.choice(instructions)
        return instruction

class RoboTwinModel:
    def __init__(self, model, infer, task_name):
        self.model = model
        self.infer = infer
        self.TASK_ENV = TASK_ENV(task_name)
    
    def random_set_language(self):
        debug_print("RoboTwinModel", "Eval under RoboTwin pipeline, set instruction by policy/{model}/deploy_policy.py", "DEBUG")
        return
    
    def update_observation_window(self, img_arr, state):
        self.observation_window = {}

        self.observation_window["observation"] = {}
        self.observation_window["observation"]["head_camera"] = {"rgb": img_arr[0]}
        self.observation_window["observation"]["right_camera"] = {"rgb": img_arr[1]}
        self.observation_window["observation"]["left_camera"] = {"rgb": img_arr[2]}
        self.observation_window["agent_pos"] = state
    
    def get_action(self):
        actions = self.infer(self.TASK_ENV, self.model, self.observation_window)
        return actions

def init():
    args = parse_args_and_config()

    print(args["test"])
    is_robotwin = args["robotwin"]
    is_video = args["video"]

    if not is_robotwin:
        model_class = get_class(f"policy.{args['model_name']}.inference_model", args["model_class"])
        model = model_class(args["model_path"], args["task_name"])
    else:
        get_model = get_class(f"policy.{args['model_name']}.deploy_policy", get_model)
        infer = get_class(f"policy.{args['model_name']}.deploy_policy", infer)
        base_model = get_model(args)
        model = RoboTwinModel(base_model, infer, args["task_name"])
        
    robot_class = get_class(f"my_robot.{args['robot_name']}", args["robot_class"])
    robot = robot_class()

    return model, robot, args["episode_num"], args["max_step"], is_video

if __name__ == "__main__":
    os.environ["INFO_LEVEL"] = "INFO" # DEBUG , INFO, ERROR
    
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