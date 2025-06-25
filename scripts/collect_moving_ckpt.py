import sys
sys.path.append("./")

import os
import json
import time
import select
import numpy as np

from my_robot.test_robot import TestRobot
from data.collect_any import CollectAny

from utils.time_scheduler import TimeScheduler
from utils.robot_worker import RobotWorker
from utils.data_handler import debug_print

ARM_INFO_NAME = ["qpos", "gripper"]

condition = {
    "save_path": "./datasets/ckpt",
    "task_name": "saving_move_path",
}

def is_enter_pressed():
    """非阻塞检测Enter键"""
    return select.select([sys.stdin], [], [], 0)[0] and sys.stdin.read(1) == '\n'

class PathCollector:
    def __init__(self, robot, condition, episode_index=0):
        # 传入的robot需要配置好对应需要采集的信息
        self.robot = robot
        self.collecter = CollectAny(condition, start_episode=0)
        self.condition = condition
        self.episode_index = episode_index  
    
    def collect(self):
        data = self.robot.get()
        self.collecter.collect(data[0], data[1])

    def save(self):
        json_data = {}

        # transform numpy array to list
        for index, episode in enumerate(self.collecter.episode):
            episode_data = episode.copy() 
            if isinstance(episode_data.get("left_arm", {}).get("qpos"), np.ndarray):
                episode_data["left_arm"]["qpos"] = episode_data["left_arm"]["qpos"].tolist()
            if isinstance(episode_data.get("left_arm", {}).get("gripper"), np.ndarray):
                episode_data["left_arm"]["gripper"] = episode_data["left_arm"]["gripper"].tolist()

            if isinstance(episode_data.get("right_arm", {}).get("qpos"), np.ndarray):
                episode_data["right_arm"]["qpos"] = episode_data["right_arm"]["qpos"].tolist()
            if isinstance(episode_data.get("right_arm", {}).get("gripper"), np.ndarray):
                episode_data["right_arm"]["gripper"] = episode_data["right_arm"]["gripper"].tolist()
            
            json_data[index] = episode_data
        
        save_path = os.path.join(self.condition["save_path"], f"{self.condition['task_name']}/")
        if not os.path.exists(save_path):
            os.makedirs(save_path)
        
        # save data
        with open(os.path.join(save_path, f"{self.episode_index}.json"), "w") as f:
            json.dump(json_data, f, indent=4)
        self.collecter.episode = []
        self.episode_index += 1

    def play(self, robot, episode_index, is_block=False):
        path = os.path.join(self.condition['save_path'], f"{self.condition['task_name']}/{episode_index}.json")
        try:
            with open(path, "r") as f:
                json_data = json.load(f)
        except:
            debug_print("path_controller", f"{path} does not exist!", "ERROR")
            return
        
        i = 0 
        for episode in json_data.values():
            
            debug_print("path_controller", f"move {i}: {episode}", "INFO")
            i += 1
            robot.move(episode)
            
            if not is_block:
                time.sleep(2)
            
            if is_block:
                continue
        debug_print("path_controller","play finished!", "INFO")
            
if __name__ == "__main__":
    os.environ["INFO_LEVEL"] = "DEBUG"

    robot = TestRobot(DoFs=6,INFO="DEBUG",start_episode=0)
    robot.set_up()

    # setting collect info
    ARM_INFO_NAME = ["qpos", "gripper"]

    robot.set_collect_type(ARM_INFO_NAME, None) 
    collector = PathCollector(robot, condition, episode_index=0)
    '''
    按Enter键进行采集
    按Space键保存并退出
    保存的json文件可以删除不想要的ckpt,不会影响操作
    '''
    while True:
        user_input = input("input: 'c' collect data, 's' save data, 'q' exit :").strip().lower()  # get input
        if user_input == 'c':
            collector.collect()  # collect once
        elif user_input == 's':
            collector.save()  # save the episode
            print("Collect finished!")
        elif user_input == 'q':
            print("Exiting...")
            break  # exit the loop
        else:
            print("invalid input!")

    # testing, run the first tarjectory
    collector.play(robot, 0, is_block=False)

    '''
    # If your robotic arm can only establish communication within a single script, 
    # you can add a set of data collectors in this script and comment out the collector.play() above.
    
    is_start = False
        
    # reset process
    time_lock = Semaphore(0)
    start_event = Event()
    finish_event = Event()
    robot_process = Process(target=RobotWorker, args=(PiperSingle, time_lock, start_event, finish_event, "robot_worker"))
    time_scheduler = TimeScheduler([time_lock], time_interval=10) # set lock

    robot_process.start()

    while not is_start:
        time.sleep(0.01)
        if is_enter_pressed():
            is_start = True
            start_event.set()
        else:
            time.sleep(1)
    
    collector.play(robot, 0, is_block=False)

    time_scheduler.start()
    while is_start:
        time.sleep(0.01)
        if is_enter_pressed():
            finish_event.set()  
            time_scheduler.stop()  
            is_start = False
    
    # destory
    if robot_process.is_alive():
        robot_process.join()
        robot_process.close()
    '''