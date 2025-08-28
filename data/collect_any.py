"""
This function stores all incoming data without any filtering or condition checks.
The storage format differs from the standard format used for dual-arm robots:
each controller/sensor corresponds to a separate group.
"""
import sys
sys.path.append("./")

import threading, os

from utils.data_handler import debug_print

import os
import numpy as np
import h5py
import json

class CollectAny:
    def __init__(self, condition=None, start_episode=0, move_check=True, sub_task=False):
        self.condition = condition
        self.episode = []
        self.episode_index = start_episode
        self.move_check = move_check
        self.last_controller_data = None
        self.sub_task = sub_task
        
        if self.sub_task:
            self.subtask_id = 0
    
    def collect(self, controllers_data, sensors_data):
        episode_data = {}
        if controllers_data is not None:    
            for controller_name, controller_data in controllers_data.items():
                episode_data[controller_name] = controller_data
        if sensors_data is not None:    
            for sensor_name, sensor_data in sensors_data.items():
                episode_data[sensor_name] = sensor_data
        
        if self.move_check:
            if self.last_controller_data is None:
                self.last_controller_data = controllers_data
                self.episode.append(episode_data)
            else:
                if self.move_check_success(controllers_data, tolerance=0.0001):
                    self.episode.append(episode_data)
                else:
                    debug_print("collect_any", f"robot is not moving, skip this frame!", "INFO")
                self.last_controller_data = controllers_data
        else:
            self.episode.append(episode_data)
        
        if self.sub_task:
            self.episode[-1]["subtask"] = {}
            self.episode[-1]["subtask"]["subtask_id"] = self.subtask_id
    
    def get_item(self, controller_name, item):
        if item in self.episode[0][controller_name]:
            return np.array([self.episode[i][controller_name][item] for i in range(len(self.episode))])
        else:
            debug_print("collect_any", f"item {item} not in {controller_name}", "ERROR")
            return None
    
    def next_subtask(self, subtask=None):
        self.subtask_id += 1
        if subtask:
            debug_print("collect_any", f"chanhge to next task:{subtask}", "INFO")
        else:
            debug_print("collect_any", f"change to next task.", "INFO")
        
    def add_extra_condition_info(self, extra_info):
        save_path = os.path.join(self.condition["save_path"], f"{self.condition['task_name']}/")
        condition_path = os.path.join(save_path, "./config.json")
        if os.path.exists(condition_path):
            with open(condition_path, 'r', encoding='utf-8') as f:
                self.condition = json.load(f)
            for key in extra_info.keys():
                if key in self.condition.keys():
                    value = self.condition[key]
                    if not isinstance(value, list):
                        value = [value]
                    value.append(extra_info[key])
                    
                    self.condition[key] = value
                else:
                    self.condition[key] = extra_info[key]
        else:
            if len(self.episode) > 0:
                for key in self.episode[0].keys():
                    self.condition[key] = list(self.episode[0][key].keys())
        with open(condition_path, 'w', encoding='utf-8') as f:
            json.dump(self.condition, f, ensure_ascii=False, indent=4)
        
    def write(self, episode_id=None):
        save_path = os.path.join(self.condition["save_path"], f"{self.condition['task_name']}/")
        if not os.path.exists(save_path):
            os.makedirs(save_path)

        condition_path = os.path.join(save_path, "./config.json")
        if not os.path.exists(condition_path):
             if len(self.episode) > 0:
                for key in self.episode[0].keys():
                    self.condition[key] = list(self.episode[0][key].keys())

             with open(condition_path, 'w', encoding='utf-8') as f:
                 json.dump(self.condition, f, ensure_ascii=False, indent=4)
        
        if not episode_id is None:
            hdf5_path = os.path.join(save_path, f"{episode_id}.hdf5")
        else:
            hdf5_path = os.path.join(save_path, f"{self.episode_index}.hdf5")
        
        # print(f"WRITE called in PID={os.getpid()} TID={threading.get_ident()}")

        with h5py.File(hdf5_path, "w") as f:
            obs = f
            # print(self.episode[0].keys())

            for controller_name in self.episode[0].keys():
                controller_group = obs.create_group(controller_name)
                for item in self.episode[0][controller_name].keys():
                    data = self.get_item(controller_name, item)
                    controller_group.create_dataset(item, data=data)
        debug_print("collect_any", f"write to {hdf5_path}", "INFO")
        # reset the episode
        self.episode = []
        self.episode_index += 1

    def move_check_success(self, controller_data: dict, tolerance: float) -> bool:
        """
        判断当前控制器状态是否与上一状态有显著差异（任一字段的任一元素差值超过容忍值，则视为成功移动）。

        参数:
            controller_data (dict): 当前控制数据，嵌套结构，值可为标量、list、np.array、或子字典。
            tolerance (float): 最大允许的静止误差。

        返回:
            bool: 如果有任一元素变化超过 tolerance，则返回 True（动作已发生）；否则 False。
        """
        for part, current_subdata in controller_data.items():
            previous_subdata = self.last_controller_data.get(part)
            if previous_subdata is None:
                return True  # 没有历史数据视为变动

            if isinstance(current_subdata, dict):
                for key, current_value in current_subdata.items():
                    previous_value = previous_subdata.get(key)
                    if previous_value is None:
                        return True  # 缺失对应字段，视为变动

                    current_arr = np.atleast_1d(current_value)
                    previous_arr = np.atleast_1d(previous_value)

                    if current_arr.shape != previous_arr.shape:
                        return True  # 尺寸变化，视为变动

                    if np.any(np.abs(current_arr - previous_arr) > tolerance):
                        return True  # 任一值超误差，视为变动
            else:
                current_arr = np.atleast_1d(current_subdata)
                previous_arr = np.atleast_1d(previous_subdata)

                if current_arr.shape != previous_arr.shape:
                    return True

                if np.any(np.abs(current_arr - previous_arr) > tolerance):
                    return True

        return False  # 所有值都在容忍范围内，无显著动作