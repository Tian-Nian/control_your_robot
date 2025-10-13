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
import glob
import re
import time

class CollectAny:
    def __init__(self, condition=None, start_episode=0, move_check=True, resume=False, time_stamp=True):
        self.condition = condition
        self.episode = []
        self.move_check = move_check
        self.last_controller_data = None
        self.resume = resume
        self.time_stamp = time_stamp
        
        # Initialize episode_index based on resume parameter
        if resume and condition is not None:
            self.episode_index = self._get_next_episode_index()
        else:
            self.episode_index = start_episode
    
    def _get_next_episode_index(self):
        """
        获取下一个可用的episode索引，通过扫描目标文件夹中的hdf5文件
        """
        save_path = os.path.join(self.condition["save_path"], f"{self.condition['task_name']}/")
        if not os.path.exists(save_path):
            debug_print("collect_any", f"Save path {save_path} does not exist, starting from episode 0", "INFO")
            return 0

        hdf5_files = glob.glob(os.path.join(save_path, "*.hdf5"))
        if not hdf5_files:
            debug_print("collect_any", f"No existing hdf5 files found in {save_path}, starting from episode 0", "INFO")
            return 0

        # 收集已有的 episode 序号
        existing_ids = set()
        for file_path in hdf5_files:
            file_name = os.path.basename(file_path)
            match = re.match(r"(\d+)\.hdf5", file_name)
            if match:
                existing_ids.add(int(match.group(1)))

        # 找到从 0 开始的最小缺失序号
        next_episode = 0
        while next_episode in existing_ids:
            next_episode += 1

        debug_print("collect_any", f"Found {len(hdf5_files)} existing episodes, next free episode id {next_episode}", "INFO")
        return next_episode

    def collect(self, controllers_data, sensors_data):
        if self.time_stamp:
            time_stamp = time.time_ns()

        episode_data = {}
        if controllers_data is not None:    
            for controller_name, controller_data in controllers_data.items():
                episode_data[controller_name] = controller_data
                if self.time_stamp:
                    episode_data[controller_name]["timestamp"] = time_stamp
        
        if sensors_data is not None:    
            for sensor_name, sensor_data in sensors_data.items():
                episode_data[sensor_name] = sensor_data
                if self.time_stamp:
                    episode_data[sensor_name]["timestamp"] = time_stamp
        
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
    
    def get_item(self, controller_name, item):
        data = None
        for ep in self.episode:
            if controller_name in ep.keys():
                if data is None:
                    data = [ep[controller_name][item]] 
                else:
                    data.append(ep[controller_name][item])
        if data is None:
            debug_print("collect_any", f"item {item} not in {controller_name}", "ERROR")
            return None

        data = np.array(data)
        return data
        
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
            mapping = {}

            for ep in self.episode:
                for outer_key, inner_dict in ep.items():
                    if isinstance(inner_dict, dict):
                        mapping[outer_key] = set(inner_dict.keys())
            
            for name, items in mapping.items():
                print(name, ": ",items)
                group = obs.create_group(name)
                for item in items:
                    data = self.get_item(name, item)
                    group.create_dataset(item, data=data)
                    
            # for controller_name in name:
            #     controller_group = obs.create_group(controller_name)
            #     for item in self.episode[0][controller_name].keys():
            #         data = self.get_item(controller_name, item)
            #         controller_group.create_dataset(item, data=data)
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
