import sys
sys.path.append("./")

import os
from typing import Dict, Any, List
import numpy as np
import time
import json
import h5py

from robot.controller.TestArm_controller import TestArmController
from robot.sensor.TestVision_sensor import TestVisonSensor

from robot.data.collect_any import CollectAny

from robot.utils.base.data_handler import debug_print, hdf5_groups_to_dict

import cv2

# add your controller/sensor type here
ALLOW_TYPES = ["arm", "mobile","image", "tactile", "teleop"]

condition = {
    "save_path": "./save/", 
    "task_name": "base", 
    "save_format": "hdf5", 
    "save_freq": 10,
}

class Robot:
    def __init__(self, 
                condition=condition,
                move_check=True, 
                start_episode=0) -> None:
          
        self.name = "base_robot"
        self.controllers = {}
        self.sensors = {}

        self.condition = condition
        self.collection = CollectAny(condition, move_check=move_check, start_episode=start_episode)

    def set_up(self):
        for controller_type in self.controllers.keys():
            if controller_type not in ALLOW_TYPES:
                debug_print(self.name, f"It's recommanded to set your controller type into our format.\nYOUR STPE:{controller_type}\n\
                            ALLOW_TYPES:{ALLOW_TYPES}", "WARNING")
        
        for sensor_type in self.sensors.keys():
            if sensor_type not in ALLOW_TYPES:
                debug_print(self.name, f"It's recommanded to set your sensor type into our format.\nYOUR STPE:{sensor_type}\n\
                            ALLOW_TYPES:{ALLOW_TYPES}", "WARNING")
        # debug_print(self.name, "set_up() should be realized by your robot class", "ERROR")
        # raise NotImplementedError
    
    def set_collect_type(self,INFO_NAMES: Dict[str, Any]):
        for key,value in INFO_NAMES.items():
            if key in self.controllers:
                for controller in self.controllers[key].values():
                    controller.set_collect_info(value)
            if key in self.sensors:
                for sensor in self.sensors[key].values():
                    sensor.set_collect_info(value)
    
    def get(self):
        controller_data = {}
        sensor_data = {}

        if self.controllers is not None:
            for type_name, controller_type in self.controllers.items():
                for controller_name, controller in controller_type.items():
                    controller_data[controller_name] = controller.get()

        if self.sensors is not None:
            for type_name, sensor_type in self.sensors.items(): 
                for sensor_name, sensor in sensor_type.items():
                    sensor_data[sensor_name] = sensor.get()

        return [controller_data, sensor_data]
    
    def collect(self, data):
        self.collection.collect(data[0], data[1])
    
    def finish(self, episode_id=None):
        extra_info = {}
        for controller_type in self.controllers.keys():
            extra_info[controller_type] = []
            for key in self.controllers[controller_type].keys():
                extra_info[controller_type].append(key)
        
        for sensor_type in self.sensors.keys():
            extra_info[sensor_type] = []
            for key in self.sensors[sensor_type].keys():
                extra_info[sensor_type].append(key)

        self.collection.add_extra_condition_info(extra_info)
        
        self.collection.write(episode_id)
    
    def move(self, move_data, key_banned=None):
        if move_data is None:
            return
        for controller_type_name, controller_type in move_data.items():
            for controller_name, controller_action in controller_type.items():
                if key_banned is None:
                    self.controllers[controller_type_name][controller_name].move(controller_action,is_delta=False)
                else:
                    controller_action = remove_duplicate_keys(controller_action, key_banned)
                    self.controllers[controller_type_name][controller_name].move(controller_action,is_delta=False)
    
    def is_start(self):
        debug_print(self.name, "your are using default func: is_start(), this will return True only", "DEBUG")
        return True

    def reset(self):
        debug_print(self.name, "your are using default func: reset(), this will return True only", "DEBUG")
        return True

    def show_pic(self, data_path, pic_name):
        episode = dict_to_list(hdf5_groups_to_dict(data_path))
        for ep in episode:
            cam = ep[pic_name]["color"]
            if isinstance(cam, bytes):
                jpeg_bytes = np.array(cam).tobytes().rstrip(b"\0")
                nparr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                cam = cv2.imdecode(nparr, 1)
            
            cv2.imshow(f"{pic_name}", cam)
            cv2.waitKey(10)

    def replay(self, data_path, key_banned=None, is_collect=False, episode_id=None):
        parent_dir = os.path.dirname(data_path)
        config_path = os.path.join(parent_dir, "config.json")
        
        with open(config_path, 'r', encoding='utf-8') as f:
            condition = json.load(f)
        
        time_interval = 1.0 / condition['save_freq']

        episode = dict_to_list(hdf5_groups_to_dict(data_path))
        
        now_time = last_time = time.monotonic()
        for ep in episode:
            while now_time - last_time < time_interval:
                now_time = time.monotonic()
                time.sleep(0.001)
            if is_collect:
                data = self.get()
                self.collect(data)
            
            self.play_once(ep, key_banned)
            last_time = time.monotonic()
        if is_collect:
            self.finish(episode_id)
    
    def play_once(self, episode: Dict[str, Any], key_banned=None):
        for controller_type, controller_group in self.controllers.items():
            for controller_name, controller in controller_group.items():
                if controller_name in episode:
                    move_data = {
                        controller_type: {
                            controller_name: episode[controller_name],
                        },
                    }
                    self.move(move_data, key_banned=key_banned)

def get_array_length(data: Dict[str, Any]) -> int:
    """获取最外层np.array的长度"""
    for value in data.values():
        if isinstance(value, dict):
            return get_array_length(value)
        elif isinstance(value, np.ndarray):
            return value.shape[0]
    raise ValueError("No np.ndarray found in data.")

def split_nested_dict(data: Dict[str, Any], idx: int) -> Dict[str, Any]:
    """提取每一帧的子结构"""
    result = {}
    for key, value in data.items():
        if isinstance(value, dict):
            result[key] = split_nested_dict(value, idx)
        elif isinstance(value, np.ndarray):
            result[key] = value[idx]
        else:
            raise TypeError(f"Unsupported type: {type(value)} at key {key}")
    return result

def dict_to_list(data: Dict[str, Any]) -> List[Dict[str, Any]]:
    length = get_array_length(data)
    return [split_nested_dict(data, i) for i in range(length)]

# def remove_duplicate_keys(source_dict: dict[str, any], keys_to_remove: list[str]) -> dict[str, any]:
def remove_duplicate_keys(source_dict, keys_to_remove):
    return {k: v for k, v in source_dict.items() if k not in keys_to_remove}

if __name__ == "__main__":
    robot = Robot()
    robot.show_pic("save/test_robot/0.hdf5","cam_head")