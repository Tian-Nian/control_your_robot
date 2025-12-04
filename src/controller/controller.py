from typing import List
import numpy as np
import time

from utils.data_handler import debug_print

class Controller:
    def __init__(self, timestamp=True):
        self.name = "controller"
        self.controller_type = "base_controller"
        # self.is_set_up = False
        self.timestamp = timestamp
    
    def set_collect_info(self, collect_info:List[str]):
        self.collect_info = collect_info
        if self.timestamp:
           self.collect_info.append("timestamp")

    # get controller infomation
    def get(self):
        if self.collect_info is None:
            raise ValueError(f"{self.name}: collect_info is not set")
        info = self.get_information()

        if self.timestamp:
            info["timestamp"] = time.time_ns()
        
        for collect_info in self.collect_info:
            if info[collect_info] is None:
                debug_print(f"{self.name}", f"{collect_info} information is None", "ERROR")
        
        debug_print(f"{self.name}", f"get data:\n{info} ", "DEBUG")
        return {collect_info: info[collect_info] for collect_info in self.collect_info}

    def move(self, move_data, is_delta=False):
        debug_print(f"{self.name}", f"get move data:\n{move_data} ", "DEBUG")
        try:
            self.move_controller(move_data, is_delta)
        except Exception as e:
            debug_print(self.name, f"move error: {e}", "WARNING")
    
   # init controller
    def set_up(self):
        raise NotImplementedError("This method should be implemented by the subclass")
    
    # print controller
    def __repr__(self):
        return f"Base Controller, can't be used directly \n \
                name: {self.name} \n \
                controller_type: {self.controller_type}"
