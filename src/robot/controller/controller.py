from typing import List, Dict, Any
from abc import ABC, abstractmethod
import numpy as np
import time

from robot.utils.base.data_handler import debug_print

class Controller(ABC):
    def __init__(self, timestamp=True):
        self.name = "controller"
        self.controller_type = "base_controller"
        # self.is_set_up = False
        self.timestamp = timestamp
        self.collect_info = None
    
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
        if getattr(self, "_cyr_safety_stopped", False):
            debug_print(self.name, "安全保护已触发，丢弃运动命令", "WARNING")
            return
        debug_print(f"{self.name}", f"get move data:\n{move_data} ", "DEBUG")
        try:
            self.move_controller(move_data, is_delta)
        except Exception as e:
            debug_print(self.name, f"move error: {e}", "WARNING")
            self.safe_stop()
            if getattr(self, "_cyr_raise_move_errors", False):
                raise

    def reset(self, start_state=None):
        """回到零位，或回到传入的 start_state。安全保护触发后不再动作。"""
        if getattr(self, "_cyr_safety_stopped", False):
            debug_print(self.name, "安全保护已触发，拒绝复位", "WARNING")
            return
        return self.apply_reset(start_state)

    @abstractmethod
    def apply_reset(self, start_state):
        """具体设备的复位动作。"""

    def safe_stop(self) -> str:
        """停下这个设备。已经停过就直接返回。

        设备自己有 stop 就调用。底盘把速度清零。
        没有这些接口时保持当前位置，并拒绝后续 move。
        """
        if getattr(self, "_cyr_safety_stopped", False):
            return "already"
        self._cyr_safety_stopped = True
        stop = getattr(self.__class__, "stop", None)
        if stop is not None:
            stop(self)
            return "stop"
        setter = getattr(self, "set_move_velocity", None)
        if callable(setter):
            setter(np.zeros(6))
            return "zero_velocity"
        return "hold"
    
    @abstractmethod
    def get_information(self) -> Dict[str, Any]:
        """Get raw information from the controller."""
        pass

    @abstractmethod
    def move_controller(self, move_data: Dict[str, Any], is_delta: bool = False):
        """Execute movement commands."""
        pass

   # init controller
    @abstractmethod
    def set_up(self):
        """Initialize the controller."""
        pass
    
    # print controller
    def __repr__(self):
        return f"{self.name}: {self.controller_type}"
        return f"Base Controller, can't be used directly \n \
                name: {self.name} \n \
                controller_type: {self.controller_type}"
