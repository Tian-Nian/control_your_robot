from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
import time

from robot.utils.base.data_handler import debug_print

class Sensor(ABC):
    def __init__(self, timestamp=True):
        self.name = "sensor"
        self.type = "sensor"
        self.timestamp = timestamp
        self.collect_info = None
    
    def set_collect_info(self, collect_info):
       self.collect_info = collect_info
       if self.timestamp:
           if self.collect_info is None:
               self.collect_info = ["timestamp"]
           else:
               self.collect_info.append("timestamp")
    
    def get(self):
        if self.collect_info is None:
            debug_print(self.name,f"collect_info is not set, if only collecting controller data, forget this warning", "WARNING")
            return None
        info = self.get_information()

        if self.timestamp:
            info["timestamp"] = time.time_ns()
        
        for collect_info in self.collect_info:
            if info[collect_info] is None:
                debug_print(f"{self.name}", f"{collect_info} information is None", "ERROR")
        
        # 由于sensor数据比较高维, 所以不输出, 只调试信息是否为None
        # debug_print(f"{self.name}", f"get data:\n{info} ", "DEBUG")
        return {collect_info: info[collect_info] for collect_info in self.collect_info}

    @abstractmethod
    def get_information(self) -> Dict[str, Any]:
        """Get raw information from the sensor."""
        pass

    def __repr__(self):
        return f"Base Sensor, can't be used directly \n \
                name: {self.name} \n \
                type: {self.type}"
    
        
        
