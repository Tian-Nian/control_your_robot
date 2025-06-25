import sys
sys.path.append("./")

from utils.data_handler import debug_print

class Sensor:
    def __init__(self):
        self.name = "sensor"
        self.type = "sensor"
    
    def set_collect_info(self, collect_info):
       self.collect_info = collect_info
    
    def get(self):
        if self.collect_info is None:
            debug_print({self.name},f"collect_info is not set, if only collecting controller data, forget this warning", "WARNING")
            return None
        info = self.get_information()
        for collect_info in self.collect_info:
            if info[collect_info] is None:
                debug_print(f"{self.name}", f"{collect_info} information is None", "ERROR")
        # 由于sensor数据比较高维, 所以不输出, 只调试信息是否为None
        # debug_print(f"{self.name}", f"get data:\n{info} ", "DEBUG")
        return {collect_info: info[collect_info] for collect_info in self.collect_info}

    def __repr__(self):
        return f"Base Sensor, can't be used directly \n \
                name: {self.name} \n \
                type: {self.type}"
    
        
        
