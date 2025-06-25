import sys
sys.path.append("./")

import numpy as np
import time
from sensor.vision_sensor import VisionSensor

from utils.data_handler import debug_print

class TestVisonSensor(VisionSensor):
    def __init__(self, name,INFO="DEBUG"):
        super().__init__()
        self.name = name
        self.INFO = INFO
    
    def set_up(self,is_depth = False):
        debug_print(self.name, f"setup success, is_depth={is_depth}",self.INFO)
        self.is_depth = is_depth


    def get_image(self):
        image = {}
        height = 480
        width = 640
        if "color" in self.collect_info:
            image["color"] = np.random.randint(0, 256, size=(height, width, 3), dtype=np.uint8)

        if "depth" in self.collect_info:
            if not self.is_depth:
                debug_print(self.name,f"should use set_up(is_depth=True) to enable collecting depth image","ERROR")
                raise ValueError
            image["depth"] = np.random.randint(0, 256, size=(height, width, 3), dtype=np.uint8)
        debug_print(self.name,f"get image success",self.INFO)

        return image

    def cleanup(self):
        try:
            if hasattr(self, 'pipeline'):
                self.pipeline.stop()
        except Exception as e:
            print(f"Error during cleanup: {str(e)}")

    def __del__(self):
        self.cleanup()

if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "INFO"
    
    cam = TestVisonSensor("test", INFO="DEBUG")
    cam.set_up()
    cam.set_collect_info(["color"])
    cam_list = []
    for i in range(1000):
        print(i)
        data = cam.get()
        time.sleep(0.1)
