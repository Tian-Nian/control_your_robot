import numpy as np
import pyrealsense2 as rs
import time
from sensor.vision_sensor import VisionSensor
from copy import copy

from utils.data_handler import debug_print


def find_device_by_serial(devices, serial):
    """Find device index by serial number"""
    for i, dev in enumerate(devices):
        if dev.get_info(rs.camera_info.serial_number) == serial:
            return i
    return None

class CVSensor(VisionSensor):
    def __init__(self, name):
        super().__init__()
        self.name = name
    
    def set_up(self,CAMERA_ID,is_depth = False):
        self.is_depth = is_depth
        try:
            pass
        except Exception as e:

            raise RuntimeError(f"Failed to initialize camera: {str(e)}")

    def get_image(self):
        image = {}
        frame = self.pipeline.wait_for_frames()

        if "color" in self.collect_info:
            color_frame = frame.get_color_frame()
            if not color_frame:
                raise RuntimeError("Failed to get color frame.")
            color_image = np.asanyarray(color_frame.get_data()).copy()
            # BGR -> RGB
            image["color"] = color_image[:,:,::-1]

        if "depth" in self.collect_info:
            if not self.is_depth:
                debug_print(self.name, f"should use set_up(is_depth=True) to enable collecting depth image","ERROR")
                raise ValueError
            else:       
                depth_frame = frame.get_depth_frame()
                if not depth_frame:
                    raise RuntimeError("Failed to get depth frame.")
                depth_image = np.asanyarray(depth_frame.get_data()).copy()
                image["depth"] = depth_image
        
        return image

if __name__ == "__main__":
    cam = RealsenseSensor("test")
    cam.set_up("419522071856")
    cam.set_collect_info(["color"])
    cam_list = []
    for i in range(1000):
        print(i)
        data = cam.get_image()
        cam_list.append(data)
        time.sleep(0.1)