import numpy as np
import pyrealsense2 as rs
import time
from sensor.vision_sensor import VisionSensor
from copy import copy

from utils.data_handler import debug_print
import cv2

def find_device_by_serial(devices, serial):
    """Find device index by serial number"""
    for i, dev in enumerate(devices):
        if dev.get_info(rs.camera_info.serial_number) == serial:
            return i
    return None

class RealsenseSensor(VisionSensor):
    def __init__(self, name):
        super().__init__()
        self.name = name
    
    def set_up(self,CAMERA_SERIAL,is_depth = False):
        self.is_depth = is_depth
        try:
            # Initialize RealSense context and check for connected devices
            self.context = rs.context()
            self.devices = list(self.context.query_devices())
            
            if not self.devices:
                raise RuntimeError("No RealSense devices found")
            
            # Initialize each camera
            serial = CAMERA_SERIAL
            device_idx = find_device_by_serial(self.devices, serial)
            if device_idx is None:
                raise RuntimeError(f"Could not find camera with serial number {serial}")
            
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Enable device by serial number
            self.config.enable_device(serial)
            # self.config.disable_all_streams()
            # Enable color stream only
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            if is_depth:
                self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            
            # Start streaming
            try:
                self.pipeline.start(self.config)
                print(f"Started camera: {self.name} (SN: {serial})")
            except RuntimeError as e:
                raise RuntimeError(f"Error starting camera: {str(e)}")
        except Exception as e:
            self.cleanup()
            raise RuntimeError(f"Failed to initialize camera: {str(e)}")

    def get_image(self):
        try:
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
        except:
            raise RuntimeError(f"Error getting image:{self.name}")
        
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
    cam = RealsenseSensor("test")
    cam.set_up("233722072561", is_depth=True)
    cam.set_collect_info(["color", "depth"])

    for i in range(100000):
        data = cam.get_image()
        depth_image = data["depth"]
        rgb =data["color"]
        depth_img = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
        cv2.imshow("dep", depth_img)
        cv2.imshow("color", rgb)
        cv2.waitKey(33)