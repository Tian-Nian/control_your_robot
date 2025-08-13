import numpy as np
import pyrealsense2 as rs
import time
from sensor.vision_sensor import VisionSensor
from copy import copy
import cv2
from cv_bridge import CvBridge
from utils.data_handler import debug_print

# from utils.ros_publisher import ROSPublisher, start_publishing
from utils.ros_subscriber import ROSSubscriber 

from sensor_msgs.msg import Image

def find_device_by_serial(devices, serial):
    """Find device index by serial number"""
    for i, dev in enumerate(devices):
        if dev.get_info(rs.camera_info.serial_number) == serial:
            return i
    return None

class VisionROSensor(VisionSensor):
    def __init__(self, name):
        super().__init__()
        self.name = name
        self.cv_bridge = CvBridge()
    
    def set_up(self, topic, is_depth = False):
        self.controller = {}
        self.is_depth = is_depth
        try:
            self.controller["subscriber"] = ROSSubscriber(topic, Image)
            if self.is_depth:
                #  self.depth_pipeline = self.ROSSubscriber(topic, Image)
                raise NotImplementedError("is_depth No Implemented")
        except Exception as e:
            self.cleanup()
            raise RuntimeError(f"Failed to initialize camera: {str(e)}")

    def get_image(self):
        image = {}
        rgb_frame = self.controller["subscriber"].get_latest_data()
        if rgb_frame is not None:
            rgb_frame = self.cv_bridge.imgmsg_to_cv2(rgb_frame, desired_encoding="bgr8")
        else:
            debug_print(self.name, "No image data!", "WARNING")
        image["color"] = rgb_frame
        # if "depth" in self.collect_info:
        #     if not self.is_depth:
        #         debug_print(self.name, f"should use set_up(is_depth=True) to enable collecting depth image","ERROR")
        #         raise ValueError
        #     else:       
        #         depth_frame = frame.get_depth_frame()
        #         if not depth_frame:
        #             raise RuntimeError("Failed to get depth frame.")
        #         depth_image = np.asanyarray(depth_frame.get_data()).copy()
        #         image["depth"] = depth_image
        
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
    import rospy
    rospy.init_node('ros_subscriber_node', anonymous=True)

    cam = VisionROSensor("test")

    cam.set_up("/camera_l/color/image_raw")
    cam.set_collect_info(["color"])
    cam_list = []
    for i in range(1000):
        print(i)
        data = cam.get_image()
        # print(data)
        if data["color"] is not None:
            # print(data["color"].type)
            cv2.imshow("data", np.array(data["color"]))
            cv2.waitKey(1)
        time.sleep(0.1)