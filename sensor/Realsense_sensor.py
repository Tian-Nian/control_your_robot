import numpy as np
import pyrealsense2 as rs
import time
from sensor.vision_sensor import VisionSensor
from copy import copy
import threading
from collections import deque
from utils.data_handler import debug_print


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
        # 添加线程控制变量
        self.frame_buffer = deque(maxlen=1)  # 仅保留最新帧
        self.keep_running = False
        self.exit_event = threading.Event()
        self.thread = None
        
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
                #start mutitheading process
                self.keep_running = True
                self.exit_event.clear()
                self.thread = threading.Thread(target=self._update_frames)
                self.thread.daemon = True
                self.thread.start()
                print(f"Started camera: {self.name} (SN: {serial})")
            except RuntimeError as e:
                raise RuntimeError(f"Error starting camera: {str(e)}")
        except Exception as e:
            self.cleanup()
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
    def _update_frames(self):
        """独立线程持续获取帧数据"""
        try:
            while not self.exit_event.is_set():
                frames = self.pipeline.wait_for_frames(5000)  # 带超时
                
                # 分离颜色和深度帧
                frame_data = {}
                if "color" in self.collect_info:
                    color_frame = frames.get_color_frame()
                    if color_frame:
                        frame_data["color"] = np.asanyarray(color_frame.get_data())[:,:,::-1]
                
                if self.is_depth and "depth" in self.collect_info:
                    depth_frame = frames.get_depth_frame()
                    if depth_frame:
                        frame_data["depth"] = np.asanyarray(depth_frame.get_data())
                
                if frame_data:
                    self.frame_buffer.append(frame_data)
                    
        except RuntimeError as e:
            if "timeout" in str(e):
                print(f"{self.name} 帧等待超时，重试中...")
            else:
                raise
        except Exception as e:
            print(f"{self.name} 捕获异常: {str(e)}")
        #
    def get_image_mp(self):
        """非阻塞获取最新帧"""
        return self.frame_buffer[-1] if self.frame_buffer else None
    def cleanup(self):
        try:
            if hasattr(self, 'pipeline'):
                self.pipeline.stop()
        except Exception as e:
            print(f"Error during cleanup: {str(e)}")
    def cleanup_mp(self):
        self.exit_event.set()
        self.keep_running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        if hasattr(self, 'pipeline'):
            self.pipeline.stop()
    def __del__(self):
        self.cleanup()

if __name__ == "__main__":
    cam = RealsenseSensor("head")
    cam1= RealsenseSensor("left")
    cam2= RealsenseSensor("right")

    cam.set_up("313522071698")
    cam1.set_up("948122073452")
    cam2.set_up("338622074268")
    
    cam.set_collect_info(["color"])
    cam1.set_collect_info(["color"])
    cam2.set_collect_info(["color"])

    cam_list = []
    cam_list1 = []
    cam_list2 = []

    for i in range(500):
        print(i)
        data = cam.get_image_mp()
        data1 = cam1.get_image_mp()
        data2 = cam2.get_image_mp()
        
        cam_list.append(data)
        cam_list1.append(data1)
        cam_list2.append(data2)
        
        time.sleep(0.1)
