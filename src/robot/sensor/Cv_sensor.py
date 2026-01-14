import cv2
import numpy as np
import time
from robot.sensor.vision_sensor import VisionSensor
from robot.utils.base.data_handler import debug_print


class CvSensor(VisionSensor):
    def __init__(self, name, encode_rgb=False):
        super().__init__(encode_rgb=encode_rgb)
        self.name = name
        self.cap = None
        self.is_depth = False

    def set_up(self, device_index=0, is_depth=False, encode_rgb=False):
        """
        初始化摄像头
        :param device_index: 摄像头索引号（0 为默认摄像头）
        :param is_depth: 是否为深度摄像头（True 时必须外部提供深度数据）
        """
        self.encode_rgb = encode_rgb
        self.is_depth = is_depth
        try:
            self.cap = cv2.VideoCapture(device_index, cv2.CAP_ANY)
            if not self.cap.isOpened():
                raise RuntimeError(f"Failed to open camera index {device_index}")
            
            # 设置分辨率和帧率
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)

            print(f"Started camera: {self.name} (Index: {device_index})")
        except Exception as e:
            self.cleanup()
            raise RuntimeError(f"Failed to initialize camera: {str(e)}")

    def get_image(self):
        """
        获取图像数据，返回 dict
        可能包含：
        - color: RGB 图像
        - depth: 深度图（如果 is_depth=True）
        """
        image = {}
        if not self.cap or not self.cap.isOpened():
            raise RuntimeError("Camera is not opened.")

        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to get frame from camera.")

        if "color" in self.collect_info:
            # OpenCV 默认是 BGR，需要转成 RGB
            image["color"] = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        if "depth" in self.collect_info:
            if not self.is_depth:
                debug_print(self.name, "should use set_up(is_depth=True) to enable collecting depth image", "ERROR")
                raise ValueError("Depth capture not enabled.")
            else:
                # 普通摄像头没有深度，需要用户自己对接深度数据，这里用全零代替
                depth_image = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint16)
                image["depth"] = depth_image

        return image

    def cleanup(self):
        """释放摄像头资源"""
        try:
            if self.cap and self.cap.isOpened():
                self.cap.release()
        except Exception as e:
            print(f"Error during cleanup: {str(e)}")

    def __del__(self):
        self.cleanup()


if __name__ == "__main__":
    cam = CvSensor("test_cv")
    cam.set_up(0)  # 默认摄像头
    cam.set_collect_info(["color"])  # 只采集彩色
    cam_list = []
    for i in range(100):
        print(i)
        data = cam.get_image()
        cam_list.append(data)
        time.sleep(0.1)
