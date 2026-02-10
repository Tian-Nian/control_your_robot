

from robot.sensor.touch_sensor import TouchSensor
import numpy as np
import serial
import threading
import cv2
import time
from scipy.ndimage import gaussian_filter
import logging
logger = logging.getLogger(__name__)

def apply_gaussian_blur(contact_map, sigma=0.1):
    return gaussian_filter(contact_map, sigma=sigma)

def temporal_filter(new_frame, prev_frame, alpha=0.2):
    """
    Apply temporal smoothing filter.
    'alpha' determines the blending factor.
    A higher alpha gives more weight to the current frame, while a lower alpha gives more weight to the previous frame.
    """
    return alpha * new_frame + (1 - alpha) * prev_frame

class Vitac3D(TouchSensor):
    #[paper](https://arxiv.org/pdf/2410.24091)
    #[hardware code](https://github.com/binghao-huang/3D-ViTac_Tactile_Hardware.git)
    def __init__(self,name):
        super().__init__()
        self.name = name
        
        self.THRESHOLD =12
        self.NOISE_SCALE =60
        self.baud=2000000
        

        self.current_data = np.zeros((16, 16), dtype=np.uint8)  # 存储处理好的uint8数据
        self.processed_data = None  # 存储滤波后的浮点数据
        self.median = None  # 存储初始化的中值
        self.initialized = False  # 初始化完成标志
        self.lock = threading.Lock()  # 线程锁保护共享数据
        self.exit_flag = threading.Event()
        
    def set_up(self, PORT,is_show):
        """设置串口"""
        self.is_show = is_show
        self.serDev = serial.Serial(PORT, self.baud, timeout=1)
        if not self.serDev.is_open:
            raise Exception(f"无法打开串口: {PORT}")
        print(f"3DVitac sensor set up on {PORT}")
        self.serDev.flush()
        if self.is_show:          
            cv2.namedWindow("Contact Data_left", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Contact Data_left", 480, 480)  # 直接使用480×480尺寸
        print(f"{self.name} sensor set up complete on {PORT}")
        self.thread = threading.Thread(target=self.readThread)
        self.thread.daemon = True
        self.thread.start()
        print(f"wait {self.name} sensor initialized")
        while not self.initialized:
            time.sleep(0.1)
        
        
        
    def readThread(self):
        data_tac = []
        current = []
        frame_count = 0
        INIT_FRAMES = 30
        
        print(f"{self.name} sensor reading thread started")
        
        while not self.initialized and not self.exit_flag.is_set():
            if self.serDev.in_waiting > 0:
                try:
                    line = self.serDev.readline().decode('utf-8', errors='ignore').strip()
                except Exception as e:
                    logger.error(f"Decode error: {e}")
                    continue
                
                # 关键帧结束检测逻辑
                if len(line) < 10:
                    if len(current) == 16:
                        try:
                            frame = np.array(current, dtype=np.int16)
                            data_tac.append(frame)
                            frame_count += 1
                            
                            if frame_count >= INIT_FRAMES:
                                with self.lock:
                                    self.median = np.median(data_tac, axis=0)
                                    self.initialized = True
                                    print(f"{self.name} initialization complete with {frame_count} frames")
                        except Exception as e:
                            logger.error(f"Frame processing error: {e}")
                    current = []
                    continue
                
                # 解析数据行
                try:
                    str_values = line.split()
                    if len(str_values) != 16:
                        continue
                    
                    int_values = [int(val) for val in str_values]
                    current.append(int_values)
                except Exception as e:
                    logger.error(f"Data parsing error: {e}")
                    continue
        
        # 实时处理阶段
        prev_frame = np.zeros((16, 16))
        print(f"{self.name} entering real-time processing")
        
        while not self.exit_flag.is_set():
            if self.serDev.in_waiting > 0:
                try:
                    line = self.serDev.readline().decode('utf-8', errors='ignore').strip()
                except:
                    continue
                
                # 帧结束检测
                if len(line) < 10:
                    if len(current) == 16:
                        try:
                            frame = np.array(current, dtype=np.int16)
                            
                            # 数据处理流程
                            contact_data = frame - self.median - self.THRESHOLD
                            contact_data = np.clip(contact_data, 0, 100)
                            
                            if np.max(contact_data) < self.THRESHOLD:
                                contact_data_norm = contact_data / self.NOISE_SCALE
                            else:
                                contact_data_norm = contact_data / np.max(contact_data)
                            
                            # 应用时间滤波
                            filtered_data = temporal_filter(contact_data_norm, prev_frame)
                            prev_frame = filtered_data
                            
                            # 转换为uint8并存储
                            data_scaled = (filtered_data * 255).astype(np.uint8)
                            
                            with self.lock:
                                self.current_data = data_scaled
                                self.processed_data = filtered_data
                        except Exception as e:
                            logger.error(f"Real-time processing error: {e}")
                    
                    current = []
                    continue
                
                # 解析数据行
                try:
                    str_values = line.split()
                    if len(str_values) != 16:
                        continue
                    
                    int_values = [int(val) for val in str_values]
                    current.append(int_values)
                except:
                    continue

    def get_touch(self):
        tac_data = {}
        with self.lock:
            data = self.current_data.copy()
            processed = self.processed_data.copy() if self.processed_data is not None else None
        if "force" in self.collect_info:
            tac_data["force"]=np.asanyarray(data).copy()
        # 显示图像（如果启用）
        if self.is_show and data is not None:
            # 重新缩放为0-255（如果经过滤波可能超出范围）
            display_data = (processed * 255).astype(np.uint8) if processed is not None else data
            colormap = cv2.applyColorMap(display_data, cv2.COLORMAP_VIRIDIS)
            cv2.imshow("Contact Data_left", colormap)
            cv2.waitKey(1)
        # 返回触摸数据
        return tac_data
    def close(self):
        self.exit_flag.set()
        if self.serDev.is_open:
            self.serDev.close()
        if self.is_show:
            cv2.destroyWindow(f"Contact Data_{self.name}")
        print(f"{self.name} sensor closed")
    def __del__(self):
        self.close()
if __name__ == "__main__":
    tac=Vitac3D("leftarm_left_tac")
    tac.set_up("/dev/ttyUSB0",is_show=True)

    tac.set_collect_info("force")
    
    for i in range(1000):
        print(i)
        data = tac.get_touch()
        
        time.sleep(1/30)