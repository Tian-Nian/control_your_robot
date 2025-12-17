import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime
import time

def save_realsense_images():
    output_dir = "realsense_captures"
    os.makedirs(output_dir, exist_ok=True)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device("233722072561")
    
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    pipeline.start(config)
    
    for _ in range(50):  # Skip the first 50 frames to allow auto-exposure to stabilize.
        pipeline.wait_for_frames()
    
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
        
    color_image = np.asanyarray(color_frame.get_data())
    cv2.imwrite("test.jpg", color_image)
    # data = []
    # for i in range(100000000000):
    #     frames = pipeline.wait_for_frames()
    #     # print(i)
    #     color_frame = frames.get_color_frame()
        
    #     color_image = np.asanyarray(color_frame.get_data())

    #     cv2.imshow('Color_Image', color_image)
    #     # cv2.imshow('Depth Image', depth_colormap)
    #     cv2.waitKey(10) 
        
if __name__ == "__main__":
    save_realsense_images()
