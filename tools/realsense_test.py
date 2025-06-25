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
    config.enable_device("336222070133")
    
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    pipeline.start(config)
    
    for _ in range(50):  # Skip the first 50 frames to allow auto-exposure to stabilize.
        pipeline.wait_for_frames()
    
    data = []
    for i in range(100):
        frames = pipeline.wait_for_frames()
        print(i)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        
    
        if not color_frame or not depth_frame:
            raise RuntimeError("unable to get frame")
        
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        data.append(color_image)
        time.sleep(0.1)
    
    # make timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # save RGB image(cv2 use BGR data format to save RGB data)
    color_filename = os.path.join(output_dir, f"color_{timestamp}.png")
    cv2.imwrite(color_filename, color_image)
    print(f"RGB saved: {color_filename}")
    
    # save depth image
    depth_colormap = cv2.applyColorMap(
        cv2.convertScaleAbs(depth_image, alpha=0.03), 
        cv2.COLORMAP_JET
    )
    depth_filename = os.path.join(output_dir, f"depth_{timestamp}.png")
    cv2.imwrite(depth_filename, depth_colormap)
    print(f"depth data saved: {depth_filename}")
    
    # save origin depth data
    depth_raw_filename = os.path.join(output_dir, f"depth_raw_{timestamp}.npy")
    np.save(depth_raw_filename, depth_image)
    print(f"origin depth data saved: {depth_raw_filename}")
    
    # # display image(optional)
    # cv2.imshow('Color Image', color_image)
    # cv2.imshow('Depth Image', depth_colormap)
    # cv2.waitKey(2000) 
    # cv2.destroyAllWindows()
        
if __name__ == "__main__":
    save_realsense_images()