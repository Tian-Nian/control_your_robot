import sys
sys.path.append("./")

import numpy as np

from sensor.teleoperation_sensor import TeleoperationSensor
from utils.data_handler import matrix_to_xyz_rpy, compute_local_delta_pose, debug_print, apply_local_delta_pose

from scipy.spatial.transform import Rotation as R
from typing import Callable, Optional

from oculus_reader import OculusReader

'''
QuestVR base code from:
https://github.com/rail-berkeley/oculus_reader.git
'''

def adjustment_matrix(transform):
    if transform.shape != (4, 4):
        raise ValueError("Input transform must be a 4x4 numpy array.")
    
    adj_mat = np.array([
        [0,0,-1,0],
        [-1,0,0,0],
        [0,1,0,0],
        [0,0,0,1]
    ])
    
    r_adj = self.tools.xyzrpy2Mat(0,0,0,   -np.pi , 0, -np.pi/2)
    
    transform = adj_mat @ transform  
    
    transform = np.dot(transform, r_adj)  
    
    return transform


class QuestSensor(TeleoperationSensor):
    def __init__(self,name):
        super().__init__()
        self.name = name
    
    def set_up(self):
        
        self.sensor = OculusReader()

        self.prev_qpos = None

    def get_state(self):
        transformations, buttons = self.sensor.get_transformations_and_buttons()
        if 'r' not in transformations:
            qpos = None
        
        right_pose = matrix_to_xyz_rpy(adjustment_matrix(transformations['r']))
        left_pose = matrix_to_xyz_rpy(adjustment_matrix(transformations['l']))
        
        qpos = np.concatenate([left_pose, right_pose])
        if self.prev_qpos is None:
            self.prev_qpos = qpos
            qpos = np.array([0,0,0,0,0,0])
        else:
            qpos = compute_local_delta_pose(self.prev_qpos, qpos)

        return {
            "end_pose":qpos,
            "extra": buttons,
        }

    def reset(self, buttons):
        debug_print(f"{self.name}", "reset success!", "INFO")
        return

if __name__ == "__main__":
    import time
    teleop = QuestSensor("left_pika")

    teleop.set_up()

    teleop.set_collect_info(["end_pose","extra"]) 
    
    while True:
        pose, buttons = teleop.get_state()["end_pose"]
        left_pose = pose[:6]
        right_pose = pose[-6:]

        teleop.reset(buttons)
        
        print("left_pose:\n", left_pose)
        print("right_pose:\n", right_pose)
        print("buttons:\n", buttons)
        time.sleep(0.1)