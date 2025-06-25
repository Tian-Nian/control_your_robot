import sys
sys.path.append("./")

import numpy as np

from sensor.teleoperation_sensor import TeleoperationSensor
from utils.data_handler import matrix_to_xyz_rpy, compute_local_delta_pose, debug_print, apply_local_delta_pose, compute_rotate_matrix

from scipy.spatial.transform import Rotation as R
from typing import Callable, Optional

from pika import sense

'''
QuestVR base code from:
https://github.com/rail-berkeley/oculus_reader.git
'''

class PikaSensor(TeleoperationSensor):
    def __init__(self,name):
        super().__init__()
        self.name = name
    
    def set_up(self, tty, device_name):
        '''
        device_name:跟插入顺序有关
        无线连接: WM0, WM1
        有线连接: T20, T21
        '''
        self.sensor = sense(tty)
        self.prev_qpos = None
        self.device_name = device_name

    def get_state(self):
        qpos = self.sensor.get_pose(self.device_name)
        # gripper = self.sensor.get_encoder_data()['rad'] / np.pi
        if self.prev_qpos is None:
            self.prev_qpos = qpos
            qpos = np.array([0,0,0,0,0,0])
        else:
            qpos = compute_local_delta_pose(self.prev_qpos, qpos)

        qpos = compute_rotate_matrix(qpos)
        return {
            "end_pose":qpos,
            # "extra": gripper,
        }

    def reset(self, buttons):
        debug_print(f"{self.name}", "reset success!", "INFO")
        return

if __name__ == "__main__":
    import time
    teleop = PikaSensor("left_pika")

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