import sys
sys.path.append('./')

# from my_robot.test_robot import TestRobot
from my_robot.agilex_piper_dual_base import PiperDual
from utils.bisocket import BiSocket
from utils.data_handler import debug_print, is_enter_pressed, hdf5_groups_to_dict
from my_robot.base_robot import dict_to_list
import socket
import time
import numpy as np

import cv2

class Replay:
    def __init__(self, hdf5_path) -> None:
        self.ptr = 0
        self.episode = dict_to_list(hdf5_groups_to_dict(hdf5_path))
    def get_data(self):
        print(self.episode[self.ptr].keys())
        data = self.episode[self.ptr], self.episode[self.ptr]
        self.ptr += 30
        return data
    
def input_transform(data):
    state = np.concatenate([
        np.array(data[0]["right_arm"]["joint"]).reshape(-1),
        np.array(data[0]["right_arm"]["gripper"]).reshape(-1),
        np.array(data[0]["left_arm"]["joint"]).reshape(-1),
        np.array(data[0]["left_arm"]["gripper"]).reshape(-1),
    ])

    data[1]["cam_head"]["color"] = cv2.cvtColor(data[1]["cam_head"]["color"], cv2.COLOR_BGR2RGB)
    data[1]["cam_left_wrist"]["color"] = cv2.cvtColor(data[1]["cam_left_wrist"]["color"], cv2.COLOR_BGR2RGB)
    data[1]["cam_right_wrist"]["color"] = cv2.cvtColor(data[1]["cam_right_wrist"]["color"], cv2.COLOR_BGR2RGB)
    
    cv2.imshow("cam_head", data[1]["cam_head"]["color"])
    cv2.imshow("cam_left_wrist", data[1]["cam_left_wrist"]["color"])
    cv2.imshow("cam_right_wrist", data[1]["cam_right_wrist"]["color"])

    cv2.waitKey(1)

    img_arr = data[1]["cam_head"]["color"], data[1]["cam_right_wrist"]["color"], data[1]["cam_left_wrist"]["color"],
    return img_arr, state

def output_transform(data):
    move_data = {
        "arm":{
            "right_arm":{ # left_arm
                "joint":data[:6],
                "gripper":data[6],
            },
            "left_arm":{
                "joint":data[7:13],
                "gripper":data[13],
            }
        },
    }
    return move_data

class Client:
    def __init__(self,robot,cntrol_freq=10):
        self.robot = robot
        self.cntrol_freq = cntrol_freq
    
    def set_up(self, bisocket:BiSocket):
        self.bisocket = bisocket

    def move(self, message):
        # print(message)
        action_chunk = message["action_chunk"]
        action_chunk = np.array(action_chunk)

        for action in action_chunk[:30]:
            move_data = output_transform(action)
            # if move_data["arm"]["left_arm"]["gripper"] < 0.5 :
            #     move_data["arm"]["left_arm"]["gripper"] = 0.0
            # if move_data["arm"]["right_arm"]["gripper"] < 0.5 :
            #     move_data["arm"]["right_arm"]["gripper"] = 0.0
            
            self.robot.move(move_data)
            time.sleep(0.1)

    def play_once(self, R=None):
        if R:
            raw_data = R.get_data()
        else:    
            time.sleep(0.1)
            raw_data = self.robot.get()
        print(raw_data)
            
        img_arr, state = input_transform(raw_data)
        data_send = {
            "img_arr": img_arr,
            "state": state
        }

        # send data
        self.bisocket.send(data_send)

        time.sleep(1 / self.cntrol_freq)

    def close(self):
        return

if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "DEBUG"
    
    ip = "192.3.8.74"
    port = 10000

    DoFs = 6
    robot = PiperDual()
    robot.set_up()
    robot.reset()

    client = Client(robot)
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((ip, port))

    bisocket = BiSocket(client_socket, client.move)
    client.set_up(bisocket)
    re = Replay("save/Make_a_beef_sandwichv2/10.hdf5")

    while True:
        if is_enter_pressed():
            break
        else:
            time.sleep(0.1)
        
    for i in range(1000):
        try:
            # while True:
            #     if is_enter_pressed():
            #         break
            #     else:
            #         time.sleep(0.1)
                
            client.play_once(re)
            # client.play_once()
            print(f"play once:{i}")

            # time.sleep(3)
            while True:
                if is_enter_pressed():
                    break
                else:
                    time.sleep(0.1)
        except:
            client.close()
    client.close()

    