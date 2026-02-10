
from my_robot.test_robot import TestRobot

from robot.utils.base.bisocket import BiSocket
from robot.utils.base.data_handler import debug_print, is_enter_pressed

import socket
import time
import numpy as np
import cv2

def images_encoding(imgs):
    encode_data = []
    padded_data = []
    max_len = 0
    for i in range(len(imgs)):
        success, encoded_image = cv2.imencode('.jpg', imgs[i])
        jpeg_data = encoded_image.tobytes()
        encode_data.append(jpeg_data)
        max_len = max(max_len, len(jpeg_data))
    # padding
    for i in range(len(imgs)):
        padded_data.append(encode_data[i].ljust(max_len, b'\0'))
    return encode_data, max_len

def input_transform(data, size=256):
    # ====== 处理 state ======
    state = np.concatenate([
        np.array(data[0]["left_arm"]["joint"]).reshape(-1),
        np.array(data[0]["left_arm"]["gripper"]).reshape(-1),
        np.array(data[0]["right_arm"]["joint"]).reshape(-1),
        np.array(data[0]["right_arm"]["gripper"]).reshape(-1),
    ])

    # ====== 处理图像 ======
    img_arr = [
        data[1]["cam_head"]["color"],
        data[1]["cam_right_wrist"]["color"],
        data[1]["cam_left_wrist"]["color"],
    ]
    if isinstance(img_arr[0], bytes):
        img_enc = img_arr
    else:
        img_enc, img_enc_len = images_encoding(img_arr)

    return img_enc, state

def output_transform(data):
    move_data = {
        "arm":{
            "left_arm":{
                "joint":data[:6],
                "gripper":data[6]
            },
            "right_arm":{
                "joint":data[7:13],
                "gripper":data[13]
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
        action_chunk = message["action_chunk"]
        action_chunk = np.array(action_chunk)

        for action in action_chunk:
            move_data = output_transform(action)
            self.robot.move(move_data)
            time.sleep(1 / self.cntrol_freq)

    def play_once(self):
        raw_data = self.robot.get()
        img_arr, state = input_transform(raw_data)
        data_send = {
            "img_arr": img_arr,
            "state": state
        }

        # send data
        # self.bisocket.send(data_send)
        self.bisocket.send_and_wait_reply(data_send, timeout=30.)
        # time.sleep(1 / self.cntrol_freq)

    def close(self):
        return

if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "INFO"
    
    ip = "127.0.0.1"
    port = 10000

    DoFs = 6
    robot = TestRobot(DoFs=DoFs, INFO="DEBUG")
    robot.set_up()

    client = Client(robot)

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((ip, port))

    bisocket = BiSocket(client_socket, client.move, enable_loop=False)
    client.set_up(bisocket)

    while True:
        if is_enter_pressed():
            break
        client.play_once()
    client.close()