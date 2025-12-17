import sys
sys.path.append('./')

from my_robot.test_robot import TestRobot
from my_robot.y1_dual_base import Y1Dual


from utils.bisocket import BiSocket
from utils.data_handler import debug_print, is_enter_pressed

import socket
import time
import numpy as np
from collections import deque
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

    img_enc, img_enc_len = images_encoding(img_arr)

    return img_enc, state

def output_transform(data):
    if data[6] < 0.1:
        data[6] = 0
    
    if data[13] < 0.1:
        data[13] = 0

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
    def __init__(self,robot,cntrol_freq=10, jump_threshold=0.1, interp_steps=3):
        self.robot = robot
        self.cntrol_freq = cntrol_freq

        self.action_queue = deque()
        self.last_action = None
        self.jump_threshold = jump_threshold
        self.interp_steps = interp_steps
    
    def set_up(self, bisocket:BiSocket):
        self.bisocket = bisocket

    def processor(self, message):
        action_chunk = np.array(message["action_chunk"])

        if self.last_action is None:
            self.last_action = action_chunk[0]

        safe_actions = []

        first_action = action_chunk[0]
        diff = np.linalg.norm(first_action - self.last_action)

        if diff > self.jump_threshold:
            interp_actions = np.linspace(self.last_action, first_action, self.interp_steps)
            for interp_act in interp_actions:
                safe_actions.append(interp_act)
        else:
            safe_actions.append(first_action)

        for i in range(1, len(action_chunk)):
            safe_actions.append(action_chunk[i])

        self.action_queue.clear()
        for a in safe_actions:
            self.action_queue.append(a)

    def step(self):
        """
        控制循环调用：
        - 从队列取一个动作
        - 执行 robot.move()
        - 将此动作作为 last_action
        """

        if len(self.action_queue) == 0:
            return  # 空队列则等待下一帧

        action = self.action_queue.popleft()

        move_data = output_transform(action)
        self.robot.move(move_data)

        # 在这里更新 last_action，保证跳变判断基于真实执行动作
        self.last_action = action

    def play_once(self, instruction):
        raw_data = self.robot.get()
        img_arr, state = input_transform(raw_data)
        data_send = {
            "img_arr": img_arr,
            "state": state,
            "instruction": instruction,
        }

        # send data
        self.bisocket.send(data_send)
        time.sleep(1 / self.cntrol_freq)

    def close(self):
        self.bisocket.close()
        return

if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "INFO"
    
    ip = "127.0.0.1"
    port = 10002

    DoFs = 6
    # robot = TestRobot(DoFs=DoFs, INFO="DEBUG")
    robot = Y1Dual()
    robot.set_up()

    robot.reset()

    client = Client(robot)

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((ip, port))

    bisocket = BiSocket(client_socket, client.processor)
    client.set_up(bisocket)

    while True:
        try:
            if is_enter_pressed():
                exit()
            client.play_once("Open the box, take out the bag inside the box, place it on the right side, fold the box, place the bag on the box.")
            for i in range(30):
                client.step()
                time.sleep(1/20)
        except:
            client.close()