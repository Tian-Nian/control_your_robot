import sys
sys.path.append('src/')

from my_robot.test_robot import TestRobot

from utils.bisocket import BiSocket
from utils.data_handler import debug_print

import socket
import time
import numpy as np

def input_transform(data):
    state = np.concatenate([
        np.array(data[0]["left_arm"]["joint"]).reshape(-1),
        np.array(data[0]["left_arm"]["gripper"]).reshape(-1),
        np.array(data[0]["right_arm"]["joint"]).reshape(-1),
        np.array(data[0]["right_arm"]["gripper"]).reshape(-1)
    ])
    
    img_arr = data[1]["cam_head"]["color"], data[1]["cam_right_wrist"]["color"], data[1]["cam_left_wrist"]["color"]
    return img_arr, state

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
        if len(self.action_queue) == 0:
            return  # 空队列则等待下一帧

        action = self.action_queue.popleft()

        move_data = output_transform(action)
        self.robot.move(move_data)

        # 在这里更新 last_action，保证跳变判断基于真实执行动作
        self.last_action = action

    def play_once(self):
        raw_data = self.robot.get()
        img_arr, state = input_transform(raw_data)
        data_send = {
            "img_arr": img_arr,
            "state": state,
        }

        # send data
        self.bisocket.send(data_send)
        time.sleep(1 / self.cntrol_freq)

    def close(self):
        self.bisocket.close()
        return

if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "DEBUG"
    
    ip = "127.0.0.1"
    port = 10000

    DoFs = 6
    robot = TestRobot(DoFs=DoFs, INFO="DEBUG")
    robot.set_up()

    client = Client(robot)

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((ip, port))

    bisocket = BiSocket(client_socket, client.processor)
    client.set_up(bisocket)

    while True:
        try:
            if is_enter_pressed():
                exit()
            client.play_once("test")
            for i in range(20):
                client.step()
                time.sleep(1/30)
        except:
            client.close()