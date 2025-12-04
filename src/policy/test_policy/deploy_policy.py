import numpy as np
import os, sys

current_file_path = os.path.abspath(__file__)
parent_directory = os.path.dirname(current_file_path)
sys.path.append(parent_directory)

from utils.data_handler import debug_print

class TestModel:
    def __init__(self, test_1, test_2, info_level="INFO"):
        self.observation_window = None
        self.INFO = info_level
        debug_print("TestModel", f"get test_info_1: {test_1}", self.INFO)
        debug_print("TestModel", f"get test_info_2: {test_2}", self.INFO)
    
    def update_observation_window(self, img_arr, state):
        debug_print("model",f"update observation windows success", self.INFO)
        pass

    def get_action(self):
        actions = np.random.rand(10, 14)
        debug_print("TestModel", f"infer action success!", self.INFO)
        return actions

    def set_language(self, instruction):
        self.instruction = instruction
    
    def reset_obsrvationwindows(self):
        debug_print("TestModel", "reset success!", self.INFO)
        return

# Encode observation for the model
def encode_obs(observation):
    input_rgb_arr = [
        observation["observation"]["head_camera"]["rgb"],
        observation["observation"]["right_camera"]["rgb"],
        observation["observation"]["left_camera"]["rgb"],
    ]
    input_state = observation["joint_action"]["vector"]

    return input_rgb_arr, input_state


def get_model(usr_args):
    test_1, test_2 = (usr_args["test_info_1"], usr_args["test_info_2"])
    return TestModel(test_1, test_2)

def eval(TASK_ENV, model, observation):

    if model.observation_window is None:
        instruction = TASK_ENV.get_instruction()
        model.set_language(instruction)

    input_rgb_arr, input_state = encode_obs(observation)
    model.update_observation_window(input_rgb_arr, input_state)

    # ======== Get Action ========

    actions = model.get_action()[:model.pi0_step]
    
    for action in actions:
        TASK_ENV.take_action(action)
        observation = TASK_ENV.get_obs()
        input_rgb_arr, input_state = encode_obs(observation)
        model.update_observation_window(input_rgb_arr, input_state)

    # ============================


def reset_model(model):
    model.reset_obsrvationwindows()
