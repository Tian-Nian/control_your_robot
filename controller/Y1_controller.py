import sys
sys.path.append("./")

from controller.arm_controller import ArmController
from utils.data_handler import debug_print, is_enter_pressed
from my_robot.base_robot import dict_to_list

from y1_sdk import Y1SDKInterface, ControlMode
import numpy as np
import time

'''
Piper base code from:
https://github.com/agilexrobotics/piper_sdk.git
'''

package_path = "/home/xspark-ai/project/y1_sdk_python/y1_ros/src/y1_controller/"

class Y1Controller(ArmController):
    def __init__(self, name):
        super().__init__()
        self.name = name
        self.controller_type = "user_controller"
        self.controller = None
    
    def set_up(self, can:str, arm_end_type=3, teleop=False):
        self.arm_end_type = arm_end_type
        if arm_end_type == 0:
            urdf_path = f"{package_path}/urdf/y10804.urdf"
        elif arm_end_type == 1:
            urdf_path = f"{package_path}/urdf/y1_gripper_t.urdf"
        elif arm_end_type == 2:
            urdf_path = f"{package_path}/urdf/y1_gripper_g.urdf"
        elif arm_end_type == 3:
            urdf_path = f"{package_path}/urdf/y1_with_gripper.urdf"

        self.controller = Y1SDKInterface(
            can_id=can,
            urdf_path=urdf_path,
            arm_end_type=arm_end_type,
            enable_arm=True,
        )

        if not self.controller.Init():
            debug_print(self.name, "Init Fail!", "ERROR")
            # exit()
        if teleop:
            self.controller.SetArmControlMode(ControlMode.GRAVITY_COMPENSATION)
        else:
            self.controller.SetArmControlMode(ControlMode.NRT_JOINT_POSITION)

    def change_mode(self, teleop):
        if teleop:
            self.controller.SetArmControlMode(ControlMode.GRAVITY_COMPENSATION)
        else:
            self.controller.SetArmControlMode(ControlMode.NRT_JOINT_POSITION)
        time.sleep(2)
        
    # 返回单位为米
    def get_state(self):
        state = {}
        
        eef = self.controller.GetArmEndPose()
        joint = self.controller.GetJointPosition()
        vel = self.controller.GetJointVelocity()

        state["qpos"] = eef
        state["joint"] = joint[:6]
        if self.arm_end_type != 0:
            state["gripper"] = joint[6] / 100

        return state

    # All returned values are expressed in meters,if the value represents an angle, it is returned in radians
    def set_position(self, position):
        self.controller.SetArmEndPose(list(position))
    
    def set_joint(self, joint):
        print(joint)
        self.controller.SetArmJointPosition(list(joint), 5)

    # The input gripper value is in the range [0, 1], representing the degree of opening.
    def set_gripper(self, gripper):
        gripper = gripper * 100
        self.controller.SetGripperStroke(gripper)
        print("set!!!!")

    def __del__(self):
        try:
            if hasattr(self, 'controller'):
                # Add any necessary cleanup for the arm controller
                pass
        except:
            pass
if __name__=="__main__":
    # left_controller = Y1Controller("test_y1_left")
    right_controller = Y1Controller("test_y1_right")
    # left_controller.set_up("can1", 3, False)
    right_controller.set_up("can0", 3, True)

    while True:
        print(right_controller.get_state()["qpos"])
        time.sleep(0.1)

    # move_data = {"qpos": [0.039362965487455576, -0.0001585010972597902, 0.19092359541622742, -3.04687070405715, -1.5548993674145568, -0.09090050481295406],
    #                       }

    move_data = {
            "joint": [0, 0, 0, 0, 0, 0],
        }
    
    right_controller.move(move_data)
    time.sleep(1)
    print(right_controller.get_state()["joint"])
    print(right_controller.get_state()["qpos"])

    # time.sleep(2)

    # from utils.data_handler import hdf5_groups_to_dict
    # data_path = "save/test/0.hdf5"
    # episode = dict_to_list(hdf5_groups_to_dict(data_path))
    # for ep in episode:
    #     left_move_data = {"joint": ep["left_arm"]["joint"],
    #                       "gripper":ep["left_arm"]["gripper"]
    #                       }
    #     right_move_data = {"joint": ep["right_arm"]["joint"],
    #                       "gripper":ep["right_arm"]["gripper"]
    #                       }

    #     left_controller.move(left_move_data)
    #     right_controller.move(right_move_data)

    #     time.sleep(0.1)
    # time.sleep(1)
    # print(controller.get_gripper())
    # print(controller.get_state())

    # controller.set_position(np.array([0.057, 0.0, 0.260, 0.0, 0.085, 0.0]))
    # time.sleep(1)
    # print(controller.get_gripper())
    # print(controller.get_state())
