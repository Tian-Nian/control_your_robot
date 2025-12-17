import sys
sys.path.append("./")

import numpy as np

from my_robot.base_robot import Robot, dict_to_list

from controller.Y1_controller import Y1Controller
from sensor.Realsense_sensor import RealsenseSensor

from data.collect_any import CollectAny
import time

# setting your realsense serial
CAMERA_SERIALS = {
    'head': '233522079294',  # Replace with actual serial number
    #'left_wrist': '941322071558',   # Replace with actual serial number
     # 'left_wrist': '233722072561',
     'left_wrist': '233722072561',
    #'right_wrist': '337322073597',   # Replace with actual serial number
    # 'right_wrist': '233622078943',   # Replace with actual serial number
    'right_wrist': '337322073597',   # Replace with actual serial number
}

# Define start position (in degrees)
START_POSITION_ANGLE_LEFT_ARM = [
    0,   # Joint 1
    0,    # Joint 2
    0,  # Joint 3
    0,   # Joint 4
    0,  # Joint 5
    0,    # Joint 6
]

# Define start position (in degrees)
START_POSITION_ANGLE_RIGHT_ARM = [
    0,   # Joint 1
    0,    # Joint 2
    0,  # Joint 3
    0,   # Joint 4
    0,  # Joint 5
    0,    # Joint 6
]

condition = {
    "save_path": "./save/",
    "task_name": "deploy_pick_dual_bottles",
    "save_format": "hdf5",
    "save_freq": 30, 
}

class Y1Dual(Robot):
    def __init__(self, condition=condition, move_check=True, start_episode=0):
        super().__init__(condition=condition, move_check=move_check, start_episode=start_episode)

        self.controllers = {
            "arm":{
                "left_arm": Y1Controller("left_arm"),
                "right_arm": Y1Controller("right_arm"),
            }
        }
        self.sensors = {
            "image": {
                "cam_head": RealsenseSensor("cam_head"),
                "cam_left_wrist": RealsenseSensor("cam_left_wrist"),
                "cam_right_wrist": RealsenseSensor("cam_right_wrist"),
            },
        }

    def reset(self):
        self.change_mode(teleop=False)
        time.sleep(1)

        move_data = {
            "arm":{
                "left_arm":{
                    # "joint": [0, 0, 0.3, 0.2, 0, 0],
                    "joint": [0, 0, 0, 0, 0, 0],
                    # "joint": [0.3, 0, 0, 0, 0, 0],
                    "gripper":  1.0,
                },
                
                "right_arm":{
                    # "joint": [0, 0, 0.3, 0.2, 0, 0],
                    "joint": [0, 0, 0, 0, 0, 0],
                    "gripper":  1.0,
                }
            }
        }
        self.move(move_data)
        time.sleep(2)
        # self.change_mode(teleop=True)
    
    def change_mode(self, teleop):
        self.controllers["arm"]["left_arm"].change_mode(teleop)
        self.controllers["arm"]["right_arm"].change_mode(teleop)
        time.sleep(1)

    def set_up(self, teleop=False):
        self.controllers["arm"]["left_arm"].set_up("can0", teleop=teleop)
        self.controllers["arm"]["right_arm"].set_up("can1", teleop=teleop)

        self.sensors["image"]["cam_head"].set_up(CAMERA_SERIALS['head'], is_depth=False)
        self.sensors["image"]["cam_left_wrist"].set_up(CAMERA_SERIALS['left_wrist'], is_depth=False)
        self.sensors["image"]["cam_right_wrist"].set_up(CAMERA_SERIALS['right_wrist'], is_depth=False)

        self.set_collect_type({"arm": ["joint","qpos","gripper"],
                               "image": ["color"]
                               })
        # time.sleep(2)
        print("set up success!")

from scipy.spatial.transform import Rotation as R
def pose_to_matrix(pose):
    """pose: [x, y, z, roll, pitch, yaw] → 4x4 matrix"""
    x, y, z, roll, pitch, yaw = pose
    T = np.eye(4)
    T[:3, :3] = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    T[:3, 3] = [x, y, z]
    return T

def matrix_to_pose(T):
    """4x4 matrix → [x, y, z, roll, pitch, yaw]"""
    x, y, z = T[:3, 3]
    roll, pitch, yaw = R.from_matrix(T[:3, :3]).as_euler('xyz')
    return np.array([x, y, z, roll, pitch, yaw])

def end_pose_transform(tarjectory_base_pose, tarjectory_pose, robot_base_pose):
    """
    Compute robot_pose = robot_base_pose ⊕ (tarjectory_pose ⊖ tarjectory_base_pose)
    
    Inputs:
        tarjectory_base_pose : [x,y,z,r,p,y]
        tarjectory_pose      : [x,y,z,r,p,y]
        robot_base_pose      : [x,y,z,r,p,y]
    Returns:
        robot_pose : [x,y,z,r,p,y]
    """
    # Convert to transformation matrices
    T_tbase = pose_to_matrix(tarjectory_base_pose)
    T_t     = pose_to_matrix(tarjectory_pose)
    T_rbase = pose_to_matrix(robot_base_pose)

    # 1. Compute delta in trajectory frame: T_delta = T_tbase^-1 * T_t
    T_delta = np.linalg.inv(T_tbase) @ T_t

    # 2. Apply delta to robot base frame: T_robot = T_rbase * T_delta
    T_robot = T_rbase @ T_delta

    # Convert back to pose
    return matrix_to_pose(T_robot)

def test_delta_eef():
    from utils.data_handler import debug_print, hdf5_groups_to_dict
    hdf5_path = "save/pick_bag_1118/_zip/10.hdf5"

    robot = Y1Dual()
    robot.set_up(teleop=False)
    robot.reset()

    left_robot_base_eef_pose = robot.get()[0]["left_arm"]["qpos"]
    right_robot_base_eef_pose = robot.get()[0]["right_arm"]["qpos"]

    episode = dict_to_list(hdf5_groups_to_dict(hdf5_path))

    left_base_eef_pose = episode[0]["left_arm"]["qpos"]
    right_base_eef_pose = episode[0]["right_arm"]["qpos"]

    for ep in episode:
        left_eef_pose = ep["left_arm"]["qpos"]
        right_eef_pose = ep["right_arm"]["qpos"]

        left_robot_eef_pose = end_pose_transform(left_base_eef_pose, left_eef_pose, left_robot_base_eef_pose)
        right_robot_eef_pose = end_pose_transform(right_base_eef_pose, right_eef_pose, right_robot_base_eef_pose)

        robot.move({
            "arm":{
                "left_arm":{
                    "qpos": left_robot_eef_pose,
                },
                "right_arm":{
                    "qpos": right_robot_eef_pose,
                }
            }
        })
        time.sleep(0.05)

if __name__ == "__main__":
    test_delta_eef()
    exit()

    robot = Y1Dual()

    hdf5_path = "save/pick_bag_1118/_zip/10.hdf5"

    # replay test
    robot.set_up(teleop=False)
    # time.sleep(2)
    robot.reset()
    # print(robot.get()[1]["cam_head"]["color"])
    # robot.replay(hdf5_path, key_banned=["joint"])
    # # robot.replay("./save/stack_bowls_three_zip_unzip/31.hdf5", key_banned=["qpos"])
    # # robot.replay("./stack_bowls_two.hdf5", key_banned=["qpos"], is_collect=True)


    exit()

    # image test
    
    # robot.show_pic(hdf5_path, "cam_head", save_video=False, fps=5)
    # robot.show_pic(hdf5_path, "cam_left_wrist", save_video=False, fps=5)
    # robot.show_pic(hdf5_path, "cam_right_wrist", save_video=False, fps=5)
    # exit()

    # collection test
    robot.set_up(teleop=True)
    time.sleep(10000)

    for i in range(100):
        print(i)
        data = robot.get()
        robot.collect(data)
        time.sleep(0.01)
    robot.finish()

    time.sleep(100)
