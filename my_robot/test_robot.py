import sys
sys.path.append("./")

import numpy as np

from my_robot.base_robot import Robot

from controller.TestArm_controller import TestArmController
from sensor.TestVision_sensor import TestVisonSensor
from utils.data_handler import debug_print, Replay
from data.collect_any import CollectAny

condition = {
    "save_path": "./save/", 
    "task_name": "uncap_error_corr_1117", 
    "save_format": "hdf5", 
    "save_freq": 10,
}

def transform(data):
    if data is None:
        return np.array([0., 0., 0., 0., 0., 0.,0., 0., 0., 0., 0., 0., 0.,0.])
    state = np.concatenate([
        np.array(data[0]["left_arm"]["joint"]).reshape(-1),
        np.array(data[0]["left_arm"]["gripper"]).reshape(-1),
        np.array(data[0]["right_arm"]["joint"]).reshape(-1),
        np.array(data[0]["right_arm"]["gripper"]).reshape(-1),
    ])
    return state

def transform_policy(data):
    if data is None:
        return np.array([0., 0., 0., 0., 0., 0.,0., 0., 0., 0., 0., 0., 0.,0.])
    state = np.concatenate([
        np.array(data["arm"]["left_arm"]["joint"]).reshape(-1),
        np.array(data["arm"]["left_arm"]["gripper"]).reshape(-1),
        np.array(data["arm"]["right_arm"]["joint"]).reshape(-1),
        np.array(data["arm"]["right_arm"]["gripper"]).reshape(-1),
    ])
    return state

class TestRobot(Robot):
    def __init__(self, condition=condition, move_check=True, start_episode=0, DoFs=6,INFO="DEBUG", replay_path=None):
        super().__init__(condition=condition, move_check=move_check, start_episode=start_episode)  
        
        self.INFO = INFO
        self.DoFs = DoFs
        self.controllers = {
            "arm": {
                "left_arm": TestArmController("left_arm",DoFs=self.DoFs,INFO=self.INFO),
                "right_arm": TestArmController("right_arm",DoFs=self.DoFs,INFO=self.INFO),
            },
        }
        self.sensors = {
            "image": {
                "cam_head": TestVisonSensor("cam_head",INFO=self.INFO),
                "cam_left_wrist": TestVisonSensor("cam_left_wrist",INFO=self.INFO),
                "cam_right_wrist": TestVisonSensor("cam_right_wrist",INFO=self.INFO),
            }, 
        }
        self.condition = condition
        self.collection = CollectAny(condition, start_episode=start_episode)

        if replay_path is not None:
            self.replay_mode = Replay(replay_path)
            self.delta_replay_frame_index = 0
            self.replay_tarjectory = []
            self.policy_tarjectory = []
        else:
            self.replay_mode = None
    
    def get(self, run=True):
        if self.replay_mode is None:
            return super().get()
        else:
            if run:
                data = self.replay_mode.get_data(bias=self.delta_replay_frame_index)
                self.delta_replay_frame_index = 0
            else:
                index = self.replay_mode._index() + self.delta_replay_frame_index
                data = self.replay_mode.get_index_data(index=index)
            return data
    
    def move(self, move_data):
        if self.replay_mode is None:
            super().move(move_data)
        else:
            super().move(move_data)
            replay_index = self.replay_mode._index() + self.delta_replay_frame_index
            self.delta_replay_frame_index += 1
            replay_tarj = transform(self.replay_mode.get_index_data(index=replay_index))
            policy_tarj = transform_policy(move_data)

            self.replay_tarjectory.append(replay_tarj)
            self.policy_tarjectory.append(policy_tarj)

    def reset(self):
        self.controllers["arm"]["left_arm"].reset(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        self.controllers["arm"]["right_arm"].reset(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    
    def set_up(self):
        self.controllers["arm"]["left_arm"].set_up()
        self.controllers["arm"]["right_arm"].set_up()
        self.sensors["image"]["cam_head"].set_up(is_depth=False)
        self.sensors["image"]["cam_left_wrist"].set_up(is_depth=False)
        self.sensors["image"]["cam_right_wrist"].set_up(is_depth=False)
        self.set_collect_type({"arm": ["joint","qpos","gripper"],
                               "image": ["color"],
                               })
    
    def is_start(self):
        return True
    
    def draw(self):
        if self.replay_mode is None:
            debug_print(self.name, "Not open replay mode, not tarjectory saved!", "WARNING")
            return
        debug_print(self.name, "Drawing start!", "INFO")
        import matplotlib.pyplot as plt
        arr1 = np.stack(self.replay_tarjectory)  # shape (N,14)
        arr2 = np.stack(self.policy_tarjectory)  # shape (N,14)
        N, D = arr1.shape

        # ---- 创建子图 ----
        fig, axes = plt.subplots(D, 1, figsize=(12, 2*D), sharex=True)

        for d in range(D):
            axes[d].plot(range(N), arr1[:,d], linestyle='-', label='list1')
            axes[d].plot(range(N), arr2[:,d], linestyle='--', label='list2')
            axes[d].set_ylabel(f'dim {d+1}')
            axes[d].legend(fontsize=8)

        axes[-1].set_xlabel("Index")
        plt.tight_layout()

        # ---- 保存图片 ----
        plt.savefig("dimension_subplots.png", dpi=300)
        plt.close()

        debug_print(self.name, "Drawing finish!", "INFO")

if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "INFO" # DEBUG , INFO, ERROR
    
    robot = TestRobot(replay_path="save/complete_traj_1117/20.hdf5")

    robot.set_up()

    
    for i in range(16):
        for i in range(10):
            move_data = {
                "arm":{
                    "left_arm":{
                        "joint":np.random.rand(6) * 3.1515926,
                        "gripper": np.random.rand(1),
                    },
                    "right_arm":{
                        "joint":np.random.rand(6) * 3.1515926,
                        "gripper": np.random.rand(1),
                    }
                }
            }
            robot.move(move_data)
        robot.draw()
        # print(len(robot.policy_tarjectory))
        # print(len(robot.replay_tarjectory))


    