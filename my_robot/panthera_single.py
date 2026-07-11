

import numpy as np

from robot.robot.base_robot import Robot

from robot.controller.Panthera_HT_controller import PantheraHTController
from robot.sensor.Realsense_sensor import RealsenseSensor

from robot.data.collect_any import CollectAny

CAMERA_SERIALS = {
    'head': '338622074268',  # Replace with actual serial number
    'wrist': '420122070816',   # Replace with actual serial number
}

# Define start position (in degrees)
START_POSITION_ANGLE_LEFT_ARM = [
    0.0,   # Joint 1
    0.0,    # Joint 2
    0.0,  # Joint 3
    0.0,   # Joint 4
    0.0,  # Joint 5
    0.0,    # Joint 6
]

# Define start position (in degrees)
START_POSITION_ANGLE_RIGHT_ARM = [
    0.0,   # Joint 1
    0.0,    # Joint 2
    0.0,  # Joint 3
    0.0,   # Joint 4
    0.0,  # Joint 5
    0.0,    # Joint 6
]

condition = {
    "robot":"panthera_single",
    "save_path": "./datasets/", 
    "task_name": "test", 
    "save_format": "hdf5", 
    "save_freq": 30, 
}


class PantheraSingle(Robot):
    def __init__(
        self,
        condition=condition,
        move_check=True,
        start_episode=0,
        enable_cameras=True,
    ):
        super().__init__(condition=condition, move_check=move_check, start_episode=start_episode)

        self.condition = condition
        self.enable_cameras = enable_cameras
        self.controllers = {
            "arm":{
                "left_arm": PantheraHTController("left_arm"),
            },
        }
        self.sensors = {"image": {}}
        if self.enable_cameras:
            self.sensors["image"] = {
                "cam_head": RealsenseSensor("cam_head"),
                "cam_wrist": RealsenseSensor("cam_wrist"),
            }

    # ============== init ==============
    def reset(self):
        self.controllers["arm"]["left_arm"].reset(np.array(START_POSITION_ANGLE_LEFT_ARM))

    def set_up(self):
        super().set_up()

        self.controllers["arm"]["left_arm"].set_up(follower=True)
        if self.enable_cameras:
            self.sensors["image"]["cam_head"].set_up(
                CAMERA_SERIALS["head"]
            )
            self.sensors["image"]["cam_wrist"].set_up(
                CAMERA_SERIALS["wrist"]
            )

        collect_type = {
            "arm": ["joint", "eef", "gripper", "velocity"],
        }
        if self.enable_cameras:
            collect_type["image"] = ["color"]
        self.set_collect_type(collect_type)
        
        print("set up success!")

    def shutdown(self, damped=False):
        for sensor_group in self.sensors.values():
            for sensor in sensor_group.values():
                cleanup = getattr(sensor, "cleanup", None)
                if cleanup is not None:
                    cleanup()

        for controller_group in self.controllers.values():
            for controller in controller_group.values():
                method_name = "brake" if damped else "stop"
                shutdown = getattr(controller, method_name, None)
                if shutdown is not None:
                    shutdown()
    
if __name__=="__main__":
    import time
    robot = PantheraSingle()
    robot.set_up()
    # collection test
    robot.reset()
    data_list = []
    for i in range(100):
        print(i)
        data = robot.get()
        robot.collect(data)
        time.sleep(0.1)
    robot.finish()
    
    # moving test
    move_data = {
        "arm":{
            "left_arm":{
            "joint":[0.0, 0.8, 0.8, 0.3, 0.0, 0.0],
            "gripper":0.2,
            },
        },
    }
    robot.move(move_data)
    time.sleep(5)
