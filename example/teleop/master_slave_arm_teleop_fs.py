import sys
sys.path.append("./")

import time
from multiprocessing import Manager, Event

from utils.data_handler import is_enter_pressed
from utils.time_scheduler import TimeScheduler
from utils.worker import Worker
from controller.TestArm_controller import TestArmController
from sensor.TestVision_sensor import TestVisonSensor
from data.collect_any import CollectAny

from my_robot.test_robot import TestRobot

condition = {
    "save_path": "./save/", 
    "task_name": "test_3", 
    "save_format": "hdf5", 
    "save_freq": 30,
    "collect_type": "teleop",
}


class MasterWorker(Worker):
    def __init__(self, process_name: str, start_event, end_event):
        super().__init__(process_name, start_event, end_event)
        self.manager = Manager()
        self.data_buffer = self.manager.dict()
    
    def handler(self):
        data_left = self.component_left.get()
        for key, value in data_left.items():
            self.data_buffer["left_"+key] = value
        
        data_right = self.component_right.get()
        for key, value in data_right.items():
            self.data_buffer["right_"+key] = value
        
    def component_init(self):
        self.component_left = TestArmController("left_master_arm")
        self.component_right = TestArmController("right_master_arm")

        self.component_left.set_up()
        self.component_right.set_up()

        self.component_left.set_collect_info(["joint", "gripper"])
        self.component_right.set_collect_info(["joint", "gripper"])

class SlaveWorker(Worker):
    def __init__(self, process_name: str, start_event, end_event, move_data_buffer: Manager):
        super().__init__(process_name, start_event, end_event)
        self.move_data_buffer = move_data_buffer
        self.manager = Manager()
        self.data_buffer = self.manager.dict()
    
    def handler(self):
        move_data = dict(self.move_data_buffer)

        left_move_data = {
            "joint": move_data["left_joint"],
            "gripper": move_data["left_gripper"],
        }
        right_move_data = {
            "joint": move_data["right_joint"],
            "gripper": move_data["right_gripper"],
        }
        
        self.component_left.move(left_move_data)
        self.component_right.move(right_move_data)

        data_left = self.component_left.get()
        data_right = self.component_left.get()

        data = {
            "left_arm": data_left,
            "right_arm": data_right,
        }

        for key, value in data.items():
            self.data_buffer["controller"]["slave_"+key] = value

    def component_init(self):
        self.component_left = TestArmController("left_slave_arm")
        self.component_right = TestArmController("right_slave_arm")

        self.component_left.set_up()
        self.component_right.set_up()

        self.component_left.set_collect_info(["joint", "qpos", "gripper"])
        self.component_right.set_collect_info(["joint", "qpos", "gripper"])

        self.data_buffer["controller"] = self.manager.dict()

class DataWorker(Worker):
    def __init__(self, process_name: str, start_event, end_event, collect_data_buffer: Manager, episode_id=0):
        super().__init__(process_name, start_event, end_event)
        self.collect_data_buffer = collect_data_buffer
        self.episode_id = episode_id
    
    def handler(self):
        data = dict(self.collect_data_buffer)
        if data == {}:
            return
        
        data["sensor"] = {}

        data["sensor"]["cam_head"] = self.cam_head.get()
        data["sensor"]["cam_left_wrist"] = self.cam_left_wrist.get()
        data["sensor"]["cam_right_wrist"] = self.cam_right_wrist.get()

        self.collection.collect(data["controller"], data["sensor"])
    
    def finish(self):
        self.collection.write()

    def component_init(self):
        self.cam_head = TestVisonSensor("cam_head")
        self.cam_left_wrist = TestVisonSensor("cam_left_wrist")
        self.cam_right_wrist = TestVisonSensor("cam_right_wrist")

        self.cam_head.set_up()
        self.cam_left_wrist.set_up()
        self.cam_right_wrist.set_up()

        self.cam_head.set_collect_info(["color"])
        self.cam_left_wrist.set_collect_info(["color"])
        self.cam_right_wrist.set_collect_info(["color"])

        self.collection = CollectAny(condition=condition, start_episode=self.episode_id, move_check=True)

if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "INFO"
    num_episode = 3
    avg_collect_time = 0

    for i in range(num_episode):
        is_start = False

        start_event, end_event = Event(), Event()

        master = MasterWorker("master_arm", start_event, end_event)
        slave = SlaveWorker("slave_arm", start_event, end_event, master.data_buffer)
        data = DataWorker("collect_data", start_event, end_event, slave.data_buffer, episode_id=i)

        time_scheduler_control = TimeScheduler(work_events=[master.forward_event], time_freq=300, end_events=[slave.next_event], process_name="time_scheduler_control")
        time_scheduler_collect = TimeScheduler(work_events=[data.forward_event], time_freq=30, end_events=[data.next_event], process_name="time_scheduler_collect")
        
        master.next_to(slave)

        master.start()
        slave.start()
        data.start()

        while not is_start:
            time.sleep(0.01)
            if is_enter_pressed():
                is_start = True
                start_event.set()
            else:
                time.sleep(1)

        time_scheduler_control.start()
        time_scheduler_collect.start()
        while is_start:
            time.sleep(0.01)
            if is_enter_pressed():
                end_event.set()  
                time_scheduler_control.stop()  
                time_scheduler_collect.stop()  
                is_start = False

        # 给数据写入一定时间缓冲
        time.sleep(1)

        master.stop()
        slave.stop()
        data.stop()