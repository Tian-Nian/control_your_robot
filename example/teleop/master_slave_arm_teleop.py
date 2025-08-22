import sys
sys.path.append("./")

import time
from multiprocessing import Manager, Event

from utils.data_handler import is_enter_pressed
from utils.time_scheduler import TimeScheduler
from utils.worker import Worker
from controller.TestArm_controller import TestArmController
from data.collect_any import CollectAny

from my_robot.test_robot import TestRobot

condition = {
    "save_path": "./save/", 
    "task_name": "test_3", 
    "save_format": "hdf5", 
    "save_freq": 10,
    "collect_type": "teleop",
}


class MasterWorker(Worker):
    def __init__(self, process_name: str, start_event, end_event):
        super().__init__(process_name, start_event, end_event)
        self.manager = Manager()
        self.data_buffer = self.manager.dict()
    
    def handler(self):
        data = self.component.get()
        for key, value in data.items():
            self.data_buffer[key] = value

    def component_init(self):
        self.component = TestArmController("left_teleop_arm")
        self.component.set_up()
        self.component.set_collect_info(["joint", "gripper"])

class SlaveWorker(Worker):
    def __init__(self, process_name: str, start_event, end_event, move_data_buffer: Manager):
        super().__init__(process_name, start_event, end_event)
        self.move_data_buffer = move_data_buffer
        self.manager = Manager()
        self.data_buffer = self.manager.dict()
    
    def handler(self):
        move_data = dict(self.move_data_buffer)
        self.component.move({"arm": 
                                {
                                    "left_arm": move_data
                                }
                            })

        data = self.component.get()

        self.data_buffer["controller"] = self.manager.dict()
        self.data_buffer["sensor"] = self.manager.dict()
        # self.data_buffer["controller"]["master_left_arm"] = self.manager.dict()

        for key, value in data[0].items():
            self.data_buffer["controller"]["slave_"+key] = value
        
        for key, value in data[1].items():
            self.data_buffer["sensor"]["slave_"+key] = value

        # for key, value in move_data.items():
        #     self.data_buffer["controller"]["master_left_arm"][key] = value
    
    def component_init(self):
        self.component = TestRobot()
        self.component.set_up()

class DataWorker(Worker):
    def __init__(self, process_name: str, start_event, end_event, collect_data_buffer: Manager, episode_id=0):
        super().__init__(process_name, start_event, end_event)
        self.collect_data_buffer = collect_data_buffer
        self.episode_id = episode_id
    
    def component_init(self):
        self.collection = CollectAny(condition=condition, start_episode=self.episode_id, move_check=True)
    
    def handler(self):
        data = dict(self.collect_data_buffer)
        self.collection.collect(data["controller"], data["sensor"])
    
    def finish(self):
        self.collection.write()

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

        time_scheduler = TimeScheduler(work_events=[master.forward_event], time_freq=30, end_events=[data.next_event])
        
        master.next_to(slave)
        slave.next_to(data)

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

        time_scheduler.start()
        while is_start:
            time.sleep(0.01)
            if is_enter_pressed():
                end_event.set()  
                time_scheduler.stop()  
                is_start = False

        # 给数据写入一定时间缓冲
        time.sleep(1)

        master.stop()
        slave.stop()
        data.stop()