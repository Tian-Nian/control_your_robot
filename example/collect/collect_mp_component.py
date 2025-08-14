import sys
sys.path.append("./")
import time

from multiprocessing import Process, Manager, Event, Semaphore

from data.collect_any import CollectAny

# from sensor.TestVision_sensor import TestVisonSensor
# from controller.TestArm_controller import TestArmController

from sensor.VisionROS_sensor import VisionROSensor
from sensor.Realsense_sensor import RealsenseSensor

from controller.Piper_controller import PiperController

from utils.time_scheduler import TimeScheduler
from utils.component_worker import ComponentWorker
from utils.data_handler import is_enter_pressed, DataBuffer

from typing import Dict, List

import multiprocessing as mp

condition = {
    "save_path": "./save/",
    "task_name": "test1",
    "save_format": "hdf5",
    "save_freq": 30, 
}


def dict2list(data: Dict[str, List]) -> List[Dict]:
    keys = list(data.keys())
    values = list(data.values())

    # 检查是否为空
    if not values:
        return []

    # 检查所有列表长度是否相等
    length = len(values[0])
    assert all(len(v) == length for v in values), "All lists must be the same length"

    # 转换
    result = []
    for i in range(length):
        item = {k: v[i] for k, v in zip(keys, values)}
        result.append(item)
    return result

if __name__ == "__main__":
    # import rospy

    # mp.set_start_method("spawn")

    # rospy.init_node('ros_subscriber_node', anonymous=True)

    import os
    os.environ["INFO_LEVEL"] = "DEBUG"
    num_episode = 3
    avg_collect_time = 0

    start_episode = 0
    collection = CollectAny(condition, start_episode=start_episode)

    for i in range(num_episode):
        is_start = False

        # 初始化共享操作
        processes = {}
        start_event = Event()
        finish_event = Event()
        manager = Manager()
        data_buffer = DataBuffer(manager)

        time_lock_vision = Event()
        time_lock_arm = Event()
        processes["vision_process_h"] = Process(target=ComponentWorker, args=(RealsenseSensor, "cam_head", ["342622301553"], ["color"], data_buffer, time_lock_vision, start_event, finish_event, "vision_worker_head"))
        processes["vision_process_l"] = Process(target=ComponentWorker, args=(RealsenseSensor, "cam_left_wrist", ["242522071124"], ["color"], data_buffer, time_lock_vision, start_event, finish_event, "vision_worker_l"))
        processes["vision_process_r"] = Process(target=ComponentWorker, args=(RealsenseSensor, "cam_right_wrist", ["244622071566"], ["color"], data_buffer, time_lock_vision, start_event, finish_event, "vision_worker_r"))

        processes["arm_process_l"] = Process(target=ComponentWorker, args=(PiperController, "left_arm", ["can_left"], ["joint", "qpos", "gripper"], data_buffer, time_lock_arm, start_event, finish_event, "arm_worker_l"))
        processes["arm_process_r"] = Process(target=ComponentWorker, args=(PiperController, "right_arm", ["can_right"], ["joint", "qpos", "gripper"], data_buffer, time_lock_arm, start_event, finish_event, "arm_worker_r"))
        time_scheduler = TimeScheduler([time_lock_vision, time_lock_arm], time_freq=condition["save_freq"]) # 可以给多个进程同时上锁
        
        # processes.append(vision_process)
        # processes.append(arm_process)

        for process in processes.values():
            process.start()

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
                finish_event.set()  
                is_start = False
                break
        
        print("11111next step!")
        

        # 销毁多进程
        for process in processes.values():
            if process.is_alive():
                process.join()
                process.close()
        
        data = data_buffer.get()
        # import pdb;pdb.set_trace()
        data = dict2list(data)
        
        time_scheduler.stop()  
        
        avg_collect_time += time_scheduler.real_time_average_time_interval
        for i in range(len(data)):
            collection.collect(data[i], None)
        collection.write()
    
    avg_collect_time /= num_episode
    extra_info = {}
    extra_info["avg_time_interval"] = avg_collect_time
    collection.add_extra_condition_info(extra_info)

    print("next step!")