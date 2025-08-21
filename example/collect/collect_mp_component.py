import sys
sys.path.append("./")
import time

from multiprocessing import Process, Manager, Event, Semaphore, Barrier

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
    "task_name": "test_mp",
    "save_format": "hdf5",
    "save_freq": 60, 
}


def dict2list(data: Dict[str, List]) -> List[Dict]:
    keys = list(data.keys())
    values = list(data.values())

    # 检查是否为空
    if not values:
        return []
    
    for k,v in data.items():
        print(k, " length: ",len(data[k]))

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
    os.environ["INFO_LEVEL"] = "INFO"
    num_episode = 10
    avg_collect_time = 0

    start_episode = 0
    collection = CollectAny(condition, move_check=False, start_episode=start_episode)

    for i in range(num_episode):
        is_start = False

        # 初始化共享操作
        processes = {}
        start_event = Event()
        finish_event = Event()
        manager = Manager()
        data_buffer = manager.dict()

        # time_lock_vision = Event()
        # time_lock_arm = Event()
        worker_barrier = Barrier(5 + 1)

        data_buffer["cam_head"] = manager.list()
        data_buffer["cam_left_wrist"] = manager.list()
        data_buffer["cam_right_wrist"] = manager.list()
        data_buffer["left_arm"] = manager.list()
        data_buffer["right_arm"] = manager.list()

        processes["vision_process_h"] = Process(target=ComponentWorker, args=(RealsenseSensor, "cam_head", ["342622301553"], ["color"], data_buffer, worker_barrier, start_event, finish_event, "vision_worker_head"))
        processes["vision_process_l"] = Process(target=ComponentWorker, args=(RealsenseSensor, "cam_left_wrist", ["242522071124"], ["color"], data_buffer, worker_barrier, start_event, finish_event, "vision_worker_l"))
        processes["vision_process_r"] = Process(target=ComponentWorker, args=(RealsenseSensor, "cam_right_wrist", ["244622071566"], ["color"], data_buffer, worker_barrier, start_event, finish_event, "vision_worker_r"))

        processes["arm_process_l"] = Process(target=ComponentWorker, args=(PiperController, "left_arm", ["can_left"], ["joint", "qpos", "gripper"], data_buffer, worker_barrier, start_event, finish_event, "arm_worker_l"))
        processes["arm_process_r"] = Process(target=ComponentWorker, args=(PiperController, "right_arm", ["can_right"], ["joint", "qpos", "gripper"], data_buffer, worker_barrier, start_event, finish_event, "arm_worker_r"))
        time_scheduler = TimeScheduler(worker_barrier, time_freq=condition["save_freq"]) # 可以给多个进程同时上锁
        
        controller_keys = ["left_arm", "right_arm"]

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
                worker_barrier.abort()  
                is_start = False
                break
        
        # 销毁多进程
        for process in processes.values():
            if process.is_alive():
                process.join()
                process.close()
        
        data = dict(data_buffer)
        # import pdb;pdb.set_trace()
        data = dict2list(data)
        
        time_scheduler.stop()  
        
        for i in range(len(data)):
            controller_Data = {k: v for k,v in data[i].items() if k in controller_keys}
            sensor_data = {k: v for k,v in data[i].items() if k not in controller_keys}
            collection.collect(controller_Data, sensor_data)
        
        collection.write()

        avg_collect_time = time_scheduler.real_time_average_time_interval
        extra_info = {}
        extra_info["avg_time_interval"] = avg_collect_time
        collection.add_extra_condition_info(extra_info)

    print("next step!")