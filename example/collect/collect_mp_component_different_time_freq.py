import sys
sys.path.append("src/")
import time

from multiprocessing import Process, Manager, Event, Semaphore, Barrier

from data.collect_any import CollectAny

from utils.time_scheduler import TimeScheduler
from utils.component_worker import ComponentWorker
from utils.data_handler import is_enter_pressed

from typing import Dict, List

condition = {
    "save_path": "./save/",
    "task_name": "test",
    "save_format": "hdf5",
    "save_freq": 10, 
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
    import multiprocessing as mp
    mp.set_start_method("spawn")

    import os
    os.environ["INFO_LEVEL"] = "DEBUG"
    num_episode = 3
    avg_collect_time = 0

    start_episode = 0
    collection = CollectAny(condition, start_episode=start_episode, move_check=False)

    manager = Manager()
    shared_data_buffer = manager.dict()
    
    for i in range(num_episode):
        is_start = False

        # 初始化共享操作
        processes = []
        start_event = Event()
        finish_event = Event()

        # time_lock_vision = Event()
        # time_lock_arm = Event()
        # 数量为组件进程数+时间控制器数(默认1个时间控制器)
        worker_barrier_vision = Barrier(1 + 1)
        worker_barrier_arm = Barrier(1 + 1)

        vision_process = Process(target=ComponentWorker, args=("sensor.TestVision_sensor", "TestVisonSensor", "test_vision", None, ["color"], shared_data_buffer, worker_barrier_vision, start_event, finish_event, "vision_worker"))
        arm_process = Process(target=ComponentWorker, args=("controller.TestArm_controller", "TestArmController", "test_arm", None, ["joint", "qpos", "gripper"], shared_data_buffer, worker_barrier_arm, start_event, finish_event, "arm_worker"))
        
        time_scheduler_vision = TimeScheduler(work_barrier=worker_barrier_vision, time_freq=30) # 可以给多个进程同时上锁
        time_scheduler_arm = TimeScheduler(work_barrier=worker_barrier_arm, time_freq=300) # 可以给多个进程同时上锁
        
        processes.append(vision_process)
        processes.append(arm_process)

        # 由于是Dict[list]形式, 需要提前在主进程申请空间
        shared_data_buffer["test_vision"] = manager.list()
        shared_data_buffer["test_arm"] = manager.list()

        for process in processes:
            process.start()

        while not is_start:
            time.sleep(0.01)
            if is_enter_pressed():
                is_start = True
                start_event.set()
            else:
                time.sleep(1)

        time_scheduler_vision.start()
        time_scheduler_arm.start()

        while is_start:
            time.sleep(0.01)
            if is_enter_pressed():
                finish_event.set()  
                time_scheduler_vision.stop()  
                time_scheduler_arm.stop()  

                is_start = False
        
        # 销毁多进程
        for process in processes:
            if process.is_alive():
                process.join()
                process.close()
        
        data = shared_data_buffer.copy()

        shared_data_buffer = manager.dict()
        # import pdb;pdb.set_trace()

        for arm_data in data["test_arm"]:
            collection.collect({"test_arm": arm_data}, None)
        
        for vision_data in data["test_vision"]:
            collection.collect(None, {"test_vision": vision_data})
        
        collection.write()

        # add extra info
        extra_info = {
            "avg_time_interval_vision": time_scheduler_vision.real_time_average_time_interval,
            "avg_time_interval_arm": time_scheduler_arm.real_time_average_time_interval,
        }
        collection.add_extra_condition_info(extra_info=extra_info)
