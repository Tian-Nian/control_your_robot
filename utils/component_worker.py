import sys
sys.path.append("./")
from controller import *
import time
from typing import *

from sensor import *
from controller import *

from multiprocessing import Event, Semaphore, Process, Value, Manager

from utils.data_handler import debug_print, DataBuffer
import rospy

import importlib

def ComponentWorker(component_class_name, component_name, component_setup_input, component_collect_info, data_buffer: DataBuffer,
                time_lock: Event, start_event: Event, finish_event: Event, process_name: str):
    '''
    组件级别的多进程同步器, 用于多进程数据采集, 如果希望是多进程的同步控制也可以稍微改下代码添加一个共享的信号输入
    输入:
    component_class: 你的组件类, 可以自定义的是controller / sensor, class
    component_name: 你希望组件的名称, 用于对应组件info的输出, str
    component_setup_input: 组件初始化需要设置的信息, List[Any]
    component_collect_info: 组件采集的数据种类, List[str]
    data_buffer: 初始化一个同步所有组件的内存空间, DataBuffer
    time_lock: 初始化对于当前组件的时间同步锁, 该锁需要分配给time_scheduler用于控制时间, multiprocessing::Semaphore
    start_event: 同步开始事件, 所有的组件共用一个, multiprocessing::Event
    finish_event: 同步结束事件, 所有的组件共用一个, multiprocessing::Event
    process_name:你希望当前进程叫什么, 用于对应子进程info的输出, str
    '''
    # rospy.init_node(f'{component_class_name}', anonymous=True)
    # moudle = importlib.import_module(component_class_name)
    # Cls = getattr(moudle, component_class_name)
    component = component_class_name(component_name)
    
    if not component_setup_input is None:
        component.set_up(*component_setup_input)
    else:
        component.set_up()
    
    component.set_collect_info(component_collect_info)
    
    debug_print(process_name ,"Press Enter to start...","INFO")
    last_time = time.monotonic()
    while not start_event.is_set():
        now = time.monotonic()
        if now - last_time > 5:  
            debug_print(process_name ,"Press Enter to start...","INFO")
            last_time = now
        
    debug_print(process_name, "Get start Event, start collecting...","INFO")
    debug_print(process_name, "To finish this episode, please press Enter. ","INFO")
    try:
        while not finish_event.is_set():
            time_lock.wait()  
            if finish_event.is_set():
                break  # Prevent exiting immediately after acquire before processing data

            debug_print(process_name, "Time lock acquired. Processing data...", "DEBUG")

            try:
                data = component.get()
                data_buffer.collect(component.name, data)
            except Exception as e:
                debug_print(process_name, f"Error: {e}", "ERROR")
            
            time_lock.clear()
            debug_print(process_name, "Data processed. Waiting for next time slot.", "DEBUG")

        debug_print(process_name, "Finish event triggered. Finalizing...","INFO")
        
    except KeyboardInterrupt:
        debug_print(process_name, "Worker terminated by user.", "WARNING")
    finally:
        debug_print(process_name, "Worker exiting.", "INFO")
    
if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "DEBUG"

    from sensor.TestVision_sensor import TestVisonSensor
    from controller.TestArm_controller import TestArmController
    from utils.time_scheduler import TimeScheduler
    from utils.data_handler import is_enter_pressed

    # 初始化共享操作
    processes = []
    start_event = Event()
    finish_event = Event()
    manager = Manager()
    data_buffer = DataBuffer(manager)

    time_lock_vision = Semaphore(0)
    time_lock_arm = Semaphore(0)
    vision_process = Process(target=ComponentWorker, args=(TestVisonSensor, "test_vision", None, ["color"], data_buffer, time_lock_vision, start_event, finish_event, "vision_worker"))
    arm_process = Process(target=ComponentWorker, args=(TestArmController, "test_arm", None, ["joint", "qpos", "gripper"], data_buffer, time_lock_arm, start_event, finish_event, "arm_worker"))
    time_scheduler = TimeScheduler([time_lock_vision, time_lock_arm], time_freq=100) # 可以给多个进程同时上锁
    
    processes.append(vision_process)
    processes.append(arm_process)

    for process in processes:
        process.start()

    is_start = False

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
            time_scheduler.stop()  
            is_start = False
    
    # 销毁多进程
    for process in processes:
        if process.is_alive():
            process.join()
            process.close()
    