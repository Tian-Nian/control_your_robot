from controller import *
import time
from typing import *

from multiprocessing import Event, Manager, Barrier

from utils.data_handler import debug_print, DataBuffer

def RobotWorker(robot_class, start_episode,
                time_lock: Barrier, start_event: Event, finish_event: Event, process_name: str, 
                data_buffer: DataBuffer = None, move_data: Manager = None):
    '''
    对于实现的机器人类进行多进程数据采集, 可以对多个机器人进行.
    输入:
    robot_class: 机器人类, my_robot::robot_class
    start_episode: 数据采集的开始序号, 只影响保存数据的后缀组号, int
    time_lock: 初始化对于当前组件的时间同步锁, 该锁需要分配给time_scheduler用于控制时间, multiprocessing::Event
    start_event: 同步开始事件, 所有的组件共用一个, multiprocessing::Event
    finish_event: 同步结束事件, 所有的组件共用一个, multiprocessing::Event
    process_name:你希望当前进程叫什么, 用于对应子进程info的输出, str
    '''
    robot = robot_class(start_episode=start_episode)
    robot.set_up()
    
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
            try:
                time_lock.wait()  
            except Exception as e:
                debug_print(process_name, f"This warining cause of Baririer.abort()", "WARNING")
            if finish_event.is_set():
                break  # Prevent exiting immediately after acquire before processing data

            debug_print(process_name, "Time lock acquired. Processing data...", "DEBUG")

            try:
                data = robot.get()
                robot.collect(data)

                if data_buffer is not None:
                    data_buffer.collect(robot.name, {**data[0], **data[1]})

                if move_data is not None:
                    robot.move(move_data)
            except Exception as e:
                debug_print(process_name, f"Error: {e}", "ERROR")

            debug_print(process_name, "Data processed. Waiting for next time slot.", "DEBUG")

        debug_print(process_name, "Finish event triggered. Finalizing...","INFO")
        robot.finish()
        debug_print(process_name, "Writing success!","DEBUG")
        
    except KeyboardInterrupt:
        debug_print(process_name, "Worker terminated by user.", "WARNING")
    finally:
        debug_print(process_name, "Worker exiting.", "INFO")
