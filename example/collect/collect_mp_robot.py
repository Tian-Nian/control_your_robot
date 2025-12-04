import sys
sys.path.append("src/")
import time
import select

from multiprocessing import Process, Event, Barrier

from my_robot.test_robot import TestRobot, condition

from utils.time_scheduler import TimeScheduler
from utils.robot_worker import RobotWorker
from utils.data_handler import is_enter_pressed

from data.collect_any import CollectAny


if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "DEBUG" # DEBUG , INFO, ERROR

    start_episode = 0
    num_episode = 10
    avg_collect_time = 0

    for episode_id in range(start_episode, start_episode + num_episode):
        is_start = False
        
        # 重置进程
        # time_lock = Event()
        time_lock= Barrier(1+1)
        start_event = Event()
        finish_event = Event()
        robot_process = Process(target=RobotWorker, args=(TestRobot, episode_id, time_lock, start_event, finish_event, "robot_worker"))
        time_scheduler = TimeScheduler(work_barrier=time_lock, time_freq=10) # 可以给多个进程同时上锁
        
        robot_process.start()
        while not is_start:
            time.sleep(0.01)
            if is_enter_pressed():
                is_start = True
                start_event.set()
            else:
                time.sleep(1)
    
        time_scheduler.start()
        while is_start:
            time.sleep(0.001)
            if is_enter_pressed():
                finish_event.set() 
                time_scheduler.stop()  
                is_start = False
        
        # 销毁多进程
        if robot_process.is_alive():
            robot_process.join()
            robot_process.close()
        
    
        # 仅用于添加额外信息
        collection = CollectAny(condition=condition,start_episode=0)
        avg_collect_time = time_scheduler.real_time_average_time_interval
        extra_info = {}
        extra_info["avg_time_interval"] = avg_collect_time
        collection.add_extra_condition_info(extra_info)