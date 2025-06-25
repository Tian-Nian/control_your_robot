import sys
sys.path.append("./")
import time
import select

from multiprocessing import Array, Process, Lock, Event, Semaphore

from utils.time_scheduler import TimeScheduler
from utils.robot_worker import RobotWorker

from my_robot.test_robot import TestRobot

from utils.data_handler import is_enter_pressed

if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "DEBUG" # DEBUG , INFO, ERROR

    num_episode = 10

    for i in range(num_episode):
        is_start = False
        
        # 重置进程
        time_lock = Semaphore(0)
        start_event = Event()
        finish_event = Event()
        start_episode = i
        robot_process = Process(target=RobotWorker, args=(TestRobot, start_episode, time_lock, start_event, finish_event, "robot_worker"))
        time_scheduler = TimeScheduler([time_lock], time_interval=10) # 可以给多个进程同时上锁
        
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
            time.sleep(0.01)
            if is_enter_pressed():
                finish_event.set()  
                time_scheduler.stop()  
                is_start = False
        
        # 销毁多进程
        if robot_process.is_alive():
            robot_process.join()
            robot_process.close()