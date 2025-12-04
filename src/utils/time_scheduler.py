import time
import multiprocessing as mp
from multiprocessing import Array, Process, Lock, Value, Event, Barrier
from typing import List, Optional
import numpy as np

DEBUG = False
Release = True

from utils.data_handler import debug_print

TIME_SLEEP = 0.00001

def worker(process_id: int, process_name: str, time_event: Event, result_array: Array, result_lock: Lock):
    '''
    测试使用的子类
    '''
    while True:
        # Block until the scheduler issues an "execution token"
        time_event.wait()  
        debug_print(process_name, "received time slot", "DEBUG")

        # Simulate task processing + data writing (with locking)
        result = np.random.randn(5,5).flatten()
        with result_lock:
            result_array[process_id*25:(process_id+1)*25] = result
        time.sleep(0.1) 

        time_event.clear()

class TimeScheduler:
    '''
    时间控制器, 用于同步不同进程之间的信号量
    work_barrier: 每个子进程的控制都需要有一个信号量控制循环操作, 这里就是将所有进程的信号量进行控制, List[Event]
    time_freq: 采集数据的频率, 实际频率可能会稍微低于该频率, 会保存最终采集平均时间间隔在数据采集的config里, int
    '''
    def __init__(
        self,
        work_events: Optional[List[Event]] = None,
        work_barrier: Optional[Barrier] = None,
        time_freq: int = 10,
        end_events: Optional[List[Event]] = None,
        end_barrier: Optional[Barrier] = None,
        process_name: str = None
    ):

        self.time_freq = int(time_freq)
        if self.time_freq <= 0:
            raise ValueError("time_freq must be positive.")

        # 工作触发两种模式：二选一
        self.work_events = work_events
        self.work_barrier = work_barrier

        # 完成同步两种模式：可都不给（则不等待）
        self.end_events = end_events
        self.end_barrier = end_barrier

        # 校验“二选一”
        if (self.work_events is None) == (self.work_barrier is None):
            raise ValueError("Exactly one of work_events or work_barrier must be provided.")

        if self.work_events is not None and len(self.work_events) == 0:
            raise ValueError("work_events must be a non-empty list when provided.")

        if self.end_events is not None and self.end_barrier is not None:
            raise ValueError("end_events and end_barrier cannot both be set; choose one.")

        # 统计信息
        self.process_name = process_name
        self.real_time_accumulate_time_interval = Value('d', 0.0)
        self.step = Value('i', 0)

        # 控制
        self.stop_event = Event()
        self._proc: Optional[Process] = None

    def time_worker(self):
        last_time = time.monotonic()
        while True:
            now = time.monotonic()
            if now - last_time >= 1 / self.time_freq:
                # 触发条件获取
                if  self.end_barrier is None and self.end_events is None:
                    # 并行自触发
                    if self.work_barrier:
                        try:
                            self.work_barrier.wait()
                        except Exception as e:
                            debug_print(self.process_name, f"{e}", "WARNING")
                            return
                    else:
                        while True:
                            if all(not event.is_set() for event in self.work_events):
                                break
                            else:
                                time.sleep(TIME_SLEEP)
                else:
                    # 链式结构触发
                    if self.end_barrier:
                        self.end_barrier.wait()
                    else:
                        while True:
                            if all(event.is_set() for event in self.end_events):
                                break
                            else:
                                time.sleep(TIME_SLEEP)
                    
                debug_print(self.process_name, f"the actual time interval is {now - last_time}", "DEBUG")
                with self.real_time_accumulate_time_interval.get_lock():
                    self.real_time_accumulate_time_interval.value = self.real_time_accumulate_time_interval.value + (now - last_time)
                with self.step.get_lock():
                    self.step.value += 1

                if now - last_time > 2 / self.time_freq:
                        debug_print(self.process_name, "The current lock release time has exceeded twice the intended time interval.\n Please check whether the corresponding component's get() function is taking too long.", "WARNING")
                        debug_print(self.process_name, f"the actual time interval is {now - last_time}", "WARNING")
                last_time = now

                # 释放触发条件
                if self.work_events:
                    for event in self.work_events:
                        event.set()
                
                if self.end_events:
                    for event in self.end_events:
                        event.clear()

    def start(self):
        '''
        开启时间同步器进程
        '''
        self.time_locker = Process(target=self.time_worker)
        self.time_locker.start()
        if self.work_barrier:
            self.work_barrier.wait()
        else:
            for event in self.work_events:
                event.set()
        

    def stop(self):
        '''
        释放该时间同步器进程
        '''
        # for event in self.work_barrier:
        #         event.set()  # 防止卡在获取锁
        # self.time_locker.terminate()
        # self.time_locker.join()
        # self.time_locker.close()
        self.time_locker = None
        with self.real_time_accumulate_time_interval.get_lock():
            self.real_time_average_time_interval = self.real_time_accumulate_time_interval.value / self.step.value
        debug_print(self.process_name, f"average real time collect interval is: {self.real_time_accumulate_time_interval.value / self.step.value}", "INFO")

if __name__ == "__main__":
    processes = []
    process_num = 2
    result_array = Array('d', process_num * 25)
    time_event = [Event() for _ in range(process_num)]
    result_lock = Lock()

    # start workers
    for i in range(process_num):
        process = Process(target=worker, args=(i, f"process_{i}", time_event[i], result_array, result_lock))
        process.start()
        processes.append(process)

    # start time scheduler
    time_scheduler = TimeScheduler(time_event, time_freq=10)
    time_scheduler.start()

    # (optional)
    try:
        time.sleep(5)
        print("Sample result snapshot:", result_array[:])
    except KeyboardInterrupt:
        print("Main process interrupted.")
    finally:
        time_scheduler.stop()
        for p in processes:
            p.terminate()
            p.join()
