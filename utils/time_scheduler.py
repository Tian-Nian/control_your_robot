import time
from threading import Thread, Lock, Event, Barrier
from typing import List, Optional
import numpy as np

DEBUG = False
Release = True

from utils.data_handler import debug_print

TIME_SLEEP = 0.00001


class TimeScheduler:
    '''
    时间控制器，用于同步不同线程之间的信号量
    work_events: 每个子线程的控制事件，列表
    work_barrier: 可选，所有线程共享Barrier同步
    time_freq: 触发频率
    end_events / end_barrier: 完成同步事件或Barrier
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
        self.real_time_accumulate_time_interval = 0.0
        self.step = 0
        self.lock = Lock()

        # 控制
        self.stop_event = Event()
        self._thread: Optional[Thread] = None

    def time_worker(self):
        last_time = time.monotonic()
        while not self.stop_event.is_set():
            now = time.monotonic()
            if now - last_time >= 1 / self.time_freq:
                # 触发条件获取
                if self.work_barrier is None and self.work_events is not None:
                    # 事件触发模式
                    while any(event.is_set() for event in self.work_events):
                        time.sleep(TIME_SLEEP)
                elif self.work_barrier is not None:
                    try:
                        self.work_barrier.wait()
                    except Exception as e:
                        debug_print(self.process_name, f"{e}", "WARNING")
                        return

                # 统计间隔
                interval = now - last_time
                with self.lock:
                    self.real_time_accumulate_time_interval += interval
                    self.step += 1

                # if interval > 2 / self.time_freq:
                #     debug_print(self.process_name,
                #                 f"The current lock release time has exceeded twice the intended interval. Actual: {interval:.4f}s",
                #                 "WARNING")

                last_time = now

                # 释放触发事件
                if self.work_events:
                    for event in self.work_events:
                        event.set()
                if self.end_events:
                    for event in self.end_events:
                        event.clear()
                if self.end_barrier:
                    try:
                        self.end_barrier.wait()
                    except Exception as e:
                        debug_print(self.process_name, f"{e}", "WARNING")

            else:
                time.sleep(TIME_SLEEP)

    def start(self):
        '''开启时间同步线程'''
        self.stop_event.clear()
        self._thread = Thread(target=self.time_worker, name=self.process_name)
        self._thread.start()
        # 启动初始事件
        if self.work_barrier:
            self.work_barrier.wait()
        else:
            for event in self.work_events:
                event.set()

    def stop(self):
        '''停止时间同步线程'''
        self.stop_event.set()
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        # 计算平均间隔
        with self.lock:
            if self.step > 0:
                self.real_time_average_time_interval = self.real_time_accumulate_time_interval / self.step
            else:
                self.real_time_average_time_interval = 0.0
        debug_print(self.process_name,
                    f"average real time collect interval: {self.real_time_average_time_interval:.6f}s",
                    "INFO")