from __future__ import annotations

import sys
sys.path.append("./")

import time

from threading import Event, Thread
from utils.data_handler import debug_print

class Worker:
    def __init__(self, process_name: str, start_event, end_event):
        self.process_name = process_name
        self.start_event = start_event
        self.end_event = end_event

        self.forward_event = Event()
        self.next_event = Event()

    def next_to(self, next_worker: Worker):
        self.next_event = next_worker.forward_event

    def start(self):
        self.process = Thread(target=self._worker, args=[])
        self.process.start()

    def component_init(self):
        raise NotImplementedError("you should realize this function")

    def handler(self):
        raise NotImplementedError("you should realize this function")
    
    def finish(self):
        return
    
    # 用于强制结束, 正常使用不需调用此函数
    def stop(self):
        # self.process.terminate()
        # self.process.join()
        # self.process.close()
        return

    def _worker(self):
        '''
        如果你的组件是Robot类, 那么对于component_name,component_setup_input与component_collect_info都可以直接设置为None, 因为机器人的初始化是默认的, 无输入参数.
        set_collect_info是只有controller与sensor拥有的函数.
        '''
        self.component_init()
        
        debug_print(self.process_name ,"Press Enter to start...","INFO")
        last_time = time.monotonic()
        while not self.start_event.is_set():
            now = time.monotonic()
            if now - last_time > 5:  
                debug_print(self.process_name ,"Press Enter to start...","INFO")
                last_time = now
            else:
                time.sleep(0.001)
            
        debug_print(self.process_name, "Get start Event, start collecting...","INFO")
        debug_print(self.process_name, "To finish this episode, please press Enter. ","INFO")
        # try:
        while not self.end_event.is_set():
            self.forward_event.wait()  
            start = time.monotonic()

            if self.end_event.is_set():
                break  # Prevent exiting immediately after acquire before processing data

            debug_print(self.process_name, "Time lock acquired. Processing data...", "DEBUG")

            # try:
            self.handler()  
            # except Exception as e:
            #     debug_print(self.process_name, f"Error: {e}", "ERROR")
            
            self.forward_event.clear()
            if self.next_event is None:
                debug_print(self.process_name, "if this worker is not the end, you should use next_to() to build up a chain of process.", "ERROR")
                break
            else:
                self.next_event.set()                
            debug_print(self.process_name, "Data processed. Waiting for next time slot.", "DEBUG")
            end = time.monotonic()
            debug_print(self.process_name, f"running time: {end - start}s", "DEBUG")

        debug_print(self.process_name, "Finish event triggered. Finalizing...","INFO")
        self.finish()

        self.next_event.set()
        debug_print(self.process_name, "Worker exiting.", "INFO")

class TestComponent_1:
    def __init__(self) -> None:
        pass

    def get(self):
        return 1

class TestComponent_2:
    def __init__(self) -> None:
        pass

    def get(self):
        return 2

class TestWorker_1(Worker):
    def __init__(self, process_name: str, start_event, end_event):
        super().__init__(process_name, start_event, end_event)
    
    def handler(self):
        print(self.component.get())
        time.sleep(0.05)

    def component_init(self):
        return TestComponent_1()

    def finish(self):
        debug_print("TestComponent_1","finish!", "INFO")

class TestWorker_2(Worker):
    def __init__(self, process_name: str, start_event, end_event):
        super().__init__(process_name, start_event, end_event)
    
    def handler(self):
        print(self.component.get())
        time.sleep(0.05)

    def component_init(self):
        return TestComponent_2()

    def finish(self):
        debug_print("TestComponent_2","finish!", "INFO")

if __name__ == "__main__":
    from utils.time_scheduler import TimeScheduler

    lock, start_event, end_event = Event(), Event(), Event()
    worker_1 = TestWorker_1("test_1", start_event, end_event)
    worker_2 = TestWorker_2("test_2", start_event, end_event)
    worker_1.next_to(worker_2)
    time_scheduler = TimeScheduler([worker_1.forward_event], time_freq=10, end_Events=[worker_2.next_event])

    start_event.set()
    worker_1.start()
    worker_2.start()
    time_scheduler.start()

    time.sleep(1)
    end_event.set()

    time_scheduler.stop()
