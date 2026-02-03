from robot.robot.base_robot import Robot
from robot.utils.base.data_handler import is_enter_pressed, debug_print, dict_to_list
from robot.utils.node.node import TaskNode
from robot.utils.node.scheduler import Scheduler

from threading import Lock
import time

ROBOT_MAP = {
    "sensor": {
        "image": 30,
    },
    "controller": {
        "arm": 120,
    }
}

class DataBuffer:
    def __init__(self):
        self._buffer = {}
        self.lock = Lock()
    
    def update(self, name, data):
        with self.lock:
            if name in self._buffer.keys():
                for k,v in data.items():
                    self._buffer[name][k].append(v)
            else:
                self._buffer[name] = data
                for k,v in self._buffer[name].items():
                    self._buffer[name][k] = [v]
    
    def get(self):
        with self.lock:
            # import pdb;pdb.set_trace()
            try:
                ret = dict_to_list(self._buffer)
            except:
                print("ERROR OCCURED!")
                import pdb;pdb.set_trace()
                exit()
            return ret
    
    def clear(self):
        with self.lock:
            self._buffer = {}

class ComponentNode(TaskNode):
    def task_init(self, component, data_buffer: DataBuffer):
        self.component = component
        self.data_buffer = data_buffer
    
    def task_step(self):
        data = self.component.get()
        self.data_buffer.update(self.component.name, data)

def init(robot: Robot):
    sensor_data_buffers = {}
    sensor_nodes = {}
    for sensor_type in ROBOT_MAP["sensor"].keys():
        sensor_nodes[sensor_type] = []
        sensor_data_buffers[sensor_type] = DataBuffer()
        for sensor_name, sensor in robot.sensors[sensor_type].items():
            sensor_node = ComponentNode(sensor_name, component=sensor, data_buffer=sensor_data_buffers[sensor_type])
            sensor_node.start()
            sensor_nodes[sensor_type].append(sensor_node)

    controller_data_buffers = {}
    controller_nodes = {}
    for controller_type in ROBOT_MAP["controller"].keys():
        controller_nodes[controller_type] = []
        controller_data_buffers[controller_type] = DataBuffer()
        for controller_name, controller in robot.controllers[controller_type].items():
            controller_node = ComponentNode(controller_name, component=controller, data_buffer=controller_data_buffers[controller_type])
            controller_node.start()
            controller_nodes[controller_type].append(controller_node)
    
    return sensor_data_buffers, sensor_nodes, controller_data_buffers, controller_nodes

def build_map(sensor_nodes, controller_nodes):
    sensor_schedulers = {}
    for sensor_type in ROBOT_MAP["sensor"].keys():
        sensor_schedulers[sensor_type] = Scheduler(entry_nodes=sensor_nodes[sensor_type], 
                                                    all_nodes=sensor_nodes[sensor_type],
                                                    final_nodes=sensor_nodes[sensor_type],
                                                    hz=ROBOT_MAP["sensor"][sensor_type])
    controller_schedulers = {}
    for controller_type in ROBOT_MAP["controller"].keys():
        controller_schedulers[controller_type] = Scheduler(entry_nodes=controller_nodes[controller_type], 
                                                    all_nodes=controller_nodes[controller_type],
                                                    final_nodes=controller_nodes[controller_type],
                                                    hz=ROBOT_MAP["controller"][controller_type])
    
    return sensor_schedulers, controller_schedulers

condition = {
    "save_path": "./save/test_dt/",
    "task_name": "new",
    "save_format": "hdf5",
    "save_freq": 10, 
}

if __name__ == "__main__":
    from my_robot.xspark_robot import XsparkRobot
    robot = XsparkRobot(move_check=False, condition=condition)
    robot.set_up(teleop=True)
    episode_num = 10

    for _ in range(episode_num):
        sensor_data_buffers, sensor_nodes, controller_data_buffers, controller_nodes = init(robot)

        sensor_schedulers, controller_schedulers = build_map(sensor_nodes, controller_nodes)

        debug_print("collect_node", "Waiting for ENTER to start...", "INFO")

        while not is_enter_pressed():
            time.sleep(0.1)
        debug_print("collect_node", "Collect start! Press ENTER to finish!", "INFO")

        for sensor_scheduler in sensor_schedulers.values():
            sensor_scheduler.start()
        
        for controller_scheduler in controller_schedulers.values():
            controller_scheduler.start()
        
        while not is_enter_pressed():
            time.sleep(0.1)  

        for sensor_scheduler in sensor_schedulers.values():
            sensor_scheduler.stop()
        
        for controller_scheduler in controller_schedulers.values():
            controller_scheduler.stop()      

        for key, sensor_data_buffer in sensor_data_buffers.items():
            print(key)
            datas = sensor_data_buffer.get()
            for data in datas:
                d = [None, data]
                # import pdb;pdb.set_trace()
                robot.collect(d)

        for controller_data_buffer in controller_data_buffers.values():
            datas = controller_data_buffer.get()
            for data in datas:
                d = [data, None]
                robot.collect(d,)
        
        robot.finish()
        


