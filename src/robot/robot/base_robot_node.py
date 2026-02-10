from robot.robot.base_robot import Robot

from robot.utils.base.data_handler import debug_print
from robot.utils.node.node import TaskNode
from robot.utils.node.scheduler import Scheduler

from threading import Lock, Event
import time

ROBOT_MAP = {
    "sensor": {
        "image": 30,
    },
    "controller": {
        "arm": 200,
    }
}

class DataBuffer:
    def __init__(self):
        self._buffer = {}
        self.show_buffer = {}
        self.lock = Lock()

    def update(self, name, data):
        self.show_buffer[name] = data
    
    def get_lastest(self):
        return self.show_buffer

class ComponentNode(TaskNode):
    def task_init(self, component, data_buffer: DataBuffer):
        self.component = component
        self.data_buffer = data_buffer
    
    def task_step(self):
        data = self.component.get()

        self.data_buffer.update(self.component.name, data)

class CollectNode(TaskNode):
    def task_init(self, controller_buffers: list[DataBuffer], sensor_buffers: list[DataBuffer], start_event: Event):
        self.controller_buffers = controller_buffers
        self.sensor_buffers = sensor_buffers
        self.controller_episode = []
        self.sensor_episode = []
        self.start_event = start_event
    
    def task_step(self):
        if self.start_event.is_set():
            controller_obs = {}
            
            for data_buffer in self.controller_buffers:
                data_dict = data_buffer.get_lastest()
                for k,v in data_dict.items():
                    controller_obs[k] = v
            self.controller_episode.append(controller_obs)

            sensor_obs = {}
            for data_buffer in self.sensor_buffers:
                data_dict = data_buffer.get_lastest()
                for k,v in data_dict.items():
                    sensor_obs[k] = v
            self.sensor_episode.append(sensor_obs)

    def _cleanup(self):
        self.sensor_episode = []
        self.controller_episode = []
    
    def get(self):
        return self.controller_episode.copy(), self.sensor_episode.copy()
        

def init(robot: Robot):
    start_event = Event()

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
    
    data_buffers = []
    data_buffers.extend(sensor_data_buffers)
    data_buffers.extend(controller_data_buffers)

    return sensor_data_buffers, sensor_nodes, controller_data_buffers, controller_nodes, start_event

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

def build_robot_node(base_robot_cls):
    class RobotNode(base_robot_cls):
        def __init__(self,
                condition = None,
                move_check=True, 
                start_episode=0):
            super().__init__(condition=condition, move_check=move_check, start_episode=start_episode)
            self.name = self.name + "_node"
        
        def set_up(self, **kargs):
            super().set_up(**kargs)
            
            (
                self.sensor_data_buffers,
                self.sensor_nodes,
                self.controller_data_buffers,
                self.controller_nodes,
                self.start_event,
            ) = init(self)

            self.sensor_schedulers, self.controller_schedulers = build_map(
                self.sensor_nodes,
                self.controller_nodes,
            )

            for s in self.sensor_schedulers.values():
                s.start()
            for c in self.controller_schedulers.values():
                c.start()

        def get(self):
            controller_data = {}

            for buf in self.controller_data_buffers.values():
                for k, v in buf.get_lastest().items():
                    controller_data[k] = v

            sensor_data = {}
            for buf in self.sensor_data_buffers.values():
                for k, v in buf.get_lastest().items():
                    sensor_data[k] = v

            return controller_data.copy(), sensor_data.copy()

        def start(self):
            if self.start_event.is_set():
                self.reset()
            
            controller_buffers = []
            for v in self.controller_data_buffers.values():
                controller_buffers.append(v)
            
            sensor_buffers = []
            for v in self.sensor_data_buffers.values():
                sensor_buffers.append(v)
            
            self.collect_node = CollectNode("COLLECT_NODE", controller_buffers=controller_buffers, sensor_buffers=sensor_buffers, start_event=self.start_event)
            self.collect_node.start()

            self.collect_scheduler = Scheduler(entry_nodes=[self.collect_node], 
                                                all_nodes=[self.collect_node],
                                                final_nodes=[self.collect_node],
                                                hz=self.collect_cfg["save_freq"])
            time.sleep(1)
            self.collect_scheduler.start()
            self.start_event.set()

            debug_print("collect_node", "Collect data start!", "INFO")

        def finish(self, episode_id=None):
            if self.start_event.is_set():
                self.start_event.clear()

                controller_episode, sensor_episode = self.collect_node.get()
                assert len(controller_episode) == len(sensor_episode)

                for controller_obs, sensor_obs in zip(controller_episode, sensor_episode):
                    self.collect((controller_obs, sensor_obs))

            super().finish(episode_id=episode_id)

        def reset(self):
            if hasattr(self, "collect_node"):
                self.collect_node._cleanup()
            
            super().reset()

    return RobotNode