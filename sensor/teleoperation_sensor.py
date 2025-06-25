import sys
sys.path.append("./")

import numpy as np

from sensor.sensor import Sensor
from typing import Dict, Any

class TeleoperationSensor(Sensor):
    def __init__(self):
        super().__init__()
        self.name = "teleoperation_sensor"
        self.sensor = None
    
    def get_information(self):
        sensor_info = {}
        state = self.get_state()
        if "end_pose" in self.collect_info:
            sensor_info["end_pose"] = state["end_pose"]
        if "velocity" in self.collect_info:
            sensor_info["velocity"] = state["velocity"]
        if "gripper" in self.collect_info:
            sensor_info["gripper"] = state["gripper"]
        if "extra" in self.collect_info:
            sensor_info["extra"] = state["extra"]
        return sensor_info
