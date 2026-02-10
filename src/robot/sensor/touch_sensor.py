

from robot.sensor.sensor import Sensor

class TouchSensor(Sensor):
    def __init__(self):
        super().__init__()
        self.name = "touch_sensor"
        self.type = "touch_sensor"
        self.collect_info = None

    def get_information(self):
        touch_info = {}
        touch = self.get_touch()
        if "force" in self.collect_info:
            touch_info["force"] = touch["force"]
        if "torque" in self.collect_info:
            touch_info["torque"] = touch["torque"]
        
        return touch_info

    
    

