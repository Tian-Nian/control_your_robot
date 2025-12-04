from sensor.sensor import Sensor

class VisionSensor(Sensor):
    def __init__(self):
        super().__init__()
        self.name = "vision_sensor"
        self.type = "vision_sensor"
        self.collect_info = None

    def get_information(self):
        image_info = {}
        image = self.get_image()
        if "color" in self.collect_info:
            image_info["color"] = image["color"]
        if "depth" in self.collect_info:
            image_info["depth"] = image["depth"]
        if "point_cloud" in self.collect_info:
            image_info["point_cloud"] = image["point_cloud"]
        
        return image_info

    
    

