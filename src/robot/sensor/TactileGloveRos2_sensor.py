

from robot.sensor.touch_sensor import TouchSensor
from robot.utils.ros.ros2_subscriber import ROS2Subscriber 
from robot.utils.base.data_handler import is_enter_pressed

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8MultiArray
import serial
import threading
import time

class TactileGloveRosSensor(TouchSensor):
    def __init__(self, name):
        super().__init__()
        self.name = name
        self.controller_type = "user_controller"
        self.controller = None

    def set_up(self, topic_name):
        subscriber = ROS2Subscriber(
            node_name='"hand_tactile_publisher"',
            topic_name=topic_name,
            msg_type=UInt8MultiArray,
        )

        self.controller = { "subscriber":subscriber}
    
    def get_touch(self):
        msg = self.controller["subscriber"].get_latest_data()
        print(msg)
        return{"force":msg.data}


if __name__ == "__main__":
    touch_sensor = TactileGloveRosSensor("left_hand")
    touch_sensor.set_up("left_hand")

    touch_sensor.set_collect_info(["force"])

    while True:
        print(touch_sensor.get())
        if is_enter_pressed():
            break
        time.sleep(0.01)