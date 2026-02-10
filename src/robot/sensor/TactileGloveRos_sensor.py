

from robot.sensor.touch_sensor import TouchSensor
from robot.utils.ros_subscriber import ROSSubscriber 
from robot.utils.base.data_handler import is_enter_pressed
from robot.utils.tactile_hand import draw
import rospy
import numpy as np
import time

from std_msgs.msg import UInt8MultiArray

class TactileGloveRosSensor(TouchSensor):
    def __init__(self, name):
        super().__init__()
        self.name = name
        self.controller_type = "user_controller"
        self.controller = None

    def set_up(self, topic_name):
        subscriber = ROSSubscriber(
            topic_name=topic_name,
            msg_type=UInt8MultiArray,
        )

        self.controller = { "subscriber":subscriber}
    
    def get_touch(self):
        msg = self.controller["subscriber"].get_latest_data()
        # print(msg)
        if msg is not None:
            msg = np.frombuffer(msg.data, dtype=np.uint8)
        return{"force":msg}

if __name__ == "__main__":
    rospy.init_node("test_node", anonymous=True)

    touch_sensor = TactileGloveRosSensor("left_hand")
    touch_sensor.set_up("left_hand")
    touch_sensor.set_collect_info(["force"])

    while True:
        data = touch_sensor.get()["force"]
        if data is not None:
            draw("left", data)
            # break
        if is_enter_pressed():
            break
        time.sleep(0.01)