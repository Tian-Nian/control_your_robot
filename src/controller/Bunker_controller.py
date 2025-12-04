from controller.mobile_controller import MobileController

import rclpy
from rclpy.node import Node

from utils.ros2_publisher import ROS2Publisher
from utils.ros2_subscriber import ROS2Subscriber 

from bunker_msgs.msg import BunkerRCState
from geometry_msgs.msg import Twist

import threading


class BunkerController(MobileController):
    def __init__(self, name):
        super().__init__()
        self.name = name
        self.controller_type = "user_controller"
        self.controller = None

    def set_up(self):
        subscriber = ROS2Subscriber(
            node_name='rc_state_listener',
            topic_name='/bunker_rc_state',
            msg_type=BunkerRCState,
        )
        publisher = ROS2Publisher('/cmd_vel', Twist, continuous=True)

        msg = Twist()
        publisher.update_msg(msg)
        self.pub_thread = threading.Thread(target=publisher.continuous_publish, args=(0.01,))
        self.pub_thread.start()
        self.controller = {"publisher":publisher, 
                           "subscriber":subscriber}

    def set_move_velocity(self, vel):
        rclpy.spin_once(self.controller["publisher"], timeout_sec=0.1)

        vel_msg = Twist()

        vel_msg.linear.x = vel[0]
        vel_msg.linear.y = vel[1]
        vel_msg.linear.z = vel[2]
        vel_msg.angular.x = vel[3]
        vel_msg.angular.y = vel[4]
        vel_msg.angular.z = vel[5]
    
        self.controller["publisher"].update_msg(vel_msg)
    
    def set_move_to(self, to):
        raise NotImplementedError ("set_move_to is not implemented")

    def get_subscriber(self):
        rclpy.spin_once(self.controller["subscriber"], timeout_sec=0.1)
        data = {}

        data["move_velocity"] = self.controller["subscriber"].get_latest_data()
    
    def stop(self):
        self.controller["publisher"].stop()

if __name__=="__main__":
    import time
    rclpy.init()

    bunker = BunkerController("bunker_mini")
    bunker.set_up()
    bunker.set_collect_info(["move_velocity"])
    bunker.move({"move_velocity": [0.05 ,0.,0.,0.,0.,0.]}) 
    time.sleep(0.5)
    bunker.move({"move_velocity": [0., 0., 0., 0., 0., 0.5]}) 
    time.sleep(0.5)
    bunker.move({"move_velocity": [0., 0., 0., 0.,0.,-0.5]}) 
    time.sleep(0.5)
    bunker.move({"move_velocity": [-0.05, 0., 0., 0., 0., 0]}) 
    time.sleep(0.5)
    print(bunker.get_subscriber())
    bunker.stop()