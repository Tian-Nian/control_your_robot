from controller.mobile_controller import MobileController

import rospy

# ros
from utils.ros_publisher import ROSPublisher, start_publishing
from utils.ros_subscriber import ROSSubscriber 

from tracer_msgs.msg import TracerRsStatus
from geometry_msgs.msg import Twist

import threading

'''
Tracer base code(ROS) from:
https://github.com/agilexrobotics/tracer_ros.git
'''

class TracerController(MobileController):
    def __init__(self, name):
        super().__init__()
        self.name = name
        self.controller_type = "user_controller"
        self.controller = None

    def set_up(self):
        subscriber = ROSSubscriber('/tracer_rs_status', TracerRsStatus)
        publisher = ROSPublisher('/cmd_vel', Twist)
        msg = Twist()
        publisher.update_msg(msg)
        self.pub_thread = threading.Thread(target=start_publishing, args=(publisher,))
        self.pub_thread.start()
        self.controller = {"publisher":publisher, 
                           "subscriber":subscriber}

    def set_move_velocity(self, vel):
        vel_msg = Twist()
        vel_msg.linear.x = vel[0]
        vel_msg.linear.y = vel[1]
        vel_msg.linear.z = vel[2]
        vel_msg.angular.x = vel[3]
        vel_msg.angular.y = vel[4]
        vel_msg.angular.z = vel[5]
        self.controller["publisher"].update_msg(vel_msg)
    
    # 
    def set_move_to(self, to):
        raise NotImplementedError ("set_move_to is not implemented")

    def get_subscriber(self):
        data = {}
        data["move_velocity"] = self.controller["subscriber"].get_latest_data()
        return data

    def stop(self):
        self.controller["publisher"].stop()

if __name__=="__main__":
    import time
    rospy.init_node("tracer_controller_node", anonymous=True)
    tracer = TracerController("tracer")
    tracer.set_up()
    tracer.set_move_velocity([-0.1,0,0,0,0,0]) 
    time.sleep(0.5)
    tracer.set_move_velocity([0.0,0,0,0,0,0.5]) 
    time.sleep(0.5)
    tracer.set_move_velocity([0.1,0,0,0,0,0]) 
    time.sleep(0.5)
    tracer.set_move_velocity([0,0,0,0,0,0]) 
    tracer.get_subscriber()
    tracer.stop()