import rospy
import threading
from typing import Callable, Optional

class ROSSubscriber:
    def __init__(self, topic_name, msg_type,call: Optional[Callable] = None):
        """
        Initialize ROS subscriber
        :param topic_name: Name of the topic to subscribe to
        :param msg_type: Type of the message
        """
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.latest_msg = None
        self.lock = threading.Lock()
        self.user_call = call
        
        self.subscriber = rospy.Subscriber(self.topic_name, self.msg_type, self.callback)
    
    def callback(self, msg):
        """
        Subscriber callback function to receive messages and update the latest data.
        :param msg: The received message
        """
        with self.lock:
            self.latest_msg = msg
            if self.user_call:
                self.user_call(self.latest_msg)

    def get_latest_data(self):
        with self.lock:
            return self.latest_msg

        
if __name__=="__main__":
    import time
    '''
    示例:
    from tracer_msgs.msg import TracerRsStatus

    ros_test = ROSSubscriber('/tracer_rs_status', TracerRsStatus)
    # 初始化 ROS 节点
    rospy.init_node('ros_subscriber_node', anonymous=True)
    for i in range(100):
        print(ros_test.get_latest_data())
        time.sleep(0.1)
    
    示例:
    from geometry_msgs.msg import PoseStamped
    ros_test = ROSSubscriber('/pika_pose_l', PoseStamped)
    rospy.init_node('ros_subscriber_node', anonymous=True)
    
    for i in range(100):
        print(ros_test.get_latest_data())
        time.sleep(0.1)
    '''
