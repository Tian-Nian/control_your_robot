import rospy
from geometry_msgs.msg import Twist
import threading

class ROSPublisher:
    def __init__(self, topic_name, msg_type, continuous=True):
        """
        Initialize ROS publisher
        :param topic_name: Name of the topic to publish
        :param msg_type: Type of the message
        """
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.publisher = None
        self.pub_msg = None
        self.shutdown_flag = False
        self.continuous = continuous

        self.publisher = rospy.Publisher(self.topic_name, self.msg_type, queue_size=10)

    def publish(self, event=None):
        if self.pub_msg is None:
            # if self.continuous:
            #     rospy.logwarn("No message to publish.")
            return
        if self.shutdown_flag:
            return
        else:
            self.publisher.publish(self.pub_msg)
            if not self.continuous:
                self.pub_msg = None

    def continuous_publish(self):
        rospy.Timer(rospy.Duration(0.01), self.publish)

    def update_msg(self, msg):
        self.pub_msg = msg
    
    def stop(self):
        self.shutdown_flag = True
        rospy.loginfo("Publisher stopped.")

def start_publishing(publisher):
    publisher.continuous_publish()

    rospy.on_shutdown(publisher.stop)

if __name__ == "__main__":
    try:
        publisher = ROSPublisher('/cmd_vel', Twist)
        # init ros node
        rospy.init_node('ros_publisher_node', anonymous=True)
        
        # init msg
        msg = Twist()
        msg.linear.x = 0.1 
        publisher.update_msg(msg)

        # start a publish thread
        pub_thread = threading.Thread(target=start_publishing, args=(publisher,))
        pub_thread.start()

        rospy.sleep(1)
        msg.linear.x = 0.0
        publisher.update_msg(msg)

        rospy.sleep(1)
        msg.linear.x = 0.1
        publisher.update_msg(msg)

        rospy.sleep(1)
        msg.linear.x = 0.0
        publisher.update_msg(msg)

        rospy.sleep(1) 
        publisher.stop()
        rospy.loginfo("Shutting down ROS publisher.")

        pub_thread.join()

    except rospy.ROSInterruptException:
        pass
