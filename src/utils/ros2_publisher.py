import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from threading import Thread, Event
import time


class ROS2Publisher(Node):
    def __init__(self, topic_name, msg_type, continuous=True):
        """
        初始化 ROS2 发布者
        :param topic_name: 发布的 topic 名称
        :param msg_type: 消息类型
        :param continuous: 是否持续发布（默认每 10ms 发布一次）
        """
        super().__init__('ros2_publisher_node')
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.continuous = continuous
        self.publisher = self.create_publisher(msg_type, topic_name, 10)
        self.pub_msg = None
        self.shutdown_event = Event()

    def publish_once(self):
        if self.pub_msg:
            self.publisher.publish(self.pub_msg)

    def continuous_publish(self, interval_sec=0.01):
        while not self.shutdown_event.is_set():
            if self.pub_msg:
                self.publisher.publish(self.pub_msg)
            time.sleep(interval_sec)

    def update_msg(self, msg):
        self.pub_msg = msg

    def stop(self):
        self.shutdown_event.set()
        self.get_logger().info("Publisher stopped.")

def main():
    rclpy.init()
    pub_node = ROS2Publisher('/cmd_vel', Twist, continuous=True)

    # 初始化消息
    msg = Twist()
    msg.linear.x = 0.02
    pub_node.update_msg(msg)

    # 启动线程持续发布
    pub_thread = Thread(target=pub_node.continuous_publish)
    pub_thread.start()

    try:
        rclpy.spin_once(pub_node, timeout_sec=0.1)
        time.sleep(1)

        msg.linear.x = 0.0
        pub_node.update_msg(msg)
        time.sleep(1)

        msg.linear.x = -0.02
        pub_node.update_msg(msg)
        time.sleep(1)

        msg.linear.x = 0.0
        pub_node.update_msg(msg)
        time.sleep(1)

    except KeyboardInterrupt:
        pass
    finally:
        pub_node.stop()
        pub_thread.join()
        pub_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
