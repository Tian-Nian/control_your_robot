import rclpy
from rclpy.node import Node
from threading import Lock
from typing import Callable, Optional


class ROS2Subscriber(Node):
    def __init__(self, node_name: str, topic_name: str, msg_type, call: Optional[Callable] = None):
        """
        ROS2 Subscriber 封装类
        :param node_name: 节点名称
        :param topic_name: 订阅的话题名
        :param msg_type: 消息类型
        :param call: 可选的回调函数
        """
        super().__init__(node_name)
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.latest_msg = None
        self.lock = Lock()
        self.user_call = call

        self.subscription = self.create_subscription(
            msg_type,
            topic_name,
            self.callback,
            10  # QoS depth
        )

    def callback(self, msg):
        with self.lock:
            self.latest_msg = msg
            if self.user_call:
                self.user_call(msg)

    def get_latest_data(self):
        with self.lock:
            return self.latest_msg

import time
from bunker_msgs.msg import BunkerRCState  # 替换为你使用的消息类型

def custom_callback(msg):
    print(f"Received: SWA={msg.swa}, SWC={msg.swc}")

def main():
    rclpy.init()

    # 创建节点和订阅器对象
    subscriber_node = ROS2Subscriber(
        node_name='rc_state_listener',
        topic_name='/bunker_rc_state',
        msg_type=BunkerRCState,
        call=custom_callback  # 可选
    )

    try:
        while rclpy.ok():
            rclpy.spin_once(subscriber_node, timeout_sec=0.1)
            msg = subscriber_node.get_latest_data()
            if msg:
                print(msg)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

