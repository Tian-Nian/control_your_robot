import sys
sys.path.append("./")

from controller.arm_controller import ArmController

# ros
from utils.ros_publisher import ROSPublisher, start_publishing
from utils.ros_subscriber import ROSSubscriber 
from threading import threading

# from ur的数据消息 import 对应消息类型， 如ros基础通讯： from geometry_msgs.msg import Twist

import numpy as np
import time

class UR5eController(ArmController):
    def __init__(self, name):
        super().__init__()
        self.name = name
        self.controller_type = "user_controller"
        self.controller = None
    
    def set_up(self):
        pos_publisher = ROSPublisher("", ) # (要发布的消息的节点, 节点数据类型) 如 ('/tracer_rs_status', TracerRsStatus) , ('/cmd_vel', Twist)
        joint_publisher = ROSPublisher("", ) 
        gripper_publisher = ROSPublisher("", ) 

        subscriber = ROSSubscriber("" ) # 同上, (要订阅的节点, 节点数据类型)

        self.pub_thread = {}
        
        self.pub_thread["joint_pub"] = threading.Thread(target=start_publishing, args=(joint_publisher,))
        self.pub_thread["pos_pub"] = threading.Thread(target=start_publishing, args=(pos_publisher,))
        self.pub_thread["gripper_pub"] = threading.Thread(target=start_publishing, args=(gripper_publisher,))

        # 开启对应线程
        for _, thread in self.pub_thread:
            thread.start()
        
        self.controller = {"joint_publisher":joint_publisher,
                            "pos_publisher":pos_publisher,
                            "gripper_publisher":gripper_publisher, 
                            "subscriber":subscriber}

    def reset(self, start_state):
        # 调用set_position或set_joint就行
        pass

    # 返回单位为米
    def get_state(self):
        state = {}
        
        # 返回的数据类型为绑定的那个数据类型
        msg = self.controller["subscriber"].get_latest_data()
        # 从中提取需要的信息, 需要自己处理下, 角度单位为m, 关节角单位为rads
        state["joint"] = msg.joint 
        state["qpos"] = msg.qpos 
        return state

    # 返回单位为0 ~1的张合度
    def get_gripper(self):
        msg = self.controller["subscriber"].get_latest_data()
         # 只需要返回0~1的张合度
        gripper = msg.gripper
        return gripper

    # 如果有基于从臂的控制, 那么这里的action应该是从臂控制信息
    def get_action(self):
        raise NotImplementedError("get_action is not implemented")

    # 单位为米
    def set_position(self, position):
        # 输入的position是numpy.array, [x,y,z,rx,ry,rz], 转化成自己要的控制数据格式
        pos_msg = None
        self.controller["pos_publisher"].update_msg(pos_msg)

    
    def set_joint(self, joint):
        # 同上操作
        joint_msg = None
        self.controller["joint_publisher"].update_msg(joint_msg)

    def set_gripper(self, joint):
        # 同上操作
        gripper_msg = None
        self.controller["gripper_publisher"].update_msg(gripper_msg)

    def __del__(self):
        try:
            if hasattr(self, 'controller'):
                # Add any necessary cleanup for the arm controller
                pass
        except:
            pass

if __name__=="__main__":
    controller = UR5eController("test_ur5e")
    controller.set_up()
    print(controller.get_state())
    
    controller.set_gripper(0.2)

    controller.set_joint(np.array([0.1,0.1,-0.2,0.3,-0.2,0.5]))
    time.sleep(1)
    print(controller.get_gripper())
    print(controller.get_state())

    controller.set_position(np.array([0.057, 0.0, 0.260, 0.0, 0.085, 0.0]))
    time.sleep(1)
    print(controller.get_gripper())
    print(controller.get_state())