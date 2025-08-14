import sys
sys.path.append("./")

from rm_msgs.msg import GetArmState_Command, Arm_Current_State, MoveJ, CartePos 
from geometry_msgs.msg import Pose, Point, Quaternion

from controller.arm_controller import ArmController
from utils.ros_publisher import ROSPublisher, start_publishing
from utils.ros_subscriber import ROSSubscriber
from utils.data_handler import debug_print

import threading
import rospy
from scipy.spatial.transform import Rotation as R
import numpy as np



class RealmanRosController(ArmController):
    def __init__(self, name):
        super().__init__()
        self.name = name
        self.controller_type = "user_controller"
        self.controller = None

    def set_up(self, arm_name):
        subscriber = ROSSubscriber(f"/{arm_name}/rm_driver/Arm_Current_State", Arm_Current_State)
        self.pub_thread = {}

        # 初始化发布获取状态消息的节点
        state_publisher = ROSPublisher(f"/{arm_name}/rm_driver/GetArmState_Cmd", GetArmState_Command, continuous=True)
        state_msg = GetArmState_Command()
        state_msg.command = ''
        state_publisher.update_msg(state_msg)
        self.pub_thread["state"] = threading.Thread(target=start_publishing, args=(state_publisher,))
        self.pub_thread["state"].start()

        # 初始化发布关节角的节点
        joint_publisher = ROSPublisher(f"/{arm_name}/rm_driver/MoveJ_Cmd", MoveJ, continuous=False)
        self.pub_thread["joint"] = threading.Thread(target=start_publishing, args=(joint_publisher,))
        self.pub_thread["joint"].start()

        # 初始化发布末端位姿的节点
        eef_publisher = ROSPublisher(f"/{arm_name}/rm_driver/MoveP_Fd_Cmd", CartePos , continuous=False)
        self.pub_thread["eef"] = threading.Thread(target=start_publishing, args=(eef_publisher,))
        self.pub_thread["eef"].start()

        try:
            from dh_gripper_msgs.msg import GripperCtrl 
            if "right" in self.name:
                gripper_publisher = ROSPublisher(f"/dh_right/gripper/ctrl", GripperCtrl , continuous=False)
            elif "left" in self.name:
                gripper_publisher = ROSPublisher(f"/dh_left/gripper/ctrl", GripperCtrl , continuous=False)
            else:
                KeyError("if you want to use gripper, left / right should be keyword in name")
            self.pub_thread["gripper"] = threading.Thread(target=start_publishing, args=(gripper_publisher,))
            self.pub_thread["gripper"].start()
        except:
            gripper_publisher = None
            debug_print(self.name, "unable initializing gripper!", "WARNING")

        self.controller = {
            "subscriber": subscriber,
            "state_publisher": state_publisher,
            "joint_publisher": joint_publisher,
            "eef_publisher": eef_publisher,
            "gripper_publisher": gripper_publisher,
        }

    def get_state(self):
        state_msg = self.controller["subscriber"].get_latest_data()
        state = {}
        state["joint"] = state_msg.joint if state_msg is not None else None
        state["qpos"] = state_msg.Pose if state_msg is not None else None
        return state
        
    def set_joint(self, joint):
        if joint is None:
            return
        debug_print(self.name, joint, "INFO")
        joint_msg = MoveJ()
        joint_msg.joint = joint
        joint_msg.speed = 0.2
        self.controller["joint_publisher"].update_msg(joint_msg)
    
    def set_position(self, position):
        if position is None:
            return
        if position.shape[0] == 6:
            r = R.from_euler('xyz', [position[-3], position[-2], position[-1]], degrees=False)
            quat = np.array(r.as_quat())
            position = np.concatenate([position[:3], quat])
        pos_msg = CartePos ()

        pose = Pose()
        pose.position = Point(x=position[0], y=position[1], z=position[2])
        pose.orientation = Quaternion(x=position[3], y=position[4], z=position[5], w=position[6])

        pos_msg.Pose = pose
        # pos_msg.speed = 0.1 

        self.controller["eef_publisher"].update_msg(pos_msg)

    def set_gripper(self, gripper):
        if self.controller["gripper_publisher"] is None:
            debug_print(self.name, "Initializing gripper failed!", "ERROR")
            return
        from dh_gripper_msgs.msg import GripperCtrl 
        gripper = int(gripper*1000)
        gripper_msg = GripperCtrl()
        gripper_msg.initialize = False
        gripper_msg.position = gripper
        gripper_msg.force = 50.0
        gripper_msg.speed = 10.0
        self.controller["gripper_publisher"].update_msg(gripper_msg)

if __name__=="__main__":
    import time

    rospy.init_node("rm_controller_node", anonymous=True)

    rm_right = RealmanRosController("right_arm")
    rm_right.set_up("rm_right")
    
    # 数据缓冲
    time.sleep(1)
    print("get state!")
    for i in range(10):
        print(f"{i}:",rm_right.get_state())
        time.sleep(0.1)

    print("eef")
    r = R.from_euler('xyz', [3.134000062942505, 1.5230000019073486, -3.075000047683716], degrees=False)
    quat = r.as_quat()
    rm_right.set_position(np.array([0.5109999775886536, -0.3499999940395355, 0.2709999978542328, quat[0], quat[1], quat[2], quat[3]]))
    for i in range(10):
        x = 0.5
        x+= 0.001
        rm_right.set_position(np.array([x, -0.3499999940395355, 0.2709999978542328, 3.134000062942505, 1.5230000019073486, -3.075000047683716]))
        time.sleep(0.1)
    print("joint")
    rm_right.set_joint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(1)
    print("gripper")
    rm_right.set_gripper(0.1)
    time.sleep(1)
