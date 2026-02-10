

from robot.controller.arm_controller import ArmController
from robot.utils.base.data_handler import debug_print
from franky import *
import numpy as np
import math
from scipy.spatial.transform import Rotation

'''
Franka base code from:
https://github.com/TimSchneider42/franky
'''

class FrankaFrankyController(ArmController):
    def __init__(self, name):
        super().__init__()
        self.name = name
        self.controller_type = "user_controller"
        self.controller = None
    
    def set_up(self, ip):
        self.controller = Robot(ip)

        # Recover from errors
        self.controller.recover_from_errors()

        # Set velocity, acceleration, and jerk to 5% of the maximum
        self.controller.relative_dynamics_factor = 0.05

        # Alternatively, you can define each constraint individually
        self.controller.relative_dynamics_factor = RelativeDynamicsFactor(
            velocity=0.1, acceleration=0.05, jerk=0.1
        )

        # Or, for more fine-grained access, set individual limits
        self.controller.translation_velocity_limit.set(3.0)
        self.controller.rotation_velocity_limit.set(2.5)
        self.controller.elbow_velocity_limit.set(2.62)
        self.controller.translation_acceleration_limit.set(9.0)
        self.controller.rotation_acceleration_limit.set(17.0)
        self.controller.elbow_acceleration_limit.set(10.0)
        self.controller.translation_jerk_limit.set(4500.0)
        self.controller.rotation_jerk_limit.set(8500.0)
        self.controller.elbow_jerk_limit.set(5000.0)
        self.controller.joint_velocity_limit.set([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])
        self.controller.joint_acceleration_limit.set([10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0])
        self.controller.joint_jerk_limit.set([5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0])

    def get_state(self):
        state = {}

        # Get the self.controller's cartesian state
        cartesian_state = self.controller.current_cartesian_state
        robot_pose = cartesian_state.pose  # Contains end-effector pose and elbow position

        ee_pose = robot_pose.end_effector_pose
        
        # Get the self.controller's joint state
        joint_state = self.controller.current_joint_state

        state["qpos"] = ee_pose
        state["joint"] = joint_state

        return state

    def set_position(self, position):
        position = position.tolist()
        if len(position) == 6:
           quat = Rotation.from_euler("xyz", position[3:]).as_quat()
           debug_print(self.name, f"Set position using EULER!", "INFO")
        elif len(position) == 7:
            quat = np.array(position[3:])
            debug_print(self.name, f"Set position using QUATERNION!", "INFO")
        else:
           debug_print(self.name, f"Invalid position length!", "ERROR")

        xyz = position[:3]
        m_cp = CartesianMotion(Affine(xyz, quat))

        try:
            self.controller.move(m_cp)
        except Exception as e:
            debug_print(self.name, f"{e}", "ERROR")
    def set_joint(self, joint):
        m_jp = JointMotion(joint.tolist())
        try:
            self.controller.move(m_jp)
        except Exception as e:
            debug_print(self.name, f"{e}", "ERROR")
    def set_gripper(self, gripper):
        raise NotImplementedError

if __name__ == "__main__":
    import time

    ip = "192.168.1.2"
    franka = FrankaFrankyController("franka")
    franka.set_up(ip)

    state = franka.get_state()
    print(state)

    # 测试位置控制 / 关节控制
    print("set position!")
    pos = [0.4, -0.2, 0.3, 0, 0, math.pi / 2]
    franka.set_position(pos)

    time.sleep(5)

    print("set joint!")
    joint = [-0.3, 0.1, 0.3, -1.4, 0.1, 1.8, 0.7]
    franka.set_joint(joint)

    time.sleep(5)