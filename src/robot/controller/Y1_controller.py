

from robot.controller.arm_controller import ArmController
from robot.utils.base.data_handler import debug_print

from y1_sdk import Y1SDKInterface, ControlMode

'''
Piper base code from:
https://github.com/agilexrobotics/piper_sdk.git
'''

package_path = "/home/tian/project/y1_sdk_python/y1_ros/src/y1_controller/"

class Y1Controller(ArmController):
    def __init__(self, name):
        super().__init__()
        self.name = name
        self.controller_type = "user_controller"
        self.controller = None
    
    def set_up(self, can:str, arm_end_type=3, teleop=False):
        if arm_end_type == 0:
            urdf_path = f"{package_path}/urdf/y10804.urdf"
        elif arm_end_type == 1:
            urdf_path = f"{package_path}/urdf/y1_gripper_t.urdf"
        elif arm_end_type == 2:
            urdf_path = f"{package_path}/urdf/y1_gripper_g.urdf"
        elif arm_end_type == 3:
            urdf_path = f"{package_path}/urdf/y10824_ee.urdf"

        self.controller = Y1SDKInterface(
            can_id=can,
            urdf_path=urdf_path,
            arm_end_type=arm_end_type,
            enable_arm=True,
        )

        if not self.controller.Init():
            debug_print(self.name, "Init Fail!", "ERROR")
            # exit()
        if teleop:
            self.controller.SetArmControlMode(ControlMode.GRAVITY_COMPENSATION)
        else:
            self.controller.SetArmControlMode(ControlMode.NRT_JOINT_POSITION)

    # 返回单位为米
    def get_state(self):
        state = {}
        
        eef = self.controller.GetArmEndPose()
        joint = self.controller.GetJointPosition()
        vel = self.controller.GetJointVelocity()

        state["eef"] = eef
        state["joint"] = joint[:6]
        state["gripper"] = joint[6] / 84

        return state

    # All returned values are expressed in meters,if the value represents an angle, it is returned in radians
    def set_position(self, position):
        self.controller.SetArmEndPose(list(position))
    
    def set_joint(self, joint):
        self.controller.SetArmJointPosition(list(joint), 2)

    # The input gripper value is in the range [0, 1], representing the degree of opening.
    def set_gripper(self, gripper):
        gripper = gripper * 84
        self.controller.SetGripperStroke(gripper)
        print("set!!!!")

    def __del__(self):
        try:
            if hasattr(self, 'controller'):
                # Add any necessary cleanup for the arm controller
                pass
        except:
            pass