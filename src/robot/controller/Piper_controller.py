

from robot.controller.arm_controller import ArmController
from robot.controller.mit_command import pack_joint_mit

from piper_sdk import *
import numpy as np
import time

_DOF = 6

'''
Piper base code from:
https://github.com/agilexrobotics/piper_sdk.git
'''

def effort_from_high_speed(message) -> np.ndarray:
    """高速反馈的 effort 单位是 0.001 N·m，这里换成 N·m。"""
    values = []
    for index in range(1, _DOF + 1):
        motor = getattr(message, f"motor_{index}", None)
        effort = getattr(motor, "effort", None)
        if effort is None:
            raise ValueError(f"高速反馈缺少 motor_{index}.effort")
        values.append(float(effort) * 0.001)
    return np.asarray(values, dtype=float)


class PiperController(ArmController):
    def __init__(self, name):
        super().__init__()
        self.name = name
        self.controller_type = "user_controller"
        self.controller = None
        self.DoFs = _DOF
        self._last_result = {}
    
    def set_up(self, can:str):
        piper = C_PiperInterface_V2(can)
        piper.ConnectPort()
        piper.EnableArm(7)
        enable_fun(piper=piper)
        self.controller = piper

    def apply_reset(self, start_state):
        if start_state is None:
            start_state = np.zeros(6)
        try:
            self.set_joint(np.asarray(start_state, dtype=float))
        except Exception:
            print("reset error")

    # 返回单位为米
    def get_state(self):
        state = {}
        eef = self.controller.GetArmEndPoseMsgs()
        joint = self.controller.GetArmJointMsgs()
        
        state["joint"] = np.array([joint.joint_state.joint_1, joint.joint_state.joint_2, joint.joint_state.joint_3,\
                                   joint.joint_state.joint_4, joint.joint_state.joint_5, joint.joint_state.joint_6]) * 0.001 / 180 * 3.1415926
        state["eef"] = np.array([eef.end_pose.X_axis, eef.end_pose.Y_axis, eef.end_pose.Z_axis, \
                                  eef.end_pose.RX_axis, eef.end_pose.RY_axis, eef.end_pose.RZ_axis]) * 0.001 / 1000
        state["gripper"] = self.controller.GetArmGripperMsgs().gripper_state.grippers_angle * 0.001 / 70
        try:
            state["force"] = effort_from_high_speed(self.controller.GetArmHighSpdInfoMsgs())
        except (AttributeError, TypeError, ValueError):
            pass
        return state

    # All returned values are expressed in meters,if the value represents an angle, it is returned in radians
    def set_position(self, position):
        x, y, z, rx, ry, rz = position*1000*1000
        x, y, z, rx, ry, rz = int(x), int(y), int(z), int(rx), int(ry), int(rz)

        self.controller.MotionCtrl_2(0x01, 0x00, 100, 0x00)
        self.controller.EndPoseCtrl(x, y, z, rx, ry, rz)
    
    def set_joint(self, joint):
        j1, j2, j3 ,j4, j5, j6 = joint * 57295.7795 #1000*180/3.1415926
        j1, j2, j3 ,j4, j5, j6 = int(j1), int(j2), int(j3), int(j4), int(j5), int(j6)
        
        self.controller.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        self.controller.JointCtrl(j1, j2, j3, j4, j5, j6)

    # The input gripper value is in the range [0, 1], representing the degree of opening.
    def set_gripper(self, gripper):
        gripper = int(gripper * 70 * 1000)
        self.controller.GripperCtrl(gripper, 1000, 0x01, 0)

    def set_mit(self, q_des, kp, kd, qd_des=None, tau_ff=None):
        """piper_sdk.JointMitCtrl。先切到 MIT：MotionCtrl_2 的 move_mode=0x04，is_mit_mode=0xAD。

        不传 tau_ff 时前馈是 0。这个控制器没有 URDF，重力要调用方自己放进 tau_ff。
        驱动侧 τ = kp (q_d - q) + kd (qd_d - qd) + t_ref。
        """
        self._require_arm()
        parts = pack_joint_mit(q_des, kp, kd, qd_des, tau_ff, _DOF)
        self.controller.MotionCtrl_2(0x01, 0x04, 100, 0xAD)
        for index in range(_DOF):
            self.controller.JointMitCtrl(
                index + 1,
                float(parts["q_des"][index]),
                float(parts["qd_des"][index]),
                float(parts["kp"][index]),
                float(parts["kd"][index]),
                float(parts["feedforward"][index]),
            )
        return parts

    def set_force(self, force):
        """纯关节力矩。kp = 0，kd = 0，t_ref 为力矩，单位 N·m。"""
        torque = np.asarray(force, dtype=float).reshape(-1)
        return self.set_mit(np.zeros(_DOF), np.zeros(_DOF), np.zeros(_DOF), np.zeros(_DOF), torque)

    def get_information(self):
        info = super().get_information()
        if self.collect_info and "result" in self.collect_info:
            info["result"] = self._last_result
        return info

    def handle_runtime_op(self, data: dict) -> None:
        op = str(data.get("op") or "")
        if op != "mit":
            raise ValueError(f"未知操作 {op}")
        parts = self.set_mit(
            data.get("q_des", np.zeros(_DOF)),
            data.get("kp", np.zeros(_DOF)),
            data.get("kd", np.zeros(_DOF)),
            data.get("qd_des"),
            data.get("tau_ff"),
        )
        self._last_result = {"kind": "mit", **{key: value.tolist() for key, value in parts.items()}}

    def _require_arm(self):
        if self.controller is None:
            raise RuntimeError(f"{self.name} 尚未连接")

    def __del__(self):
        try:
            if hasattr(self, 'controller'):
                # Add any necessary cleanup for the arm controller
                pass
        except:
            pass

def enable_fun(piper:C_PiperInterface_V2):
    enable_flag = False
    timeout = 5
    start_time = time.time()
    elapsed_time_flag = False
    while not (enable_flag):
        elapsed_time = time.time() - start_time
        print("--------------------")
        enable_flag = piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
        print("enable flag:",enable_flag)
        piper.EnableArm(7)
        piper.GripperCtrl(0,1000,0x01, 0)

        print("--------------------")
        if elapsed_time > timeout:
            print("time out....")
            elapsed_time_flag = True
            enable_flag = True
            break
        time.sleep(1)
        pass
    if(elapsed_time_flag):
        print("time out, exit!")
        exit(0)

if __name__=="__main__":
    controller = PiperController("test_piper")
    controller.set_up("can0")
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