

from pathlib import Path

import numpy as np

from robot.controller.arm_controller import ArmController, reject_unsupported_mode
from robot.controller.arm_servo import DEFAULT_MIT_KD, ArmServoLaw, MitServo
from robot.controller.mit_command import pack_joint_mit
from robot.utils.base.data_handler import debug_print

'''
Y1 SDK: y1_sdk.Y1SDKInterface，MIT 用 MitControlArm。
'''

package_path = "/home/tian/project/y1_sdk_python/y1_ros/src/y1_controller/"
_DOF = 6


def _load_y1():
    from y1_sdk import ControlMode, Y1SDKInterface

    command_cls = None
    try:
        from y1_sdk.y1_sdk import MitControlCommand as command_cls
    except ImportError:
        import y1_sdk

        command_cls = getattr(y1_sdk, "MitControlCommand", None)
        nested = getattr(y1_sdk, "y1_sdk", None)
        if command_cls is None and nested is not None:
            command_cls = getattr(nested, "MitControlCommand", None)
    if command_cls is None:
        raise ImportError("当前 y1_sdk 没有 MitControlCommand")
    return Y1SDKInterface, ControlMode, command_cls


class Y1Controller(MitServo, ArmController):
    supported_modes = ("position", "mit", "gravity_compensation", "impedance")

    def __init__(self, name):
        super().__init__()
        self.name = name
        self.controller_type = "user_controller"
        self.controller = None
        self.DoFs = _DOF
        self._mode = "position"
        self._last_result = {}
        self._drive_mit = False
        self.urdf_path = None

    def set_up(self, can: str, arm_end_type=3, teleop=False, control_mode=None, joint_K=None, joint_D=None):
        Y1SDKInterface, _control_mode, _command_cls = _load_y1()
        if arm_end_type == 0:
            urdf_path = f"{package_path}/urdf/y10804.urdf"
        elif arm_end_type == 1:
            urdf_path = f"{package_path}/urdf/y1_gripper_t.urdf"
        elif arm_end_type == 2:
            urdf_path = f"{package_path}/urdf/y1_gripper_g.urdf"
        elif arm_end_type == 3:
            urdf_path = f"{package_path}/urdf/y10824_ee.urdf"

        self.urdf_path = urdf_path
        if Path(urdf_path).is_file():
            self.bind_mit_servo(
                ArmServoLaw(
                    urdf_path,
                    mit_kd=DEFAULT_MIT_KD,
                    joint_K=joint_K,
                    joint_D=joint_D,
                    apply_scale=True,
                )
            )
        self.controller = Y1SDKInterface(
            can_id=can,
            urdf_path=urdf_path,
            arm_end_type=arm_end_type,
            enable_arm=True,
        )

        if not self.controller.Init():
            debug_print(self.name, "Init Fail!", "ERROR")
            # exit()
        if control_mode is None:
            control_mode = "gravity_compensation" if teleop else "position"
        self.set_control_mode(control_mode)

    def set_control_mode(self, mode: str) -> str:
        """重力和阻抗走 MIT 前馈。不再切 SDK 的重力模式，避免和这份前馈叠两份。"""
        target = reject_unsupported_mode(self.name, mode, self.supported_modes)
        if target in ("gravity_compensation", "impedance"):
            law = self.require_mit_servo()
            if target == "impedance":
                law.require_gains()
            self.control_mode = target
            self._mode = target
            if self.controller is not None:
                _interface, ControlMode, _command_cls = _load_y1()
                self.controller.SetArmControlMode(ControlMode.MIT_CONTROL)
                self._drive_mit = True
            return target
        self.control_mode = target
        self._mode = target
        self._drive_mit = target == "mit"
        if self.controller is None:
            return target
        _interface, ControlMode, _command_cls = _load_y1()
        sdk_mode = {
            "position": ControlMode.NRT_JOINT_POSITION,
            "mit": ControlMode.MIT_CONTROL,
        }[target]
        self.controller.SetArmControlMode(sdk_mode)
        return target

    # 返回单位为米
    def get_state(self):
        state = {}
        
        eef = self.controller.GetArmEndPose()
        joint = self.controller.GetJointPosition()
        vel = self.controller.GetJointVelocity()

        state["eef"] = eef
        state["joint"] = joint[:6]
        state["gripper"] = joint[6] / 84
        state["control_mode"] = self.control_mode
        if hasattr(self.controller, "GetJointEffort"):
            effort = np.asarray(self.controller.GetJointEffort(), dtype=float).reshape(-1)
            state["force"] = effort[:_DOF].copy()

        return state

    # All returned values are expressed in meters,if the value represents an angle, it is returned in radians
    def set_position(self, position):
        self._leave_mit()
        self.controller.SetArmEndPose(list(position))
    
    def set_joint(self, joint):
        if self.control_mode == "impedance" and self.mit_servo_ready():
            self.note_joint_target(joint)
            return
        self._leave_mit()
        self.controller.SetArmJointPosition(list(joint), 2)

    def set_mit(self, q_des, kp, kd, qd_des=None, tau_ff=None):
        """Y1SDK.MitControlArm。先切到 ControlMode.MIT_CONTROL。

        每轴一条 MitControlCommand：kp、joint_position、kd、joint_velocity、torque。
        不传 tau_ff 时力矩前馈是 0。重力补偿是另一种模式，不会自动加进这条指令。
        """
        if self.controller is None:
            raise RuntimeError(f"{self.name} 尚未连接")
        _interface, ControlMode, command_cls = _load_y1()
        parts = pack_joint_mit(q_des, kp, kd, qd_des, tau_ff, _DOF)
        if self._mode != "mit":
            self.controller.SetArmControlMode(ControlMode.MIT_CONTROL)
            self._mode = "mit"
        self.control_mode = "mit"
        self._drive_mit = True
        commands = []
        for index in range(_DOF):
            command = command_cls()
            command.kp = float(parts["kp"][index])
            command.joint_position = float(parts["q_des"][index])
            command.kd = float(parts["kd"][index])
            command.joint_velocity = float(parts["qd_des"][index])
            command.torque = float(parts["feedforward"][index])
            commands.append(command)
        self.controller.MitControlArm(commands)
        return parts

    def set_force(self, force):
        """纯关节力矩。kp = 0，kd = 0，MitControlCommand.torque 为力矩。"""
        torque = np.asarray(force, dtype=float).reshape(-1)
        return self.set_mit(np.zeros(_DOF), np.zeros(_DOF), np.zeros(_DOF), np.zeros(_DOF), torque)

    def get_information(self):
        info = super().get_information()
        if self.collect_info and "result" in self.collect_info:
            info["result"] = self._last_result
        return info

    def _send_mit_refresh(self, terms: dict) -> None:
        if self.controller is None:
            raise RuntimeError(f"{self.name} 尚未连接")
        _interface, ControlMode, command_cls = _load_y1()
        if not self._drive_mit:
            self.controller.SetArmControlMode(ControlMode.MIT_CONTROL)
            self._drive_mit = True
        commands = []
        for index in range(_DOF):
            command = command_cls()
            command.kp = float(terms["kp"][index])
            command.joint_position = float(terms["q_des"][index])
            command.kd = float(terms["kd"][index])
            command.joint_velocity = float(terms["qd_des"][index])
            command.torque = float(terms["tau_ff"][index])
            commands.append(command)
        self.controller.MitControlArm(commands)

    def handle_runtime_op(self, data: dict) -> None:
        if self.dispatch_servo_op(data):
            return
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

    def _leave_mit(self):
        if self.controller is None:
            return
        if self._mode != "mit" and not self._drive_mit:
            return
        _interface, ControlMode, _command_cls = _load_y1()
        self.controller.SetArmControlMode(ControlMode.NRT_JOINT_POSITION)
        self._mode = "position"
        self.control_mode = "position"
        self._drive_mit = False

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