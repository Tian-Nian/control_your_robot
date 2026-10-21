import numpy as np
import pytest

from robot.controller.Marvin_arm_controller import (
    MarvinArmController,
    clear_shared_robots,
    joint_torque_from_feedback,
    normalize_control_mode,
)
from robot.controller.Y1_controller import Y1Controller
from robot.controller.mit_command import pack_joint_mit
from robot.harness.gate import CommandRejected
from robot.runtime.node import _apply_mode


def test_pack_joint_mit_keeps_feedforward_separate_from_gains():
    packed = pack_joint_mit([0.2] * 6, [10] * 6, [0.8] * 6, tau_ff=[0.3, 0, 0, 0, 0, 0])
    assert np.allclose(packed["q_des"], 0.2)
    assert np.allclose(packed["kp"], 10)
    assert np.allclose(packed["feedforward"], [0.3, 0, 0, 0, 0, 0])
    with pytest.raises(ValueError):
        pack_joint_mit([0.0], [0.0], [0.0], dofs=6)


def test_runtime_accepts_force_and_cartesian_modes():
    class Arm:
        def __init__(self):
            self.moves = []

        def move(self, data, is_delta=False):
            self.moves.append(data)

    arm = Arm()
    _apply_mode(arm, "force")
    _apply_mode(arm, "cartesian")
    assert arm.moves == [{"control_mode": "force"}, {"control_mode": "cartesian"}]
    with pytest.raises(CommandRejected):
        _apply_mode(arm, "torque")


def test_y1_mit_uses_sdk_command_and_reads_effort(monkeypatch):
    class Command:
        pass

    class Modes:
        MIT_CONTROL = "MIT"
        NRT_JOINT_POSITION = "NRT"

    class Sdk:
        def __init__(self):
            self.mode = None
            self.commands = None
            self.joint = None

        def SetArmControlMode(self, mode):
            self.mode = mode

        def MitControlArm(self, commands):
            self.commands = list(commands)

        def SetArmJointPosition(self, joint, ratio):
            self.joint = (list(joint), ratio)

        def GetJointPosition(self):
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 42.0]

        def GetJointVelocity(self):
            return [0.0] * 7

        def GetArmEndPose(self):
            return [0.0] * 6

        def GetJointEffort(self):
            return [1.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    monkeypatch.setattr(
        "robot.controller.Y1_controller._load_y1",
        lambda: (None, Modes, Command),
    )
    arm = Y1Controller("y1")
    arm.controller = Sdk()
    arm.set_mit([0.2] * 6, [4] * 6, [0.5] * 6, tau_ff=[0.7, 0, 0, 0, 0, 0])
    assert arm.controller.mode == "MIT"
    assert arm.controller.commands[0].joint_position == 0.2
    assert arm.controller.commands[0].kp == 4
    assert arm.controller.commands[0].kd == 0.5
    assert arm.controller.commands[0].torque == 0.7
    assert arm.controller.commands[0].joint_velocity == 0.0
    arm.set_force([0.0, 0.4, 0, 0, 0, 0])
    assert arm.controller.commands[1].kp == 0.0
    assert arm.controller.commands[1].kd == 0.0
    assert arm.controller.commands[1].torque == 0.4
    arm.set_joint([0.1] * 6)
    assert arm.controller.mode == "NRT"
    assert arm.controller.joint[1] == 2
    assert np.isclose(arm.get_state()["force"][0], 1.25)


def test_joint_torque_feedback_is_optional():
    assert normalize_control_mode("imp_force") == "force"
    assert normalize_control_mode("cart") == "cartesian"
    data = {"outputs": [{"fb_joint_pos": [0] * 7}, {}]}
    assert joint_torque_from_feedback(data, "B") is None
    data["outputs"][1]["fb_joint_sToq"] = [1, 2, 3, 4, 5, 6, 7]
    assert np.allclose(joint_torque_from_feedback(data, "B"), [1, 2, 3, 4, 5, 6, 7])


def test_marvin_sends_cartesian_impedance_and_end_force(monkeypatch):
    monkeypatch.setattr("robot.controller.Marvin_arm_controller.time.sleep", lambda _seconds: None)
    clear_shared_robots()
    calls = []

    class Robot:
        def __init__(self):
            self.mode = {"A": 0}

        def subscribe(self, _dcss):
            return {
                "outputs": [
                    {
                        "fb_joint_pos": [0, 0, 0, 0, 0, 0, 0],
                        "fb_joint_sToq": [1.5, 0, 0, 0, 0, 0, 0],
                        "low_speed_flag": 1,
                    }
                ],
                "states": [{"cur_state": self.mode["A"]}],
            }

        def set_joint_position_cmd(self, arm, joint):
            calls.append(("joint", arm))
            return True

        def disable(self, arm):
            self.mode[arm] = 0
            return True

        def set_position_state(self, arm, velRatio, AccRatio):
            self.mode[arm] = 1
            calls.append(("position", velRatio, AccRatio))
            return True

        def set_imp_joint_state(self, arm, velRatio, AccRatio, K, D):
            self.mode[arm] = 3
            return True

        def set_imp_cart_state(self, arm, velRatio, AccRatio, K, D, rot_type, cart_ctrl_para):
            self.mode[arm] = 3
            calls.append(("cartesian", list(K), rot_type, list(cart_ctrl_para)))
            return True

        def set_imp_force_state(self, arm, fx_dir, fc_adj_lmt):
            self.mode[arm] = 3
            calls.append(("force_mode", list(fx_dir), fc_adj_lmt))
            return True

        def set_force_cmd(self, arm, force):
            calls.append(("force", force))
            return True

        def set_tool(self, arm, kine_para, dyn_para):
            return True

        def release_robot(self):
            calls.append("release")

    class Kine:
        def fk(self, joints):
            return np.eye(4)

    arm = MarvinArmController("left_arm")
    try:
        arm.set_up(
            "192.168.1.190",
            side="left",
            control_mode="position",
            force_dir=[0, 1, 0, 0, 0, 0],
            force_adj_mm=30,
            _connect=lambda _ip: (Robot(), object()),
            _kine=Kine(),
        )
        assert np.isclose(arm.get_state()["force"][0], 1.5)
        arm.set_control_mode("cartesian")
        assert calls[-1][0] == "cartesian"
        assert calls[-1][1][0] == 3000.0
        assert calls[-1][2] == 2
        arm.move({"control_mode": "force", "force": 12.0})
        assert ("force_mode", [0.0, 1.0, 0.0, 0.0, 0.0, 0.0], 30.0) in calls
        assert ("force", 12.0) in calls
        assert arm.get_information()["control_mode"] == "force"
        with pytest.raises(ValueError):
            arm.set_force([1, 2, 3, 4, 5, 6, 7])
    finally:
        arm.stop()
        clear_shared_robots()


def test_piper_mit_matches_joint_mit_ctrl():
    pytest.importorskip("piper_sdk")
    from robot.controller.Piper_controller import PiperController, effort_from_high_speed

    class Motor:
        def __init__(self, effort):
            self.effort = effort

    message = type("High", (), {})()
    for index, effort in enumerate((1500, 0, -500, 0, 0, 0), start=1):
        setattr(message, f"motor_{index}", Motor(effort))
    assert np.allclose(effort_from_high_speed(message), [1.5, 0, -0.5, 0, 0, 0])

    class Sdk:
        def __init__(self):
            self.mode = None
            self.mits = []

        def MotionCtrl_2(self, ctrl_mode, move_mode, speed, is_mit_mode):
            self.mode = (ctrl_mode, move_mode, speed, is_mit_mode)

        def JointMitCtrl(self, motor, pos, vel, kp, kd, torque):
            self.mits.append((motor, pos, vel, kp, kd, torque))

    arm = PiperController("piper")
    arm.controller = Sdk()
    arm.set_force([0.4, 0, 0, 0, 0, 0])
    assert arm.controller.mode == (0x01, 0x04, 100, 0xAD)
    assert arm.controller.mits[0] == (1, 0.0, 0.0, 0.0, 0.0, 0.4)
    arm.handle_runtime_op(
        {"op": "mit", "q_des": [0.2] * 6, "kp": [10] * 6, "kd": [0.8] * 6, "tau_ff": [0.1] * 6}
    )
    assert arm._last_result["kind"] == "mit"
    assert arm._last_result["q_des"][0] == 0.2
    assert arm.controller.mits[6][0] == 1
    assert arm.controller.mits[6][3] == 10.0
    assert arm.controller.mits[6][5] == 0.1


def test_piper_agx_move_mit_is_per_joint_and_pauses_teleop():
    pytest.importorskip("pyAgxArm")
    pytest.importorskip("agx_pinocchio")
    import threading

    from robot.controller.PiperAgx_controller import PiperAgxController, torque_from_motor_states

    class Sample:
        def __init__(self, torque):
            self.msg = type("Msg", (), {"torque": torque})()

    assert torque_from_motor_states([Sample(0.2), Sample(None)]) is None
    assert np.allclose(torque_from_motor_states([Sample(0.2), Sample(-0.1)]), [0.2, -0.1])

    class Sdk:
        joint_nums = 6

        def __init__(self):
            self.calls = []

        def move_mit(self, joint_id, p_des, v_des, kp, kd, t_ff):
            self.calls.append((joint_id, p_des, v_des, kp, kd, t_ff))

    arm = PiperAgxController("left")
    arm.controller = Sdk()
    arm.teleop = True
    arm._teleop_event = threading.Event()
    arm._teleop_event.set()
    arm.set_force([0.15, 0, 0, 0, 0, 0])
    assert arm.teleop is False
    assert not arm._teleop_event.is_set()
    assert arm.controller.calls[0] == (1, 0.0, 0.0, 0.0, 0.0, 0.15)
    assert arm.controller.calls[5][0] == 6
    arm.handle_runtime_op(
        {
            "op": "mit",
            "q_des": [0.3] * 6,
            "kp": [8] * 6,
            "kd": [0.4] * 6,
            "qd_des": [0.1] * 6,
            "tau_ff": [0.05] * 6,
        }
    )
    assert arm._last_result["feedforward"] == [0.05] * 6
    assert arm.controller.calls[6] == (1, 0.3, 0.1, 8.0, 0.4, 0.05)
