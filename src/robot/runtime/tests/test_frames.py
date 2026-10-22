import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from robot.harness.contract import ActionContract
from robot.runtime.frames import compose_delta


def test_tool_delta_uses_rotation_and_quaternion_multiply():
    identity = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    moved = compose_delta(identity, [0.02, 0.0, 0.0])
    assert moved[:3] == pytest.approx([0.02, 0.0, 0.0])
    assert np.allclose(moved[3:], identity[3:])

    quarter = Rotation.from_euler("z", np.pi / 2).as_quat()
    turned = np.array([0.0, 0.0, 0.0, quarter[3], quarter[0], quarter[1], quarter[2]])
    shifted = compose_delta(turned, [0.02, 0.0, 0.0])
    assert shifted[0] == pytest.approx(0.0, abs=1e-9)
    assert shifted[1] == pytest.approx(0.02, abs=1e-9)

    twist = compose_delta(identity, [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0])
    assert twist[3:7] == pytest.approx([0.0, 0.0, 1.0, 0.0])
    assert not np.allclose(twist[3:7], identity[3:7] + np.array([0.0, 0.0, 1.0, 0.0]))


def test_action_mode_rejects_an_empty_value():
    with pytest.raises(ValueError):
        ActionContract(mode="")


def test_solve_eef_delta_keeps_quaternion_multiply_and_position_only():
    from robot.runtime.node import _solve_eef

    seen = {}

    class Body:
        def arm_names(self):
            return ["left"]

        def flange_pose(self, arm, joints):
            return np.array([1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0])

        def ik(self, arm, joints, target, max_error_m=0.008):
            seen["target"] = np.asarray(target, dtype=float).copy()
            seen["max_error_m"] = max_error_m
            return np.array([0.1, 0.2, 0.3])

    component = type("Component", (), {})()
    component._cyr_body = Body()
    component.get_state = lambda: {"joint": [0.0, 0.0, 0.0]}
    solved, target = _solve_eef(
        {"name": "left_arm", "gate": {}},
        component,
        [0.02, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        True,
    )
    assert seen["target"][:3] == pytest.approx([1.02, 2.0, 3.0])
    assert seen["target"][3:] == pytest.approx([0.0, 0.0, 1.0, 0.0])
    assert np.allclose(target, seen["target"])
    assert solved.tolist() == [0.1, 0.2, 0.3]


def test_arm_controller_delta_uses_tool_frame_when_there_is_no_body():
    from robot.controller.TestArm_controller import TestArmController

    arm = TestArmController("left", DoFs=6, INFO="ERROR")
    arm.now_state = {
        "joint": np.zeros(6),
        "eef": np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]),
        "gripper": 0.0,
    }
    arm.move_controller({"eef": np.array([0.02, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0])}, is_delta=True)
    assert arm.now_state["eef"][:3] == pytest.approx([0.02, 0.0, 0.0])
    assert arm.now_state["eef"][3:] == pytest.approx([0.0, 0.0, 1.0, 0.0])
