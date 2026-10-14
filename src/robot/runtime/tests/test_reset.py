import numpy as np


def test_controllers_reset_to_zero_or_given_pose():
    from robot.controller.TestArm_controller import TestArmController
    from robot.controller.TestMobile_controller import TestMobileController

    arm = TestArmController("left_arm", DoFs=6, INFO="ERROR")
    arm.set_up()
    arm.move({"joint": np.ones(6)})
    arm.reset()
    assert np.allclose(arm.get_state()["joint"], np.zeros(6))
    arm.reset(np.full(6, 0.2))
    assert np.allclose(arm.get_state()["joint"], np.full(6, 0.2))

    mobile = TestMobileController("test_mobile", INFO="ERROR")
    mobile.set_up()
    mobile.move({"move_velocity": np.ones(6)})
    mobile.reset()
    assert np.allclose(mobile.get_move_velocity(), 0)
    assert np.allclose(mobile.get_position(), 0)
    mobile.reset(np.full(6, 0.3))
    assert np.allclose(mobile.get_position(), np.full(6, 0.3))
    assert np.allclose(mobile.get_move_velocity(), 0)


def test_reset_is_rejected_after_safe_stop():
    from robot.controller.TestArm_controller import TestArmController

    arm = TestArmController("left_arm", DoFs=6, INFO="ERROR")
    arm.set_up()
    arm.move({"joint": np.ones(6)})
    assert arm.safe_stop() == "hold"
    arm.reset()
    assert np.allclose(arm.get_state()["joint"], np.ones(6))
