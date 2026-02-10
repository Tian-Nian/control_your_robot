
# change to your controller and test_controller
# from robot.controller.Piper_controller import PiperController
from robot.controller.TestArm_controller import TestArmController
from robot.utils.base.data_handler import debug_print

if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "DEBUG"

    controller = TestArmController("play_arm")
    debug_print("TestArmController","TestArmController initialized", "INFO")
    controller.set_up()
    debug_print("TestArmController","TestArmController moved", "INFO")

    test_controller = TestArmController("test_arm",DoFs=6,INFO="DEBUG")

    test_controller.set_collect_info(["joint","qpos","gripper"])

    test_controller.set_up()

    test_controller.get()

    
