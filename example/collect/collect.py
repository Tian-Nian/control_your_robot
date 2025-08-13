import sys
sys.path.append("./")

import select

from my_robot.agilex_piper_dual_base import PiperDual

import time

from utils.data_handler import is_enter_pressed,debug_print


if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "DEBUG" # DEBUG , INFO, ERROR

    import rospy
    rospy.init_node('ros_subscriber_node', anonymous=True)

    robot = PiperDual()
    robot.set_up()
    num_episode = 10
    robot.condition["task_name"] = "my_test"

    for _ in range(num_episode):
        robot.reset()
        debug_print("main", "Press Enter to start...", "INFO")
        while not robot.is_start() or not is_enter_pressed():
            time.sleep(1/robot.condition["save_freq"])
        
        debug_print("main", "Press Enter to finish...", "INFO")

        while True:
            data = robot.get()
            robot.collect(data)
            
            if is_enter_pressed():
                robot.finish()
                break
            
            time.sleep(1/robot.condition["save_freq"])