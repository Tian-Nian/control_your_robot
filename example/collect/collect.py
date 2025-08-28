import sys
sys.path.append("./")

import select

from my_robot.test_robot import TestRobot

import time

from utils.data_handler import KeyListener, debug_print

import keyboard


if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "INFO" # DEBUG , INFO, ERROR

    robot = TestRobot(sub_task=True)
    robot.set_up()
    num_episode = 5

    # kc = KeyListener()
    for _ in range(num_episode):
        robot.reset()
        debug_print("main", "Press Enter to start...", "INFO")
        while not robot.is_start() or not  keyboard.is_pressed("\n"):
            time.sleep(1/robot.condition["save_freq"])
        
        debug_print("main", "Press Enter to finish...", "INFO")
        time.sleep(1)
        avg_collect_time = 0.0
        collect_num = 0
        while True:
            last_time = time.monotonic()

            data = robot.get()
            robot.collect(data)

            # key = kc.get_key()

            if keyboard.is_pressed("\n"):
                robot.finish()
                break
            elif keyboard.is_pressed(" "):
                robot.collection.next_subtask()
                
            collect_num += 1
            while True:
                now = time.monotonic()
                if now -last_time > 1/robot.condition["save_freq"]:
                    avg_collect_time += now -last_time
                    break
                else:
                    time.sleep(0.0001)
        extra_info = {}
        avg_collect_time = avg_collect_time / collect_num
        extra_info["avg_time_interval"] = avg_collect_time
        robot.collection.add_extra_condition_info(extra_info)