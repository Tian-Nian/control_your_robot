import sys
sys.path.append("./")

import select

from my_robot.agilex_piper_dual_base import PiperDual

import time

from utils.data_handler import is_enter_pressed,debug_print

condition = {
    "save_path": "./save/", 
    "task_name": "Make_a_beef_sandwichv5",  # Make_a_beef_sandwichv2
    "save_format": "hdf5", 
    "save_freq": 30,
}

if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "INFO" # DEBUG , INFO, ERROR

    # import rospy
    # rospy.init_node('ros_subscriber_node', anonymous=True)

    robot = PiperDual(condition=condition, move_check=True)
    robot.set_up()
    start_episode = 0
    num_episode = 1

    for episode_id in range(start_episode, start_episode + num_episode):
        # robot.reset()
        debug_print("main", "Press Enter to start...", "INFO")
        while not robot.is_start() or not is_enter_pressed():
            time.sleep(1/robot.condition["save_freq"])
        
        debug_print("main", "Press Enter to finish...", "INFO")
        
        avg_time = 0.0
        num = 0

        while True:
            now = time.monotonic()

            data = robot.get()
            robot.collect(data)
            
            if is_enter_pressed():
                robot.finish(episode_id)
                break
            
            while True:
                last = time.monotonic()
                if last - now >= 1/robot.condition["save_freq"]:
                    avg_time +=last - now
                    num += 1
                    break
        
        avg_time /= num
        extra_info = {}
        extra_info["avg_time_interval"] = avg_time
        robot.collection.add_extra_condition_info(extra_info)