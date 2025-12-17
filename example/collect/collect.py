import sys
sys.path.append("./")

import select

from my_robot.y1_dual_base import Y1Dual

import time

from utils.data_handler import is_enter_pressed,debug_print

condition = {
    "save_path": "./save/",
    # "task_name": "complete_task",
    "task_name": "pick_bag_1118",
    # "task_name": "replay_1",
    # "task_name": "step3",
    "save_format": "hdf5",
    "save_freq": 30, 
}

if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "INFO" # DEBUG , INFO, ERROR

    start_episode = 48
    num_episode = 100

    robot = Y1Dual(condition=condition, move_check=True, start_episode=start_episode)
    robot.set_up()

    for episode_id in range(start_episode, start_episode + num_episode):
        robot.reset()
        robot.change_mode(teleop=True)

        debug_print("main", "Press Enter to start...", "INFO")
        while not robot.is_start() or not is_enter_pressed():
            time.sleep(1/robot.condition["save_freq"])
        
        debug_print("main", "Press Enter to finish...", "INFO")

        avg_collect_time = 0.0
        collect_num = 0
        while True:
            last_time = time.monotonic()

            data = robot.get()
            robot.collect(data)
            
            if is_enter_pressed():
                robot.finish(episode_id)
                break
                
            collect_num += 1
            while True:
                now = time.monotonic()
                if now -last_time > 1/robot.condition["save_freq"]:
                    avg_collect_time += now -last_time
                    break
                else:
                    time.sleep(0.001)
        extra_info = {}
        avg_collect_time = avg_collect_time / collect_num
        extra_info["avg_time_interval"] = avg_collect_time
        robot.collection.add_extra_condition_info(extra_info)
        # time.sleep(7)