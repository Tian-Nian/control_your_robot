from robot.robot.base_robot_node import build_robot_node
from robot.utils.base.data_handler import is_enter_pressed, debug_print, dict_to_list

condition = {
    "save_path": "./save/test_dt/",
    "task_name": "new",
    "save_format": "hdf5",
    "save_freq": 10, 
}

if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "DEBUG" # DEBUG , INFO, ERROR

    robot = TestRobot()
    robot_cls = build_robot_node(TestRobot)
    robot = robot_cls(condition=condition)
    robot.set_up()

    start_episode = 0
    num_episode = 5

    for episode_id in range(start_episode, start_episode + num_episode):
        robot.reset()
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



