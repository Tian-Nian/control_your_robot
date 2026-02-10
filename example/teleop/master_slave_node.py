from robot import ROOT_PATH
import sys
sys.path.append(ROOT_PATH)

from robot.robot.base_robot import Robot
from robot.utils.base.data_handler import is_enter_pressed, debug_print
from robot.utils.node.node import TaskNode
from robot.utils.node.scheduler import Scheduler

from threading import Lock
import time

condition = {
    "save_path": "./save/",
    "task_name": "test_teleop",
    "save_format": "hdf5",
    "save_freq": 10, 
}

class RobotNode(TaskNode):
    def task_init(self, robot: Robot):
        self.robot = robot
        self.is_first = True
    
    def task_step(self):
        if not self.is_first:
            data = self.robot.get()
            self.robot.collect(data)
        else:
            self.is_first = False

class TeleopNode(TaskNode):
    def task_init(self, master_robot: Robot, slave_robot: Robot):
        self.master_robot = master_robot
        self.slave_robot = slave_robot
    
    def task_step(self):
        data = self.master_robot.get()[0]
        move_data = {
            "arm":{
                "right_arm": {
                    "joint" :data["right_arm"]["joint"],
                    "gripper" :data["right_arm"]["gripper"],
                },
                "left_arm": {
                    "joint" :data["left_arm"]["joint"],
                    "gripper" :data["left_arm"]["gripper"],
                },

            }
        }
        self.slave_robot.move(move_data)

def init(master_robot: Robot, slave_robot: Robot):
    robot_node = RobotNode("SLAVE_ROBOT", robot=slave_robot)
    robot_node.start()

    teleop_node = TeleopNode("TELEOP", master_robot=master_robot, slave_robot=slave_robot)
    teleop_node.start()

    return robot_node, teleop_node

def build_map(robot_node, teleop_node):
    robot_scheduler = Scheduler(entry_nodes=[robot_node],
                                all_nodes=[robot_node],
                                final_nodes=[robot_node],
                                hz=30) 
    
    teleop_scheduler = Scheduler(entry_nodes=[teleop_node],
                                     all_nodes=[teleop_node],
                                     final_nodes=[teleop_node],
                                     hz=100)

    return robot_scheduler, teleop_scheduler

if __name__ == "__main__":
    from my_robot.test_robot import TestRobot as TestRobotSlave
    from my_robot.test_robot_master import TestRobotMaster

    master_robot = TestRobotMaster(move_check=True)
    master_robot.set_up(teleop=True)

    slave_robot = TestRobotSlave(move_check=True, condition=condition)
    slave_robot.set_up(teleop=False)

    start_episode = 0
    num_episode = 1000

    for episode_id in range(start_episode, start_episode + num_episode):
        robot_node, teleop_node = init(master_robot, slave_robot)

        robot_scheduler, teleop_scheduler = build_map(robot_node, teleop_node)

        master_robot.reset()
        slave_robot.reset()
        debug_print("collect_node", "Waiting for ENTER to start...", "INFO")

        while not is_enter_pressed():
            time.sleep(0.1)
        debug_print("collect_node", "Collect start! Press ENTER to finish!", "INFO")
        
        robot_scheduler.start()
        teleop_scheduler.start()
        
        while not is_enter_pressed():
            time.sleep(0.1)  

        robot_scheduler.stop()
        teleop_scheduler.stop()

        slave_robot.finish(episode_id)