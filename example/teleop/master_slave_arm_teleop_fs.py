

import time
from multiprocessing import Event

from robot.utils.base.data_handler import is_enter_pressed
from robot.utils.worker.time_scheduler import TimeScheduler
from robot.utils.worker.worker import Worker
from robot.utils.worker.msgpack_shared_dict import MsgpackSharedDict
from robot.controller.TestArm_controller import TestArmController
from robot.sensor.TestVision_sensor import TestVisonSensor
from robot.data.collect_any import CollectAny

condition = {
    "save_path": "./save/",
    "task_name": "test_3",
    "save_format": "hdf5",
    "save_freq": 30,
    "collect_type": "teleop",
}


class MasterWorker(Worker):
    def __init__(self, process_name: str, start_event, end_event, shared_state: MsgpackSharedDict):
        super().__init__(process_name, start_event, end_event)
        self.shared_state = shared_state

    def handler(self):
        data_left = self.component_left.get()
        data_right = self.component_right.get()
        self.shared_state.replace(
            {
                "left_joint": data_left["joint"],
                "left_gripper": data_left["gripper"],
                "right_joint": data_right["joint"],
                "right_gripper": data_right["gripper"],
            }
        )

    def component_init(self):
        self.component_left = TestArmController("left_master_arm")
        self.component_right = TestArmController("right_master_arm")

        self.component_left.set_up()
        self.component_right.set_up()

        self.component_left.set_collect_info(["joint", "gripper"])
        self.component_right.set_collect_info(["joint", "gripper"])


class SlaveWorker(Worker):
    def __init__(
        self,
        process_name: str,
        start_event,
        end_event,
        move_data_buffer: MsgpackSharedDict,
        shared_state: MsgpackSharedDict,
    ):
        super().__init__(process_name, start_event, end_event)
        self.move_data_buffer = move_data_buffer
        self.shared_state = shared_state

    def handler(self):
        move_data = self.move_data_buffer.read()

        left_move_data = {
            "joint": move_data["left_joint"],
            "gripper": move_data["left_gripper"],
        }
        right_move_data = {
            "joint": move_data["right_joint"],
            "gripper": move_data["right_gripper"],
        }

        self.component_left.move(left_move_data)
        self.component_right.move(right_move_data)

        data_left = self.component_left.get()
        data_right = self.component_right.get()

        self.shared_state.replace(
            {
                "controller": {
                    "left_arm": data_left,
                    "right_arm": data_right,
                }
            }
        )

    def component_init(self):
        self.component_left = TestArmController("left_slave_arm")
        self.component_right = TestArmController("right_slave_arm")

        self.component_left.set_up()
        self.component_right.set_up()

        self.component_left.set_collect_info(["joint", "eef", "gripper"])
        self.component_right.set_collect_info(["joint", "eef", "gripper"])


class DataWorker(Worker):
    def __init__(
        self,
        process_name: str,
        start_event,
        end_event,
        collect_data_buffer: MsgpackSharedDict,
        episode_id=0,
    ):
        super().__init__(process_name, start_event, end_event)
        self.collect_data_buffer = collect_data_buffer
        self.episode_id = episode_id

    def handler(self):
        data = self.collect_data_buffer.read()
        if not data:
            return

        controller_data = data["controller"]
        sensor_data = {
            "cam_head": self.cam_head.get(),
            "cam_left_wrist": self.cam_left_wrist.get(),
            "cam_right_wrist": self.cam_right_wrist.get(),
        }

        self.collection.collect(controller_data, sensor_data)

    def finish(self):
        self.collection.write(episode_id=self.episode_id)

    def component_init(self):
        self.cam_head = TestVisonSensor("cam_head")
        self.cam_left_wrist = TestVisonSensor("cam_left_wrist")
        self.cam_right_wrist = TestVisonSensor("cam_right_wrist")

        self.cam_head.set_up()
        self.cam_left_wrist.set_up()
        self.cam_right_wrist.set_up()

        self.cam_head.set_collect_info(["color"])
        self.cam_left_wrist.set_collect_info(["color"])
        self.cam_right_wrist.set_collect_info(["color"])

        self.collection = CollectAny(condition=condition, start_episode=self.episode_id, move_check=False)


if __name__ == "__main__":
    import os

    os.environ["INFO_LEVEL"] = "DEBUG"
    num_episode = 3
    avg_collect_time = 0

    for i in range(num_episode):
        is_start = False

        start_event, end_event = Event(), Event()
        master_state = MsgpackSharedDict()
        slave_state = MsgpackSharedDict()

        master = MasterWorker("master_arm", start_event, end_event, master_state)
        slave = SlaveWorker("slave_arm", start_event, end_event, master_state, slave_state)
        data = DataWorker("collect_data", start_event, end_event, slave_state, episode_id=i)

        time_scheduler_control = TimeScheduler(
            work_events=[master.forward_event],
            time_freq=300,
            end_events=[slave.next_event],
            process_name="time_scheduler_control",
        )
        time_scheduler_collect = TimeScheduler(
            work_events=[data.forward_event],
            time_freq=60,
            end_events=[data.next_event],
            process_name="time_scheduler_collect",
        )

        master.next_to(slave)

        master.start()
        slave.start()
        data.start()

        while not is_start:
            time.sleep(0.01)
            if is_enter_pressed():
                is_start = True
                start_event.set()
            else:
                time.sleep(1)

        time_scheduler_control.start()
        time_scheduler_collect.start()
        while is_start:
            time.sleep(0.01)
            if is_enter_pressed():
                end_event.set()
                time_scheduler_control.stop()
                time_scheduler_collect.stop()
                is_start = False

        time.sleep(1)

        for worker in (master, slave, data):
            worker.process.join(timeout=15)
            if worker.process.is_alive():
                worker.stop()
