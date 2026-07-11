import argparse
from contextlib import contextmanager
import os
import select
import sys
import termios
import time
import tty
from multiprocessing import Event, Value
from pathlib import Path

import numpy as np

# Support both direct execution and ``python -m`` from the project root.
PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from my_robot.panthera_single import CAMERA_SERIALS, PantheraSingle
from robot.controller.Panthera_HT_controller import PantheraHTController
from robot.data.collect_any import CollectAny
from robot.sensor.Realsense_sensor import RealsenseSensor
from robot.utils.worker.msgpack_shared_dict import MsgpackSharedDict
from robot.utils.worker.time_scheduler import TimeScheduler
from robot.utils.worker.worker import Worker


DEFAULT_CONTROL_FREQUENCY = 100
MAX_CONTROL_FREQUENCY = 300
DEFAULT_COLLECT_FREQUENCY = 30
MAX_COLLECT_FREQUENCY = 60
DEFAULT_GRIPPER_CONTROL = "mit"
DEFAULT_GRIPPER_GAIN = 1.0
DEFAULT_GRIPPER_OFFSET = 0.0
MIN_GRIPPER_VELOCITY = 0.5
MAX_INITIAL_JOINT_ERROR = 0.5
DEFAULT_SUCCESS_TARGET = 3

RESULT_NONE = 0
RESULT_SUCCESS = 1
RESULT_FAIL = 2
RESULT_NAMES = {
    RESULT_SUCCESS: "success",
    RESULT_FAIL: "fail",
}

DEFAULT_CONDITION = {
    "robot": "panthera_single",
    "save_path": "./datasets/",
    "task_name": "panthera_ht_teleop_single",
    "save_format": "hdf5",
    "save_freq": DEFAULT_COLLECT_FREQUENCY,
    "collect_type": "teleop",
}


def _leader_state_from_buffer(data_buffer):
    state = data_buffer.read()
    required = (
        "joint",
        "velocity",
        "gripper",
        "gripper_velocity",
        "timestamp",
    )
    missing = [key for key in required if key not in state]
    if missing:
        raise RuntimeError(
            f"Leader state is incomplete, missing: {', '.join(missing)}"
        )

    return {
        "joint": np.asarray(state["joint"], dtype=np.float32),
        "velocity": np.asarray(state["velocity"], dtype=np.float32),
        "gripper": np.float32(state["gripper"]),
        "gripper_velocity": np.float32(state["gripper_velocity"]),
        "timestamp": int(state["timestamp"]),
    }


def _bounded_gripper_target(value, gain=1.0, offset=0.0):
    target = float(np.asarray(value).squeeze())
    if not np.isfinite(target):
        raise RuntimeError("Leader gripper feedback is NaN or Inf")
    target = gain * target + offset
    return np.float32(np.clip(target, 0.0, 1.0))


def _command_follower_gripper(follower, mode, target, velocity):
    if mode == "position":
        normalized_velocity = float(np.asarray(velocity).squeeze())
        if not np.isfinite(normalized_velocity):
            raise RuntimeError(
                "Leader gripper velocity is NaN or Inf"
            )
        follower.set_gripper(
            target,
            velocity=max(
                MIN_GRIPPER_VELOCITY,
                abs(normalized_velocity) * 2.0,
            ),
        )
        return
    if mode == "mit":
        follower.set_gripper_mit(
            position=target,
            velocity=velocity,
            torque=0.0,
            kp=follower.gripper_kp,
            kd=follower.gripper_kd,
        )
        return
    raise ValueError(f"Unsupported gripper control mode: {mode}")


def _classified_condition(condition, result_name):
    classified = dict(condition)
    base_task_name = condition["task_name"].rstrip("/")
    classified["task_name"] = f"{base_task_name}/{result_name}"
    return classified


def _count_existing_episodes(condition, result_name):
    classified = _classified_condition(condition, result_name)
    directory = (
        Path(classified["save_path"])
        / classified["task_name"]
    )
    if not directory.exists():
        return 0
    return sum(
        path.suffix == ".hdf5" and path.stem.isdigit()
        for path in directory.iterdir()
    )


def _read_key():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


@contextmanager
def _single_key_mode(stream=None):
    stream = sys.stdin if stream is None else stream
    if not stream.isatty():
        yield
        return

    file_descriptor = stream.fileno()
    previous_settings = termios.tcgetattr(file_descriptor)
    tty.setcbreak(file_descriptor)
    try:
        yield
    finally:
        termios.tcsetattr(
            file_descriptor,
            termios.TCSADRAIN,
            previous_settings,
        )


class MasterWorker(Worker):
    def __init__(
        self,
        process_name,
        start_event,
        end_event,
        shared_state,
    ):
        super().__init__(process_name, start_event, end_event)
        self.shared_state = shared_state

    def component_init(self):
        self.component = PantheraHTController("master_left_arm")
        self.component.set_up(follower=False)

    def handler(self):
        try:
            state = self.component.update_leader_compensation()
            state["timestamp"] = time.time_ns()
            self.shared_state.replace(state)
        except Exception:
            self.end_event.set()
            raise

    def finish(self):
        self.component.brake()


class SlaveWorker(Worker):
    def __init__(
        self,
        process_name,
        start_event,
        end_event,
        move_data_buffer,
        shared_state,
        gripper_control,
        gripper_gain,
        gripper_offset,
    ):
        super().__init__(process_name, start_event, end_event)
        self.move_data_buffer = move_data_buffer
        self.shared_state = shared_state
        if gripper_control not in {"position", "mit"}:
            raise ValueError(
                "gripper_control must be 'position' or 'mit'"
            )
        self.gripper_control = gripper_control
        self.gripper_gain = gripper_gain
        self.gripper_offset = gripper_offset
        self._alignment_checked = False

    def component_init(self):
        self.component = PantheraSingle(enable_cameras=False)
        self.component.set_up()
        self.follower = self.component.controllers["arm"]["left_arm"]

    def handler(self):
        try:
            leader_state = _leader_state_from_buffer(
                self.move_data_buffer
            )
            follower_state = self.follower.get_state()

            if not self._alignment_checked:
                joint_error = np.max(
                    np.abs(
                        leader_state["joint"]
                        - follower_state["joint"]
                    )
                )
                if joint_error > MAX_INITIAL_JOINT_ERROR:
                    raise RuntimeError(
                        "Leader/follower initial joint error is "
                        f"{joint_error:.3f} rad, exceeding "
                        f"{MAX_INITIAL_JOINT_ERROR:.3f} rad. "
                        "Align both arms before starting teleoperation."
                    )
                self._alignment_checked = True

            compensation_torque = (
                self.follower.get_compensation_torque(
                    joint=follower_state["joint"],
                    velocity=follower_state["velocity"],
                )
            )
            self.follower.set_joint_mit(
                position=leader_state["joint"],
                velocity=leader_state["velocity"],
                torque=compensation_torque,
                kp=self.follower.kp,
                kd=self.follower.kd,
            )
            _command_follower_gripper(
                self.follower,
                mode=self.gripper_control,
                target=_bounded_gripper_target(
                    leader_state["gripper"],
                    self.gripper_gain,
                    self.gripper_offset,
                ),
                velocity=leader_state["gripper_velocity"],
            )

            slave_state = self.follower.get_state()
            slave_state["timestamp"] = time.time_ns()
            collected_controllers = {
                "master_left_arm": leader_state,
                "slave_left_arm": slave_state,
            }
            self.shared_state.replace(
                {
                    "controller": collected_controllers,
                    "sensor": {},
                }
            )
        except Exception:
            self.end_event.set()
            raise

    def finish(self):
        self.component.shutdown(damped=True)


class DataWorker(Worker):
    def __init__(
        self,
        process_name,
        start_event,
        end_event,
        collect_data_buffer,
        condition,
        collect_event,
        write_event,
        write_result,
        write_done_event,
        episode_id=0,
        resume=False,
        move_check=False,
        enable_cameras=True,
    ):
        super().__init__(process_name, start_event, end_event)
        self.collect_data_buffer = collect_data_buffer
        self.condition = condition
        self.collect_event = collect_event
        self.write_event = write_event
        self.write_result = write_result
        self.write_done_event = write_done_event
        self.episode_id = episode_id
        self.resume = resume
        self.move_check = move_check
        self.enable_cameras = enable_cameras

    def component_init(self):
        self.sensors = {}
        if self.enable_cameras:
            self.sensors = {
                "cam_head": RealsenseSensor("cam_head"),
                "cam_wrist": RealsenseSensor("cam_wrist"),
            }
            self.sensors["cam_head"].set_up(
                CAMERA_SERIALS["head"]
            )
            self.sensors["cam_wrist"].set_up(
                CAMERA_SERIALS["wrist"]
            )
            for sensor in self.sensors.values():
                sensor.set_collect_info(["color"])

        self.category_conditions = {}
        self.next_episode_ids = {}
        for result_name in ("success", "fail"):
            category_condition = _classified_condition(
                self.condition,
                result_name,
            )
            category_collection = CollectAny(
                condition=category_condition,
                start_episode=self.episode_id,
                move_check=self.move_check,
                resume=self.resume,
            )
            self.category_conditions[result_name] = category_condition
            self.next_episode_ids[result_name] = (
                category_collection.episode_index
            )

        self.collection = CollectAny(
            condition=self.category_conditions["success"],
            start_episode=self.episode_id,
            move_check=self.move_check,
            resume=False,
        )

    def _write_classified_episode(self):
        result_name = RESULT_NAMES.get(self.write_result.value)
        if result_name is None:
            raise RuntimeError(
                "Episode result must be success or fail before writing"
            )
        if not self.collection.episode:
            raise RuntimeError("Cannot save an episode with no frames")

        self.collection.condition = self.category_conditions[result_name]
        self.collection.episode_index = self.next_episode_ids[result_name]
        self.collection.write()
        self.next_episode_ids[result_name] = (
            self.collection.episode_index
        )
        self.collection.last_controller_data = None
        self.write_result.value = RESULT_NONE

    def handler(self):
        if self.write_event.is_set():
            try:
                self._write_classified_episode()
            except Exception:
                self.end_event.set()
                raise
            else:
                self.write_event.clear()
                self.write_done_event.set()
            return

        if not self.collect_event.is_set():
            return

        data = self.collect_data_buffer.read()
        if "controller" not in data:
            return

        sensor_data = {}
        for name, sensor in self.sensors.items():
            sensor_data[f"slave_{name}"] = sensor.get()

        self.collection.collect(
            data["controller"],
            sensor_data,
        )

    def finish(self):
        try:
            if self.collection.episode:
                self.collection.episode = []
                self.collection.last_controller_data = None
        finally:
            for sensor in self.sensors.values():
                cleanup = getattr(sensor, "cleanup", None)
                if cleanup is not None:
                    cleanup()


def _check_workers(workers):
    failed = [
        worker.process_name
        for worker in workers
        if not worker.process.is_alive()
    ]
    if failed:
        raise RuntimeError(
            f"Worker exited unexpectedly: {', '.join(failed)}"
        )


def _wait_for_enter(message, workers, key_reader=_read_key):
    print(message)
    while True:
        _check_workers(workers)
        key = key_reader()
        if key in {"\n", "\r"}:
            return
        time.sleep(0.01)


def _wait_for_episode_result(
    message,
    workers,
    key_reader=_read_key,
):
    print(message)
    while True:
        _check_workers(workers)
        key = key_reader()
        if key is not None:
            normalized = key.lower()
            if normalized == "s":
                return "success"
            if normalized == "f":
                return "fail"
        time.sleep(0.01)


def _wait_for_event(event, message, workers):
    print(message)
    while not event.wait(timeout=0.05):
        _check_workers(workers)


def _wait_for_worker_idle(worker, workers, timeout=5):
    deadline = time.monotonic() + timeout
    while not worker.next_event.wait(timeout=0.05):
        _check_workers(workers)
        if time.monotonic() >= deadline:
            raise TimeoutError(
                f"Timed out waiting for {worker.process_name} to finish"
            )
    worker.next_event.clear()


def _stop_scheduler(time_scheduler):
    process = getattr(time_scheduler, "time_locker", None)
    if process is None:
        return
    if process.is_alive():
        process.terminate()
    process.join(timeout=5)
    process.close()
    time_scheduler.time_locker = None


def _join_workers(workers, timeout=30):
    for worker in workers:
        worker.forward_event.set()

    for worker in workers:
        worker.process.join(timeout=timeout)
        if worker.process.is_alive():
            worker.process.terminate()
            worker.process.join(timeout=5)
        worker.process.close()


def run_session(
    success_target,
    initial_success_count,
    initial_fail_count,
    condition,
    control_frequency,
    collect_frequency,
    enable_cameras,
    gripper_control,
    gripper_gain,
    gripper_offset,
):
    start_event = Event()
    end_event = Event()
    collect_event = Event()
    write_event = Event()
    write_result = Value("i", RESULT_NONE)
    write_done_event = Event()
    master_state = MsgpackSharedDict()
    controller_state = MsgpackSharedDict()

    master = MasterWorker(
        "master_arm",
        start_event,
        end_event,
        master_state,
    )
    slave = SlaveWorker(
        "slave_arm",
        start_event,
        end_event,
        master_state,
        controller_state,
        gripper_control=gripper_control,
        gripper_gain=gripper_gain,
        gripper_offset=gripper_offset,
    )

    data = DataWorker(
        "collect_data",
        start_event,
        end_event,
        controller_state,
        condition=condition,
        collect_event=collect_event,
        write_event=write_event,
        write_result=write_result,
        write_done_event=write_done_event,
        episode_id=0,
        resume=True,
        move_check=True,
        enable_cameras=enable_cameras,
    )
    workers = [master, slave, data]

    master.next_to(slave)
    control_scheduler = TimeScheduler(
        work_events=[master.forward_event],
        time_freq=control_frequency,
        end_events=[slave.next_event],
        process_name="panthera_ht_control",
    )
    collection_scheduler = TimeScheduler(
        work_events=[data.forward_event],
        time_freq=collect_frequency,
        end_events=[data.next_event],
        process_name="panthera_ht_collection",
    )

    for worker in workers:
        worker.start()

    control_scheduler_started = False
    collection_scheduler_started = False
    success_count = initial_success_count
    fail_count = initial_fail_count
    attempt_count = 0
    try:
        _wait_for_enter(
            "主从臂对齐后按 Enter 启动遥操作；"
            "采集中按 s=成功，按 f=失败",
            workers,
        )
        start_event.set()
        collect_event.set()
        control_scheduler.start()
        control_scheduler_started = True
        collection_scheduler.start()
        collection_scheduler_started = True

        while success_count < success_target:
            attempt_count += 1
            result_name = _wait_for_episode_result(
                f"第 {attempt_count} 次采集中："
                f"成功 {success_count}/{success_target}，"
                f"失败 {fail_count}；按 s 保存成功，按 f 保存失败",
                workers,
            )

            collect_event.clear()
            _stop_scheduler(collection_scheduler)
            collection_scheduler_started = False
            _wait_for_worker_idle(data, workers)
            write_done_event.clear()
            write_result.value = (
                RESULT_SUCCESS
                if result_name == "success"
                else RESULT_FAIL
            )
            write_event.set()
            data.forward_event.set()
            _wait_for_event(
                write_done_event,
                f"正在保存到 {result_name}/，遥操作保持运行",
                workers,
            )
            if result_name == "success":
                success_count += 1
            else:
                fail_count += 1

            print(
                f"{result_name} 保存完成："
                f"成功 {success_count}/{success_target}，"
                f"失败 {fail_count}"
            )
            if success_count >= success_target:
                break

            _wait_for_enter(
                "本条数据已保存，请复位主从臂；"
                "复位完成后按 Enter 开始下一条采集",
                workers,
            )
            collect_event.set()
            collection_scheduler.start()
            collection_scheduler_started = True
            print("下一条采集已开始")
    finally:
        collect_event.clear()
        if collection_scheduler_started:
            _stop_scheduler(collection_scheduler)
        end_event.set()
        if control_scheduler_started:
            _stop_scheduler(control_scheduler)
        _join_workers(workers, timeout=120)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Panthera HT single-arm teleoperation collection"
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=None,
        help="Deprecated alias for --success-target",
    )
    parser.add_argument(
        "--success-target",
        type=int,
        default=None,
        help=(
            "Stop when this many success episodes exist in "
            "the success folder"
        ),
    )
    parser.add_argument(
        "--control-frequency",
        type=int,
        default=DEFAULT_CONTROL_FREQUENCY,
    )
    parser.add_argument(
        "--collect-frequency",
        type=int,
        default=DEFAULT_COLLECT_FREQUENCY,
    )
    parser.add_argument(
        "--frequency",
        type=int,
        default=None,
        help="Deprecated alias for --collect-frequency",
    )
    parser.add_argument(
        "--gripper-gain",
        type=float,
        default=DEFAULT_GRIPPER_GAIN,
    )
    parser.add_argument(
        "--gripper-offset",
        type=float,
        default=DEFAULT_GRIPPER_OFFSET,
    )
    parser.add_argument(
        "--gripper-control",
        choices=("position", "mit"),
        default=DEFAULT_GRIPPER_CONTROL,
        help="Follower gripper control mode",
    )
    parser.add_argument("--save-path", default="./datasets/")
    parser.add_argument(
        "--task-name",
        default="panthera_ht_teleop_single",
    )
    parser.add_argument(
        "--no-cameras",
        action="store_true",
        help="Run arm teleoperation without RealSense collection",
    )
    args = parser.parse_args()

    if args.episodes is not None and args.success_target is not None:
        parser.error(
            "use either --episodes or --success-target, not both"
        )
    args.success_target = (
        args.success_target
        if args.success_target is not None
        else args.episodes
        if args.episodes is not None
        else DEFAULT_SUCCESS_TARGET
    )
    if args.success_target <= 0:
        parser.error("--success-target must be positive")
    if args.frequency is not None:
        args.collect_frequency = args.frequency
    if not 0 < args.control_frequency <= MAX_CONTROL_FREQUENCY:
        parser.error(
            f"--control-frequency must be between 1 and "
            f"{MAX_CONTROL_FREQUENCY} Hz"
        )
    if not 0 < args.collect_frequency <= MAX_COLLECT_FREQUENCY:
        parser.error(
            f"--collect-frequency must be between 1 and "
            f"{MAX_COLLECT_FREQUENCY} Hz"
        )
    if not np.isfinite(args.gripper_gain) or args.gripper_gain <= 0:
        parser.error("--gripper-gain must be finite and positive")
    if not np.isfinite(args.gripper_offset):
        parser.error("--gripper-offset must be finite")
    return args


def main():
    args = parse_args()
    os.environ.setdefault("INFO_LEVEL", "INFO")

    condition = dict(DEFAULT_CONDITION)
    condition["save_path"] = args.save_path
    condition["task_name"] = args.task_name
    condition["save_freq"] = args.collect_frequency

    initial_success_count = _count_existing_episodes(
        condition,
        "success",
    )
    initial_fail_count = _count_existing_episodes(
        condition,
        "fail",
    )

    print(
        f"Panthera HT 遥操作启动，"
        f"success={initial_success_count}/{args.success_target}, "
        f"fail={initial_fail_count}, "
        f"control={args.control_frequency} Hz, "
        f"collect={args.collect_frequency} Hz, "
        f"gripper={args.gripper_control}"
    )
    if initial_success_count >= args.success_target:
        print("success 文件夹已达到目标数量，无需继续采集")
        return

    try:
        with _single_key_mode():
            run_session(
                success_target=args.success_target,
                initial_success_count=initial_success_count,
                initial_fail_count=initial_fail_count,
                condition=condition,
                control_frequency=args.control_frequency,
                collect_frequency=args.collect_frequency,
                enable_cameras=not args.no_cameras,
                gripper_control=args.gripper_control,
                gripper_gain=args.gripper_gain,
                gripper_offset=args.gripper_offset,
            )
    except KeyboardInterrupt:
        print("\n用户中断遥操作")


if __name__ == "__main__":
    main()
