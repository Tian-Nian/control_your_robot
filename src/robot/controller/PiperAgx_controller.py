import sys
import threading
import json
import os
from importlib import import_module
from pathlib import Path

import numpy as np
import time
from scipy.spatial.transform import Rotation as R

from robot.controller.arm_controller import ArmController
from robot.controller.piper_gravity_config import (
    PiperTeleopGravityConfig,
    compute_piper_teleop_torque_terms,
    normalize_piper_joint_vector,
)

from robot.utils.base.data_handler import is_enter_pressed, debug_print

from agx_pinocchio import AgxPinocchio

from pyAgxArm import create_agx_arm_config, AgxArmFactory, PiperFW, ArmModel

THIRD_PARTY_PATH = Path(__file__).parent.parent.parent.parent.parent.parent / "third_party"

'''
Piper base code from:
https://github.com/agilexrobotics/pyAgxArm.git
'''

def normalize_piper_firmware_version(firmware_version=PiperFW.V183):
    # SDK 的固件分支会影响 MIT 力矩通道编码，脚本层传字符串时统一在这里收口。
    if firmware_version is None:
        return PiperFW.V183

    normalized = str(firmware_version).strip().lower()
    supported_versions = {
        PiperFW.DEFAULT: PiperFW.DEFAULT,
        PiperFW.V183: PiperFW.V183,
        PiperFW.V188: PiperFW.V188,
    }
    if normalized in supported_versions:
        return supported_versions[normalized]

    raise ValueError(
        f"Unsupported Piper firmware version: {firmware_version!r}. "
        f"Expected one of {sorted(supported_versions)}."
    )


# 基于常驻线程的重力补偿遥操控制
def run_teleop(controller, teleop_event, stop_event, urdf_path):
    pin = AgxPinocchio(str(urdf_path))

    control_frequency = 200.0
    joint_nums = controller.controller.joint_nums
    gravity_config = getattr(controller, "teleop_gravity_config", None)
    if gravity_config is None:
        gravity_config = PiperTeleopGravityConfig.for_arm_urdf(
            getattr(controller, "arm_type", ArmModel.PIPER),
            urdf_path,
            env=os.environ,
        )
    mit_kd = normalize_piper_joint_vector(gravity_config.mit_kd, "teleop_mit_kd", joint_nums)

    while not stop_event.is_set():
        feedback = controller.get_cached_feedback()
        if feedback is not None:
            joint_angles = feedback["joint"]
            break
        time.sleep(0.01)

    # 计算世界坐标系到基座坐标系的旋转矩阵
    roll, pitch, yaw = 0, 0, 0
    R_world_base = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_matrix()

    try:
        while not stop_event.is_set():            
            if not teleop_event.is_set():
                time.sleep(0.01)
                continue
            
            start_time = time.time()

            feedback = controller.get_cached_feedback(max_age_s=0.2)
            if feedback is None:
                time.sleep(0.01)
                continue

            joint_angles = feedback["joint"]
            joint_velocities = feedback["joint_velocities"]

            gravity_torque = pin.gravity_compensation(
                joint_angles,
                joint_velocities,
                R_world_base,
                include_velocity_bias=False,
            )
            torque_terms = compute_piper_teleop_torque_terms(gravity_torque, gravity_config)
            torque_ff = torque_terms["gravity_ff"]

            try:
                for joint_id in range(1, controller.controller.joint_nums + 1):
                    joint_idx = joint_id - 1
                    actual_torque = torque_ff[joint_idx]
                    actual_kd = mit_kd[joint_idx]
                    # 小 kd 走 MIT 速度阻尼通道，t_ff 保持纯重力前馈，避免把阻尼重复叠进力矩项。
                    controller.controller.move_mit(joint_id, 0, 0, 0, actual_kd, actual_torque)

            except Exception as e:
                print(f"应用重力补偿失败: {e}")

            t = 1.0 / control_frequency
            elapsed_time = time.time() - start_time
            if elapsed_time < t:
                time.sleep(t - elapsed_time)
            else:
                print(f"警告：控制循环超时 {elapsed_time:.3f}s > {t:.3f}s")

    except KeyboardInterrupt:
        print("\n用户中断, 停止重力补偿")
        for joint_id in range(1, controller.controller.joint_nums + 1):
            try:
                controller.controller.move_mit(joint_id, joint_angles[joint_id - 1], 0, 10, 0.8, 0)
            except Exception:
                pass

    except Exception as e:
        print(f"程序运行出错: {e}")
        for joint_id in range(1, controller.controller.joint_nums + 1):
            try:
                controller.controller.move_mit(joint_id, joint_angles[joint_id - 1], 0, 10, 0.8, 0)
            except Exception:
                pass


class PiperAgxController(ArmController):
    def __init__(self, name):
        super().__init__()
        self.name = name
        self.controller_type = "user_controller"
        self.controller = None
        self.end_effector = None
        self.fk_solver = None
        self.teleop = False
        self._teleop_thread = None
        self._teleop_event = None
        self._teleop_stop_event = None
        self._debug_lock = threading.Lock()
        self._debug_dir = None
        self._gripper_debug_path = None
        self._feedback_lock = threading.Lock()
        self._cached_joint = None
        self._cached_joint_velocity = None
        self._cached_feedback_ns = 0
        self._last_joint_command = None
        self._last_joint_command_ns = 0
        self._last_gripper_command = None
        self._last_gripper_command_ns = 0
        self._joint_deadband = float(os.environ.get("XONE_PIPER_JOINT_DEADBAND", "0.0001"))
        self._gripper_deadband = float(os.environ.get("XONE_PIPER_GRIPPER_DEADBAND", "0.02"))
        self._joint_min_interval_ns = int(float(os.environ.get("XONE_PIPER_JOINT_MIN_INTERVAL_MS", "8.0")) * 1_000_000)
        self._gripper_min_interval_ns = int(float(os.environ.get("XONE_PIPER_GRIPPER_MIN_INTERVAL_MS", "20.0")) * 1_000_000)
        self._joint_track_log_every = max(1, int(os.environ.get("XONE_PIPER_TRACK_LOG_EVERY", "200")))
        self._joint_track_warn_error = float(os.environ.get("XONE_PIPER_TRACK_WARN_ERROR", "0.03"))
        self._gripper_track_warn_error = float(os.environ.get("XONE_PIPER_GRIPPER_TRACK_WARN_ERROR", "0.2"))
        self.teleop_gravity_config = None
        self._joint_feedback_counter = 0
        self._gripper_feedback_counter = 0

    def _setup_debug_loggers(self):
        debug_dir = os.environ.get("XONE_PIPER_DEBUG_DIR")
        if not debug_dir:
            return None

        os.makedirs(debug_dir, exist_ok=True)
        self._debug_dir = debug_dir
        self._gripper_debug_path = os.path.join(debug_dir, f"{self.name}_gripper_trace.jsonl")
        self._append_debug_jsonl(
            self._gripper_debug_path,
            {
                "type": "header",
                "controller": self.name,
                "timestamp": time.time(),
            },
        )
        return None

    def _close_debug_loggers(self):
        self._debug_dir = None
        self._gripper_debug_path = None
        return None

    def _append_debug_jsonl(self, path, record):
        if path is None:
            return

        with self._debug_lock:
            with open(path, "a", encoding="utf-8") as handle:
                handle.write(json.dumps(record, ensure_ascii=False) + "\n")

    def _log_debug_event(self, *args, **kwargs):
        return None

    def _log_joint_target_update(self, *args, **kwargs):
        return None

    def _log_joint_command_send(self, *args, **kwargs):
        return None

    def _log_gripper_target_update(self, *args, **kwargs):
        if self._gripper_debug_path is None:
            return None

        record = {
            "type": "gripper_target_update",
            "controller": self.name,
            "timestamp": time.time(),
            "monotonic_ns": time.monotonic_ns(),
        }
        record.update(kwargs)
        self._append_debug_jsonl(self._gripper_debug_path, record)
        return None

    def _log_gripper_command_send(self, *args, **kwargs):
        if self._gripper_debug_path is None:
            return None

        record = {
            "type": "gripper_command_send",
            "controller": self.name,
            "timestamp": time.time(),
            "monotonic_ns": time.monotonic_ns(),
        }
        record.update(kwargs)
        self._append_debug_jsonl(self._gripper_debug_path, record)
        return None

    def _wait_for_message(self, getter, timeout=1.0, interval=0.01, description="message"):
        start_time = time.monotonic()
        last_value = None

        while time.monotonic() - start_time < timeout:
            last_value = getter()
            if last_value is not None:
                return last_value
            time.sleep(interval)

        raise RuntimeError(f"Timed out waiting for Piper {description}.")

    def _collect_enable_diagnostics(self):
        arm_status = self.controller.get_arm_status()
        driver_states = [self.controller.get_driver_states(i) for i in range(1, 7)]

        return {
            "arm_ok": self.controller.is_ok(),
            "arm_fps": self.controller.get_fps(),
            "gripper_ok": self.end_effector.is_ok(),
            "gripper_fps": self.end_effector.get_fps(),
            "arm_status_ready": arm_status is not None,
            "driver_states_ready": [state is not None for state in driver_states],
            "joint_enable_status": self.controller.get_joints_enable_status_list(),
        }

    def _wait_until_enabled(self, timeout=3.0, interval=0.1):
        start_time = time.monotonic()
        last_diagnostics = self._collect_enable_diagnostics()

        while time.monotonic() - start_time < timeout:
            if self.controller.enable():
                return

            time.sleep(interval)
            last_diagnostics = self._collect_enable_diagnostics()

        raise RuntimeError(
            f"Failed to enable Piper on CAN channel {self.controller.get_channel()} within {timeout:.1f}s. "
            f"Diagnostics: {last_diagnostics}. "
            "This usually means the CAN device is up but the Piper arm is not returning valid arm/driver feedback. "
            "Check arm power, emergency stop state, CAN cabling, and whether can_left/can_right is mapped to the correct USB-CAN adapter."
        )

    def _start_teleop_thread(self):
        if self._teleop_thread is not None and self._teleop_thread.is_alive():
            return

        self._teleop_event = threading.Event()
        self._teleop_stop_event = threading.Event()

        urdf_path = self.urdf_path
        self._teleop_thread = threading.Thread(
            target=run_teleop,
            args=(self, self._teleop_event, self._teleop_stop_event, urdf_path),
            daemon=True,
        )
        self._teleop_thread.start()

    def _update_feedback_cache(self, joint):
        joint = np.asarray(joint, dtype=float)
        now_ns = time.monotonic_ns()
        joint_velocity = np.zeros_like(joint)

        with self._feedback_lock:
            if self._cached_joint is not None and self._cached_feedback_ns > 0:
                dt = (now_ns - self._cached_feedback_ns) / 1_000_000_000
                if dt > 1e-6:
                    joint_velocity = (joint - self._cached_joint) / dt
                elif self._cached_joint_velocity is not None:
                    joint_velocity = self._cached_joint_velocity.copy()

            self._cached_joint = joint.copy()
            self._cached_joint_velocity = joint_velocity.copy()
            self._cached_feedback_ns = now_ns

    def get_cached_feedback(self, max_age_s=None):
        with self._feedback_lock:
            if self._cached_joint is None or self._cached_feedback_ns <= 0:
                return None

            age_s = (time.monotonic_ns() - self._cached_feedback_ns) / 1_000_000_000
            if max_age_s is not None and age_s > max_age_s:
                return None

            return {
                "joint": self._cached_joint.copy(),
                "joint_velocities": self._cached_joint_velocity.copy() if self._cached_joint_velocity is not None else np.zeros_like(self._cached_joint),
                "age_s": age_s,
            }

    def _set_teleop_enabled(self, enabled):
        if self._teleop_event is None:
            return

        if enabled:
            self._teleop_event.set()
        else:
            self._teleop_event.clear()

    def _stop_teleop_thread(self):
        if self._teleop_stop_event is not None:
            self._teleop_stop_event.set()

        if self._teleop_thread is not None:
            self._teleop_thread.join(timeout=1.0)

        self._teleop_thread = None
        self._teleop_event = None
        self._teleop_stop_event = None

    def set_up(self, can: str, arm_type=ArmModel.PIPER, teleop=False, firmware_version=PiperFW.V183):
        self._setup_debug_loggers()
        firmware_version = normalize_piper_firmware_version(firmware_version)

        if arm_type == ArmModel.PIPER_X or arm_type == "piper_x":
            self.urdf_path = Path(THIRD_PARTY_PATH) / "agilex-arm-gravity-compensation" / "piper_x_description" / "urdf" / "piper_x_description_with_teach.urdf"
            arm_type = ArmModel.PIPER_X
        elif arm_type == "piper" or arm_type == ArmModel.PIPER:
            self.urdf_path = Path(THIRD_PARTY_PATH) / "agilex-arm-gravity-compensation" / "piper_description" / "urdf" / "piper_description_with_teach.urdf"
            arm_type = ArmModel.PIPER
        else:
            raise ValueError(f"Unsupported arm type: {arm_type}")
        self.arm_type = arm_type
        self.teleop_gravity_config = PiperTeleopGravityConfig.for_arm_urdf(
            arm_type,
            self.urdf_path,
            env=os.environ,
        )

        cfg = create_agx_arm_config(
            robot=arm_type,
            firmeware_version=firmware_version,
            interface="socketcan",
            channel=can,
        )
        self.controller = AgxArmFactory.create_arm(cfg)
        self.end_effector = self.controller.init_effector(self.controller.OPTIONS.EFFECTOR.AGX_GRIPPER)

        self.controller.connect()

        self.controller.set_follower_mode()

        self._wait_until_enabled()

        self.controller.set_motion_mode()

        self._start_teleop_thread()

        self.change_mode(teleop=teleop)

    def get_state(self):
        state = {}

        joint_msg = self._wait_for_message(
            self.controller.get_joint_angles,
            description="joint angles",
        )
        joint = joint_msg.msg

        eef = self.controller.get_flange_pose().msg

        gripper_msg = self._wait_for_message(
            self.end_effector.get_gripper_status,
            description="gripper status",
        )
        gripper_value = getattr(gripper_msg.msg, "value", None)
        gripper_width = getattr(gripper_msg.msg, "width", None)
        gripper = gripper_value

        if gripper is None and gripper_width is not None:
            gripper = gripper_width

        state["joint"] = np.array(joint)
        state["eef"] = np.array(eef)
        state["gripper"] = gripper * 10.
        self._update_feedback_cache(state["joint"])
        self._maybe_log_joint_tracking(state["joint"])
        self._maybe_log_gripper_tracking(state["gripper"])
        self._log_gripper_target_update(
            source="get_state",
            teleop=self.teleop,
            raw_value=gripper_value,
            raw_width=gripper_width,
            scaled_gripper=state["gripper"],
        )
        return state

    def get_teleop_state(self):
        state = self.get_state()
        return {
            "joint": state["joint"],
            "gripper": state["gripper"],
        }

    def set_position(self, position):
        self.controller.move_p(position.tolist())

    def _maybe_log_joint_tracking(self, actual_joint):
        if self.teleop:
            return

        if self._last_joint_command is None:
            return

        self._joint_feedback_counter += 1
        joint_error = float(np.max(np.abs(np.asarray(actual_joint, dtype=float) - self._last_joint_command)))
        command_age_ms = (time.monotonic_ns() - self._last_joint_command_ns) / 1_000_000
        if joint_error >= self._joint_track_warn_error or self._joint_feedback_counter % self._joint_track_log_every == 0:
            level = "WARNING" if joint_error >= self._joint_track_warn_error else "DEBUG"
            debug_print(
                self.name,
                f"JOINT_TRACKING error_max={joint_error:.6f} age_ms={command_age_ms:.3f}",
                level,
            )

    def _maybe_log_gripper_tracking(self, actual_gripper):
        if self.teleop:
            return

        if self._last_gripper_command is None:
            return

        self._gripper_feedback_counter += 1
        actual_width = float(actual_gripper) / 10.0
        gripper_error = abs(actual_width - self._last_gripper_command)
        command_age_ms = (time.monotonic_ns() - self._last_gripper_command_ns) / 1_000_000
        if gripper_error >= self._gripper_track_warn_error or self._gripper_feedback_counter % self._joint_track_log_every == 0:
            level = "WARNING" if gripper_error >= self._gripper_track_warn_error else "DEBUG"
            debug_print(
                self.name,
                f"GRIPPER_TRACKING error={gripper_error:.6f} age_ms={command_age_ms:.3f}",
                level,
            )

    def set_joint(self, joint):
        joint = np.asarray(joint, dtype=float)
        self.controller.move_js(joint.tolist())
        return

    def set_gripper(self, gripper):
        target_width = gripper / 10
        now_ns = time.monotonic_ns()
        self.end_effector.move_gripper(target_width)

        if self._last_gripper_command is not None:
            gripper_delta = abs(float(target_width) - self._last_gripper_command)
            # if gripper_delta < self._gripper_deadband and now_ns - self._last_gripper_command_ns < self._gripper_min_interval_ns: # 限制微小变化和高频率命令
            if now_ns - self._last_gripper_command_ns < self._gripper_min_interval_ns: # 只限制高频率命令，允许微小变化cd 
                return

        self._log_gripper_target_update(
            source="set_gripper",
            teleop=self.teleop,
            requested_gripper=gripper,
            command_width=target_width,
        )
        self.end_effector.move_gripper(target_width)
        self._last_gripper_command = float(target_width)
        self._last_gripper_command_ns = now_ns
        self._log_gripper_command_send(
            source="set_gripper",
            teleop=self.teleop,
            requested_gripper=gripper,
            command_width=target_width,
        )

    def change_mode(self, teleop):
        if self.teleop == teleop:
            return

        self.teleop = teleop
        if teleop:
            self._set_teleop_enabled(True)
            self.end_effector.disable_gripper() # 夹爪力矩清空
        else:
            self._set_teleop_enabled(False)

    def reset(self, joint):
        joint = np.asarray(joint, dtype=float)
        self.controller.move_js(joint.tolist())

    def __del__(self):
        try:
            self._close_debug_loggers()
            self._stop_teleop_thread()
            if hasattr(self, "controller"):
                pass
        except Exception:
            pass


if __name__ == "__main__":
    controller = PiperController("test_piper")
    controller.set_up("can_left", arm_type=ArmModel.PIPER_X,teleop=False)
    controller.set_collect_info(["joint", "eef", "gripper"])

    print(controller.get())
    time.sleep(10)
    move_data = {
        "joint": np.array([0., 0., 0., 0., 0., 0.]),
        "gripper": 1.,
    }

    controller.move(move_data)
    time.sleep(2)

    controller.change_mode(teleop=True)

    while not is_enter_pressed():
        print(controller.get())
        time.sleep(1)

    controller.change_mode(teleop=False)

    controller.move(move_data)

    time.sleep(1)
