from typing import List, Dict, Any
from abc import ABC, abstractmethod
import numpy as np
import time

from robot.utils.base.data_handler import debug_print

class Controller(ABC):
    SMOOTHERSTEP_PEAK = 15.0 / 8.0
    _RESET_HZ = 50.0
    _RESET_MIN_DURATION_S = 0.8
    _RESET_MAX_JOINT_SPEED_RAD_S = 1.7
    _RESET_WARN_DURATION_S = 3.0

    def __init__(self, timestamp=True):
        self.name = "controller"
        self.controller_type = "base_controller"
        # self.is_set_up = False
        self.timestamp = timestamp
        self.collect_info = None
    
    def set_collect_info(self, collect_info:List[str]):
        self.collect_info = collect_info
        if self.timestamp:
           self.collect_info.append("timestamp")

    # get controller infomation
    def get(self):
        if self.collect_info is None:
            raise ValueError(f"{self.name}: collect_info is not set")
        info = self.get_information()

        if self.timestamp:
            info["timestamp"] = time.time_ns()
        
        for collect_info in self.collect_info:
            if info[collect_info] is None:
                debug_print(f"{self.name}", f"{collect_info} information is None", "ERROR")
        
        debug_print(f"{self.name}", f"get data:\n{info} ", "DEBUG")
        return {collect_info: info[collect_info] for collect_info in self.collect_info}

    def move(self, move_data, is_delta=False):
        if getattr(self, "_cyr_safety_stopped", False):
            debug_print(self.name, "安全保护已触发，丢弃运动命令", "WARNING")
            return
        debug_print(f"{self.name}", f"get move data:\n{move_data} ", "DEBUG")
        try:
            self.move_controller(move_data, is_delta)
        except Exception as e:
            debug_print(self.name, f"move error: {e}", "WARNING")
            self.safe_stop()
            if getattr(self, "_cyr_raise_move_errors", False):
                raise

    def reset(self, start_state=None):
        """回到零位，或回到传入的 start_state。安全保护触发后不再动作。"""
        if getattr(self, "_cyr_safety_stopped", False):
            debug_print(self.name, "安全保护已触发，拒绝复位", "WARNING")
            return
        return self.apply_reset(start_state)

    @staticmethod
    def smootherstep(u: float) -> float:
        """两端速度为零：u^3 (6u^2 - 15u + 10)。"""
        u = float(np.clip(u, 0.0, 1.0))
        return u * u * u * (u * (u * 6.0 - 15.0) + 10.0)

    @classmethod
    def iter_smootherstep(cls, duration: float, hz: float):
        """按绝对时间给出 smootherstep 采样。"""
        duration = float(duration)
        hz = float(hz)
        if not np.isfinite(duration) or duration < 0.0:
            raise ValueError("轨迹时长必须是有限的非负数")
        if not np.isfinite(hz) or hz <= 0.0:
            raise ValueError("轨迹频率必须是有限的正数")
        n_steps = max(int(np.ceil(duration * hz)), 1)
        started = time.monotonic()
        for step in range(1, n_steps + 1):
            remaining = started + step * duration / n_steps - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            yield cls.smootherstep(step / n_steps)

    @classmethod
    def smootherstep_joint_duration(cls, start_joint, target_joint, *, max_joint_speed: float) -> float:
        """按 smootherstep 的峰值速度，给出最短时长。"""
        start = np.asarray(start_joint, dtype=float).reshape(-1)
        target = np.asarray(target_joint, dtype=float).reshape(-1)
        max_joint_speed = float(max_joint_speed)
        if start.size == 0 or start.shape != target.shape:
            raise ValueError("复位的起点和终点关节长度必须相同，且不能为空")
        if not np.all(np.isfinite(start)) or not np.all(np.isfinite(target)):
            raise ValueError("复位关节必须是有限数")
        if not np.isfinite(max_joint_speed) or max_joint_speed <= 0.0:
            raise ValueError("复位关节速度必须是有限的正数")
        max_delta = float(np.max(np.abs(target - start)))
        return max_delta * cls.SMOOTHERSTEP_PEAK / max_joint_speed

    def execute_joint_smootherstep(self, start_joint, target_joint, *, duration: float, hz: float) -> None:
        """整段关节轨迹都通过 set_joint 下发。"""
        start = np.asarray(start_joint, dtype=float).reshape(-1)
        target = np.asarray(target_joint, dtype=float).reshape(-1)
        if start.size == 0 or start.shape != target.shape:
            raise ValueError("复位的起点和终点关节长度必须相同，且不能为空")
        if not np.all(np.isfinite(start)) or not np.all(np.isfinite(target)):
            raise ValueError("复位关节必须是有限数")
        delta = target - start
        for alpha in self.iter_smootherstep(duration, hz):
            if getattr(self, "_cyr_safety_stopped", False):
                return
            self.set_joint(start + alpha * delta)

    def interpolate_set_joint(self, current, start_state) -> None:
        """从当前关节平滑走到目标。没给目标时用 _home、_init_pose，再不行就用零位。"""
        start = np.asarray(current, dtype=float).reshape(-1)
        if start_state is None:
            home = getattr(self, "_home", None)
            if home is None:
                home = getattr(self, "_init_pose", None)
            target = np.zeros(start.size) if home is None else np.asarray(home, dtype=float).reshape(-1)
        else:
            target = np.asarray(start_state, dtype=float).reshape(-1)
        if float(np.max(np.abs(target - start))) < 1e-6:
            self.set_joint(target)
            return
        duration = self.smootherstep_joint_duration(
            start,
            target,
            max_joint_speed=self._RESET_MAX_JOINT_SPEED_RAD_S,
        )
        duration = max(self._RESET_MIN_DURATION_S, duration)
        if duration > self._RESET_WARN_DURATION_S:
            delta = float(np.max(np.abs(target - start)))
            debug_print(self.name, f"复位轨迹较长：{duration:.2f}s，最大关节差 {delta:.3f} rad", "WARNING")
        self.execute_joint_smootherstep(start, target, duration=duration, hz=self._RESET_HZ)

    @abstractmethod
    def apply_reset(self, start_state):
        """具体设备的复位动作。"""

    def safe_stop(self) -> str:
        """停下这个设备。已经停过就直接返回。

        设备自己有 stop 就调用。底盘把速度清零。
        没有这些接口时保持当前位置，并拒绝后续 move。
        """
        if getattr(self, "_cyr_safety_stopped", False):
            return "already"
        self._cyr_safety_stopped = True
        stop = getattr(self.__class__, "stop", None)
        if stop is not None:
            stop(self)
            return "stop"
        setter = getattr(self, "set_move_velocity", None)
        if callable(setter):
            setter(np.zeros(6))
            return "zero_velocity"
        return "hold"
    
    @abstractmethod
    def get_information(self) -> Dict[str, Any]:
        """Get raw information from the controller."""
        pass

    @abstractmethod
    def move_controller(self, move_data: Dict[str, Any], is_delta: bool = False):
        """Execute movement commands."""
        pass

   # init controller
    @abstractmethod
    def set_up(self):
        """Initialize the controller."""
        pass
    
    # print controller
    def __repr__(self):
        return f"{self.name}: {self.controller_type}"
        return f"Base Controller, can't be used directly \n \
                name: {self.name} \n \
                controller_type: {self.controller_type}"
