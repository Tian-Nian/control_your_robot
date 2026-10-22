"""舞肌 Hand 2。UDP 连接后使能，关节指令走 joint_command，不在这里做遥操作。"""

from __future__ import annotations

import importlib
import logging
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

from robot.controller.controller import Controller

_N_JOINTS = 20


def connect_hand(side: str, address: str):
    wuji_sdk = _load_wuji_sdk()
    text = str(address).strip()
    if ":" not in text:
        text = f"{text}:7447"
    host = text.rsplit(":", 1)[0]
    if not _reachable(host):
        raise RuntimeError(
            f"{host} 不通。臂和手不在同一块网卡，先执行 "
            "X-Robot-Pipeline_deploy/scripts/setup_wuji_host_routes.sh apply"
        )
    return wuji_sdk.SdkManager.instance().connect(
        address=text,
        device_name=f"wuji_hand_2_{side}",
    )


def joints_from_frame(frame) -> np.ndarray:
    flat = np.zeros(_N_JOINTS, dtype=np.float64)
    for joint in getattr(frame, "joints", None) or []:
        idx = _stream_nid_to_model(int(joint.nid))
        if 0 <= idx < _N_JOINTS:
            flat[idx] = float(joint.position)
    return flat


class WujiHandController(Controller):
    def __init__(self, name: str = "wuji_hand"):
        super().__init__()
        self.name = name
        self.controller_type = "wuji_hand"
        self._device = None
        self._pub = None
        self._command = None
        self._sub = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._joint: np.ndarray | None = None
        self._goal: np.ndarray | None = None
        self._last_cmd: np.ndarray | None = None
        self._max_step = 0.08
        self._last_enable_t = 0.0

    def set_up(
        self,
        side: str,
        address: str,
        kp: float = 3.0,
        kd: float = 0.1,
        effort_limit: float = 1.5,
        max_joint_step_rad: float = 0.08,
        _connect=None,
    ):
        side = str(side).lower()
        if side not in ("left", "right"):
            raise ValueError(f"side 只能是 left 或 right，得到 {side}")
        self._max_step = max(0.0, float(max_joint_step_rad))
        connect = connect_hand if _connect is None else _connect
        device = connect(side, address)
        self._device = device
        self._command = _load_wuji_sdk().JointCommand
        clear_fault = getattr(device, "clear_fault", None)
        if callable(clear_fault):
            try:
                clear_fault()
            except Exception:
                pass
        device.mit_params().set((float(kp), float(kd)))
        device.effort_limit().set(float(effort_limit))
        device.enable()
        self._last_enable_t = time.monotonic()
        self._pub = device.joint_command().publish()
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name=f"{self.name}_joint_states", daemon=True)
        self._thread.start()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            with self._lock:
                if self._joint is not None:
                    self._goal = self._joint.copy()
                    self._publish_toward_goal()
                    return
            time.sleep(0.01)
        raise RuntimeError(f"{self.name} 没有读到关节")

    def get_information(self):
        self._publish_toward_goal()
        return {"joint": self.get_joint()}

    def get_joint(self) -> np.ndarray:
        with self._lock:
            if self._joint is None:
                raise RuntimeError(f"{self.name} 没有关节反馈")
            return self._joint.copy()

    def move_controller(self, move_data, is_delta=False):
        if "joint" not in move_data:
            return
        target = np.asarray(move_data["joint"], dtype=float).reshape(-1)
        if is_delta:
            target = self.get_joint() + target
        self.set_joint(target)

    def set_joint(self, joint):
        if self._pub is None or self._command is None:
            raise RuntimeError(f"{self.name} 尚未连接")
        target = np.asarray(joint, dtype=float).reshape(-1)
        if target.size != _N_JOINTS:
            raise ValueError(f"{self.name} 需要 {_N_JOINTS} 个关节，得到 {target.size}")
        self._goal = target.copy()
        logging.getLogger(f"cyr.{self.name}").info("关节目标已更新")
        self._publish_toward_goal()

    def _publish_toward_goal(self) -> None:
        if self._pub is None or self._command is None or self._goal is None:
            return
        if getattr(self, "_cyr_safety_stopped", False):
            return
        goal = self._goal
        if self._last_cmd is None:
            command = goal.copy()
        elif self._max_step > 0.0:
            command = self._last_cmd + np.clip(goal - self._last_cmd, -self._max_step, self._max_step)
        else:
            command = goal.copy()
        self._last_cmd = command
        self._ensure_enabled()
        self._pub.send(
            [self._command(position=float(value), velocity=0.0, effort=0.0) for value in command]
        )

    def _ensure_enabled(self) -> None:
        device = self._device
        if device is None:
            return
        now = time.monotonic()
        if now - self._last_enable_t < 1.0:
            return
        device.enable()
        self._last_enable_t = now

    def apply_reset(self, start_state):
        self.interpolate_set_joint(self.get_joint(), start_state)

    def stop(self) -> None:
        self._stop.set()
        self._pub = None
        sub = self._sub
        self._sub = None
        if sub is not None:
            try:
                sub.close()
            except Exception:
                pass
        thread = self._thread
        if thread is not None:
            thread.join(timeout=1.0)
        device = self._device
        self._device = None
        if device is None:
            return
        disable = getattr(device, "disable", None)
        if callable(disable):
            try:
                disable()
            except Exception:
                pass
        device.disconnect()

    def _loop(self) -> None:
        device = self._device
        if device is None:
            return
        try:
            sub = device.joint_states().subscribe()
        except Exception:
            return
        self._sub = sub
        while not self._stop.is_set():
            try:
                frame = sub.recv()
            except Exception:
                break
            if frame is None:
                time.sleep(0.002)
                continue
            joint = joints_from_frame(frame)
            with self._lock:
                self._joint = joint
        try:
            sub.close()
        except Exception:
            pass


def _stream_nid_to_model(nid: int) -> int:
    if nid <= 0 or nid > 24 or nid % 5 == 0:
        return -1
    return 4 * (nid // 5) + (nid % 5) - 1


def _reachable(host: str) -> bool:
    result = subprocess.run(
        ["ping", "-c", "1", "-W", "1", host],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def _load_wuji_sdk():
    site = _deploy_wuji_site()
    if site is not None:
        path = str(site)
        if path not in sys.path:
            sys.path.insert(0, path)
    return importlib.import_module("wuji_sdk")


def _deploy_wuji_site() -> Path | None:
    override = os.environ.get("CYR_WUJI_SITE")
    if override:
        return Path(override)
    prefix = Path(sys.prefix).resolve()
    if prefix.name == "xone_new":
        return None
    minor = f"python{sys.version_info.major}.{sys.version_info.minor}"
    site = prefix.parent / "xone_new" / "lib" / minor / "site-packages"
    if (site / "wuji_sdk").is_dir():
        return site
    return None
