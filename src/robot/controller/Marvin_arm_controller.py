"""天机单臂。同一台控制器只连一次 UDP。默认关节阻抗，也可切到位置模式。"""

from __future__ import annotations

import os
import sys
import threading
import time
from pathlib import Path

import numpy as np

from robot.controller.arm_controller import ArmController

_DOF = 7
_IDLE = 0
_POSITION = 1
_TORQUE = 3
_SWITCH_POS = 101
_ARM_INDEX = {"A": 0, "B": 1}
_SIDE_ARM = {"left": "A", "right": "B"}
_ARM_SIDE = {"A": "left", "B": "right"}
_DEFAULT_K = [16.0, 16.0, 16.0, 8.0, 8.0, 8.0, 8.0]
_DEFAULT_D = [1.8, 1.8, 1.8, 1.8, 1.8, 1.8, 1.8]
# 笛卡尔阻抗的 K 单位：平移 N/m，旋转 N·m/rad。D 是 0~1 的比例。demo 用的就是这组。
_DEFAULT_CART_K = [3000.0, 3000.0, 3000.0, 100.0, 100.0, 100.0, 20.0]
_DEFAULT_CART_D = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
_DEFAULT_FORCE_DIR = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
_DEFAULT_FORCE_ADJ_MM = 50.0
_DEFAULT_TOOL = {
    "left": [0.919, 9.783, 12.470, 91.993, 0.006, 0.0, 0.0, 0.029, 0.0, 0.0],
    "right": [0.930, 6.574, -8.853, 84.767, 0.021, 0.0, 0.0, 0.008, 0.0, 0.0],
}
_DEFAULT_HOME = {
    "left": [2.0933333333, -1.5707963268, -1.5707963268, -2.0933333333, 0.0, 0.0, 0.0],
    "right": [-2.0933333333, -1.5707963268, 1.5707963268, -2.0933333333, 0.0, 0.0, 0.0],
}
_R_UNIFIED_TO_NATIVE = {
    "left": np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]),
    "right": np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]]),
}
_SHARED: dict[str, dict] = {}
_SHARED_LOCK = threading.Lock()


def connect_arms(robot_ip: str):
    robot_cls, dcss_cls = _load_sdk()
    robot = robot_cls()
    if not robot.connect(robot_ip=robot_ip, log_switch=0):
        raise RuntimeError(f"Marvin 连接失败 ip={robot_ip}")
    dcss = dcss_cls()
    for _ in range(5):
        data = robot.subscribe(dcss)
        if data and data.get("outputs"):
            return robot, dcss
    robot.release_robot()
    raise RuntimeError(f"Marvin 没有反馈 ip={robot_ip}")


def acquire_robot(robot_ip: str, connect=None):
    opener = connect_arms if connect is None else connect
    with _SHARED_LOCK:
        entry = _SHARED.get(robot_ip)
        if entry is None:
            robot, dcss = opener(robot_ip)
            entry = {"robot": robot, "dcss": dcss, "refs": 0, "lock": threading.Lock()}
            _SHARED[robot_ip] = entry
        entry["refs"] += 1
        return entry["robot"], entry["dcss"], entry["lock"]


def release_robot(robot_ip: str) -> None:
    robot = None
    with _SHARED_LOCK:
        entry = _SHARED.get(robot_ip)
        if entry is None:
            return
        entry["refs"] -= 1
        if entry["refs"] <= 0:
            robot = entry["robot"]
            _SHARED.pop(robot_ip, None)
    if robot is not None:
        robot.release_robot()


def clear_shared_robots() -> None:
    with _SHARED_LOCK:
        _SHARED.clear()


def joints_from_feedback(data: dict, arm_id: str) -> tuple[np.ndarray, int]:
    idx = _ARM_INDEX[arm_id]
    joint_deg = np.asarray(data["outputs"][idx]["fb_joint_pos"], dtype=float).reshape(-1)
    if joint_deg.size < _DOF:
        raise RuntimeError(f"Marvin {arm_id} 关节长度 {joint_deg.size}")
    mode = int(data["states"][idx]["cur_state"])
    return np.radians(joint_deg[:_DOF]), mode


def flange_pose(native_T_mm, side: str) -> np.ndarray:
    native = np.asarray(native_T_mm, dtype=float).reshape(4, 4).copy()
    native[:3, 3] /= 1000.0
    rotation = _R_UNIFIED_TO_NATIVE[side].T
    unified_r = rotation @ native[:3, :3]
    unified_t = rotation @ native[:3, 3]
    qx, qy, qz, qw = _quat_xyzw(unified_r)
    return np.array([unified_t[0], unified_t[1], unified_t[2], qw, qx, qy, qz], dtype=float)


def normalize_control_mode(mode: str) -> str:
    name = str(mode).strip().lower()
    if name in ("impedance", "imp", "imp_joint", "teleop", "teleop_imp"):
        return "impedance"
    if name in ("position", "pos"):
        return "position"
    if name in ("cartesian", "imp_cart", "cart"):
        return "cartesian"
    if name in ("force", "imp_force"):
        return "force"
    raise ValueError(f"control_mode 只能是 position、impedance、cartesian 或 force，得到 {mode!r}")


def joint_torque_from_feedback(data: dict, arm_id: str) -> np.ndarray | None:
    """SDK 反馈 fb_joint_sToq，关节实际扭矩。没有这一字段时返回 None。"""
    output = data["outputs"][_ARM_INDEX[arm_id]]
    torque = output.get("fb_joint_sToq")
    if torque is None:
        return None
    values = np.asarray(torque, dtype=float).reshape(-1)
    if values.size < _DOF:
        raise RuntimeError(f"Marvin {arm_id} 力矩长度 {values.size}")
    return values[:_DOF].copy()


class MarvinArmController(ArmController):
    supported_modes = ("position", "impedance", "cartesian", "force")

    def __init__(self, name: str = "marvin_arm"):
        super().__init__()
        self.name = name
        self.DoFs = _DOF
        self.side = "left"
        self.arm_id = "A"
        self._robot_ip = ""
        self._robot = None
        self._dcss = None
        self._cmd_lock = None
        self._kine = None
        self._held = False
        self._enabled = False
        self._mode = "impedance"
        self.control_mode = "impedance"
        self._joint_K = list(_DEFAULT_K)
        self._joint_D = list(_DEFAULT_D)
        self._cart_K = list(_DEFAULT_CART_K)
        self._cart_D = list(_DEFAULT_CART_D)
        self._cart_rot = 2
        self._cart_para = [0.0] * _DOF
        self._force_dir = list(_DEFAULT_FORCE_DIR)
        self._force_adj_mm = _DEFAULT_FORCE_ADJ_MM
        self._vel_ratio = 50
        self._acc_ratio = 50
        self._home = np.asarray(_DEFAULT_HOME["left"], dtype=float)

    def set_up(
        self,
        robot_ip: str,
        side: str,
        arm_id: str | None = None,
        control_mode: str = "impedance",
        joint_K: list | None = None,
        joint_D: list | None = None,
        vel_ratio: int = 50,
        acc_ratio: int = 50,
        tool_dyn: list | None = None,
        home_joint: list | None = None,
        cart_K: list | None = None,
        cart_D: list | None = None,
        cart_rot: int = 2,
        cart_ctrl: list | None = None,
        force_dir: list | None = None,
        force_adj_mm: float = _DEFAULT_FORCE_ADJ_MM,
        _connect=None,
        _kine=None,
    ):
        self.side, self.arm_id = _resolve_arm(side, arm_id)
        self._robot_ip = robot_ip
        self._mode = normalize_control_mode(control_mode)
        if self._mode not in self.supported_modes:
            raise ValueError(f"{self.name} 不支持 {control_mode!r}，只有 {list(self.supported_modes)}")
        self.control_mode = self._mode
        self._joint_K = [float(v) for v in (joint_K if joint_K is not None else _DEFAULT_K)]
        self._joint_D = [float(v) for v in (joint_D if joint_D is not None else _DEFAULT_D)]
        self._cart_K = [float(v) for v in (cart_K if cart_K is not None else _DEFAULT_CART_K)]
        self._cart_D = [float(v) for v in (cart_D if cart_D is not None else _DEFAULT_CART_D)]
        self._cart_rot = int(cart_rot)
        self._cart_para = [0.0] * _DOF if cart_ctrl is None else [float(v) for v in cart_ctrl]
        self._force_dir = [float(v) for v in (force_dir if force_dir is not None else _DEFAULT_FORCE_DIR)]
        self._force_adj_mm = float(force_adj_mm)
        self._vel_ratio = int(vel_ratio)
        self._acc_ratio = int(acc_ratio)
        self._home = np.asarray(
            home_joint if home_joint is not None else _DEFAULT_HOME[self.side],
            dtype=float,
        )
        self._robot, self._dcss, self._cmd_lock = acquire_robot(robot_ip, _connect)
        self._held = True
        self._kine = _open_kine(self.name, self.arm_id) if _kine is None else _kine
        self._apply_tool(tool_dyn if tool_dyn is not None else _DEFAULT_TOOL[self.side])
        self._engage(self._mode)

    def get_information(self):
        state = self.get_state()
        info = {
            "joint": state["joint"],
            "eef": state["eef"],
            "mode": state["mode"],
            "control_mode": state["control_mode"],
        }
        if "force" in state:
            info["force"] = state["force"]
        return info

    def get_state(self) -> dict:
        data = self._read()
        joint, mode = joints_from_feedback(data, self.arm_id)
        state = {
            "joint": joint,
            "eef": flange_pose(self._kine.fk(joints=np.degrees(joint).tolist()), self.side),
            "mode": mode,
            "control_mode": self._mode,
        }
        torque = joint_torque_from_feedback(data, self.arm_id)
        if torque is not None:
            state["force"] = torque
        return state

    def move_controller(self, move_data, is_delta=False):
        payload = dict(move_data)
        mode = payload.pop("control_mode", None)
        if mode is not None:
            self.set_control_mode(mode)
        if payload:
            super().move_controller(payload, is_delta)

    def set_joint(self, joint: np.ndarray):
        if not self._enabled:
            raise RuntimeError(f"{self.name} 尚未使能")
        self._command_joint(np.asarray(joint, dtype=float))

    def set_force(self, force: np.ndarray):
        """末端力。SDK 是 set_imp_force_state 加 set_force_cmd，单位 N。

        方向和行程在 set_up 的 force_dir、force_adj_mm。关节力矩没有单独的指令口，
        驱动在扭矩状态里按阻抗自己生成，读回来的是 fb_joint_sToq。
        """
        values = np.asarray(force, dtype=float).reshape(-1)
        if values.size != 1:
            raise ValueError(
                f"{self.name} 的 set_force 是末端力，只要 1 个数，单位 N，得到 {values.size}"
            )
        if not self._enabled or self._mode != "force":
            self.set_control_mode("force")
        ok = self._with_sdk(lambda robot: robot.set_force_cmd(arm=self.arm_id, force=float(values[0])))
        if not ok:
            raise RuntimeError(f"{self.name} 力指令失败")

    def set_control_mode(self, mode: str) -> str:
        target = normalize_control_mode(mode)
        if target not in self.supported_modes:
            raise ValueError(f"{self.name} 不支持 {mode!r}，只有 {list(self.supported_modes)}")
        if self._enabled and target == self._mode:
            self.control_mode = target
            return target
        if target == "position":
            self._switch_position()
        else:
            if self._cur_state() not in (_POSITION, _TORQUE):
                self._switch_position()
            if target == "impedance":
                self._switch_impedance()
            elif target == "cartesian":
                self._switch_cartesian()
            else:
                self._switch_force()
        self._mode = target
        self.control_mode = target
        self._enabled = True
        return target

    def stop(self) -> None:
        robot = self._robot
        self._robot = None
        self._enabled = False
        if robot is not None:
            try:
                joint, _mode = joints_from_feedback(robot.subscribe(self._dcss), self.arm_id)
                robot.set_joint_position_cmd(arm=self.arm_id, joint=np.degrees(joint).tolist())
            except Exception:
                pass
        if self._held:
            self._held = False
            release_robot(self._robot_ip)

    def _engage(self, mode: str) -> None:
        joint, _mode = joints_from_feedback(self._read(), self.arm_id)
        self._command_joint(joint)
        time.sleep(0.1)
        self._disable_to_idle()
        self._switch_position()
        if mode == "impedance":
            self._switch_impedance()
        elif mode == "cartesian":
            self._switch_cartesian()
        elif mode == "force":
            self._switch_force()
        self._enabled = True
        self._mode = mode

    def _command_joint(self, joint: np.ndarray) -> None:
        values = np.asarray(joint, dtype=float).reshape(-1)
        if values.size != _DOF:
            raise ValueError(f"关节需要 {_DOF} 个数，得到 {values.size}")
        ok = self._with_sdk(
            lambda robot: robot.set_joint_position_cmd(arm=self.arm_id, joint=np.degrees(values).tolist())
        )
        if not ok:
            raise RuntimeError(f"{self.name} 关节指令失败")

    def _switch_position(self) -> None:
        ok = self._with_sdk(
            lambda robot: robot.set_position_state(
                arm=self.arm_id,
                velRatio=self._vel_ratio,
                AccRatio=self._acc_ratio,
            )
        )
        if not ok:
            raise RuntimeError(f"{self.name} 切到位置模式失败")
        time.sleep(0.5)
        self._wait_state((_POSITION, _SWITCH_POS))
        if self._cur_state() == _SWITCH_POS:
            self._wait_state(_POSITION)

    def _switch_impedance(self) -> None:
        if len(self._joint_K) != _DOF or len(self._joint_D) != _DOF:
            raise ValueError(f"joint_K/joint_D 需要 {_DOF} 个数")
        ok = self._with_sdk(
            lambda robot: robot.set_imp_joint_state(
                arm=self.arm_id,
                velRatio=self._vel_ratio,
                AccRatio=self._acc_ratio,
                K=list(self._joint_K),
                D=list(self._joint_D),
            )
        )
        if not ok:
            raise RuntimeError(f"{self.name} 切到阻抗模式失败")
        time.sleep(0.5)
        self._wait_state(_TORQUE)

    def _switch_cartesian(self) -> None:
        if len(self._cart_K) != _DOF or len(self._cart_D) != _DOF:
            raise ValueError(f"cart_K/cart_D 需要 {_DOF} 个数")
        if self._cart_rot not in (0, 1, 2):
            raise ValueError(f"cart_rot 只能是 0、1、2，得到 {self._cart_rot}")
        if len(self._cart_para) != _DOF:
            raise ValueError(f"cart_ctrl 需要 {_DOF} 个数，得到 {len(self._cart_para)}")
        ok = self._with_sdk(
            lambda robot: robot.set_imp_cart_state(
                arm=self.arm_id,
                velRatio=self._vel_ratio,
                AccRatio=self._acc_ratio,
                K=list(self._cart_K),
                D=list(self._cart_D),
                rot_type=self._cart_rot,
                cart_ctrl_para=list(self._cart_para),
            )
        )
        if not ok:
            raise RuntimeError(f"{self.name} 切到笛卡尔阻抗失败")
        time.sleep(0.5)
        self._wait_state(_TORQUE)

    def _switch_force(self) -> None:
        if len(self._force_dir) != 6:
            raise ValueError(f"force_dir 需要 6 个数，得到 {len(self._force_dir)}")
        if self._force_adj_mm < 0:
            raise ValueError(f"force_adj_mm 要 >= 0，得到 {self._force_adj_mm}")
        direction = list(self._force_dir)
        limit = float(self._force_adj_mm)
        ok = self._with_sdk(
            lambda robot: robot.set_imp_force_state(arm=self.arm_id, fx_dir=direction, fc_adj_lmt=limit)
        )
        if not ok:
            raise RuntimeError(f"{self.name} 切到力控失败")
        time.sleep(0.5)
        self._wait_state(_TORQUE)

    def _disable_to_idle(self) -> None:
        if self._cur_state() == _IDLE:
            return
        self._wait_low_speed()
        self._with_sdk(lambda robot: robot.disable(arm=self.arm_id))
        time.sleep(0.2)
        try:
            self._wait_state(_IDLE, timeout_s=3.0)
        except RuntimeError:
            pass
        time.sleep(0.5)

    def _apply_tool(self, tool_dyn: list) -> None:
        dyn = [float(v) for v in tool_dyn]
        if len(dyn) == 9:
            dyn.append(0.0)
        dyn = dyn[:10]
        if len(dyn) != 10:
            raise ValueError(f"tool_dyn 需要 10 个数，得到 {len(dyn)}")
        self._with_sdk(
            lambda robot: robot.set_tool(arm=self.arm_id, kine_para=[0.0] * 6, dyn_para=dyn)
        )

    def _with_sdk(self, fn):
        if self._robot is None or self._cmd_lock is None:
            raise RuntimeError(f"{self.name} 尚未连接")
        with self._cmd_lock:
            return fn(self._robot)

    def _read(self) -> dict:
        data = self._with_sdk(lambda robot: robot.subscribe(self._dcss))
        if not data:
            raise RuntimeError(f"{self.name} 反馈为空")
        return data

    def _wait_low_speed(self, timeout_s: float = 3.0) -> bool:
        idx = _ARM_INDEX[self.arm_id]
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            flag = self._read()["outputs"][idx].get("low_speed_flag", 1)
            if isinstance(flag, (bytes, bytearray, list, tuple)):
                stationary = bool(flag) and int(flag[0]) == 1
            else:
                stationary = int(flag) == 1
            if stationary:
                return True
            time.sleep(0.05)
        return False

    def _cur_state(self) -> int:
        _joint, mode = joints_from_feedback(self._read(), self.arm_id)
        return mode

    def _wait_state(self, targets, timeout_s: float = 3.0) -> int:
        if isinstance(targets, int):
            targets = (targets,)
        deadline = time.monotonic() + timeout_s
        last = -1
        while time.monotonic() < deadline:
            last = self._cur_state()
            if last in targets:
                return last
            time.sleep(0.05)
        raise RuntimeError(f"{self.name} 等待状态 {targets} 超时，当前 {last}")


def _resolve_arm(side: str, arm_id: str | None) -> tuple[str, str]:
    resolved_side = str(side).lower()
    if resolved_side not in _SIDE_ARM:
        raise ValueError(f"side 只能是 left 或 right，得到 {side!r}")
    resolved_id = _SIDE_ARM[resolved_side] if arm_id is None else str(arm_id).upper()
    if resolved_id not in _ARM_INDEX:
        raise ValueError(f"arm_id 只能是 A 或 B，得到 {arm_id!r}")
    if _ARM_SIDE[resolved_id] != resolved_side:
        raise ValueError(f"{resolved_side} 臂对应 {_SIDE_ARM[resolved_side]}，得到 {resolved_id}")
    return resolved_side, resolved_id


def _open_kine(name: str, arm_id: str):
    _load_sdk()
    from SDK_PYTHON.fx_kine import Marvin_Kine

    cfg = _sdk_root() / "CommonConfig" / "ccs_m6_40.MvKDCfg"
    if not cfg.is_file():
        raise FileNotFoundError(f"{name}: 找不到运动学配置 {cfg}")
    kine = Marvin_Kine()
    kine.log_switch(0)
    loaded = kine.load_config(arm_type=0 if arm_id == "A" else 1, config_path=str(cfg))
    if not loaded:
        raise RuntimeError(f"{name}: 运动学配置加载失败")
    ready = kine.initial_kine(
        robot_type=loaded["TYPE"][0],
        dh=loaded["DH"][0],
        pnva=loaded["PNVA"][0],
        j67=loaded["BD"][0],
    )
    if not ready:
        raise RuntimeError(f"{name}: 运动学初始化失败")
    return kine


def _quat_xyzw(rotation: np.ndarray) -> tuple[float, float, float, float]:
    matrix = np.asarray(rotation, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        qw = (matrix[2, 1] - matrix[1, 2]) / scale
        qx = 0.25 * scale
        qy = (matrix[0, 1] + matrix[1, 0]) / scale
        qz = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        qw = (matrix[0, 2] - matrix[2, 0]) / scale
        qx = (matrix[0, 1] + matrix[1, 0]) / scale
        qy = 0.25 * scale
        qz = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        qw = (matrix[1, 0] - matrix[0, 1]) / scale
        qx = (matrix[0, 2] + matrix[2, 0]) / scale
        qy = (matrix[1, 2] + matrix[2, 1]) / scale
        qz = 0.25 * scale
    return float(qx), float(qy), float(qz), float(qw)


def _load_sdk():
    root = _sdk_root()
    python_dir = root / "SDK_PYTHON"
    if not python_dir.is_dir():
        raise ImportError(f"找不到 Marvin SDK: {python_dir}")
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    library = str(python_dir)
    current = os.environ.get("LD_LIBRARY_PATH", "")
    if library not in current.split(os.pathsep):
        os.environ["LD_LIBRARY_PATH"] = library + (os.pathsep + current if current else "")
    from SDK_PYTHON.fx_robot import Concise_Marvin_Robot, DCSS

    return Concise_Marvin_Robot, DCSS


def _sdk_root() -> Path:
    env = os.environ.get("MARVIN_SDK_ROOT")
    if env:
        return Path(env)
    return (
        Path(__file__).resolve().parents[4]
        / "X-Robot-Pipeline_deploy"
        / "third_party"
        / "TJ_FX_ROBOT_CONTRL_SDK"
    )
