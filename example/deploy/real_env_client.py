import argparse
import copy
import inspect
import os
import sys
import time
from collections.abc import Mapping
from datetime import datetime
from typing import Any

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

_deploy_dir = os.path.dirname(os.path.abspath(__file__))
if _deploy_dir not in sys.path:
    sys.path.insert(0, _deploy_dir)

from robot_client_server.model_client import ModelClient
from XPolicyLab.utils.process_data import get_robot_action_dim_info
from robot.robot import get_robot
from robot.data.collect_any import CollectAny


def _encode_color_if_needed(color, is_jpeg: bool):
    """Return JPEG bytes when is_jpeg is enabled, otherwise keep the original value."""
    if not is_jpeg or color is None:
        return color
    if isinstance(color, (bytes, bytearray)):
        return bytes(color)
    success, encoded_image = cv2.imencode(".jpg", color)
    if success:
        return encoded_image.tobytes()
    return None


class _PreviewFrameHub:
    """Latest JPEG frame per camera serial (for real-robot preview daemon only)."""

    _JPEG_MAGIC = b"\xff\xd8"

    def __init__(self):
        import threading

        self._cond = threading.Condition()
        self._frames: dict[str, tuple[bytes, int]] = {}

    def publish(self, serial: str, jpeg_bytes: bytes, timestamp_ns: int) -> None:
        if not serial or not jpeg_bytes or not jpeg_bytes.startswith(self._JPEG_MAGIC):
            return
        with self._cond:
            self._frames[serial] = (jpeg_bytes, int(timestamp_ns))
            self._cond.notify_all()

    def latest(self, serial: str) -> tuple[bytes, int] | None:
        with self._cond:
            return self._frames.get(serial)

    def wait_for_frame(
        self,
        serial: str,
        after_timestamp_ns: int = 0,
        timeout_s: float = 1.0,
    ) -> tuple[bytes, int] | None:
        deadline = time.monotonic() + timeout_s
        with self._cond:
            while True:
                item = self._frames.get(serial)
                if item is not None and item[1] > after_timestamp_ns:
                    return item
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return item
                self._cond.wait(timeout=min(remaining, 0.1))

    def clear(self) -> None:
        with self._cond:
            self._frames.clear()
            self._cond.notify_all()


_PREVIEW_FRAME_HUB = _PreviewFrameHub()

Batch_Size = 1

# Real-env-only mappings (do not edit env_cfg / XPolicyLab).
# arx_x5: client input type -> config/{name}.yml runtime robot config.
# ENV_CFG_ACTION_DIM_ALIAS: align action-dim lookup with policy server (dual_x5).
ENV_CFG_TO_BASE_CFG = {
    "replay_robot": "replay_robot",
}
ENV_CFG_ACTION_DIM_ALIAS = {
    "arx_x5": "dual_x5",
}
GRIPPER_BIAS = {
    "left_arm": {
        "gripper": -0.03,
    },
    "right_arm": {
        "gripper": -0.03,
    },
}

# Real-robot episode length per task (steps).
# Limits = ceil(Q90 * 1.5 / 10) * 10 from RoboDojo real-robot episode stats.
TASK_STEP_LIMIT = {
    # piper x
    "disassemble_LEGO": 1300,
    "cap_pen": 970,
    "classify_objects": 1580,
    "hang_mugs": 1640,
    "sweep_blocks": 780,
    "pack_objects_into_backpack": 1220,
    # piper
    "fill_pen_holder": 1600,
    "put_objects_into_basket": 1540,
    "stack_and_cover_blocks": 1670,
    "stack_bowls": 1220,
    "stand_up_bottles": 1190,
    "insert_charger": 2240,
    # arx
    "insert_tubes": 1840,
    "make_bread": 2140,
    "make_food": 2770,
    "pack_and_pour_fruit": 1540,
    "cover_blocks": 2070,
    "store_in_safe": 1280,
}

# Real-robot language instruction per task (English).
TASK_INSTRUCTION = {
    # piper x
    "disassemble_LEGO": (
        "Take apart the assembled LEGO structure and place all the pieces into the bowl."
    ),
    "cap_pen": "Pick up the pen and the pen cap, then put the cap on the pen.",
    "classify_objects": (
        "Classify the two types of objects on the table and place them into the two baskets."
    ),
    "hang_mugs": "Hang the mugs on the mug rack.",
    "sweep_blocks": (
        "Pick up the broom, hand it over to the right hand, then use the dustpan to sweep the blocks."
    ),
    "pack_objects_into_backpack": "Place all the objects on the table into the backpack.",
    # piper
    "fill_pen_holder": "Pick up the pen holder and place all the pens into it.",
    "put_objects_into_basket": "Place all the objects on the table into the basket.",
    "stack_and_cover_blocks": "Stack the blocks on the table, then cover them with the cup.",
    "stack_bowls": "Stack the bowls on the table.",
    "stand_up_bottles": "Stand the bottle upright.",
    "insert_charger": (
        "Insert the charger plug into the power strip, then connect the charging cable to the plug."
    ),
    # arx
    "insert_tubes": "Pick up the test tubes on the table and insert them into the test tube rack.",
    "make_bread": (
        "Pick up the two slices of bread from the bowl and place them into the toaster, "
        "then place the two small bowls on the plate."
    ),
    "make_food": (
        "Take the cutting board out and place it on the table. Then place the steak and the knife "
        "on the cutting board, put the vegetables and shrimp into the pot, place the pot on the stove, "
        "and finally cover it with the lid."
    ),
    "pack_and_pour_fruit": (
        "Place all the fruits into the blue bowl, then pour the fruits from the blue bowl into the large white bowl."
    ),
    "cover_blocks": (
        "Cover the three blocks from left to right, then uncover them again in the order of red, green, and blue."
    ),
    "store_in_safe": "Place all the objects into the safe, then close the safe.",
}

VALID_ACTION_TYPES = frozenset({"joint", "ee"})
EE_POSE_DIM = 7

def should_prompt_episode_result(deploy_cfg: dict) -> bool:
    """Prompt in legacy TCP TTY mode; websocket collector judges success/fail on Web."""
    protocol = deploy_cfg.get("protocol", "ws")
    if protocol == "robodojo_ws":
        protocol = "ws"
    return protocol != "ws" and sys.stdin.isatty()

def validate_deploy_cfg(deploy_cfg: Mapping[str, Any]) -> str:
    """Validate and normalize action_type from dispatch / deploy_cfg."""
    raw = deploy_cfg.get("action_type")
    trial_id = deploy_cfg.get("trial_id")
    policy_name = deploy_cfg.get("policy_name")
    if raw is None or (isinstance(raw, str) and not raw.strip()):
        raise ValueError(
            "dispatch action_type is empty or missing "
            f"(trial={trial_id!r}, policy={policy_name!r}). "
            "Set action_type to 'joint' or 'ee' in the dispatch payload."
        )
    normalized = str(raw).strip().lower()
    if normalized not in VALID_ACTION_TYPES:
        raise ValueError(
            f"invalid action_type={raw!r}, expected 'joint' or 'ee' "
            f"(trial={trial_id!r}, policy={policy_name!r})."
        )
    return normalized


def normalize_action_type_value(value: Any) -> str | None:
    if value is None:
        return None
    if isinstance(value, str):
        stripped = value.strip()
        if not stripped:
            return None
        return stripped.lower()
    return str(value).strip().lower() or None


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _parse_additional_info(info_str):
    result = {}
    if not info_str:
        return result
    for part in info_str.split(","):
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        result[key.strip()] = value.strip()
    return result


def _resolve_real_base_cfg(deploy_cfg, root_dir):
    if deploy_cfg.get("base_cfg"):
        return deploy_cfg["base_cfg"]
    if deploy_cfg.get("real_base_cfg"):
        return deploy_cfg["real_base_cfg"]

    env_cfg_type = deploy_cfg["env_cfg_type"]
    if env_cfg_type in ENV_CFG_TO_BASE_CFG:
        return ENV_CFG_TO_BASE_CFG[env_cfg_type]

    raise ValueError(
        f"Cannot resolve real robot base config for env_cfg_type={env_cfg_type!r}. "
        f"Add a mapping in ENV_CFG_TO_BASE_CFG (supported: {list(ENV_CFG_TO_BASE_CFG)})."
    )


def _default_intrinsic(width, height):
    return [
        [float(width * 0.96), 0.0, float(width / 2.0)],
        [0.0, float(height * 1.28), float(height / 2.0)],
        [0.0, 0.0, 1.0],
    ]


def _default_extrinsic():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _decode_color_image(color):
    """Decode single-frame color: raw HWC array or JPEG bytes."""
    if color is None:
        return None
    if isinstance(color, (bytes, bytearray, memoryview)):
        return cv2.imdecode(np.frombuffer(bytes(color), dtype=np.uint8), cv2.IMREAD_COLOR)

    image = np.asarray(color)
    if image.dtype.kind in ("S", "U", "O"):
        raw = image.tobytes().rstrip(b"\0")
        if not raw:
            return None
        return cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)

    if image.ndim == 4 and image.shape[0] == 1:
        image = image[0]
    if image.ndim == 1:
        raw = image.tobytes().rstrip(b"\0")
        if not raw:
            return None
        return cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image.ndim == 3 and image.shape[-1] in (1, 3, 4):
        return np.asarray(image[..., :3], dtype=np.uint8)
    return None


def _vision_color_for_transport(color):
    """Send JPEG bytes on the wire; keep already-compressed inputs unchanged."""
    if color is None:
        return None
    if isinstance(color, (bytes, bytearray, memoryview)):
        return bytes(color)

    image = np.asarray(color)
    if image.dtype.kind in ("S", "U", "O"):
        raw = image.tobytes().rstrip(b"\0")
        return raw if raw else None
    if image.ndim == 1:
        raw = image.tobytes().rstrip(b"\0")
        return raw if raw else None
    if image.ndim == 3:
        return _encode_color_if_needed(image, is_jpeg=True)
    return None


def _compress_vision_for_transport(vision):
    compressed = {}
    for cam_name, cam_data in vision.items():
        if not isinstance(cam_data, dict):
            continue
        color = _vision_color_for_transport(cam_data.get("color"))
        if color is None:
            continue
        height, width = cam_data.get("shape", (None, None))
        if height is None or width is None:
            decoded = _decode_color_image(color)
            if decoded is None:
                continue
            height, width = decoded.shape[:2]
        compressed[cam_name] = {
            "color": color,
            "depth": cam_data.get("depth"),
            "intrinsic_matrix": cam_data.get("intrinsic_matrix"),
            "extrinsics_matrix": cam_data.get("extrinsics_matrix"),
            "shape": (int(height), int(width)),
        }
    return compressed


def rpy2xyzwxyz(xyzrpy):
    """Convert xyz + roll/pitch/yaw to xyz + quaternion (w, x, y, z)."""
    xyzrpy = np.asarray(xyzrpy, dtype=np.float64)

    if xyzrpy.ndim == 1:
        xyzrpy = xyzrpy.reshape(1, -1)

    if xyzrpy.ndim != 2 or xyzrpy.shape[1] != 6:
        raise ValueError(f"Expected input shape (N, 6), got {xyzrpy.shape}")

    xyz = xyzrpy[:, :3]
    roll = xyzrpy[:, 3]
    pitch = xyzrpy[:, 4]
    yaw = xyzrpy[:, 5]

    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    quat = np.stack([qw, qx, qy, qz], axis=1)
    return np.concatenate([xyz, quat], axis=1)


def xyzwxyz_to_xyzrpy(xyzwxyz):
    """Convert xyz + quaternion (w, x, y, z) back to xyz + roll/pitch/yaw."""
    pose = np.asarray(xyzwxyz, dtype=np.float64).reshape(-1)
    if pose.shape[0] != 7:
        raise ValueError(f"Expected ee_pose shape (7,), got {pose.shape}")

    xyz = pose[:3]
    qw, qx, qy, qz = pose[3:7]
    rpy = R.from_quat([qx, qy, qz, qw]).as_euler("xyz", degrees=False)
    return np.concatenate([xyz, rpy])


def _as_controller_value(value):
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    return arr[0] if arr.size == 1 else arr


def _base_cfg_path(root_dir: str, base_cfg_name: str) -> str:
    candidate = os.fspath(base_cfg_name)
    if candidate.endswith((".yml", ".yaml")) or os.path.isabs(candidate):
        return candidate
    for subdir in ("config", "example/deploy"):
        path = os.path.join(root_dir, subdir, f"{base_cfg_name}.yml")
        if os.path.isfile(path):
            return path
    return os.path.join(root_dir, "config", f"{base_cfg_name}.yml")


def _load_real_base_cfg(deploy_cfg: Mapping[str, Any], root_dir: str) -> dict:
    if deploy_cfg.get("hdf5_paths"):
        from my_robot.replay_robot import build_replay_robot_config

        replay_save_path = deploy_cfg.get("replay_save_path", "save/eval/")
        if not os.path.isabs(replay_save_path):
            replay_save_path = os.path.join(root_dir, replay_save_path)

        return build_replay_robot_config(
            hdf5_paths=deploy_cfg["hdf5_paths"],
            action_type=deploy_cfg.get("action_type", "joint"),
            eval_num=int(deploy_cfg.get("eval_episode_num", deploy_cfg.get("eval_num", 1))),
            policy_name=str(deploy_cfg.get("policy_name", "replay_eval")),
            save_path=replay_save_path,
            hdf5_format=deploy_cfg.get("hdf5_format", "auto"),
        )

    base_cfg_name = _resolve_real_base_cfg(deploy_cfg, root_dir)
    base_cfg_path = _base_cfg_path(root_dir, base_cfg_name)
    if not os.path.isfile(base_cfg_path):
        raise FileNotFoundError(f"Robot base config not found: {base_cfg_path}")
    return _load_yaml(base_cfg_path)


def _set_robot_collect_task(robot: Any, base_cfg: dict, task_name: str) -> None:
    """Rebind per-trial HDF5 collection without recreating hardware."""
    if base_cfg.get("robot", {}).get("type") == "replay_robot":
        return
    base_cfg.setdefault("collect", {})
    base_cfg["collect"]["task_name"] = task_name
    collect_cfg = base_cfg["collect"]
    robot.collect_cfg = collect_cfg

    old_collector = getattr(robot, "collector", None)
    handler = getattr(old_collector, "handler", None)
    robot.collector = CollectAny(collect_cfg)
    if handler is not None:
        robot.collector._add_data_transform_pipeline(handler)

    collect_node = getattr(robot, "collect_node", None)
    if collect_node is not None:
        collect_node._cleanup()


def _expected_camera_names_for_robot(robot: Any) -> set[str]:
    sensors = getattr(robot, "sensors", None)
    if not isinstance(sensors, dict):
        return set()
    return set(sensors.get("image", {}).keys())


def _wait_for_robot_camera_frames(robot: Any, timeout_s=10.0) -> None:
    expected = _expected_camera_names_for_robot(robot)
    if not expected:
        return

    deadline = time.monotonic() + timeout_s
    while True:
        sensor_data = RealEnv._sensor_data_from_raw_obs(robot.get_obs())
        ready = {
            name
            for name in expected
            if isinstance(sensor_data.get(name), dict)
            and sensor_data[name].get("color") is not None
        }
        if ready == expected:
            print(f"[RealEnv] Cameras streaming: {sorted(ready)}")
            return
        if time.monotonic() >= deadline:
            missing = sorted(expected - ready)
            raise RuntimeError(
                f"Cameras produced no frames within {timeout_s:.0f}s: "
                f"missing {missing}. Check USB connection / "
                "'align batch failed' errors in logs."
            )
        time.sleep(0.1)


class RealEnv:
    def __init__(
        self,
        deploy_cfg,
        *,
        setup_cameras=True,
        robot=None,
        base_cfg=None,
        owns_robot=True,
    ):
        self.success_num, self.episode_num = 0, 0
        self.episode_step = 0
        self._stop_check = None
        self._recording_episode = False
        self._owns_robot = owns_robot
        self.deploy_cfg = deploy_cfg
        self.root_dir = deploy_cfg["root_dir"]
        self.is_first_frame = True

        policy_name = str(deploy_cfg["policy_name"]).replace("/", "_").replace("\\", "_")
        task_name = str(deploy_cfg["task_name"]).replace("/", "_").replace("\\", "_")
        seed = deploy_cfg.get("seed", 0)
        self.save_dir = os.path.join(
            self.root_dir, "eval_results", policy_name, task_name, str(seed)
        )
        os.makedirs(self.save_dir, exist_ok=True)

        env_cfg_type = deploy_cfg["env_cfg_type"]
        dim_env_cfg_type = ENV_CFG_ACTION_DIM_ALIAS.get(env_cfg_type, env_cfg_type)
        self.robot_action_dim_info = get_robot_action_dim_info(dim_env_cfg_type)
        self.action_type = validate_deploy_cfg(deploy_cfg)
        deploy_cfg["action_type"] = self.action_type
        print(
            "[RealEnv] action_type="
            f"{self.action_type!r} (policy={deploy_cfg.get('policy_name')!r}, "
            f"trial={deploy_cfg.get('trial_id')!r}, task={deploy_cfg.get('task_name')!r}, "
            "source=dispatch)"
        )
        task_name = deploy_cfg.get("task_name")
        self.episode_step_limit = int(
            deploy_cfg.get("real_step_limit")
            or TASK_STEP_LIMIT.get(task_name, 2000)
        )

        self.base_cfg = base_cfg if base_cfg is not None else _load_real_base_cfg(deploy_cfg, self.root_dir)
        _set_robot_collect_task(robot, self.base_cfg, deploy_cfg["task_name"]) if robot is not None else None
        self.base_cfg.setdefault("collect", {})
        self.base_cfg["collect"]["task_name"] = deploy_cfg["task_name"]

        protocol = deploy_cfg.get("protocol", "ws")
        if protocol == "robodojo_ws":
            protocol = "ws"
        if protocol == "ws":
            from client_server.ws import WsModelClient

            policy_server_url = deploy_cfg.get("policy_server_url")
            if policy_server_url is None:
                host = deploy_cfg.get("host", "localhost")
                policy_server_url = f"ws://{host}:{int(deploy_cfg['port'])}"
            self.model_client = WsModelClient(
                url=policy_server_url,
                evaluation_id=deploy_cfg.get("evaluation_id", "real-eval"),
                trial_id=deploy_cfg.get("trial_id", f"{deploy_cfg.get('task_name', 'trial')}"),
                action_case_id=deploy_cfg.get("action_case_id"),
                repeat_index=deploy_cfg.get("repeat_index"),
            )
        else:
            self.model_client = ModelClient(
                host=deploy_cfg.get("host", "localhost"),
                port=int(deploy_cfg["port"]),
            )

        self.robot = robot if robot is not None else get_robot(base_cfg=self.base_cfg)
        self.is_replay_robot = self.base_cfg.get("robot", {}).get("type") == "replay_robot"
        if robot is None:
            set_up_kwargs: dict[str, object] = {"teleop": False}
            if "setup_cameras" in inspect.signature(self.robot.set_up).parameters:
                set_up_kwargs["setup_cameras"] = setup_cameras and not self.is_replay_robot
            try:
                self.robot.set_up(**set_up_kwargs)
            except Exception as exc:
                trigger_fatal = getattr(self.robot, "trigger_fatal", None)
                if callable(trigger_fatal):
                    self.robot.trigger_fatal(f"robot set_up failed: {exc}")
                raise
            if setup_cameras and not self.is_replay_robot:
                self._wait_for_camera_frames()

    def _expected_camera_names(self):
        return _expected_camera_names_for_robot(self.robot)

    @staticmethod
    def _sensor_data_from_raw_obs(raw_obs):
        if isinstance(raw_obs, dict):
            return raw_obs.get("vision", {})
        if isinstance(raw_obs, (list, tuple)) and len(raw_obs) == 2:
            return raw_obs[1]
        return {}

    def _wait_for_camera_frames(self, timeout_s=10.0):
        """Block until every camera published its first frame.

        The aligned Orbbec batch is all-or-nothing: until the slowest camera
        delivers a frame, the sensor buffer stays empty for *all* cameras and
        the first get_obs() would hand the policy an observation without
        images (KeyError: 'Could not find any image for candidates ...').
        """
        _wait_for_robot_camera_frames(self.robot, timeout_s=timeout_s)

    def set_stop_check(self, stop_check):
        self._stop_check = stop_check

    @staticmethod
    def _arm_specs(num_arms):
        if num_arms == 1:
            return [("", "arm")]
        if num_arms == 2:
            return [("left_", "left_arm"), ("right_", "right_arm")]
        raise ValueError(f"Unsupported arm count: {num_arms}")

    def _resolve_action_type(self, action):
        action_type = self.action_type
        if action_type not in VALID_ACTION_TYPES:
            raise ValueError(f"Unsupported action_type: {action_type!r}")

        arm_keys = {
            "joint": ("arm_joint_state", "ee_joint_state"),
            "ee": ("ee_pose", "ee_joint_state"),
        }
        return action_type, arm_keys[action_type]

    @staticmethod
    def _require_non_empty_action_array(value: Any, *, key: str, action_type: str) -> np.ndarray:
        if value is None:
            raise ValueError(
                f"action payload missing value for {key!r} "
                f"(action_type={action_type!r})."
            )
        arr = np.asarray(value, dtype=np.float64).reshape(-1)
        if arr.size == 0:
            raise ValueError(
                f"action payload has empty array for {key!r} "
                f"(action_type={action_type!r})."
            )
        return arr

    def _validate_action_payload(
        self,
        action: Mapping[str, Any],
        action_type: str,
        arm_suffix: str,
        gripper_suffix: str,
    ) -> None:
        num_arms = len(self.robot_action_dim_info["arm_dim"])
        arm_dims = self.robot_action_dim_info["arm_dim"]

        for arm_index, (state_prefix, _controller_name) in enumerate(
            self._arm_specs(num_arms)
        ):
            arm_key = f"{state_prefix}{arm_suffix}"
            gripper_key = f"{state_prefix}{gripper_suffix}"
            if arm_key not in action:
                raise ValueError(
                    f"action payload missing required key {arm_key!r} "
                    f"for action_type={action_type!r} "
                    f"(trial={self.deploy_cfg.get('trial_id')!r})."
                )
            if gripper_key not in action:
                raise ValueError(
                    f"action payload missing required key {gripper_key!r} "
                    f"for action_type={action_type!r} "
                    f"(trial={self.deploy_cfg.get('trial_id')!r})."
                )

            arm_values = self._require_non_empty_action_array(
                action[arm_key], key=arm_key, action_type=action_type
            )
            self._require_non_empty_action_array(
                action[gripper_key], key=gripper_key, action_type=action_type
            )

            if action_type == "joint":
                expected_dim = int(arm_dims[arm_index])
                if arm_values.size != expected_dim:
                    raise ValueError(
                        f"action payload {arm_key!r} has dim {arm_values.size}, "
                        f"expected {expected_dim} for action_type='joint'."
                    )
            elif arm_values.size != EE_POSE_DIM:
                raise ValueError(
                    f"action payload {arm_key!r} has dim {arm_values.size}, "
                    f"expected {EE_POSE_DIM} for action_type='ee'."
                )

    def _log_first_action_payload(self, action: Any) -> None:
        if not isinstance(action, dict):
            print(
                f"[RealEnv] first action is not a dict (type={type(action).__name__}), "
                f"action_type={self.action_type!r}, trial={self.deploy_cfg.get('trial_id')!r}"
            )
            return

        shapes: dict[str, tuple[int, ...]] = {}
        for key in sorted(action):
            try:
                shapes[key] = np.asarray(action[key]).shape
            except Exception:
                shapes[key] = ("<unreadable>",)

        print(
            "[RealEnv] first action payload "
            f"keys={sorted(action.keys())}, shapes={shapes}, "
            f"action_type={self.action_type!r}, trial={self.deploy_cfg.get('trial_id')!r}"
        )

    def _assert_episode_executed(self) -> None:
        if self.episode_step > 0:
            return
        if self._stop_check is not None and self._stop_check():
            print(
                "[RealEnv] Episode ended with 0 steps (stop requested before first action)."
            )
            return

        raise RuntimeError(
            "trial completed with 0 action steps "
            f"(action_type={self.action_type!r}, policy={self.deploy_cfg.get('policy_name')!r}, "
            f"trial={self.deploy_cfg.get('trial_id')!r}, task={self.deploy_cfg.get('task_name')!r}). "
            "Check dispatch action_type and policy action output format."
        )

    def _build_arm_action(self, action, controller_name, arm_key, gripper_key, action_type):
        arm_entry = {"gripper": _as_controller_value(action[gripper_key])}

        if action_type == "joint":
            arm_entry["joint"] = np.asarray(action[arm_key], dtype=np.float64)
        else:
            # Real arm controllers use 6D xyz+rpy internally; policy/data stay xyzwxyz.
            arm_entry["eef"] = xyzwxyz_to_xyzrpy(action[arm_key])

        return controller_name, arm_entry

    def _action_for_robot(self, action):
        """Replay keeps native xyzwxyz flat actions; real robot uses arm controller layout."""
        if self.is_replay_robot:
            return action
        return self._action_to_move_data(action)

    def _action_to_move_data(self, action):
        num_arms = len(self.robot_action_dim_info["arm_dim"])
        action_type, (arm_suffix, gripper_suffix) = self._resolve_action_type(action)

        self._validate_action_payload(action, action_type, arm_suffix, gripper_suffix)

        move_data = {"arm": {}}
        for state_prefix, controller_name in self._arm_specs(num_arms):
            arm_key = f"{state_prefix}{arm_suffix}"
            gripper_key = f"{state_prefix}{gripper_suffix}"

            controller_name, arm_entry = self._build_arm_action(
                action, controller_name, arm_key, gripper_key, action_type
            )
            move_data["arm"][controller_name] = arm_entry

        return move_data

    def _build_state(self, controller_data):
        state = {}
        arm_names = list(controller_data.keys())

        if "left_arm" in controller_data and "right_arm" in controller_data:
            mapping = [("left_arm", "left_"), ("right_arm", "right_")]
        elif len(arm_names) == 1:
            mapping = [(arm_names[0], "")]
        else:
            raise ValueError(f"Unsupported controller layout for real env: {arm_names}")

        for arm_name, prefix in mapping:
            arm_data = controller_data[arm_name]
            joint = np.asarray(arm_data.get("joint", []), dtype=np.float32).reshape(-1)
            gripper = np.atleast_1d(np.asarray(arm_data.get("gripper", []), dtype=np.float32)).reshape(-1)
            eef = np.asarray(arm_data.get("eef", np.zeros(6, dtype=np.float32)), dtype=np.float32).reshape(-1)
            if eef.shape[0] < 6:
                eef = np.pad(eef, (0, 6 - eef.shape[0]))

            state[f"{prefix}arm_joint_state"] = joint
            state[f"{prefix}ee_joint_state"] = gripper
            state[f"{prefix}ee_pose"] = rpy2xyzwxyz(eef)[0]

        return state

    def _build_vision(self, sensor_data):
        vision = {}
        for cam_name, cam_data in sensor_data.items():
            raw_color = cam_data.get("color")
            image = _decode_color_image(raw_color)
            if image is None:
                continue
            height, width = image.shape[:2]
            jpeg_color = _vision_color_for_transport(image)
            if jpeg_color is None:
                continue
            vision[cam_name] = {
                "color": jpeg_color,
                "depth": cam_data.get("depth"),
                "intrinsic_matrix": _default_intrinsic(width, height),
                "extrinsics_matrix": _default_extrinsic(),
                "shape": (height, width),
            }
        return vision

    @staticmethod
    def _is_xspark_obs(raw_obs):
        return (
            isinstance(raw_obs, dict)
            and raw_obs.get("data_format_version") == "xspark"
            and "vision" in raw_obs
            and "state" in raw_obs
        )

    def get_obs(self, env_idx=0):
        raw_obs = self.robot.get_obs(env_idx=env_idx)
        if self._is_xspark_obs(raw_obs):
            obs = dict(raw_obs)
            obs["vision"] = _compress_vision_for_transport(obs.get("vision", {}))
            obs["env_idx"] = env_idx
            obs.setdefault(
                "additional_info",
                {"frequency": self.base_cfg.get("collect", {}).get("save_freq", 25)},
            )
            return obs

        if isinstance(raw_obs, (list, tuple)) and len(raw_obs) == 2:
            controller_data, sensor_data = raw_obs
        else:
            controller_data, sensor_data = raw_obs, {}

        if not self.is_first_frame:
            for controller_name, controller_action in controller_data.items():
                if controller_name in GRIPPER_BIAS.keys():
                    for k in GRIPPER_BIAS[controller_name].keys():
                        if k in controller_action.keys():
                            controller_action[k] -= GRIPPER_BIAS[controller_name][k]
        self.is_first_frame = False
        obs = {
            "vision": self._build_vision(sensor_data),
            "state": self._build_state(controller_data),
            "instruction": TASK_INSTRUCTION.get(self.deploy_cfg.get("task_name")),
            "additional_info": {
                "frequency": self.base_cfg.get("collect", {}).get("save_freq", 25),
            },
            "data_format_version": "v1.0",
            "env_idx": env_idx,
        }
        return obs

    def get_obs_batch(self, env_idx_list):
        if len(env_idx_list) == 1:
            return [self.get_obs(env_idx_list[0])]
        raise ValueError(f"RealEnv only support single env eval, got {len(env_idx_list)} envs")

    def _start_episode_recording(self):
        if self.is_replay_robot or self._recording_episode:
            return

        self._recording_episode = True
        print("[RealEnv] HDF5 recording started.")

    def _finish_episode_recording(self):
        if not self._recording_episode:
            return

        finish = getattr(self.robot, "finish", None)
        if not callable(finish):
            self._recording_episode = False
            print("[RealEnv] Robot collector finish unavailable; HDF5 recording skipped.")
            return

        # Collect one terminal frame with sentinel action so manual collection has
        # an explicit episode-end sample in HDF5.
        try:
            view_data = self.robot.get_obs()
            if not (isinstance(view_data, (list, tuple)) and len(view_data) == 2):
                raise TypeError(
                    "robot.get_obs() must return [controller_data, sensor_data] "
                    f"for collection, got {type(view_data).__name__}."
                )

            action_key = "joint" if self.action_type == "joint" else "eef"
            arm_dims = list(self.robot_action_dim_info.get("arm_dim", []))
            if len(arm_dims) == 1:
                arm_specs = [("arm", int(arm_dims[0]))]
            elif len(arm_dims) == 2:
                arm_specs = [
                    ("left_arm", int(arm_dims[0])),
                    ("right_arm", int(arm_dims[1])),
                ]
            else:
                raise ValueError(f"Unsupported arm dim layout for terminal collect: {arm_dims}")

            terminal_actions = {"arm": {}}
            for controller_name, arm_dim in arm_specs:
                state_dim = arm_dim if action_key == "joint" else 6
                terminal_actions["arm"][controller_name] = {
                    action_key: -1 * np.ones(state_dim, dtype=np.float64),
                    "gripper": -1.0,
                }

            controller_data, sensor_data = view_data
            controller_data = dict(controller_data)
            controller_data.update(self._policy_action_record(terminal_actions["arm"]))

            self.robot.collect([controller_data, sensor_data])
        except Exception as exc:
            print(f"[RealEnv] terminal frame collection failed: {exc}")

        try:
            finish()
            print("[RealEnv] HDF5 recording saved.")
        finally:
            self._recording_episode = False

    def eval_one_episode(self):
        policy_name = self.deploy_cfg["policy_name"]
        eval_module = __import__(
            f"XPolicyLab.policy.{policy_name}.deploy",
            fromlist=["eval_one_episode"],
        )
        if not hasattr(eval_module, "eval_one_episode"):
            raise AttributeError(f"Missing eval_one_episode in policy module {policy_name}")
        self._start_episode_recording()
        try:
            eval_module.eval_one_episode(TASK_ENV=self, model_client=self.model_client)
            self._assert_episode_executed()
        except Exception as e:
            print(f"[RealEnv] Exception during episode evaluation: {e}")
        finally:
            self._finish_episode_recording()

    def eval_one_episode_batch(self):
        policy_name = self.deploy_cfg["policy_name"]
        eval_module = __import__(
            f"XPolicyLab.policy.{policy_name}.deploy",
            fromlist=["eval_one_episode_batch"],
        )
        if not hasattr(eval_module, "eval_one_episode_batch"):
            raise AttributeError(f"Missing eval_one_episode_batch in policy module {policy_name}")
        self._start_episode_recording()
        try:
            eval_module.eval_one_episode_batch(
                TASK_ENV=self, model_client=self.model_client
            )
            self._assert_episode_executed()
        finally:
            self._finish_episode_recording()

    def _format_action_step(self) -> str:
        if self.is_replay_robot and hasattr(self.robot, "frames"):
            total_frames = len(self.robot.frames)
            frame_idx = min(getattr(self.robot, "frame_idx", 0), total_frames)
            return (
                f"[RealEnv] Replay step {self.episode_step}, "
                f"frame {frame_idx}/{total_frames}"
            )
        return (
            f"[RealEnv] Action Step: {self.episode_step} / "
            f"{self.episode_step_limit} (step_limit)"
        )

    @staticmethod
    def _policy_action_record(arm_actions: Mapping[str, Any]) -> dict[str, dict[str, np.ndarray]]:
        """Return top-level keys with nested arm-like structure.

        Example:
        policy_left_arm_action: {"joint" or "eef", "gripper"}
        policy_right_arm_action: {"joint" or "eef", "gripper"}
        """
        policy_actions: dict[str, dict[str, np.ndarray]] = {}
        for controller_name, controller_action in arm_actions.items():
            if not isinstance(controller_action, Mapping):
                continue

            nested: dict[str, np.ndarray] = {}
            for item_name in ("joint", "eef", "gripper"):
                if item_name not in controller_action:
                    continue
                value = np.asarray(controller_action[item_name], dtype=np.float64).reshape(-1)
                if item_name == "gripper" and value.size == 1:
                    nested[item_name] = float(value[0])
                else:
                    nested[item_name] = value

            if not nested:
                continue

            policy_actions[f"policy_{controller_name}_action"] = {
                **nested
            }

        return policy_actions

    def take_action(self, action):
        if self.episode_step == 0 and not self.is_replay_robot:
            self._log_first_action_payload(action)

        if self.is_replay_robot:
            if self.episode_step == 0 or (self.episode_step + 1) % 100 == 0:
                print(self._format_action_step())
            self.robot.move(action)
            time.sleep(1 / 25)
            self.episode_step += 1
            return

        # take gripper bias to close gripper more firmly, which can improve success rate for some tasks (e.g., block stacking)
        actions = self._action_for_robot(action)
        for controller_name, controller_action in actions["arm"].items():
            if controller_name in GRIPPER_BIAS.keys():
                for k in GRIPPER_BIAS[controller_name].keys():
                    if k in controller_action.keys():
                        controller_action[k] += GRIPPER_BIAS[controller_name][k]
        
        # collect eval video and action
        view_data = self.robot.get_obs()
        if not (isinstance(view_data, (list, tuple)) and len(view_data) == 2):
            raise TypeError(
                "robot.get_obs() must return [controller_data, sensor_data] "
                f"for collection, got {type(view_data).__name__}."
            )

        controller_data, sensor_data = view_data
        controller_data = dict(controller_data)
        controller_data.update(self._policy_action_record(actions["arm"]))
        
        self.robot.collect([controller_data, sensor_data])

        # take robot action
        self.robot.move(actions)
        time.sleep(1 / 25)
        self.episode_step += 1

    def take_action_batch(self, action_list, env_idx_list):
        if len(action_list) != 1 or len(env_idx_list) != 1:
            raise NotImplementedError("Real robot batch evaluation only supports batch size 1.")
        self.take_action(action_list[0])

    def is_episode_end(self):
        if self._stop_check is not None and self._stop_check():
            print(
                f"[RealEnv] Episode End (stop requested): steps={self.episode_step}"
            )
            return True

        if self.is_replay_robot:
            replay_done = self.robot.frame_idx >= len(self.robot.frames)
            if replay_done:
                print(
                    f"[RealEnv] Episode End (replay): "
                    f"frames={self.robot.frame_idx}/{len(self.robot.frames)}, "
                    f"steps={self.episode_step}"
                )
            return replay_done

        step_done = self.episode_step >= self.episode_step_limit
        if step_done:
            print(
                f"[RealEnv] Episode End: step {self.episode_step} "
                f">= limit {self.episode_step_limit}"
            )
        return step_done

    def _finalize_replay_trajectory_plot(self) -> None:
        """Save or locate replay trajectory plot under root_dir."""
        if not self.is_replay_robot:
            return
        robot = self.robot
        pred_actions = getattr(robot, "pred_actions", None)
        if not pred_actions:
            print("[RealEnv] Replay finished with 0 actions; no trajectory plot saved.")
            return

        draw = getattr(robot, "draw", None)
        frame_idx = int(getattr(robot, "frame_idx", 0))
        total_frames = len(getattr(robot, "frames", []) or [])
        if callable(draw) and frame_idx < total_frames:
            draw()

        draw_dir = os.path.abspath(getattr(robot, "draw_dir", self.root_dir))
        saved = sorted(
            (
                os.path.join(draw_dir, name)
                for name in os.listdir(draw_dir)
                if name.lower().endswith((".jpg", ".jpeg", ".png"))
            ),
            key=os.path.getmtime,
        ) if os.path.isdir(draw_dir) else []
        if saved:
            print(f"[RealEnv] Replay trajectory plot: {saved[-1]}")
        else:
            print(f"[RealEnv] Replay trajectory plot dir (empty): {draw_dir}")

    def finish_episode(self):
        self._finalize_replay_trajectory_plot()
        if not should_prompt_episode_result(self.deploy_cfg):
            self.episode_num += 1
            print(
                "[RealEnv] Episode finished (collector judges success/fail: skip prompt). "
                f"total={self.episode_num}"
            )
            return

        print("\n[RealEnv] Episode finished. Please input result (1=success, 2=fail): ", end="")
        result = input().strip()
        while result not in {"1", "2"}:
            print("Invalid input. Please enter 1 or 2: ", end="")
            result = input().strip()

        is_success = result == "1"
        if is_success:
            self.success_num += 1
        self.episode_num += 1
        success_rate = self.success_num / self.episode_num
        print(
            f"\033[92mSuccess Rate: {success_rate:.2%} "
            f"(success={self.success_num}, total={self.episode_num})\033[0m"
        )

        log_path = os.path.join(self.save_dir, "eval_result.txt")
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(log_path, "a", encoding="utf-8") as f:
            f.write(
                f"[{timestamp}] result={'success' if is_success else 'fail'} "
                f"success={self.success_num} total={self.episode_num} "
                f"success_rate={success_rate:.4f}\n"
            )

    def get_running_env_idx_list(self):
        return list(range(Batch_Size))

    def reset(self):
        self.model_client.call(func_name="reset")
        self.episode_step = 0
        self.robot.reset()
        self.is_first_frame = True

    def cleanup(self):
        self._finish_episode_recording()
        close = getattr(getattr(self, "model_client", None), "close", None)
        if callable(close):
            close()
        if not self._owns_robot:
            return
        cleanup = getattr(self.robot, "cleanup", None)
        if callable(cleanup):
            cleanup()


class PersistentRealRobotRuntime:
    """Daemon-owned real robot runtime shared by preview and trials."""

    def __init__(self, *, root_dir: str, base_cfg_name: str):
        self.root_dir = root_dir
        self.base_cfg_name = base_cfg_name
        self.base_cfg = _load_yaml(_base_cfg_path(root_dir, base_cfg_name))
        self.robot = None
        self._lock = None
        self._hub = _PREVIEW_FRAME_HUB
        serials = self.base_cfg.get("robot", {}).get("CAMERA_SERIALS") or {}
        self._serial_by_role = {str(role): str(serial) for role, serial in serials.items()}
        self._last_error = None

    def start(self) -> None:
        import threading

        self._lock = threading.Lock()
        runtime_cfg = copy.deepcopy(self.base_cfg)
        runtime_cfg.setdefault("collect", {})
        runtime_cfg["collect"]["task_name"] = (
            runtime_cfg["collect"].get("task_name") or "runtime_idle"
        )
        sync_cfg = dict(runtime_cfg.get("robot", {}).get("CAMERA_SYNC") or {})
        sync_cfg["use_async_frames"] = True
        runtime_cfg.setdefault("robot", {})["CAMERA_SYNC"] = sync_cfg
        self.robot = get_robot(base_cfg=runtime_cfg)
        set_up_kwargs: dict[str, object] = {"teleop": False}
        if "setup_cameras" in inspect.signature(self.robot.set_up).parameters:
            set_up_kwargs["setup_cameras"] = True
        try:
            self.robot.set_up(**set_up_kwargs)
        except Exception as exc:
            self.robot.trigger_fatal(f"robot set_up failed: {exc}")
            raise
        _wait_for_robot_camera_frames(self.robot)
        self.base_cfg = runtime_cfg
        print(f"[PersistentRealRobotRuntime] started base_cfg={self.base_cfg_name!r}")

    def run_trial(self, deploy_cfg: dict[str, Any], *, stop_check=None) -> dict[str, Any]:
        if self.robot is None or self._lock is None:
            self.start()
        assert self._lock is not None
        with self._lock:
            env = RealEnv(
                deploy_cfg,
                robot=self.robot,
                base_cfg=self.base_cfg,
                owns_robot=False,
            )
            if stop_check is not None:
                env.set_stop_check(stop_check)
            hdf5_path = None
            try:
                if stop_check is not None and stop_check():
                    total_steps = 0
                else:
                    env.reset()
                    if deploy_cfg.get("eval_batch", False):
                        env.eval_one_episode_batch()
                    else:
                        env.eval_one_episode()
                    total_steps = env.episode_step
                    env.reset()
                    env.finish_episode()
                hdf5_path = self._hdf5_path_from_env(env)
            finally:
                env.cleanup()
            result = {
                "status": "completed",
                "trial_id": deploy_cfg.get("trial_id"),
                "steps": total_steps,
                "eval_env": deploy_cfg.get("eval_env", "real"),
                "policy_name": deploy_cfg.get("policy_name"),
            }
            if hdf5_path:
                result["hdf5_path"] = hdf5_path
            return result

    @staticmethod
    def _hdf5_path_from_env(env: RealEnv) -> str | None:
        collector = getattr(getattr(env, "robot", None), "collector", None)
        if collector is None:
            return None
        episode_index = getattr(collector, "episode_index", 0)
        cfg = getattr(collector, "collect_cfg", None)
        if not isinstance(cfg, Mapping) or episode_index <= 0:
            return None
        try:
            path = os.path.abspath(
                os.path.join(
                    cfg["save_dir"],
                    cfg["task_name"],
                    cfg["type"],
                    f"{episode_index - 1}.hdf5",
                )
            )
        except (KeyError, TypeError):
            return None
        return path if os.path.isfile(path) else None

    def reset_idle(self) -> None:
        if self.robot is None:
            self.start()
        self.robot.reset()

    def cleanup(self) -> None:
        if self.robot is None:
            return
        cleanup = getattr(self.robot, "cleanup", None)
        if callable(cleanup):
            cleanup()
        self.robot = None

    def roles(self):
        return tuple(self._serial_by_role.keys())

    def ensure_started(self) -> None:
        if self.robot is None:
            self.start()

    def pause(self) -> None:
        return

    def resume_async(self) -> None:
        return

    def frame(self, role: str):
        serial = self._serial_by_role.get(role)
        return self._hub.latest(serial) if serial else None

    def wait_frame(self, role: str, after_timestamp_ns: int = 0, timeout_s: float = 1.0):
        serial = self._serial_by_role.get(role)
        return self._hub.wait_for_frame(serial, after_timestamp_ns, timeout_s) if serial else None

    def placeholder_jpeg(self) -> bytes:
        import cv2

        canvas = np.full((480, 640, 3), 32, dtype=np.uint8)
        cv2.putText(canvas, "preview offline", (170, 250), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 200, 200), 2, cv2.LINE_AA)
        ok, encoded = cv2.imencode(".jpg", canvas)
        if not ok:
            raise RuntimeError("failed to encode preview placeholder")
        return encoded.tobytes()

    def status(self) -> dict:
        now_ns = time.time_ns()
        cameras = {}
        for role, serial in self._serial_by_role.items():
            item = self._hub.latest(serial)
            cameras[role] = {
                "serial": serial,
                "frame_age_ms": max(0, (now_ns - item[1]) // 1_000_000) if item else None,
            }
        return {
            "active": self.robot is not None,
            "paused": False,
            "last_error": self._last_error,
            "cameras": cameras,
        }


def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ("yes", "true", "t", "1"):
        return True
    if v.lower() in ("no", "false", "f", "0"):
        return False
    raise argparse.ArgumentTypeError("Boolean value expected.")


def _build_deploy_cfg(args_cli):
    deploy_cfg = {key: value for key, value in vars(args_cli).items() if value is not None}

    if args_cli.deploy_config and os.path.isfile(args_cli.deploy_config):
        file_cfg = _load_yaml(args_cli.deploy_config)
        for key, value in file_cfg.items():
            if value is not None:
                deploy_cfg.setdefault(key, value)

    deploy_cfg.update(
        {key: value for key, value in vars(args_cli).items() if value is not None and key != "deploy_config"}
    )
    deploy_cfg.update(_parse_additional_info(args_cli.additional_info))
    return deploy_cfg


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset_name", required=True, type=str)
    parser.add_argument("--task_name", required=True, type=str)
    parser.add_argument("--env_cfg_type", type=str, required=True)
    parser.add_argument("--policy_name", type=str, required=True)
    parser.add_argument("--protocol", choices=("legacy_tcp", "ws", "robodojo_ws"), default="ws")
    parser.add_argument("--host", type=str, default="localhost")
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--policy_server_url", type=str)
    parser.add_argument("--evaluation_id", type=str, default="real-eval")
    parser.add_argument("--action_case_id", type=str)
    parser.add_argument("--trial_id", type=str)
    parser.add_argument("--repeat_index", type=int)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--eval_episode_num", type=int, default=10)
    parser.add_argument("--eval_batch", type=str2bool, default=False)
    parser.add_argument("--additional_info", type=str, default="")
    parser.add_argument("--deploy_config", type=str, default=None)
    parser.add_argument("--root_dir", type=str, required=True)
    args_cli = parser.parse_args()

    root_dir = os.path.abspath(args_cli.root_dir)
    for path in (
        os.path.join(root_dir, "src"),
        os.path.join(root_dir, "XPolicyLab"),
        root_dir,
    ):
        if path not in sys.path:
            sys.path.insert(0, path)

    deploy_cfg = _build_deploy_cfg(args_cli)
    deploy_cfg["root_dir"] = root_dir
    deploy_cfg["host"] = args_cli.host
    deploy_cfg["port"] = args_cli.port
    if deploy_cfg.get("action_type") is not None:
        normalized = normalize_action_type_value(deploy_cfg["action_type"])
        if normalized is not None:
            deploy_cfg["action_type"] = normalized

    real_env = RealEnv(deploy_cfg)
    eval_batch = bool(deploy_cfg.get("eval_batch", False))
    eval_episode_num = int(deploy_cfg.get("eval_episode_num", args_cli.eval_episode_num))

    try:
        for idx in range(eval_episode_num):
            print(f"\033[94m🚀 Running Episode {idx}\033[0m")
            real_env.reset()
            if not eval_batch:
                real_env.eval_one_episode()
            else:
                real_env.eval_one_episode_batch()
            real_env.finish_episode()
    finally:
        close = getattr(real_env.model_client, "close", None)
        if callable(close):
            close()
