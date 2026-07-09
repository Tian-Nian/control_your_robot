from __future__ import annotations

import glob
import os
import random
import time
from typing import Any, Dict, List, Optional

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

from robot.robot.base_robot import Robot, dict_to_list, get_array_length
from robot.utils.base.data_handler import (
    debug_print,
    hdf5_groups_to_dict,
)

# xspark HDF5 stores time-series with plural keys; single-frame obs uses singular keys.
_STATE_KEY_MAP = {
    "left_arm_joint_states": "left_arm_joint_state",
    "left_ee_joint_states": "left_ee_joint_state",
    "left_ee_poses": "left_ee_pose",
    "right_arm_joint_states": "right_arm_joint_state",
    "right_ee_joint_states": "right_ee_joint_state",
    "right_ee_poses": "right_ee_pose",
}
_VISION_KEY_MAP = {
    "colors": "color",
    "depths": "depth",
    "intrinsic_matrices": "intrinsic_matrix",
    "extrinsics_matrices": "extrinsics_matrix",
}


def build_replay_robot_config(
    *,
    hdf5_paths: str,
    action_type: str = "joint",
    eval_num: int = 1,
    shuffle: bool = False,
    policy_name: str = "replay_eval",
    save_path: str = "save/eval/",
    hdf5_format: str = "auto",
    save_freq: int = 30,
) -> Dict[str, Any]:
    """Build a config dict for ReplayRobot without YAML registration."""
    return {
        "collect": {
            "type": "replay",
            "save_freq": save_freq,
            "task_name": "replay",
        },
        "robot": {
            "type": "replay_robot",
            "action_type": action_type,
            "hdf5_paths": hdf5_paths,
            "eval_num": eval_num,
            "shuffle": shuffle,
            "policy_name": policy_name,
            "save_path": save_path,
            "hdf5_format": hdf5_format,
        },
    }


def create_replay_robot(
    *,
    hdf5_paths: str,
    action_type: str = "joint",
    eval_num: int = 1,
    shuffle: bool = False,
    policy_name: str = "replay_eval",
    save_path: str = "save/eval/",
    hdf5_format: str = "auto",
    save_freq: int = 30,
) -> "ReplayRobot":
    """Instantiate ReplayRobot from explicit parameters (no config registry)."""
    base_config = build_replay_robot_config(
        hdf5_paths=hdf5_paths,
        action_type=action_type,
        eval_num=eval_num,
        shuffle=shuffle,
        policy_name=policy_name,
        save_path=save_path,
        hdf5_format=hdf5_format,
        save_freq=save_freq,
    )
    return ReplayRobot(base_config)


def _split_frame_group(group: Dict[str, Any], idx: int, episode_len: int) -> Dict[str, Any]:
    frame_group = {}
    for key, value in group.items():
        if isinstance(value, dict):
            frame_group[key] = _split_frame_group(value, idx, episode_len)
        elif isinstance(value, np.ndarray):
            if value.ndim == 0 or value.shape[0] != episode_len:
                frame_group[key] = value
            else:
                frame_group[key] = value[idx]
        else:
            frame_group[key] = value
    return frame_group


def _detect_hdf5_format(raw: Dict[str, Any]) -> str:
    if "state" in raw and "vision" in raw:
        return "xspark"
    return "real"


def _load_frames(hdf5_path: str, data_format: str = "auto") -> tuple[List[Dict[str, Any]], str]:
    raw = hdf5_groups_to_dict(hdf5_path)
    fmt = data_format if data_format in {"xspark", "real"} else _detect_hdf5_format(raw)
    if fmt == "xspark":
        groups = {name: raw[name] for name in ("state", "action", "vision") if name in raw}
        if not groups:
            raise ValueError(f"HDF5 missing state/action/vision groups: {hdf5_path}")
        length = get_array_length(groups)
        scalars = {key: value for key, value in raw.items() if key not in groups}
        frames = [
            {
                **scalars,
                **{name: _split_frame_group(group, idx, length) for name, group in groups.items()},
            }
            for idx in range(length)
        ]
        return frames, "xspark"
    if not raw:
        raise ValueError(f"HDF5 is empty: {hdf5_path}")
    return dict_to_list(raw), "real"


def _arm_eef_value(arm_data: Dict[str, Any]):
    """Read end-effector pose; accept legacy ``qpos`` key in old HDF5 files."""
    if "eef" in arm_data:
        return arm_data["eef"]
    if "qpos" in arm_data:
        return arm_data["qpos"]
    raise KeyError("arm data missing 'eef' (or legacy 'qpos')")


def _split_real_frame(frame: Dict[str, Any]) -> tuple[Dict[str, Any], Dict[str, Any]]:
    controller_data: Dict[str, Any] = {}
    sensor_data: Dict[str, Any] = {}
    for key, value in frame.items():
        if not isinstance(value, dict):
            continue
        if key.startswith("cam_"):
            sensor_data[key] = value
        elif any(field in value for field in ("joint", "eef", "qpos", "gripper")):
            controller_data[key] = value
    return controller_data, sensor_data


def _eef_to_policy_pose(eef) -> np.ndarray:
    eef = np.asarray(eef, dtype=np.float64).reshape(-1)
    if eef.shape[0] == 7:
        return eef
    if eef.shape[0] < 6:
        eef = np.pad(eef, (0, 6 - eef.shape[0]))
    xyz = eef[:3]
    quat = R.from_euler("xyz", eef[3:6], degrees=False).as_quat()
    return np.concatenate([xyz, [quat[3], quat[0], quat[1], quat[2]]])


def _decode_color_image(color) -> np.ndarray | None:
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


def _normalize_xspark_cam_obs(cam_data: Dict[str, Any]) -> Dict[str, Any] | None:
    cam_obs: Dict[str, Any] = {}

    color_src = cam_data.get("colors")
    if color_src is None:
        color_src = cam_data.get("color")
    if color_src is not None:
        image = _decode_color_image(color_src)
        if image is not None:
            cam_obs["color"] = image
            height, width = image.shape[:2]
            cam_obs["shape"] = (height, width)

    depth_src = cam_data.get("depths")
    if depth_src is None:
        depth_src = cam_data.get("depth")
    if depth_src is not None:
        cam_obs["depth"] = np.asarray(depth_src)

    for plural_key, singular_key in _VISION_KEY_MAP.items():
        if plural_key in {"colors", "depths"}:
            continue
        if plural_key in cam_data:
            cam_obs[singular_key] = np.asarray(cam_data[plural_key])
        elif singular_key in cam_data:
            cam_obs[singular_key] = np.asarray(cam_data[singular_key])

    if "shape" not in cam_obs:
        stored_shape = cam_data.get("shape")
        if stored_shape is not None:
            shape = np.asarray(stored_shape).reshape(-1)
            if shape.size >= 2:
                cam_obs["shape"] = (int(shape[0]), int(shape[1]))

    return cam_obs if "color" in cam_obs else None


def _decode_text(value) -> str:
    if isinstance(value, np.ndarray):
        item = value.reshape(-1)[0]
        if isinstance(item, bytes):
            return item.decode("utf-8")
        return str(item)
    if isinstance(value, bytes):
        return value.decode("utf-8")
    return str(value) if value is not None else ""


def _flatten_policy_action(action, action_type: str) -> np.ndarray:
    if isinstance(action, np.ndarray):
        return np.asarray(action, dtype=np.float64).reshape(-1)

    parts = []
    for side in ("left", "right"):
        if action_type == "ee":
            parts.append(np.asarray(action[f"{side}_ee_pose"], dtype=np.float64).reshape(-1))
        else:
            parts.append(np.asarray(action[f"{side}_arm_joint_state"], dtype=np.float64).reshape(-1))
        parts.append(np.asarray(action[f"{side}_ee_joint_state"], dtype=np.float64).reshape(-1))
    return np.concatenate(parts)


def _gt_action_xspark(frame: Dict[str, Any], action_type: str) -> np.ndarray:
    if action_type == "ee":
        state = frame["state"]
        return np.concatenate(
            [
                np.asarray(state["left_ee_poses"], dtype=np.float64).reshape(-1),
                np.asarray(state["left_ee_joint_states"], dtype=np.float64).reshape(-1),
                np.asarray(state["right_ee_poses"], dtype=np.float64).reshape(-1),
                np.asarray(state["right_ee_joint_states"], dtype=np.float64).reshape(-1),
            ]
        )
    action = frame["action"]
    return np.concatenate(
        [
            np.asarray(action["left_arm_joint_states"], dtype=np.float64).reshape(-1),
            np.asarray(action["left_ee_joint_states"], dtype=np.float64).reshape(-1),
            np.asarray(action["right_arm_joint_states"], dtype=np.float64).reshape(-1),
            np.asarray(action["right_ee_joint_states"], dtype=np.float64).reshape(-1),
        ]
    )


def _gt_action_real(frame: Dict[str, Any], action_type: str) -> np.ndarray:
    parts = []
    for arm_name in ("left_arm", "right_arm"):
        arm_data = frame[arm_name]
        if action_type == "ee":
            parts.append(_eef_to_policy_pose(_arm_eef_value(arm_data)))
        else:
            parts.append(np.asarray(arm_data["joint"], dtype=np.float64).reshape(-1))
        parts.append(np.asarray(arm_data["gripper"], dtype=np.float64).reshape(-1))
    return np.concatenate(parts)


def _move_data_to_policy_action(move_data: Dict[str, Any], action_type: str) -> Dict[str, Any]:
    """Convert deploy ``move_data`` layout to flat policy action keys."""
    if not isinstance(move_data, dict) or "arm" not in move_data:
        return move_data

    action: Dict[str, Any] = {}
    for side, prefix in (("left_arm", "left"), ("right_arm", "right")):
        entry = move_data["arm"].get(side)
        if not entry:
            continue
        if action_type == "ee" and "eef" in entry:
            eef = np.asarray(entry["eef"], dtype=np.float64).reshape(-1)
            action[f"{prefix}_ee_pose"] = _eef_to_policy_pose(eef)
        else:
            action[f"{prefix}_arm_joint_state"] = np.asarray(entry["joint"], dtype=np.float64).reshape(-1)
        action[f"{prefix}_ee_joint_state"] = np.asarray(entry["gripper"], dtype=np.float64).reshape(-1)
    return action


def _gt_action(frame: Dict[str, Any], action_type: str, data_format: str) -> np.ndarray:
    if data_format == "real":
        return _gt_action_real(frame, action_type)
    return _gt_action_xspark(frame, action_type)


class ReplayRobot(Robot):
    """Offline HDF5 replay robot for policy evaluation (no hardware)."""

    def __init__(self, base_config: Optional[Dict[str, Any]] = None, **kwargs):
        if base_config is None:
            if not kwargs.get("hdf5_paths"):
                raise ValueError("ReplayRobot requires hdf5_paths via base_config or kwargs")
            base_config = build_replay_robot_config(**kwargs)

        robot_config = base_config.get("robot", {})
        collect_cfg = base_config.get("collect", {})
        condition = {
            "save_freq": collect_cfg.get("save_freq", 30),
            "task_name": collect_cfg.get("task_name", "replay"),
            "save_path": robot_config.get("save_path", "save/eval/"),
            "save_format": "hdf5",
        }
        super().__init__(condition=condition, move_check=False)

        self.base_config = base_config
        self.robot_config = robot_config
        self.collect_cfg = collect_cfg
        self.name = "replay_robot"

        self.hdf5_dir = self.robot_config["hdf5_paths"]
        self.eval_num = int(self.robot_config.get("eval_num", 1))
        self.policy_name = self.robot_config.get("policy_name", "replay_eval")
        self.action_type = self.robot_config.get("action_type", "joint")
        self.shuffle = bool(self.robot_config.get("shuffle", False))

        save_path = self.robot_config.get("save_path", "save/eval/")
        self.draw_dir = os.path.join(save_path, self.policy_name)
        os.makedirs(self.draw_dir, exist_ok=True)

        self.episode_paths = sorted(glob.glob(os.path.join(self.hdf5_dir, "*.hdf5")))
        if not self.episode_paths:
            raise FileNotFoundError(f"No .hdf5 files under {self.hdf5_dir!r}")

        if self.eval_num > len(self.episode_paths):
            self.eval_num = len(self.episode_paths)
        if self.shuffle:
            self.episode_paths = random.sample(self.episode_paths, self.eval_num)
        else:
            self.episode_paths = self.episode_paths[: self.eval_num]

        self.hdf5_format = str(self.robot_config.get("hdf5_format", "auto"))
        self.episode_idx = -1
        self.episode_path = ""
        self.frames: List[Dict[str, Any]] = []
        self.frame_data_format = "xspark"
        self.frame_idx = 0
        self.pred_actions: List[np.ndarray] = []
        self._load_next_episode()

    def _load_next_episode(self):
        self.episode_idx = (self.episode_idx + 1) % len(self.episode_paths)
        self.episode_path = self.episode_paths[self.episode_idx]
        self.frames, self.frame_data_format = _load_frames(self.episode_path, self.hdf5_format)
        self.frame_idx = 0
        self.pred_actions = []
        debug_print(
            self.name,
            f"Loaded episode {self.episode_idx}: {self.episode_path} "
            f"({len(self.frames)} frames, format={self.frame_data_format})",
            "INFO",
        )

    def set_up(self, teleop=False):
        return

    def reset(self):
        self.episode_idx = -1
        self._load_next_episode()
        return True

    def get_obs(self, env_idx: int = 0):
        frame = self.frames[min(self.frame_idx, len(self.frames) - 1)]

        if self.frame_data_format == "real":
            return _split_real_frame(frame)

        frequency = int(self.collect_cfg.get("save_freq", 30))
        state = {}
        for src_key, dst_key in _STATE_KEY_MAP.items():
            if src_key in frame.get("state", {}):
                state[dst_key] = np.asarray(frame["state"][src_key], dtype=np.float32).reshape(-1)

        vision = {}
        for cam_name, cam_data in frame.get("vision", {}).items():
            cam_obs = _normalize_xspark_cam_obs(cam_data)
            if cam_obs is not None:
                vision[cam_name] = cam_obs

        return {
            "data_format_version": "xspark",
            "instruction": _decode_text(frame.get("instruction", "")),
            "state": state,
            "vision": vision,
            "additional_info": {"frequency": frequency},
            "env_idx": env_idx,
        }

    def get(self):
        return self.get_obs()

    def is_episode_end(self) -> bool:
        return self.frame_idx >= len(self.frames)

    def move(self, move_data, advance_steps=None, key_banned=None):
        if move_data is None:
            return

        policy_action = _move_data_to_policy_action(move_data, self.action_type)
        self.pred_actions.append(_flatten_policy_action(policy_action, self.action_type))
        self.frame_idx += 1
        if self.is_episode_end():
            self.draw()

    def collect(self, data):
        """ReplayRobot does not write HDF5 during offline eval."""
        return

    def draw(self):
        if not self.frames:
            return

        episode_name = os.path.splitext(os.path.basename(self.episode_path))[0]
        pic_path = os.path.join(
            self.draw_dir,
            f"{episode_name}_{time.strftime('%Y%m%d_%H%M%S')}.jpg",
        )

        gt_traj = np.array(
            [_gt_action(frame, self.action_type, self.frame_data_format) for frame in self.frames]
        )
        pred_traj = np.array(self.pred_actions) if self.pred_actions else np.empty((0, gt_traj.shape[1]))
        steps = np.arange(len(self.pred_actions))

        action_dim = gt_traj.shape[1]
        cols = 4 if action_dim > 8 else 2
        rows = int(np.ceil(action_dim / cols))
        fig, axes = plt.subplots(rows, cols, figsize=(cols * 5, rows * 3.2), squeeze=False)
        axes = axes.flatten()

        for dim_idx in range(action_dim):
            ax = axes[dim_idx]
            ax.plot(np.arange(gt_traj.shape[0]), gt_traj[:, dim_idx], color="0.75", linewidth=1.2, label="gt")
            if pred_traj.size > 0 and dim_idx < pred_traj.shape[1]:
                ax.plot(steps, pred_traj[:, dim_idx], color="#d62728", linewidth=1.2, label="model")
            ax.set_title(f"dim_{dim_idx}")
            ax.set_xlabel("frame")
            ax.grid(True, alpha=0.25)
            if dim_idx == 0:
                ax.legend(loc="best", fontsize="small")

        for dim_idx in range(action_dim, len(axes)):
            axes[dim_idx].axis("off")

        fig.suptitle(
            f"Replay | {episode_name} | frames={gt_traj.shape[0]} | steps={pred_traj.shape[0]}",
            fontsize=13,
        )
        fig.tight_layout(rect=[0, 0.02, 1, 0.97])
        fig.savefig(pic_path, dpi=200, bbox_inches="tight")
        plt.close(fig)
        debug_print(self.name, f"Trajectory comparison saved: {pic_path}", "INFO")
