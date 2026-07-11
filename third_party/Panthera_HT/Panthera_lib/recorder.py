#!/usr/bin/env python3
"""
轨迹记录与回放工具（无脏行版）
"""
import time, json, os
import numpy as np
from typing import List, Optional


class Recorder:
    def __init__(self, path: str = None, flush_interval: float = 0.2):
        if path is None:
            path = time.strftime("trajectory_%Y%m%d_%H%M%S.jsonl")
        self.path = path
        self.fd = open(path, "w", encoding="utf-8")
        self.t0 = None
        self.flush_interval = flush_interval
        self.last_flush = time.perf_counter()

    # ---------------- 记录 ----------------
    def log(self, pos: List[float], vel: List[float] = None, gripper_pos: float = None, gripper_vel: float = None):
        t = time.perf_counter()
        if self.t0 is None:
            self.t0 = t
        # 一次性生成合法 JSON 字符串，避免半截写入
        data = {"t": t - self.t0, "pos": list(pos)}
        if vel is not None:
            data["vel"] = list(vel)
        if gripper_pos is not None:
            data["gripper_pos"] = gripper_pos
        if gripper_vel is not None:
            data["gripper_vel"] = gripper_vel
        line = json.dumps(data, ensure_ascii=False)
        self.fd.write(line + "\n")
        if t - self.last_flush >= self.flush_interval:
            self.fd.flush()
            self.last_flush = t

    def close(self):
        if self.fd and not self.fd.closed:
            self.fd.flush()
            self.fd.close()
            print(f"[Recorder] 轨迹已保存 → {self.path}")

    # ---------------- 静态回放 ----------------
    @staticmethod
    def play(
        robot,
        filepath: str,
        kp: List[float],
        kd: List[float],
        fc: Optional[List[float]] = None,
        fv: Optional[List[float]] = None,
        vel_threshold: float = 0.0,
        tau_limit: Optional[List[float]] = None,
        gripper_kp: float = 5.0,
        gripper_kd: float = 0.5,
        playback_dt: float = 0.01,
        smooth_window: int = 7,
        mode: str = "mit",
    ):
        # 回放模式选择使用电机位置速度模式时会用到
        max_torque = [21.0, 36.0, 36.0, 21.0, 10.0, 10.0]
        with open(filepath, "r", encoding="utf-8") as f:
            frames = [json.loads(line) for line in f if line.strip()]
        if not frames:
            print("[Player] 空文件，无数据回放")
            return

        frames = Recorder._prepare_playback_frames(frames, playback_dt, smooth_window)
        print(f"[Player] 共 {len(frames)} 帧, dt={playback_dt:.3f}s, mode={mode}")

        # 移动到起始点
        first_frame = frames[0]
        print("[Player] 正在移动到轨迹起点...")

        # 关节移动到起点（使用 Joint_Pos_Vel 模式）
        start_pos = first_frame["pos"]
        move_vel = [0.5] * len(start_pos)  # 缓慢速度 0.5 rad/s

        # 夹爪移动到起点（如果有夹爪数据）
        if "gripper_pos" in first_frame:
            gripper_start_pos = first_frame["gripper_pos"]
            print(f"[Player] 夹爪移动到起点: {gripper_start_pos:.3f} rad")
            robot.gripper_control(gripper_start_pos, 0.5, 0.5)
            time.sleep(2.0)  # 等待夹爪到达

        # 缓慢移动到起点，等待到达
        robot.Joint_Pos_Vel(start_pos, move_vel, max_torque, iswait=True, tolerance=0.05, timeout=30.0)

        print("[Player] 已到达起点，开始回放...")
        t0 = time.perf_counter()
        
        for f in frames:
            while time.perf_counter() - t0 < f["t"]:
                time.sleep(0.0005)

            # 关节控制
            if mode == "mit":
                robot_torque = np.array(robot.get_Gravity(f["pos"]))
                if fc is not None and fv is not None:
                    robot_torque += robot.get_friction_compensation(f["vel"], fc, fv, vel_threshold)
                if tau_limit is not None:
                    robot_torque = np.clip(robot_torque, -np.array(tau_limit), np.array(tau_limit))
                robot.pos_vel_tqe_kp_kd(f["pos"], f["vel"], robot_torque, kp, kd)
            elif mode == "posvel":
                robot.Joint_Pos_Vel(f["pos"], f["vel"], max_torque)
            else:
                raise ValueError("mode 必须是 'mit' 或 'posvel'")

            # 夹爪控制（如果有夹爪数据）
            if "gripper_pos" in f:
                robot.gripper_control_MIT(f["gripper_pos"], f["gripper_vel"], 0.0, gripper_kp, gripper_kd)

        print("[Player] 回放完成")

    @staticmethod
    def _prepare_playback_frames(frames, playback_dt: float, smooth_window: int):
        if playback_dt <= 0:
            raise ValueError("playback_dt 必须大于 0")

        t = np.array([f["t"] for f in frames], dtype=float)
        t = t - t[0]
        pos = np.array([f["pos"] for f in frames], dtype=float)

        # 去掉重复或倒退时间戳，避免插值异常
        keep = np.concatenate(([True], np.diff(t) > 1e-6))
        t = t[keep]
        pos = pos[keep]
        kept_frames = [f for f, k in zip(frames, keep) if k]

        if len(t) < 2:
            return kept_frames

        new_t = np.arange(0.0, t[-1] + playback_dt * 0.5, playback_dt)
        new_pos = np.column_stack([
            np.interp(new_t, t, pos[:, i])
            for i in range(pos.shape[1])
        ])
        new_pos = Recorder._moving_average(new_pos, smooth_window)
        new_vel = np.gradient(new_pos, new_t, axis=0)

        has_gripper_pos = "gripper_pos" in kept_frames[0]
        has_gripper_vel = "gripper_vel" in kept_frames[0]
        if has_gripper_pos:
            gripper_pos = np.array([f["gripper_pos"] for f in kept_frames], dtype=float)
            new_gripper_pos = np.interp(new_t, t, gripper_pos)
            new_gripper_pos = Recorder._moving_average(new_gripper_pos[:, None], smooth_window)[:, 0]
            if has_gripper_vel:
                new_gripper_vel = np.gradient(new_gripper_pos, new_t)
            else:
                new_gripper_vel = np.zeros_like(new_gripper_pos)

        prepared = []
        for i, timestamp in enumerate(new_t):
            item = {
                "t": float(timestamp),
                "pos": new_pos[i].tolist(),
                "vel": new_vel[i].tolist(),
            }
            if has_gripper_pos:
                item["gripper_pos"] = float(new_gripper_pos[i])
                item["gripper_vel"] = float(new_gripper_vel[i])
            prepared.append(item)
        return prepared

    @staticmethod
    def _moving_average(values: np.ndarray, window: int):
        if window <= 1:
            return values
        if window % 2 == 0:
            window += 1
        pad = window // 2
        padded = np.pad(values, [(pad, pad), (0, 0)], mode="edge")
        kernel = np.ones(window) / window
        return np.apply_along_axis(lambda x: np.convolve(x, kernel, mode="valid"), 0, padded)
