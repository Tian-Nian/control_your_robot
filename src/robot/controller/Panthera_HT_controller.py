from pathlib import Path

import numpy as np

from robot.controller.arm_controller import ArmController
from third_party.Panthera_HT.Panthera_lib import Panthera
import time

class PantheraHTController(ArmController):
    JOINT_COUNT = 6
    DEFAULT_MAX_TORQUE = np.array(
        [21.0, 36.0, 36.0, 21.0, 10.0, 10.0],
        dtype=np.float32,
    )
    DEFAULT_TAU_LIMIT = np.array(
        [15.0, 30.0, 30.0, 15.0, 5.0, 5.0],
        dtype=np.float32,
    )

    def __init__(self, name):
        super().__init__()
        self.name = name
        self.controller_type = "user_controller"
        self.controller = None
        self.follower = False
        self.role = None
        self._stopped = True

    def set_up(self, follower: bool = False):
        if self.controller is not None:
            raise RuntimeError(
                "Panthera HT controller is already set up; "
                "create separate controller instances for leader and follower"
            )

        project_root = Path(__file__).resolve().parents[3]
        config_name = "Follower.yaml" if follower else "Leader.yaml"
        config_path = (
            project_root
            / "third_party"
            / "Panthera_HT"
            / "robot_param"
            / config_name
        )
        if not config_path.is_file():
            raise FileNotFoundError(
                f"Panthera HT config file not found: {config_path}"
            )

        self.controller = Panthera(str(config_path))
        if self.controller.motor_count != self.JOINT_COUNT:
            raise RuntimeError(
                "Panthera HT controller requires 6 arm motors, "
                f"got {self.controller.motor_count}"
            )

        self.follower = bool(follower)
        self.role = "follower" if self.follower else "leader"
        self.default_velocity = np.full(
            self.JOINT_COUNT,
            0.5,
            dtype=np.float32,
        )
        self.max_torque = np.asarray(
            getattr(
                self.controller,
                "max_torque",
                self.DEFAULT_MAX_TORQUE,
            ),
            dtype=np.float32,
        )
        self.tau_limit = self.DEFAULT_TAU_LIMIT.copy()

        # Gravity and friction compensation parameters shared by both roles.
        self.Fc = np.array(
            [0.15, 0.12, 0.12, 0.12, 0.04, 0.04],
            dtype=np.float32,
        )
        self.Fv = np.array(
            [0.05, 0.05, 0.05, 0.03, 0.02, 0.02],
            dtype=np.float32,
        )
        self.vel_threshold = 0.02

        # Leader zero-stiffness MIT parameters.
        self.zero_pos = np.zeros(self.JOINT_COUNT, dtype=np.float32)
        self.zero_vel = np.zeros(self.JOINT_COUNT, dtype=np.float32)
        self.zero_kp = np.zeros(self.JOINT_COUNT, dtype=np.float32)
        self.zero_kd = np.zeros(self.JOINT_COUNT, dtype=np.float32)
        self.leader_gripper_target = 0.75
        self.leader_gripper_kp = 0.2
        self.leader_gripper_kd = 0.02

        # Follower joint and gripper tracking gains.
        self.kp = np.array(
            [10.0, 21.0, 21.0, 16.0, 13.0, 1.0],
            dtype=np.float32,
        )
        self.kd = np.array(
            [1.0, 2.0, 2.0, 0.9, 0.8, 0.1],
            dtype=np.float32,
        )
        self.gripper_kp = 4.0
        self.gripper_kd = 0.4
        self._stopped = False
        return self

    def get_state(self):
        self._ensure_ready()
        state = {}
        state = {
            "joint": self._joint_vector(
                self.controller.get_current_pos(),
                "joint",
            ),
            "eef":[0.0] * 6,
            "velocity": self._joint_vector(
                self.controller.get_current_vel(),
                "velocity",
            ),
            "gripper": np.float32(
                self.controller.get_current_pos_gripper() / 2.0
            ),
            "gripper_velocity": np.float32(
                self.controller.get_current_vel_gripper() / 2.0
            ),
        }
        return state

    def get_compensation_torque(self, joint=None, velocity=None):
        self._ensure_ready()
        if joint is None:
            joint = self.controller.get_current_pos()
        if velocity is None:
            velocity = self.controller.get_current_vel()

        joint = self._joint_vector(joint, "measured joint position")
        velocity = self._joint_vector(velocity, "velocity")
        gravity = self._joint_vector(
            self.controller.get_Gravity(joint),
            "gravity torque",
        )
        friction = self._joint_vector(
            self.controller.get_friction_compensation(
                velocity,
                self.Fc,
                self.Fv,
                self.vel_threshold,
            ),
            "friction torque",
        )
        return np.clip(
            gravity + friction,
            -self.tau_limit,
            self.tau_limit,
        ).astype(np.float32)

    def update_leader_compensation(self):
        """Run one leader gravity/friction compensation control frame."""
        self._ensure_ready()
        if self.follower:
            raise RuntimeError(
                "update_leader_compensation() requires a leader controller"
            )

        state = self.get_state()
        compensation_torque = self.get_compensation_torque(
            joint=state["joint"],
            velocity=state["velocity"],
        )
        self.set_joint_mit(
            position=self.zero_pos,
            velocity=self.zero_vel,
            torque=compensation_torque,
            kp=self.zero_kp,
            kd=self.zero_kd,
        )
        self.set_gripper_mit(
            position=self.leader_gripper_target,
            velocity=0.0,
            torque=0.0,
            kp=self.leader_gripper_kp,
            kd=self.leader_gripper_kd,
        )
        return state

    def set_joint(
        self,
        joint,
        velocity=None,
        max_torque=None,
        iswait=False,
    ):
        self._ensure_ready()
        position = self._joint_position(joint)
        if velocity is None:
            velocity = self.default_velocity
        velocity = self._joint_vector(velocity, "velocity")
        if max_torque is None:
            max_torque = self.max_torque
        max_torque = self._joint_vector(max_torque, "max_torque")

        success = self.controller.Joint_Pos_Vel(
            position,
            velocity,
            max_torque,
            iswait=bool(iswait),
        )
        if success is False:
            raise RuntimeError("Panthera HT joint command failed")
        self._stopped = False
        return True

    def set_joint_mit(self, position, velocity, torque, kp, kd):
        self._ensure_ready()
        position = self._joint_position(position)
        velocity = self._joint_vector(velocity, "velocity")
        torque = self._joint_vector(torque, "torque")
        kp = self._joint_vector(kp, "kp")
        kd = self._joint_vector(kd, "kd")
        torque = np.clip(
            torque,
            -self.tau_limit,
            self.tau_limit,
        ).astype(np.float32)

        success = self.controller.pos_vel_tqe_kp_kd(
            position,
            velocity,
            torque,
            kp,
            kd,
        )
        if success is False:
            raise RuntimeError("Panthera HT MIT joint command failed")
        self._stopped = False
        return True

    def set_gripper(
        self,
        gripper,
        velocity=0.5,
        max_torque=0.5,
    ):
        self._ensure_ready()
        normalized_position = self._normalized_gripper(
            gripper,
            "gripper",
        )
        velocity = self._finite_scalar(velocity, "gripper velocity")
        max_torque = self._finite_scalar(
            max_torque,
            "gripper max torque",
        )

        success = self.controller.gripper_control(
            normalized_position * 2.0,
            velocity,
            max_tqu=max_torque,
        )
        if success is False:
            raise RuntimeError("Panthera HT gripper command failed")
        self._stopped = False
        return True

    def set_gripper_mit(
        self,
        position,
        velocity,
        torque,
        kp,
        kd,
    ):
        self._ensure_ready()
        normalized_position = self._normalized_gripper(
            position,
            "gripper position",
        )
        normalized_velocity = self._finite_scalar(
            velocity,
            "gripper velocity",
        )
        torque = self._finite_scalar(torque, "gripper torque")
        kp = self._finite_scalar(kp, "gripper kp")
        kd = self._finite_scalar(kd, "gripper kd")

        success = self.controller.gripper_control_MIT(
            normalized_position * 2.0,
            normalized_velocity * 2.0,
            torque,
            kp,
            kd,
        )
        if success is False:
            raise RuntimeError("Panthera HT MIT gripper command failed")
        self._stopped = False
        return True

    def reset(self, start_state):
        return self.set_joint(start_state, iswait=True)

    def brake(self):
        if self.controller is None or self._stopped:
            return False

        try:
            set_brake = getattr(self.controller, "set_brake", None)
            if callable(set_brake):
                for _ in range(3):
                    set_brake()
            else:
                motors = getattr(self.controller, "Motors", None)
                if motors is None:
                    raise AttributeError("SDK does not expose brake control")
                for _ in range(3):
                    for motor in motors:
                        motor.brake()
                    self.controller.motor_send_cmd()
        except Exception as exc:
            raise RuntimeError(
                "Panthera HT brake command failed"
            ) from exc

        self._stopped = True
        return True

    def stop(self):
        if self.controller is None or self._stopped:
            return False

        try:
            self.controller.set_stop()
        except AttributeError as exc:
            raise RuntimeError(
                "Panthera HT SDK does not provide set_stop()"
            ) from exc
        except Exception as exc:
            raise RuntimeError("Panthera HT stop command failed") from exc

        self._stopped = True
        return True

    def _ensure_ready(self):
        if self.controller is None:
            raise RuntimeError("Panthera HT controller is not set up")

    def _joint_vector(self, value, name):
        vector = np.asarray(value, dtype=np.float32).reshape(-1)
        if vector.shape != (self.JOINT_COUNT,):
            raise ValueError(
                f"{name} must contain {self.JOINT_COUNT} values"
            )
        if not np.all(np.isfinite(vector)):
            raise ValueError(f"{name} contains NaN or Inf")
        return vector

    def _joint_position(self, value):
        position = self._joint_vector(value, "joint position")
        limits = getattr(self.controller, "joint_limits", None)
        if limits is None:
            return position

        lower = self._joint_vector(limits["lower"], "joint lower limits")
        upper = self._joint_vector(limits["upper"], "joint upper limits")
        if np.any(position < lower) or np.any(position > upper):
            raise ValueError(
                "joint position exceeds Panthera HT joint limits"
            )
        return position

    @staticmethod
    def _finite_scalar(value, name):
        scalar = float(np.asarray(value).squeeze())
        if not np.isfinite(scalar):
            raise ValueError(f"{name} must be finite")
        return scalar

    def _normalized_gripper(self, value, name):
        scalar = self._finite_scalar(value, name)
        if scalar < 0.0 or scalar > 1.0:
            raise ValueError(f"{name} must be in the range [0, 1]")
        return scalar

    def __del__(self):
        try:
            self.stop()
        except Exception:
            pass


if __name__ == "__main__":
    controller = PantheraHTController("test_panthera_ht")
    # controller.set_up(follower=True)
    # print(controller.get_state())
    pos1 = [0.0, 0.8, 0.8, 0.3, 0.0, 0.0]
    # controller.set_joint(pos1)
    # time.sleep(1)
    # controller.set_gripper(gripper=0.1)
    # time.sleep(1)
    # zero_pos = [0.0] * 6
    # controller.reset(start_state=zero_pos)
    #主臂测试
    try:
        controller.set_up(follower=False)
        controller.set_joint(pos1)
        print("Leader 初始状态:")
        print(controller.get_state())

        frequency = 30.0
        period = 1.0 / frequency

        print("开始 Leader 重力补偿")
        print("机械臂将进入低刚度状态，可以缓慢拖动")
        print("按 Ctrl+C 结束")

        while True:
            loop_start = time.monotonic()
            print(controller.update_leader_compensation())
            elapsed = time.monotonic() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n结束 Leader 重力补偿")

    finally:
        controller.stop()
