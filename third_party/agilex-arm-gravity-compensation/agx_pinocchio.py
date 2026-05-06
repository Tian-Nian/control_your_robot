import os

import numpy as np
import pinocchio as pin


class AgxPinocchio:
    def __init__(self, urdf_path=None):
        package_dirs = [os.path.dirname(os.path.dirname(urdf_path))]
        self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path, package_dirs)
        self.robot.data = self.robot.model.createData()

    def gravity_compensation(
        self,
        q: np.ndarray,
        v: np.ndarray = None,
        base_orientation: np.ndarray = None,
        include_velocity_bias: bool = False,
    ) -> np.ndarray:
        """
        重力补偿计算，支持基座任意姿态。

        :param q: 关节角度
        :param v: 关节速度，仅在 include_velocity_bias=True 时参与计算
        :param base_orientation: 基座姿态（四元数xyzw或旋转矩阵），None表示水平基座
        :param include_velocity_bias: 是否复现旧的 RNEA(q, v, 0) 速度偏置行为；默认只计算 RNEA(q, 0, 0)
        :return: 重力补偿力矩
        """
        q = np.asarray(q, dtype=float)

        if base_orientation is not None:
            try:
                # 将重力向量从世界坐标系变换到基座坐标系
                base_orientation = np.asarray(base_orientation, dtype=float)
                if base_orientation.shape == (4,):
                    # 四元数 [x, y, z, w]
                    R = pin.Quaternion(
                        base_orientation[3],
                        base_orientation[0],
                        base_orientation[1],
                        base_orientation[2],
                    ).toRotationMatrix()
                elif base_orientation.shape == (3, 3):
                    # 旋转矩阵
                    R = base_orientation
                else:
                    raise ValueError("基座姿态格式错误")

                # 世界坐标系中的重力向量 [0, 0, -9.81]
                gravity_world = np.array([0.0, 0.0, -9.81])

                # 变换到基座坐标系
                gravity_base = R.T @ gravity_world

                # 设置新的重力向量
                self.robot.model.gravity.linear = gravity_base

                # 计算重力补偿力矩
                q_temp = np.zeros(self.robot.model.nq)
                q_temp[: len(q)] = q
                v_temp = np.zeros(self.robot.model.nv)
                if include_velocity_bias:
                    if v is None:
                        raise ValueError("include_velocity_bias=True 时必须提供 v")
                    v = np.asarray(v, dtype=float)
                    v_temp[: len(v)] = v

                zero_acc = np.zeros(self.robot.model.nv)
                tau_ff = pin.rnea(self.robot.model, self.robot.data, q_temp, v_temp, zero_acc)[: len(q)]

                return tau_ff

            except Exception as e:
                raise e
        else:
            # 使用默认重力方向
            self.robot.model.gravity.linear = np.array([0.0, 0.0, -9.81])
            q_temp = np.zeros(self.robot.model.nq)
            q_temp[: len(q)] = q
            v_temp = np.zeros(self.robot.model.nv)
            if include_velocity_bias:
                if v is None:
                    raise ValueError("include_velocity_bias=True 时必须提供 v")
                v = np.asarray(v, dtype=float)
                v_temp[: len(v)] = v

            zero_acc = np.zeros(self.robot.model.nv)
            return pin.rnea(self.robot.model, self.robot.data, q_temp, v_temp, zero_acc)[: len(q)]
