#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import numpy as np
from agx_pinocchio import AgxPinocchio
from scipy.spatial.transform import Rotation as R
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW


def main():
    # 定义机械臂模型的URDF文件路径，根据末端执行器的安装情况修改
    # urdf_path = "piper_description/urdf/piper_description.urdf"
    # urdf_path = "piper_description/urdf/piper_no_gripper_description.urdf"
    # urdf_path = "piper_description/urdf/piper_description_with_teach.urdf"
    # urdf_path = "piper_x_description/urdf/piper_x_description.urdf"
    urdf_path = "piper_x_description/urdf/piper_x_description_no_gripper.urdf"

    # 初始化逆运动学求解器（用于重力补偿计算）
    pin = AgxPinocchio(urdf_path)

    # 控制频率
    control_frequency = 200.0

    # 初始化机械臂接口
    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER_X,
        firmeware_version=PiperFW.V183,
        interface="socketcan",
        channel="can0",
    )
    robot = AgxArmFactory.create_arm(cfg)
    robot.connect()

    # 等待机械臂使能
    while not robot.enable():
        time.sleep(0.01)
    
    print("机械臂使能成功")
    
    # 获取当前关节角度
    while robot.get_joint_angles() is None:
        joint_angles = np.array(robot.get_joint_angles().msg)
        time.sleep(0.01)
        print(joint_angles)

    # 计算世界坐标系到基座坐标系的旋转矩阵
    roll, pitch, yaw = 0, 0, 0  # 单位：deg
    R_world_base = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_matrix()

    print("开始重力补偿控制循环...")
    
    try:
        while True:
            start_time = time.time()

            # 获取当前关节角度和速度
            joint_angles = np.array(robot.get_joint_angles().msg)

            joint_velocities = np.zeros(robot.joint_nums)
            joint_torques = np.zeros(robot.joint_nums)
            for i in range(1, robot.joint_nums + 1):
                ms = robot.get_motor_states(i)
                if ms is not None:
                    joint_velocities[i - 1] = ms.msg.velocity
                    joint_torques[i - 1] = ms.msg.torque

            # 计算重力补偿扭矩
            gravity_torque = pin.gravity_compensation(joint_angles, joint_velocities, R_world_base)

            # 应用重力补偿扭矩
            try: 
                for joint_id in range(1, robot.joint_nums + 1):
                    joint_idx = joint_id - 1
                    actual_torque = gravity_torque[joint_idx]
                    robot.move_mit(joint_id, 0, 0, 0, 0, actual_torque)
                
                print(f"目标力矩 - 反馈力矩: {np.round(gravity_torque - joint_torques, 3).tolist()}")
                    
            except Exception as e:
                print(f"应用重力补偿失败: {e}")
            
            # 控制频率
            t = 1.0 / control_frequency
            elapsed_time = time.time() - start_time
            if elapsed_time < t:
                time.sleep(t - elapsed_time)
            else:
                print(f"警告：控制循环超时 {elapsed_time:.3f}s > {t:.3f}s")
                
    except KeyboardInterrupt:
        print("\n用户中断，停止重力补偿")
        for joint_id in range(1, robot.joint_nums + 1):
            try:
                robot.move_mit(joint_id, joint_angles[joint_id - 1], 0, 10, 0.8, 0)
                # robot.move_mit(joint_id, 0, 0, 0, 0, 0)
            except:
                pass
        
    except Exception as e:
        print(f"程序运行出错: {e}")
        for joint_id in range(1, robot.joint_nums + 1):
            try:
                robot.move_mit(joint_id, joint_angles[joint_id - 1], 0, 10, 0.8, 0)
                # robot.move_mit(joint_id, 0, 0, 0, 0, 0)
            except:
                pass


if __name__ == "__main__":
    main()
    