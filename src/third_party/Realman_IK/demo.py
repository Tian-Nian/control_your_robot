# coding:UTF-8
import numpy as np
import time
from ik_rbtdef import *
from ik_rbtutils import *
from robotic_arm_package.robotic_arm import *
from ik_qp import *

if __name__ == '__main__':

    dT = 0.02 # 单位:sec

    # 实例化逆解库             
    qp = QPIK("RM65B", dT)    
    qp.set_install_angle([90, 180, 0], 'deg')

    # 限制肘部朝外
    qp.set_elbow_min_angle(3, 'deg')

    # 设置运行过程中的关节速度约束
    qp.set_dq_max_weight([1.0, 1.0, 1.0, 0.1, 1.0, 1.0])

    # 连接机器人, 此案例为real_robot遥操作sim_robot
    sim_robot = Arm(RM65, "192.168.1.19")
    sim_robot.Movej_Cmd([0, 0, 90, 0, 90, 0], 20, 0, 0, True)

    real_robot = Arm(RM65, "192.168.1.18")
    real_robot.Movej_Cmd([0, 0, 90, 0, 90, 0], 20, 0, 0, True)

    # 读取当前机械臂位姿数据
    ret = sim_robot.Get_Current_Arm_State()
    q = np.array(ret[1])*deg2rad
    pose = ret[2]

    ret = real_robot.Get_Current_Arm_State()
    pose_cmd = ret[2]
    last_pose_cmd = pose_cmd

    while True:
        start_time = time.time()

        # 读取当前机械臂位姿数据
        ret = real_robot.Get_Current_Arm_State()
        pose_cmd = ret[2]

        d_rx = pose_cmd[3] - last_pose_cmd[3]
        d_ry = pose_cmd[4] - last_pose_cmd[4]
        d_rz = pose_cmd[5] - last_pose_cmd[5]

        if d_rx > np.pi*0.9:
            d_rx -= 2*np.pi
        elif d_rx < -np.pi*0.9:
            d_rx += 2*np.pi

        if d_ry > np.pi*0.9:
            d_ry -= 2*np.pi
        elif d_ry < -np.pi*0.9:
            d_ry += 2*np.pi

        if d_rz > np.pi*0.9:
            d_rz -= 2*np.pi
        elif d_rz < -np.pi*0.9:
            d_rz += 2*np.pi
        
        dx = pose_cmd[0] - last_pose_cmd[0]
        dy = pose_cmd[1] - last_pose_cmd[1]
        dz = pose_cmd[2] - last_pose_cmd[2]

        last_pose_cmd = pose_cmd

        # 注意: 实际应用中, 姿态应使用旋转矩阵更新, 此处只是为了演示逆解过程
        # pose[0] += dx
        # pose[1] += dy
        # pose[2] += dz
        # pose[3] += d_rx
        # pose[4] += d_ry
        # pose[5] += d_rz
        pose[0] = pose_cmd[0]
        pose[1] = pose_cmd[1]
        pose[2] = pose_cmd[2]
        pose[3] = pose_cmd[3]
        pose[4] = pose_cmd[4]
        pose[5] = pose_cmd[5]

        x    = pose[0] 
        y    = pose[1]
        z    = pose[2]
        rx   = pose[3]
        ry   = pose[4]
        rz   = pose[5]
        euler = [rx, ry, rz]
        R = euler_to_matrix(euler)
        T = [[R[0][0], R[0][1], R[0][2], x],
            [R[1][0], R[1][1], R[1][2], y],
            [R[2][0], R[2][1], R[2][2], z],
            [0, 0, 0, 1]]
        
        # 调用QP求解器求解逆运动学
        q = qp.sovler(q,T)

        # 发送指令到机械臂
        q_deg = [x*rad2deg for x in q]
        sim_robot.Movej_CANFD(q_deg, False)

        end_time = time.time()
        elapsed_time = end_time - start_time
        if elapsed_time < dT:
            time.sleep(dT - elapsed_time)