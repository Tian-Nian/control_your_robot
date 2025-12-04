#! -*-conding=: UTF-8 -*-
#
################################################################################
#                                                                              #
# Author   : Daryl/Ray/Leon                                                    #
# Date     : 2024/09/26                                                        #
# Copyright: Copyright (c) 2018-2024 RealMan Co., Ltd.. All rights reserved.   #
#                                                                              #
################################################################################
import qpSWIFT
import numpy as np
from ik_rbtdef import *
from ik_rbtutils import *

class QPIK():
    '''
        f(x) = 0.5*x'*P*x + c'*x
        subject to G*x <= h
               
                 ||
               \ || /
                 \/
        
        f(dq) = ||Jdq - dX||^2
        subject to G*dq <= h

        where,
            P = J^T * W^T * W * J
            c = -2 * J^T * W^T * W * dX
            G = [I, -I, I, -I]^T
            h = [dq_w.*dq_max, -dq_w.*dq_max, q_max - q_ref, q_ref - q_min]^T
    '''
    def __init__(self, type, dT):
        self.dT = dT
        self.robot = Robot(type)

        self.q_max, self.q_min = self.robot.get_qlim()
        self.dq_max = self.robot.get_dq_max().transpose()

        self.W = np.eye(6)
        self.dq_weight = np.ones(self.robot.dof)

        # 限制腕部速度.
        if self.robot.dof == 7:
            self.dq_weight[4] = 0.1
        else:
            self.dq_weight[3] = 0.1

    def set_7dof_q3_max_angle(self,angle,unit = 'rad'):
        if unit == 'deg':
            self.q_max[2] = angle * deg2rad
        else:
            self.q_max[2] = angle

    def set_7dof_q3_min_angle(self,angle,unit = 'rad'):
        if unit == 'deg':
            self.q_min[2] = angle * deg2rad
        else:
            self.q_min[2] = angle

    def set_7dof_elbow_max_angle(self,angle,unit = 'rad'):
        if unit == 'deg':
            self.q_max[3] = angle * deg2rad
        else:
            self.q_max[3] = angle

    def set_7dof_elbow_min_angle(self,angle,unit = 'rad'):
        if unit == 'deg':
            self.q_min[3] = angle * deg2rad
        else:
            self.q_min[3] = angle
    
    def set_6dof_elbow_max_angle(self, angle, unit='rad'):
        if unit == 'deg':
            self.q_max[2] = angle * deg2rad
        else:
            self.q_max[2] = angle

    def set_6dof_elbow_min_angle(self, angle, unit='rad'):
        if unit == 'deg':
            self.q_min[2] = angle * deg2rad
        else:
            self.q_min[2] = angle
    
    def set_joint_limit_max(self, angle, unit = 'rad'):
        if len(angle)!= self.robot.dof:
            raise Exception(f"[ERROR] joint_limit_max size should be {self.robot.dof}.")
        for i in range(self.robot.dof):
            if unit == 'deg':
                self.q_max[i] = angle[i] * deg2rad
            else:
                self.q_max[i] = angle[i]

    def set_joint_limit_min(self,angle, unit = 'rad'):
        if len(angle)!= self.robot.dof:
            raise Exception(f"[ERROR] joint_limit_min size should be {self.robot.dof}.")
        for i in range(self.robot.dof):
            if unit == 'deg':
                self.q_min[i] = angle[i] * deg2rad
            else:
                self.q_min[i] = angle[i]

    def set_install_angle(self, angle, unit='rad'):
        self.robot.set_install_angle(angle, unit)

    def set_work_cs_params(self, pose):
        self.robot.set_work_cs_params(pose)
    
    def set_tool_cs_params(self, pose):
        self.robot.set_tool_cs_params(pose)

    def set_error_weight(self, weight):
        if len(weight)!= 6:
            raise Exception(f"[ERROR] weight size should be {6}.")
        for i in range(6):
            if weight[i] > 1.0:
                weight[i] = 1.0
            elif weight[i] < 0.0:
                weight[i] = 0.0
            self.W[i,i] = weight[i]
    
    def set_joint_velocity_limit(self, dq_max, unit='rad'):
        if len(dq_max)!= self.robot.dof:
            raise Exception(f"[ERROR] dq_max size should be {self.robot.dof}.")
        for i in range(self.robot.dof):
            if unit == 'deg':
                self.dq_max[i] = dq_max[i] * deg2rad
            else:
                self.dq_max[i] = dq_max[i]
    
    def set_dq_max_weight(self, weight):
        if len(weight)!= self.robot.dof:
            raise Exception(f"[ERROR] weight size should be {self.robot.dof}.")
        for i in range(self.robot.dof):
            if weight[i] > 1.0:
                weight[i] = 1.0
            elif weight[i] < 0.0:
                weight[i] = 0.0
            self.dq_weight[i] = weight[i]    

    def fkine(self, q):
        return self.robot.fkine(q)    
    
    def __Limt(self,q_ref):
        self.q_ref = np.array(q_ref)

        G1 =      np.eye(self.robot.dof) # 关节速度约束
        G2 = -1 * np.eye(self.robot.dof)
        G3 =      np.eye(self.robot.dof) # 关节限位约束
        G4 = -1 * np.eye(self.robot.dof)

        self.G = np.concatenate([G1,G2,G3,G4])

        self.delta_q_max = np.zeros(self.robot.dof)
        
        for i in range(self.robot.dof):
            self.delta_q_max[i] = self.dq_max[i] * self.dq_weight[i] * self.dT
        
        h1 = self.delta_q_max
        h2 = self.delta_q_max
        h3 = self.q_max - self.q_ref
        h4 = self.q_ref - self.q_min

        self.h = np.concatenate([h1,h2,h3,h4])

    def sovler(self, q_ref, Td, max_iter = 150):

        self.q_sovle = np.zeros(self.robot.dof)
      
        self.Jaco = self.robot.jacob_Jw(q_ref) 
        self.Jaco = np.array(self.Jaco)

        Tc = self.fkine(q_ref)

        self.DX = angle_axis_diff(Tc,Td)
        self.__Limt(q_ref)
 
        self.c = -2*np.dot(self.Jaco.transpose() @ self.W.transpose() @ self.W, self.DX)
        self.P =  2* self.Jaco.transpose() @ self.W.transpose() @ self.W @ self.Jaco

        opts = {"MAXITER": max_iter, "VERBOSE": 0,"OUTPUT": 1}
        self.k = qpSWIFT.run(self.c, self.h, self.P, self.G, np.zeros([1,self.robot.dof]), np.zeros(1), opts)

        is_nan = False
        for j in range(self.robot.dof):
            is_nan = is_nan or np.isnan(self.k['sol'][j])

        is_success = True
        for j in range(self.robot.dof):
            if np.abs( self.k['sol'][j]) > 1.0 * self.q_max[j]:
                is_success = False

        if self.k['basicInfo']['ExitFlag'] == 1 or self.k['basicInfo']['ExitFlag'] == 3 or is_nan:
            for j in range(self.robot.dof):
                self.q_sovle[j] = q_ref[j]
            return self.q_sovle
    
        for j in range(self.robot.dof):
            self.q_sovle[j] = q_ref[j] + self.k['sol'][j]
        return self.q_sovle

###########################  以下程序针对RM65的参数设置，75的在最下面  ####################
def RM65_Demo():

    dT = 0.01 # 用户数据的下发周期(透传周期)，与实际机械臂周期对应，为了安全起见先设置小一点，10ms为默认值，如果机械臂跟随效果差可以调大

    # 声明求解器类，第一个参数可选为("RM65B","RM65SF","RM75B","RM75SF")
    robot = QPIK("RM65B", dT)

    # 设置安装角度，工作坐标系以及工具坐标系，根据实际情况自己设置
    robot.set_install_angle([90, 180, 0], 'deg')
    robot.set_work_cs_params([0, 0, 0, 0, 0, 0, 0])
    robot.set_tool_cs_params([0, 0, 0, 0, 0, 0, 0])

    # 设置关节限位，与实际情况一致(可选)，如果不设置会对应默认机械臂限位
    robot.set_joint_limit_max([ 175,  130,  130,  175,  125,  300], 'deg') 
    robot.set_joint_limit_min([-175, -130, -130, -175, -125, -300], 'deg')

    # 对于RM65，肘部为3轴，根据安装角度自己设置，如果肘部内拐是正值，需要设置肘部最小值，那么根据如下设置方式即可，不要设置为0，
    # 因为肘部打直会抖，如果肘部内拐为负值，需要设置为 robot.set_6dof_elbow_max_angle(-3, 'deg')
    robot.set_6dof_elbow_min_angle(3, 'deg')

    # 每个关节在一个周期内的移动最大增量权重，每个值的大小为0-1，默认值为0.6，如果肘部打直后缩回很慢，可以调整第3个值大一点
    robot.set_dq_max_weight([0.6,0.6,0.6,0.1,0.6,0.6])

    robot.set_error_weight([1, 1, 1, 1, 1, 1]) #(可选)

    q_ref = np.array([0, 25, 90, 0, 65, 0]) * deg2rad
    Td = robot.fkine(q_ref)
    Td[0, 3] = Td[0, 3] + 0.01

    q_sol = robot.sovler(q_ref, Td)

    print(f"q_ref: {q_ref}")
    print(f"q_sol: {q_sol}")
    print(f"T_ref:\n{Td}")
    print(f"T_sol:\n{robot.robot.fkine(q_sol)}")



###########################  以下程序针对RM75的参数设置  ####################
def RM75_Demo():

    dT = 0.01 # 用户数据的下发周期(透传周期)，与实际机械臂周期对应，为了安全起见先设置小一点，10ms为默认值，如果机械臂跟随效果差可以调大

    # 声明求解器类，第一个参数可选为("RM65B","RM65SF","RM75B","RM75SF")
    robot = QPIK("RM75B", dT)

    # 设置安装角度，工作坐标系以及工具坐标系，根据实际情况自己设置
    robot.set_install_angle([90, 180, 0], 'deg')
    robot.set_work_cs_params([0, 0, 0, 0, 0, 0, 0])
    robot.set_tool_cs_params([0, 0, 0, 0, 0, 0, 0])

    # 此程序限制的是4轴，在3轴靠近0的情况下，根据安装角度自己设置，如果肘部内拐是正值，那么需要定义肘部(4轴)的最小角度
    # 那么根据如下设置方式即可，不要设置为0，因为肘部打直会抖，如果肘部内拐为负值，需要设置为 robot.set_7dof_elbow_max_angle(-3, 'deg')
    robot.set_7dof_elbow_min_angle(3, 'deg')

    # 此程序限制的是3轴，由于对4轴的限位是受到3轴的影响，因此3轴的限位最好在0附近，所给的值为参考值，可以根据实际需要改动
    robot.set_7dof_q3_min_angle(-30,'deg')
    robot.set_7dof_q3_max_angle(30,'deg')

    # 每个关节在一个周期内的移动最大增量权重，每个值的大小为0~1之间，默认值为0.6，如果肘部打直后缩回很慢，可以调整第四个值大一点
    robot.set_dq_max_weight([0.6,0.6,0.6,0.6,0.6,0.6,0.6])

    q_ref = np.array([0, 25, 90, 0, 65, 0 ,3]) * deg2rad
    Td = robot.fkine(q_ref)
    Td[0, 3] = Td[0, 3] + 0.01

    q_sol = robot.sovler(q_ref, Td)

    print(f"q_ref: {q_ref}")
    print(f"q_sol: {q_sol}")
    print(f"T_ref:\n{Td}")
    print(f"T_sol:\n{robot.robot.fkine(q_sol)}")

##########################################################################################
#####   所有程序在调试前务必要在仿真模式下测试，如果没问题再上真实机械臂测试  ！！！   #####
##########################################################################################
if __name__ == '__main__':
    # RM65_Demo()
    RM75_Demo()
