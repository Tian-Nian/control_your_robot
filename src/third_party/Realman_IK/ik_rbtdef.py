#! -*-conding=: UTF-8 -*-
#
################################################################################
#                                                                              #
# Author   : Leon                                                              #
# Date     : 2023/11/30                                                        #
# Copyright: Copyright (c) 2018-2024 RealMan Co., Ltd.. All rights reserved.   #
#                                                                              #
################################################################################
from math import pi
import numpy as np
from ik_rbtutils import *
# from ik_loadlib import *

ROBOT_DOF_MAX = 7

def get_mdh_Ti(i, dh, theta):
    d = dh.d[i]
    a = dh.a[i]
    alpha = dh.alpha[i]
    offset = dh.offset[i]
    q  = theta[i] + offset

    Ti  = np.mat(np.eye(4,4))
    s_q = np.sin(q)
    c_q = np.cos(q)
    s_alpha = np.sin(alpha)
    c_alpha = np.cos(alpha)

    # n
    Ti[0,0] = c_q
    Ti[1,0] = s_q * c_alpha
    Ti[2,0] = s_q * s_alpha

    # o
    Ti[0,1] = -s_q
    Ti[1,1] =  c_q * c_alpha
    Ti[2,1] =  c_q * s_alpha

    # a
    Ti[0,2] = 0
    Ti[1,2] = -s_alpha
    Ti[2,2] =  c_alpha

    # t
    Ti[0,3] =  a
    Ti[1,3] = -s_alpha * d
    Ti[2,3] =  c_alpha * d

    return Ti

class MDH():
    def __init__(self,d,a,alpha,offset):
        self.d = d
        self.a = a
        self.alpha = alpha
        self.offset = offset

class Robot():
    dof = 7
    __T = np.mat(np.eye(4,4))
    __Ti = [__T,__T,__T,__T,__T,__T,__T]
    __Twork = __T
    __Tbase = __T
    __Ttool = __T
    __install_angle = np.array([0,0,0])
    __work_params   = np.array([0,0,0,0,0,0])
    __tool_params   = np.array([0,0,0,0,0,0])
    
    def __init__(self, type):

        if type == 'RM65B':
            self.dof = 6
            d = [0.2405, 0, 0, 0.210, 0, 0.144]
            a = [0, 0, 0.256, 0, 0, 0]
            alpha  = [0, pi/2, 0, pi/2, -pi/2, pi/2]
            offset = [0, pi/2, pi/2, 0, 0, 0]
            self.__qlim_min = np.array([-178, -130, -135, -178, -128, -360]) * deg2rad
            self.__qlim_max = np.array([178, 130, 135, 178, 128, 360]) * deg2rad
            self.__dq_max   = np.array([180, 180, 220, 220, 220, 220]) * deg2rad
        elif type == 'RM65SF':
            self.dof = 6
            d = [0.2405, 0, 0, 0.210, 0, 0.1725]
            a = [0, 0, 0.256, 0, 0, 0]
            alpha  = [0, pi/2, 0, pi/2, -pi/2, pi/2]
            offset = [0, pi/2, pi/2, 0, 0, 0]
            self.__qlim_min = np.array([-178, -130, -135, -178, -128, -360]) * deg2rad
            self.__qlim_max = np.array([178, 130, 135, 178, 128, 360]) * deg2rad
            self.__dq_max   = np.array([180, 180, 220, 220, 220, 220]) * deg2rad
        elif type == 'RML63B':
            self.dof = 6
            d = [0.172, 0, 0, 0.405, 0, 0.115]
            a = [0, -0.086, 0.380, 0.069, 0, 0]
            alpha  = [0, -pi/2, 0, pi/2, -pi/2, -pi/2]
            offset = [0, -pi/2, pi/2, 0, pi, pi]
            self.__qlim_min = np.array([-178, -178, -178, -178, -178, -360]) * deg2rad
            self.__qlim_max = np.array([178, 178, 145, 178, 178, 360]) * deg2rad
            self.__dq_max   = np.array([180, 180, 220, 220, 220, 220]) * deg2rad
        elif type == 'RML63SF':
            self.dof = 6
            d = [0.172, 0, 0, 0.405, 0, 0.1435]
            a = [0, -0.086, 0.380, 0.069, 0, 0]
            alpha  = [0, -pi/2, 0, pi/2, -pi/2, -pi/2]
            offset = [0, -pi/2, pi/2, 0, pi, pi]
            self.__qlim_min = np.array([-178, -178, -178, -178, -178, -360]) * deg2rad
            self.__qlim_max = np.array([178, 178, 145, 178, 178, 360]) * deg2rad
            self.__dq_max   = np.array([180, 180, 220, 220, 220, 220]) * deg2rad
        elif type == 'RM75B':
            self.dof = 7
            d = [0.2405, 0, 0.256, 0, 0.210, 0, 0.1440]
            a = [0, 0, 0, 0, 0, 0, 0]
            alpha  = [0, -pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2]
            offset = [0, 0, 0, 0, 0, 0, 0]
            self.__qlim_min = np.array([-178, -130, -178,-135, -178, -128, -360]) * deg2rad
            self.__qlim_max = np.array([178, 130, 178, 135, 178, 128, 360]) * deg2rad
            self.__dq_max   = np.array([180, 180, 220, 220, 220, 220, 220]) * deg2rad
        elif type == 'RM75SF':
            self.dof = 7
            d = [0.2405, 0, 0.256, 0, 0.210, 0, 0.1725]
            a = [0, 0, 0, 0, 0, 0, 0]
            alpha  = [0, -pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2]
            offset = [0, 0, 0, 0, 0, 0, 0]
            self.__qlim_min = np.array([-178, -130, -178,-135, -178, -128, -360]) * deg2rad
            self.__qlim_max = np.array([178, 130, 178, 135, 178, 128, 360]) * deg2rad
            self.__dq_max   = np.array([180, 180, 220, 220, 220, 220, 220]) * deg2rad
        else:
            raise Exception(f"[ERROR] Unknown type.")
        
        self.__type = type
        self.__dh = MDH(d=d,a=a,alpha=alpha,offset=offset)
    
    # 获取机器人类型
    def get_robot_type(self):
        return self.__type

    # 获取MDH参数
    def get_mdh(self):
        return self.__dh

    def get_Ti(self, i):
        return self.__Ti[i]
    
    # 获取关节限位
    def get_qlim(self, type='rad'):
        if type=='deg':
            return self.__qlim_max*rad2deg,self.__qlim_min*rad2deg
        return self.__qlim_max,self.__qlim_min
    
    # 设置关节限位
    def set_qlim(self,q_max,q_min,type='rad'):
        q_max = np.array(q_max)
        q_min = np.array(q_min)
        if type=='deg':
            self.__qlim_max = q_max * deg2rad
            self.__qlim_min = q_min * deg2rad
        else:
            self.__qlim_max = q_max
            self.__qlim_min = q_min
    
    # 设置关节最大速度
    def get_dq_max(self, type='rad'):
        if type=='deg':
            return self.__dq_max*rad2deg
        return self.__dq_max
    
    # 获取关节最大速度
    def set_dq_max(self, dq_max, type='rad'):
        dq_max = np.array(dq_max)
        if type=='deg':
            self.__dq_max = dq_max * deg2rad
        else:
            self.__dq_max = dq_max
    
    # 获取安装角度
    def get_install_angle(self):
        return self.__install_angle
    
    # 设置安装角度
    def set_install_angle(self, angle, unit='rad'):
        angle_ = np.array(angle)

        if unit == 'deg':
            angle_ = angle_ * deg2rad

        self.__install_angle = angle_
        Tbase = pose_to_matrix([0,0,0,angle_[0],angle_[1],angle_[2]])
        self.__Tbase = Tbase
    
    # 获取工作坐标系参数
    def get_work_cs_params(self):
        return self.__work_params
    
    # 设置工作坐标系参数
    def set_work_cs_params(self, pose):
        self.__work_params = np.array(pose)
        self.__Twork = pose_to_matrix_inv(pose)

    # 获取工具坐标系参数
    def get_tool_cs_params(self):
        return self.__tool_params
    
    # 设置工作坐标系参数
    def set_tool_cs_params(self, pose):
        self.__tool_params = np.array(pose)
        self.__Ttool = pose_to_matrix(pose)
    
    def get_Twork(self):
        return self.__Twork
    
    def get_Tbase(self):
        return self.__Tbase
    
    def get_Ttool(self):
        return self.__Ttool
    
    # 正运动学
    def fkine(self, q)->np.mat:
        T0n = np.mat(np.eye(4,4))
        for i in range(0, self.dof):
            self.__Ti[i] = get_mdh_Ti(i,self.__dh,q)
            T0n = T0n @ self.__Ti[i]

        self.__T = self.__Twork @ self.__Tbase @ T0n @ self.__Ttool
        return self.__T
    
    # tool系下的雅可比
    def jacob_Jn(self,q)->np.mat:
        n = self.dof
        q = np.array(q)
        
        J = np.mat(np.zeros((6, n), dtype=q.dtype))
        U = self.__Ttool

        for j in range(n - 1, -1, -1):
            J[0,j] = -U[0, 0] * U[1, 3] + U[1, 0] * U[0, 3]
            J[1,j] = -U[0, 1] * U[1, 3] + U[1, 1] * U[0, 3]
            J[2,j] = -U[0, 2] * U[1, 3] + U[1, 2] * U[0, 3]
            J[3,j] = U[2, 0]
            J[4,j] = U[2, 1]
            J[5,j] = U[2, 2]

            U = get_mdh_Ti(j, self.__dh, q) @ U

        return np.mat(J)
    
    # work系下的雅可比
    def jacob_Jw(self,q)->np.mat:
        Twt = self.fkine(q)
        JT  = np.mat(np.zeros((6,6)))
        Rwt = T2r(Twt)
        Jn = self.jacob_Jn(q)
        JT[0:3,0:3] = Rwt
        JT[3:, 3:]  = Rwt
        return JT * Jn
    
    # def ikine_analysis_solution(self, Twt, q0=np.zeros(ROBOT_DOF_MAX), is_cartesian_space_planning=True):
    #     '''
    #     解析解

    #     @param Twt 工作坐标系下描述的齐次变换矩阵
    #     @param q0  参考角度, unit: rad, default: [0,0,0,0,0,0]
    #     @param is_cartesian_space_planning  是否启用迭代选取q3, unit: bool, default: True
    #                                         如果是笛卡尔连续轨迹, 则启用;
    #                                         如果是只求某一点位姿逆解, 则不启用.

    #     @return q   逆解角度, unit: rad
    #             res 逆解状态, success:1 failed:0
    #     '''
    #     T0w = np.linalg.inv(self.__Twork * self.__Tbase)
    #     Ttn = np.linalg.inv(self.__Ttool)
    #     T0n = T0w * Twt * Ttn

    #     if self.__type == 'RM65B':
    #         q_max = [self.__qlim_max[0], self.__qlim_max[1], self.__qlim_max[2], self.__qlim_max[3], self.__qlim_max[4], self.__qlim_max[5]]
    #         q_min = [self.__qlim_min[0], self.__qlim_min[1], self.__qlim_min[2], self.__qlim_min[3], self.__qlim_min[4], self.__qlim_min[5]]
    #         q,res = lib_ikine(RobotType.RM65, SensorType.B, q0, T0n.tolist(), q_max, q_min, not is_cartesian_space_planning)
    #     elif self.__type == 'RM65SF':
    #         q_max = [self.__qlim_max[0], self.__qlim_max[1], self.__qlim_max[2], self.__qlim_max[3], self.__qlim_max[4], self.__qlim_max[5]]
    #         q_min = [self.__qlim_min[0], self.__qlim_min[1], self.__qlim_min[2], self.__qlim_min[3], self.__qlim_min[4], self.__qlim_min[5]]
    #         q,res = lib_ikine(RobotType.RM65, SensorType.SF, q0, T0n.tolist(), q_max, q_min, not is_cartesian_space_planning)
    #     elif self.__type == 'RML63B':
    #         q_max = [self.__qlim_max[0], self.__qlim_max[1], self.__qlim_max[2], self.__qlim_max[3], self.__qlim_max[4], self.__qlim_max[5]]
    #         q_min = [self.__qlim_min[0], self.__qlim_min[1], self.__qlim_min[2], self.__qlim_min[3], self.__qlim_min[4], self.__qlim_min[5]]
    #         q,res = lib_ikine(RobotType.RML63II, SensorType.B, q0, T0n.tolist(), q_max, q_min, not is_cartesian_space_planning)
    #     elif self.__type == 'RML63SF':
    #         q_max = [self.__qlim_max[0], self.__qlim_max[1], self.__qlim_max[2], self.__qlim_max[3], self.__qlim_max[4], self.__qlim_max[5]]
    #         q_min = [self.__qlim_min[0], self.__qlim_min[1], self.__qlim_min[2], self.__qlim_min[3], self.__qlim_min[4], self.__qlim_min[5]]
    #         q,res = lib_ikine(RobotType.RML63II, SensorType.SF, q0, T0n.tolist(), q_max, q_min, not is_cartesian_space_planning)
    #     elif self.__type == 'RM75B':
    #         q_max = [self.__qlim_max[0], self.__qlim_max[1], self.__qlim_max[2], self.__qlim_max[3], self.__qlim_max[4], self.__qlim_max[5], self.__qlim_max[6]]
    #         q_min = [self.__qlim_min[0], self.__qlim_min[1], self.__qlim_min[2], self.__qlim_min[3], self.__qlim_min[4], self.__qlim_min[5], self.__qlim_min[6]]
    #         q,res = lib_ikine(RobotType.RM75, SensorType.B, q0, T0n.tolist(), q_max, q_min, not is_cartesian_space_planning)
    #     elif self.__type == 'RM75SF':
    #         q,res = lib_ikine(RobotType.RM75, SensorType.SF, q0, T0n.tolist(), q_max, q_min, not is_cartesian_space_planning)
    #     else:
    #         raise Exception(f"[ERROR] Unknown type.")

    #     max_dq = np.max(np.abs([q[i]-q0[i] for i in range(self.dof)]))
    #     if max_dq > 90.0*deg2rad or res != 0:
    #         q = q0
    #     return q,res
    
    # # 逆运动学
    # def ikine(self, Td, q0=None, is_cartesian_space_planning=True):
    #     '''
    #     逆解函数.

    #     @param Td 工作坐标系下描述的齐次变换矩阵
    #     @param q0  参考角度, unit: rad
    #     @param is_cartesian_space_planning  是否启用迭代选取q3, unit: bool, default: True
    #                                         如果是笛卡尔连续轨迹, 则启用;
    #                                         如果是只求某一点位姿逆解, 则不启用.
    #     @return q   逆解角度, unit: rad
    #             res 逆解状态
    #                         成功:0 
    #                         失败:-1 
    #     '''
    #     if q0 is None:
    #         q0 = [0, 0, 0, 0, 0, 0, 0]

    #     Td = np.mat(Td)

    #     # 调用解析解
    #     q,res=self.ikine_analysis_solution(Td, q0, is_cartesian_space_planning)
    #     return q[:self.dof], res
            
