import sys
sys.path.append('third_party/Realman_IK/')

import qpSWIFT
import numpy as np
from ik_rbtdef import *
from ik_rbtutils import *
import time
from ik_qp import *

from typing import List, Optional

class RM_IK:
    """
    基于 qpSWIFT 的 QP 逆运动学求解器封装类。
    """

    def __init__(self, model_name: str, dT: float):
        """
        初始化逆运动学求解器。

        参数:
            model_name (str): 机械臂型号，例如 "RM65B"
            dT (float): 控制周期 (秒)
        """
        self.robot = QPIK(model_name, dT)

    def set_up(
        self,
        install_angle: Optional[List[float]] = None,
        work_cs_params: Optional[List[float]] = None,
        tool_cs_params: Optional[List[float]] = None,
        joint_limit_max: Optional[List[float]] = None,
        joint_limit_min: Optional[List[float]] = None,
        dq_max_weight: Optional[List[float]] = None,
        error_weight: Optional[List[float]] = None,
    ):
        """
        设置逆解求解相关参数。

        参数:
            install_angle (List[float]): 安装角度，单位为 deg
            work_cs_params (List[float]): 工作坐标系参数
            tool_cs_params (List[float]): 工具坐标系参数
            joint_limit_max (List[float]): 各关节最大限位（deg）
            joint_limit_min (List[float]): 各关节最小限位（deg）
            dq_max_weight (List[float]): 各关节最大速度权重
            error_weight (List[float]): 末端误差权重
        """
        if install_angle:
            self.robot.set_install_angle(install_angle, 'deg')
        else:
            self.robot.set_install_angle([-37, 0, 180], 'deg') #这里请您直接将示教器安装信息对应填上,注意单位!!!!!!!!
    
        if work_cs_params:
            self.robot.set_work_cs_params(work_cs_params)
        else:
            self.robot.set_work_cs_params([0, 0, 0, 0, 0, 0, 0])
        if tool_cs_params:
            self.robot.set_tool_cs_params(tool_cs_params)
        else:
            self.robot.set_tool_cs_params([0, 0, 0, 0, 0, 0, 0])
        if joint_limit_max:
            self.robot.set_joint_limit_max(joint_limit_max, 'deg')
        else:
            self.robot.set_joint_limit_max([ 178,  130,  135,  178,  128,  360], 'deg')  #这里默认是机械臂的默认最大最小关节限位
        if joint_limit_min:
            self.robot.set_joint_limit_min(joint_limit_min, 'deg')
        else:
            self.robot.set_joint_limit_min([-178, -130, -135, -178, -128, -360], 'deg')
        if dq_max_weight:
            self.robot.set_dq_max_weight(dq_max_weight)
        else:
            self.robot.set_dq_max_weight([1,1,1,1,1,1])

        if error_weight:
            self.robot.set_error_weight(error_weight)
        else:
            self.robot.set_error_weight([1, 1, 1, 1, 1, 1]) #(可选)

    def compute_fk(self, q_ref):
        """
        使用给定的关节角进行正向运动学求解。

        参数:
            q_ref (np.ndarray): 关节角（单位 rad）

        返回:
            np.ndarray: 末端位姿矩阵
        """
        return self.robot.fkine(q_ref)

    def solve(self, q_ref, Td):
        """
        使用当前参考姿态和目标末端位姿计算逆解。

        参数:
            q_ref (np.ndarray): 当前参考关节角（rad）
            Td (np.ndarray): 目标末端位姿矩阵 (4x4)

        返回:
            np.ndarray: 求解出的关节角（rad）
        """
        return self.robot.sovler(q_ref, Td)

def RM65_Demo():
    dT = 0.033 # 用户数据的下发周期(透传周期)
    #这里一定要注意,如果你末端执行1cm的笛卡尔空间x,y,z位移,那么请注意, 假如你的位移很小是1cm左右,
    #但是你的周期是0.005,那么得到: 1cm/0.005 = 0.01m/0.005 = 2m/s , 要知道咱臂长应该也就1m左右,让末端在1s的时间走2m显然是不可能的,
    #这个可以参考示教器界面配置->安全配置中的最大线速度为0.25m/s,请让您的下发位置除以dT也就是速度不要超过它.

    # 声明求解器类，第一个参数可选为("RM65B","RM65SF","RM75B","RM75SF")
    robot = QPIK("RM65B", dT)

    # 设置安装角度，工作坐标系以及工具坐标系，根据实际情况自己设置
    robot.set_install_angle([-37, 0, 180], 'deg') #这里请您直接将示教器安装信息对应填上,注意单位!!!!!!!!
    robot.set_work_cs_params([0, 0, 0, 0, 0, 0, 0])
    robot.set_tool_cs_params([0, 0, 0, 0, 0, 0, 0])

    # 设置关节限位，与实际情况一致(可选)，如果不设置会对应默认机械臂限位
    robot.set_joint_limit_max([ 178,  130,  135,  178,  128,  360], 'deg')  #这里默认是机械臂的默认最大最小关节限位
    robot.set_joint_limit_min([-178, -130, -135, -178, -128, -360], 'deg')

    # 以下这条语句为限制RM65关节3的角度,因为机械臂打直后属于机械臂的边界奇异区,虽然咱的包也可以避,但是我还是不建议您真正的让肘部完全打直,因为
    # 这会极大的限制机械臂的运动导致较大的末端位姿误差,这里一定要注意,如果您设置的限位,请让机械臂在运动的起始构型要处于您设置的限位内!!!!!!!!
    # 这里默认注释,需要的话根据情况打开
    # robot.set_6dof_elbow_min_angle(3, 'deg')

    # 关节速度权重,这个的话是因为担心您觉得机械臂速度太快而设置的接口,默认为1也就是全速运行,注意哈,如果设置的不合适较小的话会造成跟踪的误差的,
    # 因为您相当于把关节限速了,本来就需要以某个速度到达的情况,限速后当然就有可能发生到不了的情况
    robot.set_dq_max_weight([1,1,1,1,1,1])

    # 这里哈是调整末端位姿的误差的,就比如避奇异时,您是希望末端x,y,z跟踪误差小,那就相应的把r,p,y部分调小,其他的情况一样考虑即可
    robot.set_error_weight([1, 1, 1, 1, 1, 1]) #(可选)

    #下面就是demo代码了,请您阅读
    sim_robot = Arm(RM65, "192.168.1.19") #这里IP请对应上您的 
    
    # 运行哪个demo取消掉注释即可,默认当前回字运动demo起作用
    # 回字运动部分
    sim_robot.Movej_Cmd([0, 25 , 90 , 0 , 65 , 0], 20, 0, 0, True)
    q_ref = np.array([0, 25, 90, 0 , 65 , 0]) * deg2rad
    Td = robot.fkine(q_ref)
    d = 40
    for i in range(4*d):
        start_time = time.time() 
        
        if( i < d):
            Td[0, 3] = Td[0, 3] + 0.002
        elif(i>=d and i<2*d):
            Td[1, 3] = Td[1, 3] + 0.002
        elif(i >= 2*d and i < 3*d):
            Td[0, 3] = Td[0, 3] - 0.002
        elif(i >= 3*d and i < 4*d):
            Td[1, 3] = Td[1, 3] - 0.002
      
        q_sol = robot.sovler(q_ref, Td)
            
        q_ref = q_sol
        q_sol = rad2deg*q_sol   #这里注意,q_sol解出来是弧度制的,你想通过透传下发需转换为度!!!!!!!!!!!!!!
    # 回字运动部分
#---------------------------------------------------------------------------------------------------#
    # 奇异规避部分
    # sim_robot.Movej_Cmd([0, 25 , 90 , -40.453 , 0 , 0], 20, 0, 0, True)
    # q_ref = np.array([0, 25, 90, -40.453 , 0 , 0]) * deg2rad
    # Td = robot.fkine(q_ref)
    # d = 80
    # for i in range(d):
    #     start_time = time.time() 
        
    #     if( i < d):
    #         Td[1, 3] = Td[1, 3] + 0.002

      
    #     q_sol = robot.sovler(q_ref, Td)
            
    #     q_ref = q_sol
    #     q_sol = rad2deg*q_sol   #这里注意,q_sol解出来是弧度制的,你想通过透传下发需转换为度!!!!!!!!!!!!!!
    # 奇异规避部分

        sim_robot.Movej_CANFD(q_sol,1)

        end_time = time.time()
        elapsed_time = end_time - start_time
        if elapsed_time < dT:
            time.sleep(dT - elapsed_time)

if __name__ == '__main__':
    from robotic_arm_package.robotic_arm import *
    RM65_Demo()