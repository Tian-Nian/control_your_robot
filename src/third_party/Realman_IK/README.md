# Teleoperation IK

**Author:**    Daryl/Ray/Leon

**Update:**   2024/09/29

### I 安装qp求解器工具包

```shell
cd qp-tools\python 
python setup.py install  
```

若安装出错,请根据提示安装对应的缺省环境.

安装通过后,可通过`import qpSWIFT`语句检查工具是否安装成功.


---

### II 项目结构

- qp-tools:       包含qp求解器工具包
- ik_qp.py:       用于带约束的逆运动学求解器定义, 包括各种参数的设置接口, 本工程核心文件
- ik_rbtdef.py:   定义了RM65\RM75等各机型的MDH参数及常用机器人学算法, 若修改DH参数可在此文件中修改.
- ik_rbtutils.py: 定义了工具函数, 用于计算位姿差、四元数\欧拉角与齐次变换矩阵的转换等.
- ik_loadlib.py:  定义了加载动态链接库的相关函数, 用于调用动态链接库中的解析逆解函数, 当前已屏蔽.
- demo.py:        示例程序, 展示如何使用ik_qp.py求解器求解遥操作过程中的逆运动学问题.

---

### III ik_qp.py使用说明

1. 使用前先定义一个QPIK的类:
   ```python
   robot = QPIK(type, dT)
   ```

   输入参数: 
      - type: 机器人类型,可选参数为("RM65B","RM65SF","RM75B","RM75SF")
      - dT: 用户数据的下发周期(即控制周期,需与透传周期保持一致), unit: second

2. 设置机器人相关参数

   - 设置安装角度：
      ```python
      robot.set_install_angle(angle)
      ```
      - 输入参数:
         - angle: 机器人安装角度, unit: rad, 默认为[0, 0, 0]

   - 设置工作坐标系：
      ```python
      robot.set_work_cs_params(pose)
      ```
      - 输入参数:
         - pose: 工作坐标系相对于基坐标系的位姿[x, y, z, r, p, y], unit: m\rad, 默认为[0, 0, 0, 0, 0, 0]

   - 设置工具坐标系：
      ```python
      robot.set_tool_cs_params(pose)
      ```
      - 输入参数:
         - pose: 工具坐标系相对于末端坐标系的位姿[x, y, z, r, p, y], unit: m\rad, 默认为[0, 0, 0, 0, 0, 0]

3. 由于qp方法可以进行带约束求解, ik_qp.py中提供了修改求解过程中各关节位置限制范围的接口与关节速度限制范围接口:

   - 设置最大关节位置限制范围接口:    
      ```python
         robot.set_joint_limit_max(angle) 
      ```

      - 输入参数:
         - angle: 用户自定义求解过程中关节限制范围的上限, unit: rad, 默认为原机械臂关节上限

   - 设置最小关节位置限制范围接口:    
      ```python
      robot.set_joint_limit_min(angle)
      ```
   
      - 输入参数:
         - angle: 用户自定义求解过程中关节限制范围的下限,unit: rad, 默认为原机械臂关节位置下限

   - 设置rm65系列机械臂肘部关节最大位置限制范围接口（注意要在set_joint_limit_max和set_joint_limit_min设置）:
      ```python
      robot.set_6dof_elbow_max_angle(angle)
      ```
      - 输入参数:
         - angle: 用户自定义求解过程中肘部关节限制范围的上限, unit: rad, 默认为原机械臂肘部关节上限
      
   - 设置rm65系列机械臂肘部关节最小位置限制范围接口（注意要在set_joint_limit_max和set_joint_limit_min设置）:
      ```python
      robot.set_6dof_elbow_min_angle(angle)
      ```
      - 输入参数:
         - angle: 用户自定义求解过程中肘部关节限制范围的下限,unit: rad, 默认为原机械臂肘部关节位置下限
   
   - 设置rm75系列机械臂肘部关节最大位置限制范围接口（注意要在set_joint_limit_max和set_joint_limit_min设置）:
      ```python
      robot.set_7dof_elbow_max_angle(angle)
      ```
      - 输入参数:
         - angle: 用户自定义求解过程中肘部关节限制范围的上限,unit: rad, 默认为原机械臂肘部关节位置下限
      
   - 设置rm75系列机械臂肘部关节最小位置限制范围接口（注意要在set_joint_limit_max和set_joint_limit_min设置）:
      ```python
      robot.set_7dof_elbow_min_angle(angle)
      ```
      - 输入参数:
         - angle: 用户自定义求解过程中肘部关节限制范围的下限,unit: rad, 默认为原机械臂肘部关节位置下限
         
   
   通过以上四个接口, 可以约束求解过程中rm65(对应关节3)与rm75(对应关节4)肘部关节的位置范围, 从而使其在遥操作过程中的动作更拟人.
   
   以RM65机械臂为例, 如下图所示, 机械臂主要操作区域为胸前橙色区域, 此时肘部(蓝色圆圈)朝外, 关节3角度大于0, 要想肘部一直朝外, 那么应该调用`robot.set_6dof_elbow_min_angle(3度)`, 至于具体最大设置为3度还是最小设置为-3度,视实际情况而定, 这还取决于机械臂的安装方式. 这里之所以没有完全设置为0度,是防止因为用户操作使得机械臂打直后肘部产生来回摆动的问题。
   
   对于RM75机械臂而言, 肘部位置除了受关节4的角度限制外, 还受零空间的影响, 可以根据实际情况设置关节3的范围(关节3角度直接影响肘部位置), 假设RM75机械臂初始位置同下图, 此时关节3角度为0, 那么可以设置:
   
   ```python
   robot.set_7dof_q3_min_angle(-np.pi/6)
   robot.set_7dof_q3_max_angle(np.pi/6)
   ```
   
   这样, 关节3位置被限制为[-π/6, π/6], 即肘部一直朝外.


   <img src="img/Snipaste_2024-09-26_17-53-09.png" align="middle" width="80%"/>


   - 修改关节速度限制范围接口:
      ```python
      robot.set_joint_velocity_limit(velocity)
      ```

      - 输入参数:
         - velocity: 用户自定义求解过程中关节速度限制范围, unit: rad/s, 默认为原机械臂关节速度限制

   - 设置关节速度权重接口:
      ```python
      robot.set_dq_max_weight(weight)
      ```

      - 输入参数:
         - weight: 用户定义的速度权重,即关节i的最大速度会被设置为`(dq_max_new[i] = dq_max_now[i] * weight[i])`, 范围:0~1, 对于6DOF机械臂, 关节4默认为0.1, 其余关节默认为1.0, 对于7DOF机械臂, 关节5默认为0.1, 其余关节默认为1.0, 如此限制的目的是为了限制机械臂腕部速度, 防止因腕部奇异导致的腕部快速转动,但需注意结合仿真测试的实际情况进行调试。
      
      > **注意:** 由于关节速度限制的设置会影响求解结果, 因此, 建议在实际使用中根据实际需求进行约束, 例如: 当给定位姿超出机械臂操作范围时, 由于多解原因, 会导致肘部关节来回摆动, 此时可以通过约束肘部关节的速度权重来限制摆动, 但此时在正常工作空间内可能会影响动作的响应速度(因为肘部关节在最大程度上影响位置).

3. 用户还可以根据实际情况设定求解权重:

   - 设定求解权重接口:
      ```python
      robot.set_error_weight(weight)
      ```
      - 输入参数:
         - weight: 求解权重,为一个六维向量,分别对应末端位姿的x,y,z,r,p,y,对应的权重值设的越大, 在求解过程中会更加注重该位姿参数误差收敛至0,范围:0~1, 默认为全1向量

4. 求解接口:

   ```python
   q_solve = robot.sovler(q_ref, Td)
   ```
   - 输入参数:
      - q_ref: 上一时刻求解的关节角度, unit: rad
      - Td: 当前时刻期望的机械臂末端位姿的4X4的齐次变换矩阵
   
   - 返回值:
      - q_sovle: 求解结果, unit: rad

---


### IV 示例代码
(rm65系列)
```python
import ik_qp

if __name__ == '__main__':
    dT = 0.01 # 10ms
    robot = QPIK("RM65B", dT)

    robot.set_install_angle([90, 180, 0], 'deg')

    robot.set_work_cs_params([0, 0, 0, 0, 0, 0, 0])
    robot.set_tool_cs_params([0, 0, 0, 0, 0, 0, 0])

    robot.set_joint_limit_max([ 175,  130,  130,  175,  125,  300], 'deg')
    robot.set_joint_limit_min([-175, -130, -130, -175, -125, -300], 'deg')

    robot.set_6dof_elbow_min_angle(3, 'deg')
    robot.set_6dof_elbow_max_angle(178, 'deg')

    robot.set_dq_max_weight([1, 1, 1, 1, 1, 1])
    robot.set_error_weight([1, 1, 1, 1, 1, 1])

    q_ref = np.array([0, 25, 90, 0, 65, 0]) * deg2rad
    Td = robot.fkine(q_ref)
    Td[0, 3] = Td[0, 3] + 0.01

    q_sol = robot.sovler(q_ref, Td)

    print(f"q_ref: {q_ref}")
    print(f"q_sol: {q_sol}")
    print(f"T_ref:\n{Td}")
    print(f"T_sol:\n{robot.fkine(q_sol)}")
```

(rm75系列)
```python
import ik_qp

if __name__ == '__main__':
    dT = 0.01 # 10ms
    robot = QPIK("RM75B", dT)

    robot.set_install_angle([90, 180, 0], 'deg')

    robot.set_work_cs_params([0, 0, 0, 0, 0, 0, 0])
    robot.set_tool_cs_params([0, 0, 0, 0, 0, 0, 0])

    robot.set_joint_limit_max([ 178,  130,  178,  135,  178,  128, 360], 'deg')
    robot.set_joint_limit_min([-178, -130, -178, -135, -178, -128, -360], 'deg')

    robot.set_7dof_elbow_min_angle(3, 'deg')
    robot.set_7dof_elbow_max_angle(178, 'deg')

    robot.set_7dof_q3_min_angle(-30,'deg')
    robot.set_7dof_q3_max_angle(30,'deg')

    robot.set_dq_max_weight([1, 1, 1, 1,1, 1, 1])
    robot.set_error_weight([1, 1, 1, 1, 1, 1])

    q_ref = np.array([0, 25, 0, 90, 0, 65, 0]) * deg2rad
    Td = robot.fkine(q_ref)
    Td[0, 3] = Td[0, 3] + 0.01

    q_sol = robot.sovler(q_ref, Td)

    print(f"q_ref: {q_ref}")
    print(f"q_sol: {q_sol}")
    print(f"T_ref:\n{Td}")
    print(f"T_sol:\n{robot.fkine(q_sol)}")
```
---


### V 其他说明

1. 若给定的位姿超出机械臂的操作范围, 肘部关节(以RM65为例, 肘部关节为关节3)会发生小幅度摆动(多解问题), 导致求解结果不稳定. 因此, 建议在实际使用中尽可能的限制具体传给机械臂的位姿范围, 避免此类问题发生.

2. 要确保QPIK的DH参数与实际机械臂的DH参数一致, 否则可能导致错误的求解结果.

3. 确保正确的设置了约束条件, 否则可能导致错误的求解结果, 建议机械臂在网页示教器界面设置仿真模式, 若遥操作动作响应正常, 再上实机测试.

4. 对于周期控制, 常见的用户大多都是直接用sleep(dT)来操作, 忽略了实际的控制程序执行时间, 一种建议的编程方式为:

   ```python
   import time
   
   dT = 0.01 # 10ms
   while True:
      t_start = time.time()
      
      # 控制程序
   
      t_end = time.time()
      t_sleep = dT - (t_end - t_start)
      if t_sleep > 0:
         time.sleep(t_sleep)
   ```