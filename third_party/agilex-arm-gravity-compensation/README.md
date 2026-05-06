# 重力补偿

## 安装匹诺曹库

```bash
sudo apt install ros-$ROS_DISTRO-pinocchio ros-$ROS_DISTRO-hpp-fcl ros-$ROS_DISTRO-coal
```

## 激活CAN模块

```bash
bash can_activate.sh
```

## 运行程序

运行前需先安装新的SDK，SDK代码仓库：[agilexrobotics/pyAgxArm](https://github.com/agilexrobotics/pyAgxArm)

```bash
python3 main.py
```

> 注意：SDK目前已在内部处理了力矩系数问题，move_mit的力矩对外统一为1:1，已经无需关注力矩系数问题，只需关注初始化时的固件版本选择即可。

## 程序默认参数（需根据实际情况修改）

- `机械臂类型`：piper_x，无夹爪
- `固件版本选择`：1.8-3到1.8-7
- `机械臂底座安装姿态`：水平安装
