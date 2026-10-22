---
name: add-device
description: >-
  把新机械臂或传感器接到 control_your_robot：控制器放 src/robot/controller，传感器放 src/robot/sensor，用 YAML class_path 决定能否收命令，并申报 position、mit、gravity_compensation、impedance。
  在用户要接入新设备、改伺服模式、从配置启动某种控制，或使用 toolkits 里的重力补偿、阻抗、手眼之前使用。
---

# 接入设备

运行时约定在仓库根目录的 `agent.md`。这里只写代码里看不出的合同。

## 放哪里

- 会动的设备放 `src/robot/controller/`，继承 `ArmController`。
- 只观测的设备放 `src/robot/sensor/`。
- YAML 的 `class_path` 含 `.sensor.` 时，`Session` 拒绝命令。`class_path` 在 `robot.controller` 下才收 `cyr call`。
- 构造参数进 `args`，`set_up` 的参数进 `setup`。启动模式是 `setup.control_mode`。

## 四种模式

标准名只有这四个，保持英文：

- `position`
- `mit`
- `gravity_compensation`
- `impedance`

每个臂在类上声明 `supported_modes`，只写这台设备的 SDK 真有的模式。`set_up` 和 `set_control_mode` 收到列表外的名字就报错，并把这台设备实际支持的模式写出来。

当前设备：

| 设备 | supported_modes |
|------|-----------------|
| PiperSim | `position`、`mit`、`gravity_compensation`、`impedance` |
| PiperAgx | `position`、`mit`、`gravity_compensation`、`impedance` |
| Piper（piper_sdk） | `position`、`mit`、`gravity_compensation`、`impedance` |
| Y1 | `position`、`mit`、`gravity_compensation`、`impedance` |
| Marvin | `position`、`impedance`。`cartesian` 和 `force` 留在天机自己的切换里，也写在它的 `supported_modes` 上，不改名成上面四个之一 |

`position` 是默认。Piper 仿真的 `config/runtime/piper_sim.yml` 不写 `control_mode`，关节命令仍然直接改到目标。重力补偿和阻抗用 `piper_sim_gravity.yml`、`piper_sim_impedance.yml`。

状态里要能读到 `control_mode`。工具包靠这个字段判断当前模式，所以示例配置的 `publish` 带上它。

## 伺服放在哪一侧

主机上要按周期积分力矩时，实现 `servo_step(dt)`。运行时在 `get()` 之前调用，`dt` 是该组件的 `1/rate_hz`。没有这个方法的设备保持原样。

仿真里的速度保护记在状态的 `servo_fault`，不要让它把服务打进安全锁存。

能发关节 MIT 的臂继承 `MitServo`。重力和阻抗由它每个周期算出 kp、kd、q_des 和前馈，再交给驱动。`servo_step` 只刷新这条指令，不在主机上积分。PiperAgx 用原来的 200 Hz 线程刷新，不在 `servo_step` 里再发一份。没有 URDF 时，`set_control_mode` 拒绝重力和阻抗。真机的 `joint_K`、`joint_D` 必须写在配置里。Marvin 没有关节 MIT，不走这个基类。

## 工具包

目录是 `toolkits/<name>/`，说明是该目录的 `agent.md`。

动手前先读那份说明，并运行它的检查。检查失败时，按它打印的问题问用户。不要跳过硬件条件，也不要假设仿真能看见标定板。

PiperSim 没有相机。手眼检查失败时，问相机组件名，以及眼在手上还是眼在手外。重力补偿若负载不在 URDF 里，用 cyr 下发关节目标，臂到位停稳后采集力矩，再辨识质量和质心。
