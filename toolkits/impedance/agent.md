# 关节阻抗

先跑检查。`ok` 为假时，把 `questions` 问给用户，不要往下做。

```bash
python toolkits/impedance/check.py
```

## 条件

- 软件：`supported_modes` 含 `impedance`，`setup` 里有 `joint_K` 和 `joint_D`，`publish` 里有 `joint` 和 `control_mode`。
- 硬件：PiperSim 没有额外硬件。真机要已上电，工作空间清空。`ask` 非空时先问用户，确认后加 `--confirm-ready`。

## 通过以后

1. 启动阻抗配置：

```bash
cyr start --config config/runtime/piper_sim_impedance.yml
```

2. 再跑检查，确认模式是 `impedance`。
3. 把第 2 轴目标加上 0.1 rad。臂应跟上，而不是一下子跳到目标：

```bash
python toolkits/impedance/run.py
```

仿真的 `joint_K` 默认 `[16, 16, 16, 8, 8, 8]`，`joint_D` 默认 `[1.8, 1.8, 1.8, 1.8, 1.8, 1.8]`。力矩里已经包含 `g(q)`。真机不套用这组默认值，`joint_K` 和 `joint_D` 要写在配置里，前馈仍是重力补偿。

没标定末端就不能进阻抗。`set_payload` 之后才放行。负载的质量和质心已经写在 URDF 里时，`setup` 设 `tool_in_urdf: true`，这时不再另加一份法兰负载。`piper_sim_impedance.yml` 属于这一种：仿真模型里没有额外末端。
