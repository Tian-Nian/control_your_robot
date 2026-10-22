# 重力补偿

连杆自己的重力矩来自 URDF：静止时逆动力学 τ = g(q)，用的是文件里每个连杆的质量和质心。Piper 准备姿态下大约是 `[0, -1.37, -1.88, 0.03, -0.31, 0.02]` N·m。这份数只托得住 URDF 里已经写上的连杆。

手上的负载、换过的夹爪、或者写错的质心，都不在这份数里。质量标轻了，松手会下坠；标重了，臂会被往上抬。质心标错时，不同姿态的残余力矩方向不一样。静止力矩里看不见惯量，辨识只做质量和质心。

先跑检查。负载还没写进 URDF 时，检查会要求先采集，不要直接看臂停住。

```bash
python toolkits/gravity_compensation/check.py
```

## 条件

- 软件：`supported_modes` 含 `gravity_compensation`，能读关节。采集还要能读该姿态的关节力矩：真机是 `force`，仿真是 `op: hold_torque` 写进 `result`。
- 硬件：没有重力补偿时人搬不动臂。点位用 `cyr call` 下发，臂自己走过去，停稳后再记力矩。至少 8 个，腕的朝向要拉开。姿态挤在一起时，质心有的方向解不出来。

只补偿 URDF 里已经写好的连杆时，检查加 `--tool-in-urdf`。

## 负载不在 URDF 里

1. 切到 `position`，臂才能执行关节目标：

```bash
cyr call left_arm '{"control_mode":"position"}'
```

2. 人下发点位。每条都是一次 `cyr call`，臂运动过去：

```bash
cyr call left_arm '{"joint":[0, 1.3, -1, 0, 0.5, 0]}'
```

走到停稳后记录这一拍的关节力矩。换一个差得开的目标再发下一条。采集脚本会等这些点：

```bash
python toolkits/gravity_compensation/collect.py --service piper_sim --arm left_arm
```

点位已经用 `cyr` 发过时，在脚本里按 Enter 就记当前姿态。也可以把 6 个关节贴进脚本，它用同一种 `{"joint": ...}` 命令下发，到点后记录。够了输入 `done`。残差是测到的力矩减去连杆 g(q)，对法兰负载的质量 m 和质心 c 做最小二乘。τ = J(q)^T [m g; (R c) × (m g)]。

仿真用同一组关节目标走一遍：

```bash
python toolkits/gravity_compensation/collect.py --demo
```

`--demo` 在法兰上挂一块 URDF 里没有的 0.35 kg 负载。辨识前，补偿力矩和真实负载对不上，腕会很快碰到速度保护。辨识后同一块负载能被托住。

3. 辨识结果写回补偿：`{"op":"set_payload","mass":...,"com":[...]}`。`collect.py` 会自己发。
4. 再切到 `gravity_compensation`，然后看臂是否停住：

```bash
cyr call left_arm '{"control_mode":"gravity_compensation"}'
python toolkits/gravity_compensation/run.py
```

## 负载已经在 URDF 里

1. 启动 `config/runtime/piper_sim_gravity.yml`。
2. `python toolkits/gravity_compensation/check.py --tool-in-urdf`
3. `python toolkits/gravity_compensation/run.py` 看臂留在准备姿态。`result.torque` 是当前补偿力矩。

真机写回的是 MIT 前馈：连杆 g(q) 加辨识出的法兰负载，再乘这台臂的 `torque_scale`。主机不积分。没有 URDF 的臂不做这一步。换夹爪以后重新采集。
