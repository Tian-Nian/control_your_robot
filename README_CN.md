[![中文](https://img.shields.io/badge/中文-简体-blue)](./README_CN.md)
[![English](https://img.shields.io/badge/English-English-green)](./README.md)

# 控制你的机器人

这个分支把控制器和传感器跑成一个共享内存服务。客户端按组件名读最新观测，并下发指令。

```bash
pip install -e .
cyr start --config config/runtime/test_robot.yml
cyr snapshot
cyr stop
```

要用本仓库安装出来的 `cyr`。Debian 上的 `/usr/bin/cyr` 是另一个程序。

## 命令

| 命令 | 作用 |
|------|------|
| `cyr start --config FILE` | 后台启动 |
| `cyr serve FILE` | 前台运行 |
| `cyr stop` | 安全停止设备并关掉服务 |
| `cyr restart` | 用上次的配置重启 |
| `cyr status` | 查看服务是否在运行 |
| `cyr hmi` | 打开终端状态界面。它不会启动服务 |
| `cyr echo NAME` | 打印一个组件的最新观测 |
| `cyr snapshot` | 把全部组件打成一行 JSON |
| `cyr call NAME JSON` | 发送一条指令。退出码 0 表示节点已收下 |

```bash
cyr call left_arm '{"joint":[0,0,0,0,0,0]}'
cyr call left_arm '{"op":"reset"}'
cyr call left_arm '{"joint":[0,0,0,0,0,0]}' --delta
```

`accepted: true` 表示组件进程已经取走这条指令，不表示机械臂或手已经到位。

Python：

```python
from robot.runtime.session import Session

session = Session.attach()
session.snapshot()
session.call("left_arm", {"joint": [0, 0, 0, 0, 0, 0]})
session.reset("left_arm")
session.close()
```

快照里的图像只保留 shape、dtype 和字节数。

## 配置

见 `config/runtime/test_robot.yml`。顶层是 `name`、`shm_name`、`cmd_capacity`、`components`。每个组件有 `name`、`class_path`、`class_name`、`args`、`setup`、`publish`、`rate_hz`、`max_bytes`。`setup` 会传给 `set_up`。

`robot.sensor` 只发布观测。`robot.controller` 同时接受 `move` 和 `reset`。

服务把发现记录写到 `$XDG_STATE_HOME/cyr/runtime.json`，日志追加到 `$XDG_STATE_HOME/cyr/cyr.log`。

## Marvin 实验台

`config/runtime/marvin_lab.yml`：

- `left_arm` / `right_arm`：一台天机控制器，地址 `192.168.1.190`，左右臂在同一个进程里。启动后进入关节阻抗（`K` 为 16/8，`D` 为 1.8，速度和加速度比例 50），并带上原来的工具动力学。用 `{"control_mode":"position"}` 或 `{"control_mode":"impedance"}` 切换。关节单位是弧度。`eef` 是米制 xyz 加四元数 wxyz。
- `left_hand` / `right_hand`：舞肌 Hand 2，走 UDP，20 个关节，顺序和读回来的一致。进程会按控制周期持续刷新最新目标，每拍最多走 `max_joint_step_rad`。
- `left_glove` / `right_glove`：墨现触觉手套，发布原始压力。
- `cam_head` / `cam_left_wrist` / `cam_right_wrist`：奥比彩色相机。

```bash
cyr start --config config/runtime/marvin_lab.yml
cyr call left_arm '{"control_mode":"position"}'
cyr call left_hand '{"joint":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}'
```

运动指令失败会锁存安全状态并关掉整个服务。停止服务时天机臂保持当前反馈，舞肌手去使能。

## 测试

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest src/robot/runtime/tests -q --tb=line
```

## 目录

| 路径 | 内容 |
|------|------|
| `src/robot/runtime/` | 服务、共享内存、会话接口、终端界面 |
| `src/robot/controller/` | 机械臂、底盘、灵巧手和测试控制器 |
| `src/robot/sensor/` | 相机、触觉和测试传感器 |
| `config/runtime/` | `test_robot.yml`、`marvin_lab.yml` |
| `third_party/` | 厂商库 |
| `agent.md` | 给编码代理的说明 |
