[![中文](https://img.shields.io/badge/中文-简体-blue)](./README_CN.md)
[![English](https://img.shields.io/badge/English-English-green)](./README.md)

# 控制你的机器人

这个分支把控制器和传感器跑成一个共享内存服务。客户端按组件名读最新观测，并下发指令。`main` 仍是采集和部署栈。这个分支里没有那套入口。

```bash
pip install -e .
cyr start --config config/runtime/test_robot.yml
cyr snapshot
cyr stop
```

要用本仓库安装出来的 `cyr`。Debian 上的 `/usr/bin/cyr` 是另一个程序。用 `which cyr` 确认。

没有硬件也能跑。`config/runtime/test_robot.yml` 用进程内的测试设备。`config/runtime/marvin_lab.yml` 是天机实验台。

## 命令

| 命令 | 作用 |
|------|------|
| `cyr start --config FILE` | 后台启动一台服务。同一个 `shm_name` 不会再启动一次 |
| `cyr serve FILE` | 前台运行 |
| `cyr stop` | 安全停止当前所有服务并清掉记录 |
| `cyr restart` | 恰好有一台服务时，用它上次的配置重启 |
| `cyr status` | 列出正在运行的服务。没有服务时退出码是 3 |
| `cyr hmi` | 打开终端状态界面。它不会启动服务，而且只有恰好一台服务时才会接上 |
| `cyr echo NAME` | 打印一个组件的最新原始观测。图像只显示形状 |
| `cyr snapshot` | 把全部组件打成一行 JSON。长数组只留形状 |
| `cyr call NAME JSON` | 发送一条指令。退出码 0 只在 `accepted` 为真时出现 |

```bash
cyr call left_arm '{"joint":[0,0,0,0,0,0]}'
cyr call left_arm '{"op":"reset"}'
cyr call left_arm '{"joint":[0,0,0,0,0,0]}' --delta
cyr call left_arm '{"op":"release"}'
cyr call left_arm '{"joint":[0,0,0,0,0,0]}' --timeout 15
```

`cyr hmi`：`q` 退出界面。`x` 之后按 `y` 或回车，会停掉当前所有服务。`n` 取消。方向键或 `k` / `j` 移动选中项。

命令行只附着唯一的一台服务。同时跑多台时，`echo`、`snapshot`、`call` 会失败。这时用 Python 的 `Session.attach(name)`。

## `accepted` 是什么意思

`cyr call` 和 `Session.call` 返回：

```json
{"accepted": true, "seq": 3, "error": "", "state": {}}
```

`accepted: true` 表示组件进程已经取走这条指令；开了关节跟踪时，还表示反馈已经进了容差。它不表示之后的传感器帧仍然停在这个目标上。

门控或跟踪失败时 `accepted` 为假。这个组件保持当前指令，后续命令都会被拒绝，直到发 `{"op":"release"}`。服务里的其他组件继续跑。驱动抛异常则不同：安全状态锁存，整个服务关掉。读观测失败只记在组件的 `error` 里，不锁存。

跟踪还在走的时候，节点不取下一条命令。新命令留在环里，直到这次跟踪结束、超时，或者环满后丢掉最旧的一条。

## Python 会话

```python
from robot.runtime.session import Session

session = Session.attach()                 # 或多台时 Session.attach("marvin_lab")
session.describe()
session.snapshot()                         # 图像和长度超过 8 的数组只留形状
session.read_raw("cam_head")               # 保留像素
session.call("left_arm", {"joint": [0, 0, 0, 0, 0, 0]}, timeout=15)
session.reset("left_arm", timeout=15)
session.close()
```

`attach(name)` 按服务的 `name` 或 `shm_name` 选择。只有一台时可以不写名字。多台时必须写。

`call` 先在 `timeout`（默认 2 秒）内等节点取走命令。这条命令一旦进入关节跟踪，等待会延长到该组件的 `motion.timeout_s` 再加 0.5 秒。超时要留够节点取出命令的时间。实验台天机臂的跟踪上限是 12 秒，长距离运动用 `--timeout 15` 或 `timeout=15`。

## 策略用的 harness

`robot.harness` 放在 `Session` 上面。它对齐各组件观测，把训练动作拆成组件命令，也可以记下一条 episode。

```python
from robot.harness import RobotHarness
from robot.runtime.session import Session

session = Session.attach("marvin_lab")
robot = RobotHarness(session, mode="joint", control_hz=15, max_skew_ms=80)
observation = robot.get_obs()
robot.start_episode("episode.json")
outcome = robot.step({
    "left_arm_joint": [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "left_ee_joint_state": [0.0] * 20,
})
robot.finish_episode(True, "放好了")
session.close()
```

`get_obs()` 保留图像像素，并给出 `skew_ms`、`within_skew`、`missing`。组件配置了身体文件时，观测里还有 `flange` 和 `palm`：米制 xyz 加四元数 wxyz。

`mode="joint"` 拒绝只有 `eef` 的臂目标。`mode="eef"` 仍把 `eef` 交给运行时；组件配了身体模型时，运行时解关节。

训练字段和组件命令的对应关系：

| 字段 | 命令 |
|------|------|
| `left_arm` / `right_arm` | `{"joint": ...}` 和/或 `{"eef": ...}` |
| `left_arm_joint`、`right_arm_joint` | 臂的 `joint` |
| `left_ee_pose`、`right_ee_pose` | 臂的 `eef`，长度 7，xyz + wxyz |
| `left_hand` / `right_hand`、`left_hand_joint`、`right_hand_joint`、`left_ee_joint_state`、`right_ee_joint_state` | 手的 `joint`，长度 20 |

手的顺序是拇指、食指、中指、无名指、小指。每指是 flex、abd、pip、dip。这和手控制器读回来的顺序一致。

`Episode` 写成 JSON。二维及以上的数组，或超过 32 个数的数组，只留形状和 dtype。20 维手关节会完整留下。

## 配置

见 `config/runtime/test_robot.yml`。不认识的字段会被拒绝。

顶层是 `name`、`shm_name`、`cmd_capacity`（默认 8）、`components`。

每个组件有 `name`、`class_path`、`class_name`、`args`、`setup`、`publish`、`rate_hz`、`max_bytes`（至少 64）。可选 `body`、`gate`、`motion`。

`setup` 传给 `set_up`。`args` 用来构造类。`publish` 从 `get()` 里选出要发布的字段。类路径里含 `.sensor.` 的只发布。控制器接受 `move` 和 `reset`。

`body` 是身体描述文件的路径，相对这份运行时 YAML。例如 `config/bodies/marvin.yml`。

`gate` 在下发前检查关节命令：

| 字段 | 含义 |
|------|------|
| `arm` | 身体文件里的臂名，例如 `left` |
| `lower`、`upper` | 没有身体模型时使用的限位 |
| `max_step_rad` | 没开跟踪时，一条命令允许的最大关节变化。默认 0.2 |
| `min_clearance_m` | 配置的碰撞对至少要留出的距离 |
| `max_fk_error_m` | IK 之后，或和命令里的 `eef` 相比，允许的 xyz 误差。默认 0.008 |

间隙看的是这条被命令的臂，其他臂固定在各自的 `home`。不用其他臂当前的真实关节。

`motion.enabled` 为真时，关节目标按 `max_step_rad`（默认 0.05）一步步走近，直到反馈进入 `tol_rad`（默认 0.02），或 `timeout_s` 到期。第一拍之前会把整段关节空间轨迹拿去查限位和间隙。超时只让这个组件保持当前位置，不锁存整台服务。

`{"op":"release"}` 解除保持。`{"op":"reset"}` 走到 `start_state`；没给 `start_state` 时走到身体文件里的 `home`。开了跟踪时，复位走同一条限速轨迹。

组件有身体模型、命令里只有 `eef` 没有 `joint` 时，只按位置做逆解。四元数不参与求解。`eef` 的 `--delta` 加在当前法兰位姿上再逆解。`joint` 的 `--delta` 先加成绝对关节，再做门控。

`control_mode` 接受 `impedance`（`imp`、`imp_joint`、`teleop`、`teleop_imp`）和 `position`（`pos`）。关节门控通过之后才会切换模式。

## 身体文件

`config/bodies/marvin.yml` 描述天机 M6：关节名、home、基座、法兰、掌心，以及从各基座到统一坐标系的旋转。限位来自 URDF，除非身体文件里写了 `lower` 和 `upper`。位姿和间隙需要 `pinocchio`，本包不安装它。这份文件里的 URDF 路径在仓库外面。没有 URDF 时，相关测试会跳过。

位姿是米制 xyz 加四元数 wxyz。`flange_to_palm` 是掌心原点在法兰坐标系下的位置，单位米。

## Marvin 实验台

`config/runtime/marvin_lab.yml`，共享内存名 `cyr_marvin_lab`。

| 名字 | 类型 | 说明 |
|------|------|------|
| `left_arm`、`right_arm` | 天机控制器 | 地址 `192.168.1.190`，A/B 臂，同一个进程，共用一条 UDP。启动后是关节阻抗（`K` 为 16/8，`D` 为 1.8，速度和加速度比例 50），并带上原来的工具动力学。关节单位是弧度。`eef` 是米制 xyz 加四元数 wxyz。门控：未跟踪时单步 0.2 rad，FK 误差 8 mm，间隙 40 mm。跟踪：每拍 0.05 rad，容差 0.02 rad，12 秒。`reset` 走到身体文件里的 home。停止服务时保持当前反馈，不去使能 |
| `left_hand`、`right_hand` | 舞肌 Hand 2 | UDP `192.168.1.110:7447` 和 `192.168.1.111:7447`。20 个关节，顺序和读回来的一致。进程按 `rate_hz` 刷新最新目标，每拍最多 `max_joint_step_rad`（0.08）。没有身体门控。停止服务时去使能 |
| `left_glove`、`right_glove` | 墨现传感器 | `/dev/moxian_left`、`/dev/moxian_right`，发布原始 `pressure` |
| `cam_head`、`cam_left_wrist`、`cam_right_wrist` | 奥比相机 | 只出彩色，按序列号区分 |

```bash
cyr start --config config/runtime/marvin_lab.yml
cyr call left_arm '{"control_mode":"position"}'
cyr call left_hand '{"joint":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}'
cyr call left_arm '{"op":"reset"}' --timeout 15
```

舞肌 SDK：当前环境名不是 `xone_new` 时，手的进程会去同级前缀里的 `xone_new` 加载 `wuji-sdk`。用 `CYR_WUJI_SITE` 覆盖。天机 SDK 根目录是 `MARVIN_SDK_ROOT`，否则用旁边的 `X-Robot-Pipeline_deploy`。

## 运行时行为

- 每台服务一块共享内存，只保留最新值。编码是 `msgpack_numpy`。
- 每个组件一个进程。同一 `robot_ip` 的天机左右臂例外，它们在一个进程里。
- 观测是各组件自己的最新帧，带 `timestamp_ns`。组件之间不同步，除非用 `RobotHarness.get_obs()`。
- 命令环默认 8 条。满了丢掉最旧的。
- 组件异常退出会锁存安全状态，让其他设备安全停止，拒绝后续命令，并关掉服务。
- 可以同时跑多台服务。发现记录在 `$XDG_STATE_HOME/cyr/runtime.json`（默认 `~/.local/state/cyr/runtime.json`）的 `services` 里。日志追加到 `cyr.log`。设备标准输出追加到 `driver.log`。
- 旧控制器的调试打印用 `INFO_LEVEL`：`DEBUG`、`INFO`、`ERROR`。

## 测试

在仓库根目录执行。不要用会自动加载 ROS pytest 插件的 Python。

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest src/robot/runtime/tests src/robot/harness/tests -q --tb=line
```

没有天机 URDF 或 `pinocchio` 时，对应测试会跳过。

## 目录

| 路径 | 内容 |
|------|------|
| `src/robot/runtime/` | 服务、共享内存、会话接口、`cyr`、终端界面 |
| `src/robot/harness/` | 身体模型、命令门控、关节跟踪、动作合同、对齐观测、episode 记录 |
| `src/robot/controller/` | 机械臂、底盘、灵巧手和测试控制器 |
| `src/robot/sensor/` | 相机、触觉和测试传感器 |
| `config/runtime/` | `test_robot.yml`、`marvin_lab.yml` |
| `config/bodies/` | 门控和 harness 用的身体描述 |
| `third_party/` | 厂商库 |
| `agent.md` | 给编码代理的说明 |
| `developer_README.md` | 旧的采集 / 部署说明去了哪里 |
