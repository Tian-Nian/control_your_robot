[![中文](https://img.shields.io/badge/中文-简体-blue)](./README_CN.md)  
[![English](https://img.shields.io/badge/English-English-green)](./README.md)

[Chinese WIKI](https://tian-nian.github.io/control_your_robot-doc/)

# 控制你的机器人!

该项目旨在帮助各位进入具身智能领域后，能从控制机械臂开始，走到数据采集，再到 VLA 模型的训练与部署。

# 近期更新 (2026-10)

共享内存运行时在 [`feat/shm-runtime`](https://github.com/Tian-Nian/control_your_robot/tree/feat/shm-runtime)。控制器和传感器跑成一个服务，客户端用 `cyr` 或 `robot.runtime.session.Session` 读最新观测、下发指令。该分支还会在下发前检查关节命令，按步长跟踪到位，并提供给策略用的 `robot.harness`。

`main` 仍然是下面这份采集、xspark 和 XPolicyLab 部署说明。这个分支上没有 `cyr`，也没有 `config/runtime/`。

# 近期更新 (2026-07)

本分支新增 [XPolicyLab](https://github.com/XPolicyLab/XPolicyLab) 部署支持、Xspark 数据转换、遥操作 IPC 优化，以及若干协议与命名修复。

## XPolicyLab 接入

上游项目：[XPolicyLab](https://github.com/XPolicyLab/XPolicyLab)

### 端到端流程

```
control_your_robot 采集
        ↓
tools/transform2xspark.py          # 原始 HDF5 → xspark v1.0
        ↓
tools/write_xpolicy_instruction.py # 补写 instruction（转换时未写入则在此补充）
        ↓
复制/软链到 XPolicyLab 目录树      # data/{bench}/{task}/{env_cfg}/data/episode_*.hdf5
        ↓
XPolicyLab policy/<POLICY>/process_data.sh + train.sh
        ↓
离线 replay 评测 (ReplayRobot) 或 真机部署 (real_env_client)
```

**步骤 1 — 采集原始数据**

```bash
python example/collect/collect.py
# 或遥操作：
python example/teleop/master_slave_arm_teleop_fs.py
```

原始布局：`save/{task_name}/*.hdf5`，扁平 group（`left_arm/`、`cam_head/` 等）。

**步骤 2 — 转换为 xspark v1.0**

```bash
python tools/transform2xspark.py save/test_robot \
  --save-dir ./data \
  --task-name stack_bowls \
  --skip-video
```

输出布局：

```
data/stack_bowls/
├── data/episode_0000000.hdf5
├── preview_video/                 # 可选
└── config.json
```

**步骤 3 — 语言指令（转换阶段可选）**

`transform2xspark.py` 仅在找到以下指令来源时写入 `instruction` / `instructions`：

- `--instruction` 显式指定
- sidecar JSON（`0.json`、`info.json`）
- `task_info/{task_name}.json` 或 `task_instructions/{task_name}.json`

若均未找到，episode 仍会完成转换，但 **不写入 instruction 字段**，可后期补充：

```bash
python tools/write_xpolicy_instruction.py data/stack_bowls/data \
  --task-name stack_bowls \
  --verify
```

**步骤 4 — 放入 XPolicyLab 训练目录**

```bash
mkdir -p ../XPolicyLab/data/RoboDojo/stack_bowls/piper_x/data
cp data/stack_bowls/data/episode_*.hdf5 ../XPolicyLab/data/RoboDojo/stack_bowls/piper_x/data/
```

随后按目标 policy 的 README 执行，例如：

```bash
cd ../XPolicyLab/policy/Pi_05
bash install.sh
bash process_data.sh RoboDojo stack_bowls piper_x joint
bash train.sh RoboDojo stack_bowls piper_x joint 0 0
```

**步骤 5 — 离线评测（HDF5 replay）**

使用 `my_robot/replay_robot.py` 与 `example/deploy/replay_robot.yml`（复制后按任务修改）。`hdf5_paths` 指向 **`data/`** 子目录，并设置 `hdf5_format: xspark`。Replay 从 HDF5 顶层 `instruction` 读取 prompt，以 `obs["instruction"]` 发给 policy server。

**步骤 6 — 真机部署**

分机部署流程见 XPolicyLab policy README：

```bash
# 策略机
bash setup_eval_policy_server.sh ...

# 机器人/采集端 — 通过 task_env shim 加载 example/deploy/real_env_client.py
bash setup_eval_env_client.sh ...
```

协议：WebSocket，`client_server.ws.WsModelClient`（非旧版 `robodojo_ws`）。

### xspark v1.0 episode 结构（XPolicyLab 兼容）

| 位置 | 类型 | 说明 |
|------|------|------|
| `instruction` | UTF-8 字符串 | 训练/离线评测前必需；可后期补写 |
| `instructions` | UTF-8 字符串数组 | 与 instruction 相同文本的单元素列表 |
| `data_format_version` | `"v1.0"` | 轨迹协议版本 |
| `additional_info/frequency` | int | 控制/采集频率（通常 25 或 30） |
| `vision/{cam}/colors` | `(T,)` JPEG 字节 | 三路相机 |
| `state/*_joint_states` | `(T, DOF)` | 关节 + 夹爪 |
| `state/*_ee_poses` | `(T, 7)` | xyz + 四元数 **wxyz** |
| `action/*` | 与 state 同名 | `action[t] = state[t+1]` |

相关代码：

- `example/deploy/real_env_client.py` — 真机环境客户端
- `my_robot/replay_robot.py` + `example/deploy/replay_robot.yml` — 离线 HDF5 replay
- `task_env/real_env_client.py` — XPolicyLab robodojo 导入 shim
- `src/robot/utils/base/xpolicy_hdf5.py` — 元数据读写/校验

## Xspark 数据管线

- `X_spark_format_pipeline` — `src/robot/utils/base/data_transform_pipeline.py`
- `tools/transform2xspark.py` — 批量 raw → xspark
- `tools/write_xpolicy_instruction.py` — 补写/更新 instruction 与元数据
- 兼容 legacy：`qpos` → `eef`，`slave_*_arm` → `left_arm` / `right_arm`
- 加速：`--skip-video` 或 `XSPARK_SKIP_PREVIEW=1`

```bash
python tools/transform2xspark.py save/test_robot --save-dir ./data --skip-video
python tools/write_xpolicy_instruction.py data/xspark/data --task-name xspark --verify
```

## 遥操作性能优化

- `example/teleop/master_slave_arm_teleop_fs.py` 使用 **MsgpackSharedDict** 替代 `Manager().dict()`，避免高频 IPC 序列化开销
- 进程优雅退出，避免 HDF5 写入中断
- 依赖：`msgpack>=1.0.5`（见 `pyproject.toml` / `requirements.txt`）

## 修复与重构

- 末端位姿字段统一：**`qpos` → `eef`**
- XPolicyLab 协议更新：**`robodojo_ws` → `ws`**
- Agent 开发指南：**`agent.md`**

## 快速上手!
由于本项目实现了部分测试样例, 如机械臂测试样例, 视觉模拟样例, 完整机器人模拟样例, 因此可以在没有任何实体的情况下快速了解本项目的整体框架.
不接真机时，安装可以这样：
```
pip install -r requirements.txt
# 或
pip install -e .
```  
本项目有特殊的调试参数, 分为:"DEBUG", "INFO", "ERROR", 如果想要完整看到数据的流程, 可以设置为"DEBUG".
```bash
export INFO_LEVEL="DEBUG"
```
或者可以在对应main函数中引入:
```python
import os
os.environ["INFO_LEVEL"] = "DEBUG" # DEBUG , INFO, ERROR
```
1. 数据采集测试
```bash
# 单线程。函数内部的耗时会堆积到下一次采集
python example/collect/collect.py
# 用节点并行采集
python example/collect/collect_node.py
```

`example/collect/collect_mp_robot.py` 和 `example/collect/collect_mp_component.py` 已经不在这个分支上。

2. 模型部署测试
```bash
# 通用部署脚本，在仓库根目录
bash deploy.sh
# 离线回灌。先用 collect.py 至少采三条轨迹
bash eval_offline.sh
```

`example/deploy/robot_on_test.py` 已经不在这个分支上。真机客户端是 `example/deploy/real_env_client.py`。

3. 远程部署数据传输
```bash
# 先启动服务器, 模仿推理端(允许多次连接, 监听端口)
python scripts/server.py
# 本地, 获取数据并执行指令(示例只执行了10次)
python scripts/client.py
```

4. 一些有意思的代码
```python
# 采集对应的关键点, 并且进行轨迹重演
python scripts/collect_moving_ckpt.py 
# sapien仿真, 请参考 src/robot/planner/README.md
```

5. 调试对应的一些代码
```bash
# 包在 src/ 下面。先 pip install -e .，或带上 PYTHONPATH
PYTHONPATH=src python -m robot.controller.TestArm_controller
PYTHONPATH=src python -m robot.sensor.TestVision_sensor
PYTHONPATH=src python -m my_robot.test_robot
```

6. 一些数据转化脚本
```bash
# 在执行完 python example/collect/collect.py, 路径下有轨迹后
python scripts/convert2rdt_hdf5.py save/test_robot/ save/rdt/

# Xspark + XPolicyLab 数据准备
python tools/transform2xspark.py save/test_robot --save-dir ./data --skip-video
python tools/write_xpolicy_instruction.py data/xspark/data --task-name xspark --verify
```

7. 遥操作采集（同频 / 异频）
```bash
# 同频
python example/teleop/master_slave_arm_teleop.py
# 异频（300Hz 控制 / 60Hz 采集，msgpack IPC）
python example/teleop/master_slave_arm_teleop_fs.py
```

### 🤖 设备支持情况

#### 🎛️ 控制器
`src/robot/controller/` 里已经有这些类：

| 机械臂 | 底盘 | 手 |
|--------|------|----|
| Agilex Piper、Piper Agx | Agilex Tracer 2.0 | `dexhand_controller.py` 只是基类，不是成品手 |
| RealMan 65B，含 ROS 版本 | Agilex Bunker | |
| Daran ALOHA、Y1 | | |
| Franka（`FrankaFranky_controller.py`）、UR（`URUrx_controller.py`） | | |

这个分支没有 JAKA 控制器。舞肌 Hand 2 在 [`feat/shm-runtime`](https://github.com/Tian-Nian/control_your_robot/tree/feat/shm-runtime)，不在 `main`。

#### 📡 传感器
`src/robot/sensor/` 里已经有 RealSense、OpenCV、ROS 图像、Vitac3D、ROS / ROS 2 触觉手套、Pika 和 Quest。奥比和墨现在 `feat/shm-runtime`。需要新的传感器可以提 issue，也欢迎提交配置。

## 表格形式
| 目录 | 说明 | 主要内容 |
|------|------|----------|
| **📂 controller** | 机器人控制器封装 | 机械臂、底盘等设备的控制 `class` |
| **📂 sensor** | 传感器封装 | RealSense、OpenCV、ROS 图像、Vitac3D、触觉手套、Pika、Quest |
| **📂 utils** | 工具函数库 | 辅助功能封装（如数学计算、日志等） |
| **📂 data** | 数据采集模块 | 数据记录、处理的 `class` |
| **📂 my_robot** | 机器人集成封装 | 完整机器人系统的组合 `class` |
| **📂 policy** | VLA 模型策略 | Vision-Language-Action 模型相关代码 |
| **📂 scripts** | 实例化脚本 | 主要运行入口、测试代码 |
| **📂 third_party** | 第三方依赖 | 需要编译的外部库 |
| **📂 planner** | 路径规划模块 | `curobo` 规划器封装 + 仿真机械臂代码 |
| **📂 example** | 示例代码 | 数据采集、模型部署等示例 |