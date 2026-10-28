# 默认 harness：RoboHarness 式工具工作区

2026-10-28 起，`bash examples/piper_mujoco.sh agent` 默认进入 `robowork`，替换之前不断叠加约束的分层 agent。参考版本为 [BinceQu/RoboHarness a0936867](https://github.com/BinceQu/RoboHarness/tree/a0936867e0db7d0ef3eabe10688468edaea190d8/harness/v2/ARX_X5)。场景、相机和可视化由独立设备配置管理；agent 不负责重建场景。

## 安装与首次使用

在代码仓库根目录安装独立环境（需要支持依赖包的 Python 3；不要使用系统 Python 2）：

```bash
python3 -m venv .venv-harness
source .venv-harness/bin/activate
python -m pip install -r requirements/harness-vision.txt
python -m pip install --no-deps -e .
```

依赖文件包含 MuJoCo、Rerun、Pinocchio 4.1.0 和局部图像跟踪所用 OpenCV。MuJoCo 仍是可选依赖，基础教学 Piper Sim 不需要它。仅测试物理场景可改装 `requirements/harness-mujoco.txt`；视觉 agent 仍需要完整 vision 环境。请在 editable checkout 中运行，场景与 URDF 从仓库读取。

模型接口需要接收图像并返回 JSON，只有文本能力的接口不能完成此实验。在仓库外的 `~/.config/control_your_robot/vision_api.json` 创建配置，例如：

```json
{
  "base_url": "http://127.0.0.1:3425/v1",
  "model": "codex/gpt-6.1-sol",
  "api_key_env": "CYR_VISION_API_KEY"
}
```

在启动终端中设置 `CYR_VISION_API_KEY` 为自己的密钥（也可在上述私有文件使用 `api_key`，不要提交密钥文件）。本地代理需事先启动。配置文件可用 `--api-config PATH` 替换；`--base-url`、`--model`、`--api-key-env` 覆盖文件设置。切换接口地址不会自动沿用另一个服务器的文件密钥；应同时显式指定密钥环境变量。没有配置时程序回退 DeepSeek 默认接口/模型，不代表任意模型均支持图像。

推荐首次顺序：

```bash
# 隔离服务、自行清理，不调用模型，也不影响已有场景
python examples/piper_mujoco_harness.py
python examples/piper_mujoco_harness.py --pick-test

# 启动可交互的常驻场景、查看可视化
bash examples/piper_mujoco.sh start
bash examples/piper_mujoco.sh viewer

# 只观察 3 轮，验证通路，不调用模型或执行自主运动
bash examples/piper_mujoco.sh agent --dry-run --max-rounds 3

# 开始真实视觉控制；前台终端按 Enter 停止
bash examples/piper_mujoco.sh agent --task '夹住并抬起橙色碗，以新图像确认碗随夹爪离桌。'
```

不要将交互 agent 的 stdin 重定向到 `/dev/null`；EOF 会被当作停止，具体来源写入日志。

## 功能入口与参数

| 入口 | 用途与影响 |
| --- | --- |
| `bash examples/piper_mujoco.sh start` | 启动或复用同一场景，并确保只读 Rerun 打开；已运行时不重置 |
| `... status` / `... stop` | 查看服务 / 停止服务；停止不等于保存物理快照 |
| `... restart` | 加载新的驱动、场景或配置，重置物体 |
| `... viewer` | 独立只读 Rerun：自由 3D 场景、头部 RGB、腕部 RGB |
| `... viewer --reset-view` | 重建只读 viewer、恢复布局，不重置物理场景 |
| `... agent` | 默认单 agent 工具工作区，自动打开 viewer |
| `... resume save/vision_ID` | 在仍运行的场景中继续原任务，重新观测和规划 |
| `... report save/vision_ID` | 离线汇总动作、失败与退出原因，不连接模型/设备 |
| `... report --recent 3 --out /tmp/report.json` | 汇总最近 3 份日志；输出文件必须不存在 |
| `... pick-test` | 在当前场景执行固定物理抓取测试，会运动；可重复基线优先使用上面的隔离脚本 |
| `... teleop` | 历史 layered 工具的人工调试终端；不等价于默认 robowork 持续夹持执行器 |
| `cyr call left_arm JSON` | 同一设备的关节/法兰/夹爪/reset；详见本文后面的完整示例 |

表内 `...` 表示 `bash examples/piper_mujoco.sh`。脚本使用 `.venv-harness`，继承与直接 `cyr` 相同的发现目录，默认 `~/.local/state/cyr`，不再偷偷覆盖 `XDG_STATE_HOME`。`stop` 会停止该目录内所有已登记服务；多服务并存时普通 CLI 不能选择组件所属服务，应使用 `Session.attach(name)`。

默认 agent 参数：`--task` 设置任务，`--max-rounds N` 限制本次循环（0 不限，默认 0），`--interval SEC` 设置轮间间隔（默认 1，范围 0.2～60），`--out PATH` 指定新日志目录，`--dry-run` 使用本地观察策略。`--resume` 保留原任务，不能换成另一个任务或重放旧动作。`--architecture layered` 才启用旧阶段/等待/恢复预算；旧参数不能混入默认模式。完整选项可运行 `python examples/piper_mujoco_agent.py --help`。

人工调试 `teleop`：输入 `x+`/`y-`/`z+` 沿工具轴移动 1 cm；`x+ 3` 指定 3 cm（1～5 cm）；`rx+`/`ry-`/`rz+` 每次旋转 0.04 rad；`j1+` 到 `j6-` 每次关节增减 0.04 rad；`forward/back/left/right/up/down` 沿基座方向移动；`open/close` 控制夹爪，`vertical` 请求朝下；空行退出。这些限制只属于历史人工终端，不是新 agent 的动作幅度限制。持续夹持测试使用 robowork 或 `cyr call`。

## 常见问题

| 现象 | 检查与处理 |
| --- | --- |
| `runtime already running` | 先用同一 wrapper `status`；需要沿用场景就 attach，确需复位才 restart。不要手工删除活跃共享内存 |
| viewer 没出现或自由视角异常 | `viewer --reset-view`；它只重启只读窗口，不修改相机安装或场景 |
| API 400/401/403 | 检查图像/JSON 支持、模型名、接口地址和密钥来源；非重试错误会停止，不循环伪装工具失败 |
| IK 不收敛 | 当前计划没有执行；用新图/深度调整目标、朝向或关节，重新规划 |
| 夹爪闭合但未到 0 | 可能接触物体；看实测开度与图像，不以 0 为抓稳标准 |
| 看似无原因退出 | 查 `run_summary.json` 的退出原因、`stop_source`、`runtime_error`、`shutdown_error`，再运行 report |
| 反馈延迟或租约丢失 | 查看 control/runtime 日志；短暂发布延迟允许等新帧，真正过期或控制权丢失仍停止 |
| 想接着尝试 | 使用 resume；它不恢复旧计划、夹爪意图或物理快照，服务被重启后不能当成原物理场景续接 |

## 先明确迁移范围

默认运行架构已换成：**一个 agent + 按需技能 + 显式工具 + 当前 RGB-D/机器人状态 + 工具历史**。工具出错交回 agent 自己重新观察和决策，没有旧版 Planner/Controller/Verifier 的阶段监督器。

这不是把整个上游 ARX X5 软件原样改名：上游双臂机器人、RGB-D 体积过滤/自碰撞检查、native Bash 任意 Python 编程接口依赖其运行环境。本地仍通过现有兼容模型 API 调用结构化工具，采用单臂 Piper/Pinocchio/MuJoCo 适配器。`plan_grasp_rgbd` 是公开深度表面→Piper 接近位姿的基础规划器，**不是**完整移植上游 `plan_grasp_point_filter_rgbd_lite`。结果明确返回 `collision_checked=false`，不能宣称已经具备上游全部能力。

## 启动与续跑

```bash
cd /Users/niantian/projects/control_your_robot
bash examples/piper_mujoco.sh agent
```

仅更新 agent 代码时，退出旧 agent 后重新运行即可；更新控制器、物理场景或 runtime YAML 时必须先 `bash examples/piper_mujoco.sh restart`，该操作会重置碗。入口仍使用 `.venv-harness`、当前私有模型配置和只读 Rerun。新默认无轮数上限，直到 Enter、agent 调用 finish，或底层运行故障；需要有限实验可加 `--max-rounds 100`。

```bash
bash examples/piper_mujoco.sh resume save/vision_你的目录
bash examples/piper_mujoco.sh report save/vision_你的目录
```

续跑只接现有服务，从日志读取任务与轮数，重新观察；不恢复旧阶段、重规划预算、待执行计划、跟踪身份或夹爪意图。接管时先确认保持，初始夹爪目标取当前开度；要持续夹持应由 agent 基于新观测显式 close。旧日志可转入新模式，不会重放历史命令。

旧实现仅留作对照：`bash examples/piper_mujoco.sh agent --architecture layered`。原有 max-waits/max-wait-replans/max-replans/stage-rounds/max-failures/verification-interval 等参数仅属旧模式，新模式传入会报清楚的参数错误，避免误以为仍生效。

## 你这次遇到的退出逻辑如何改变

| 旧默认 | 新默认 |
| --- | --- |
| 连续 wait / 主动 replan 共用三次预算 | 不存在等待恢复计数；agent 可重新观察、选点和规划 |
| 阶段到期、两次停滞规划、重复动作封锁 | 不存在阶段监督器、停滞预算或动作黑名单 |
| 强制独立 closed_upward_probe 验收 | agent 自己对比新图与工具结果；没有隐藏第二模型否决/推进阶段 |
| 1～5 cm、固定调姿幅度 | 接受连续米/度向量、绝对 EEF 位姿、绝对关节角；没有模型动作的厘米下限 |
| 默认纯 RGB，深度查询另开 | 公共 RGB-D 可用，选点深度由工具返回 |
| 动作结束 stop，下一动作保持实测夹爪开度 | 普通动作结束不发 stop；闭合目标跨动作持续保持 |
| 默认最多 500 轮 | 默认不限轮数，可显式 --max-rounds |

保留的是底层执行必需条件：物理关节范围、有限数值、有效 IK、当前计划身份、健康及时的反馈、独占控制租约和 Enter 停止。它们不推断任务是否成功，也不计恢复次数。反馈过期、租约丢失或无法确认停止属于运行故障，不能继续盲发命令。当前持续闭合执行器仅允许 Piper MuJoCo，不能直接用于实体臂。

## 工具循环

```text
最新 head/wrist RGB + 公开深度 + 关节/夹爪/FK
  → agent 选择技能和工具
  → 只读选点/测深/规划 或 显式执行/夹爪命令
  → 具体测量、错误和新图
  → agent 自主调整，直到 finish 或 Enter
```

每轮同时提供明确标注的上一帧历史 RGB 与当前 RGB，供 agent 比较碗是否随夹爪运动；历史图不能再点击。没有预先硬塞任务阶段。agent 使用 `activate_skill(name="pick-up-object")` 加载抓取说明，结束时 `deactivate_skill`。抓取流程参考上游 [pick-up-object](https://github.com/BinceQu/RoboHarness/blob/a0936867e0db7d0ef3eabe10688468edaea190d8/harness/v2/ARX_X5/skills/pick-up-object/SKILL.md)：选当前可见物体表面→规划→检查结果→执行→腕图检查→显式闭合→抬起后观察物体随动。

| 工具 | 关键参数/结果 |
| --- | --- |
| capture_camera / read_proprioception | 新双图与实测状态 |
| activate_skill / deactivate_skill | 当前单一任务技能 |
| read_depth | camera、image_id、u/v；返回该公开深度像素的基座三维点 |
| track_point / forget_track | 带当前图像身份的 RGB 局部跟踪；不提供语义识别 |
| plan_eef | frame、delta_m、rot_deg；支持 base/tool/head_camera/wrist_camera |
| plan_pose | pose_base_wxyz=[x,y,z,qw,qx,qy,qz] |
| plan_grasp_rgbd | 当前 camera/image_id/u/v，可选 yaw_deg、approach_m |
| exec_plan_pose | plan_id；单次消费并执行已预检轨迹 |
| move_joints | 六轴绝对 joint_rad |
| open_gripper / close_gripper | 持续目标开度 1/0；返回实测值，grasp_detected=unknown |
| finish | outcome=completed/abandoned、evidence；记录 agent 结论，不伪装独立验收 |

工具参数 schema 随每轮状态提供。调用示例：

```json
{"tool":"plan_eef","arguments":{"frame":"tool","delta_m":[0,0,0.002],"rot_deg":[0,3,0]},"reason":"根据当前腕图，用2毫米接近并调整姿态"}
```

所有点击采用上游的 **0～1000 归一化坐标**，绑定当前 image_id。原图不缩放，转换使用其宽高；不要沿用旧版原始像素参数。跟踪返回的像素仍是原图测量值，不能当作厘米。

基座 +X 向前、+Y 向机器人左、+Z 向上。工具方向取当前 FK；相机光学 +X 图像右、+Y 图像下、+Z 离开相机。`rot_deg` 是轴乘角的旋转向量，不是欧拉角；旋转绕工具原点。轨迹内部仍分段、限制执行速度，内部段不是模型动作幅度限制。

## 规划与 RGB-D 的真实边界

`plan_grasp_rgbd` 从所选像素及相邻公开深度中检查表面支持，构造工具朝下的接近点和表面目标，再用 Piper IK 检查。默认接近距离 6 cm，可调 yaw。所点表面是**工具原点目标**，不是已标定的指垫接触中心；碗沿的实际包夹需要 agent 继续结合腕图修正。它没有取碗中心、仿真物体位姿、分割标签或接触信息。

预览在冻结 RGB 上投影工具原点/XYZ 轴，标注为历史计划图；不是实时画面，也不是夹爪体积或碰撞证书。原始 RGB 保持不变。`plan_pose`/`plan_eef` 同样只预检 FK/IK 和关节范围。任意运动/夹爪指令会让旧计划失效；执行前检查实测关节是否改变。需要先打开时，应先 open，再创建计划。

## 持续闭合为何重要

旧 PositionExecutor 在每次成功/超时后 stop，模拟器会保持实测夹爪开度。夹爪接触碗时实测开度不为零，后续动作可能失去继续闭合的意图。

新 LatchedExecutor 保存的是命令值：close=0。即使实测开度仍是 0.145，后续机械臂运动依旧发送 gripper=0。闭合观察期结束后报告实测开度，不把“没有到零”自动当成失败，也不把它当成持物证据。执行期间与模型思考期间，健康反馈和有效租约允许持续更新同一目标；Enter、故障或退出才进入停止与释放流程。普通运动到位不再触发 stop。

这遵循上游 [close_gripper](https://github.com/BinceQu/RoboHarness/blob/a0936867e0db7d0ef3eabe10688468edaea190d8/harness/v2/ARX_X5/tools/close_gripper.py) 的“夹爪指令保持、grasp_detected unknown”语义。它仍是模拟器的位置驱动目标，不应称作经过力传感器验收的恒力夹持。

## 日志和验证

新目录包含 `run_context.json`、`turns.jsonl`、`model_calls.jsonl`、`control.jsonl`、`run_summary.json`、`perception/{head,wrist}/rgb.png` 与 `depth.npy`、当前状态 `perception/latest.json`、冻结计划预览 PNG。模型输入不包含 API 密钥或模拟器物体/接触真值。

`agent_outcome=completed` 只是 agent 的视觉判断；`independently_verified=false` 明确表示没有隐藏 evaluator 验收。脚本固定抓取测试的真值只存在测试日志，不流入 agent。

隔离测试记录：

- `save/robowork_latch_fe34a80128`：固定已知场景轨迹抓取，闭合实测约 0.145，抬升过程中命令持续为 0；碗抬起约 11.2 cm 后释放落桌。使用测试专用真值验收，不是自主模型成功。
- `save/robowork_loop_7e770d0030`：真实 MuJoCo 中加载技能、规划并执行 2 mm 位移、显式结束。无在线 API 调用，临时服务已清理。

实现：`src/robot/harness/robowork.py`、`robowork_executor.py`；提示：`prompts/robowork.txt`、`robowork_pick.txt`。旧模块仍用于显式 layered 和历史回归，不参与新默认调度。

最终回归：386 passed、2 skipped（外部 Marvin 资产未提供）；随后历史/当前双图对照补充的专项回归 9 passed。两个仓库 diff 空白检查与启动脚本语法检查通过。未调用在线模型作自主抓取验收。


## Rerun 自由视角显示（2026-10-28）

自由视角仅显示 `world/geoms/**` 的机械臂、实体桌面和碗；不将 RGB 投影平面与深度点云叠加到模型视图。默认观察点 `[1.05,-1.10,0.85]`，旋转中心 `[0.25,0,0.12]`，Z 轴朝上，可自由旋转缩放；下方分别显示原始 head/wrist RGB。相机标定、传给 agent 的图像与仿真状态不受影响，深度仍保留在记录中。

恢复初始视角或加载新 viewer 代码：

```bash
bash examples/piper_mujoco.sh viewer --reset-view
```

该命令只重启校验过身份的只读显示 bridge，不重启机器人服务，不获取控制租约。MuJoCo 场景回归 17 项通过。


### 模型接口 HTTP 400 与重试

启用 JSON 输出模式时，兼容网关可能要求输入明确包含 JSON 一词。传输层现在统一追加 JSON 输出指令，任务提示改动不会再遗漏该要求。HTTP 400/401/403 等不可重试错误会记录状态并结束，退出原因 `model_api_rejected`；408、429、5xx 等暂时错误使用退避等待，Enter 可立即中断。它们属于 API 请求处理，不是机器人任务恢复预算。相关回归 47 项通过。


### 接触闭合与 IK 不收敛（2026-10-28）

`vision_9217bbd029` 的抬升预检在第 7 个内部段不收敛，整条路径尚未执行。之前 Pinocchio 抛出的 RuntimeError 穿过工具层导致退出，现仅在本地 IK 调用边界转为 `kind=ik_not_converged`，附失败段、尝试位姿与 `motion_written=false`。模型收到新图后继续决策；传输/租约/反馈故障仍会停止，不能通过捕获所有 RuntimeError 忽略。

闭合是持续命令的定时观察，结果把 `gripper_observation_complete`、`joint_arrived`、实测开度、关节偏移分开；接触导致的非零开度和关节偏移不会笼统标为闭合到位超时，也不能据此认定抓稳。机械臂运动的到位容差仍保留。停止确认观察窗口由 1 秒延长到 3 秒，稳定性标准不变；若仍未确认，shutdown_error 记录进 summary。有原始异常时将清理错误附加到它，避免覆盖根因；无原始异常时仍明确抛出停止未确认。

验证：日志实际姿态的只读 IK 复现；34 项相关回归通过；隔离固定抓取 `save/robowork_latch_96a697e880` 抬升约 11.17 cm 后释放，临时服务已清理。未操作用户正在运行的场景，未调用在线模型验证自主成功。


### 明确区分停止来源

旧版本的 `user_stop` 可能来自 Enter、标准输入 EOF 或后台监控故障，不能据此断定用户按键。`vision_141ccca104` 第 16/17 轮测深成功，已提交第 18 轮请求但未收到可执行结果；旧日志只记录共用停止信号，无法追溯更具体来源。

新版终端与 run_summary 分别记录 `enter_pressed`、`stdin_eof`、`stdin_error`、`runtime_monitor_fault`、`external_stop`，以及 `stop_source` 和 `runtime_error`；后台故障同时写入 control.jsonl 的 background_fault 事件。EOF 仍停止，不会为了继续跑而忽略关闭的终端。模型推理期间停止仍丢弃迟到动作。


### 短暂状态发布延迟（feedback-wait-v1）

`vision_84127d399e` 的第 16 轮位姿规划成功，退出摘要包含 `shutdown_error: stale or future feedback`，并使用旧版共用 user_stop 标签。其进程在停止来源修订前启动，运行中修改 Python 文件不会自动更新已导入代码；该旧日志不足以还原停止信号最初来自哪里。

MuJoCo RGB-D 渲染与状态发布共享循环，之前一次读取超过 100 ms 就终止后台监控。模拟器专用执行器现在遇到正向过期但不超过 500 ms 的反馈时，最多等待 0.4 秒新帧；**不接受旧反馈、不在等待中写入/续发目标**。恢复后仍须满足原 100 ms 新鲜度标准；持续过期、未来时间戳、租约丢失、伺服故障仍会停止。到位连续稳定计时在发布间隔过大时重置，不能将缺帧时间算作稳定。

日志增加 feedback_gap_wait / feedback_gap_recovered；错误包含实测 age 与阈值。启动打印 feedback-wait-v1，run_context 记录 runner_revision，方便确认加载版本。此策略仅用于模拟器 LatchedExecutor，普通 PositionExecutor 的时效判定不变。


### 薄壁抓取的持续闭合预紧力

`vision_1086d8e004` 中闭合后的目标保持为 0，后续抬升没有改成张开，但仍可能因夹持几何或力不足滑落。旧驱动仅用位置弹簧：每指 K=500 N/m，静止在 1 mm 开度时闭合驱动力仅约 0.5 N。

Piper MuJoCo 现支持 `gripper_preload_n`，默认每指 2 N；close=0 时给位置执行器增加闭合方向常量偏置。总力仍受 MJCF 原有 ±10 N 上限约束，没有通过外加力绕过上限，也没有焊接或固定物体。开度指令大于 0、runtime hold 和 stop 都撤掉预紧力。设置为 0 可对照旧位置弹簧行为。算法遵循 [MuJoCo affine actuator force](https://mujoco.readthedocs.io/en/latest/computation.html) 的力模型；这是仿真驱动增强，不是实体 Piper 恒力控制实现。

新增公开状态 `gripper_command`、`gripper_actuator_force_n`、`gripper_preload_n`，新版模型输入中以 gripper_drive 标记为执行器输出，而不是接触力或持物证据。驱动仍需 agent 显式闭合；停止/退出后不承诺继续施加夹持力，悬空物体可能掉落。

**应用需要重启仿真服务，会重置当前场景。** 先结束旧 agent，并避免将正在悬空持物的场景当作可恢复状态：

```bash
bash examples/piper_mujoco.sh restart
bash examples/piper_mujoco.sh agent
```

运行中的旧驱动不会因只重启 agent 而改变。相机位置和 Rerun 布局未修改。


持续保持对照定位了主要滑落原因：原 soft-contact 摩擦模型存在缓慢蠕滑。固定同一抓取轨迹，0/2/4/6 N 预紧力都只能短暂抬升，随后掉落，不能仅靠加力修复。NoSlip=3 的实验虽通过了 15 秒保持，却破坏了三碗叠放稳定性，因此**未采用 NoSlip 作为最终设置**。

最终使用椭圆摩擦锥下 `impratio=100`，维持 `noslip_iterations=0`，提高切向摩擦约束相对法向接触的阻抗，不增加摩擦系数。配合 2 N/指预紧力，离线固定轨迹保持 15 秒只下降约 1.24 mm，三碗叠放也通过了 4 秒与 10 秒观测。机制见 [MuJoCo slow slippage](https://mujoco.readthedocs.io/en/latest/modeling.html)。它降低数值慢滑，不保证任意几何都能夹住，也不保证无限时间零滑移。

场景 XML 与生成器同步；抓取回归扩展为抬升后保持 15 秒（下滑不得超过 3 mm），再释放。先前“抬起约 11 cm”的短时记录只证明瞬时抬升，不能作为持续抓稳证据。

最终独立 runtime 记录：`save/preload_hold_a181a7c7ff`，144 次采样覆盖 15 秒保持，最低抬升 12.503 cm，较保持起点下滑约 0.90 mm，末次执行器闭合力约 3.92 N/指；随后张开释放，临时服务清理完毕。场景 22 项、视觉/工具循环 56 项回归通过。未重启用户现有场景，未调用在线模型作自主抓取验收。

## 使用 cyr call 手动控制同一 MuJoCo 场景

先在 agent 终端按 Enter，等它释放控制租约。不要让 agent 和手动命令同时控制机械臂。
第一次更新到支持排队轨迹的版本时，执行 `bash examples/piper_mujoco.sh restart` 加载新代码与配置；这会重建场景、重置碗。之后手动控制不需要重启。

在新的终端执行以下环境准备。正常使用无需设置 `XDG_STATE_HOME`；曾经设置旧路径的终端应先按下面迁移说明关闭旧服务，再 unset。自定义该变量时，所有启动和控制终端必须保持一致。

```bash
cd /Users/niantian/projects/control_your_robot
source .venv-harness/bin/activate
unset XDG_STATE_HOME  # 使用默认服务目录；仅旧服务迁移期间才使用旧路径
cyr start --config config/runtime/piper_mujoco.yml
cyr call left_arm '{"op":"release"}'
cyr echo left_arm
```

`release` 清除普通运动保持状态，允许接收下一条指令；它不张开夹爪，也不能绕过安全停机或其他进程的控制租约。agent 退出、指令被拒绝或跟踪超时后，可先检查原因，再调用 `release`。安全保护触发时应先处理故障。

### 关节位置与复位

单位为 rad，六个数按第 1～6 轴排列。下面的演示应在夹爪空载、周围无障碍时运行。

```bash
# 回到 READY 姿态 [0, 1.3, -1, 0, 0.5, 0]，不是全零姿态
cyr call left_arm '{"op":"reset"}'

# 绝对关节目标：第一轴转到 0.12 rad
cyr call left_arm '{"joint":[0.12,1.3,-1,0,0.5,0]}'

# 相对当前实测关节位置：第一轴减小 0.06 rad
cyr call left_arm '{"joint":[-0.06,0,0,0,0,0]}' --delta

# reset 到指定关节位置
cyr call left_arm '{"op":"reset","start_state":[0,1.3,-1,0,0.5,0]}'
```

排队的关节/末端目标由运行时按每步最多 0.02 rad 插值，最大关节误差不超过 0.005 rad 时完成，跟踪超时为 15 秒。它允许单次发送大于 0.02 rad 的完整目标；harness 的流式目标仍沿用原来的单步限制。该流程不提供完整的避障规划，关节空间插值也不保证末端走直线。

`reset` 只复位机械臂，保留夹爪目标和碗的当前物理状态，仿真时间继续前进。恢复整个初始场景使用 `bash examples/piper_mujoco.sh restart`。

### 末端位置与姿态

`eef` 的绝对值是**基座坐标系中的法兰位姿**，格式为 `[x,y,z,qw,qx,qy,qz]`，位置单位 m，姿态是单位四元数。这里的控制点是法兰，不是 harness 的指尖工具点。基座 +Z 向上；+X/+Y 沿基座坐标轴，不等同于图像的左右上下。

下面绝对位姿接近 READY 法兰位置，可在上述 reset 后演示：

```bash
cyr call left_arm '{"eef":[0.2704102193,0.0014152113,0.3129706029,0.4159810225,0.0000018367,0.9093732946,-0.0000015168]}'

# 沿当前法兰局部 +X 移动 1 mm，保持姿态
cyr call left_arm '{"eef":[0.001,0,0]}' --delta

# 在当前位置绕当前法兰局部 Z 旋转 1 度
cyr call left_arm '{"eef":[0,0,0,0.9999619231,0,0,0.0087265355]}' --delta
```

`--delta` 下平移按 `p_new = p + R × delta_xyz` 计算，姿态增量四元数右乘；不是基座 XYZ 增量，也不是欧拉角。需要基座方向平移时，从 `cyr echo left_arm` 的当前 `eef` 修改对应 XYZ，再发送完整绝对位姿。局部 IK 先求解、检查关节限制，再执行；不可达时拒绝，不会先执行一部分目标。它只验证目标 IK 和关节路径限制，不保证没有碰撞。

### 夹爪与持续夹持

```bash
cyr call left_arm '{"gripper":1}'  # 完全张开
cyr call left_arm '{"gripper":0.5}'  # 半开目标
cyr call left_arm '{"gripper":0}'  # 持续闭合并施加预紧
```

夹爪 0 为闭合、1 为张开；它不是角度或牛顿。闭合目标持续生效，不需要反复发送 `close`，之后关节、末端运动和 arm-only reset 都不会覆盖它。默认每指预紧 2 N，总执行器输出受每指 10 N 上限约束；位置/阻尼项也计入总输出。夹住物体时实际开度可大于 0，不能要求它一定归零。

夹爪命令的 `accepted:true` 表示目标已下发，不等于已到位或已抓稳；检查 `gripper_command`、实测 `gripper` 和双视角图像。`gripper_actuator_force_n` 是仿真执行器输出，不是接触力或抓取成功证据。停止/安全保持可能撤销预紧，不能承诺停止后悬空物体仍被夹住。

关节目标也可以与夹爪目标放在同一条 JSON 中，例如 `{"joint":[0,1.3,-1,0,0.5,0],"gripper":0}`。夹爪从运动开始就接收该目标，不是在机械臂到位后才闭合。抓取时应按实际需要分开发送命令。

## 服务目录迁移、可视化与 reset

新启动脚本不再覆盖 `XDG_STATE_HOME`，默认与原生 `cyr` 一样使用 `~/.local/state/cyr`。自动化隔离测试仍使用临时目录。若旧版本服务仍在 `save/mujoco-runtime` 中运行，脚本会提示 PID 和迁移步骤，不会偷偷停止它。

要迁移，先按 Enter 退出 agent，然后在代码仓库根目录执行（会重建场景、重置碗）：

```bash
XDG_STATE_HOME="$PWD/save/mujoco-runtime" .venv-harness/bin/python -m robot.runtime stop
unset XDG_STATE_HOME
bash examples/piper_mujoco.sh start
source .venv-harness/bin/activate
cyr status
```

若当前场景必须保留，可以暂时继续在各终端设置旧路径；等实验结束再迁移。不要只 unset 后启动第二份同名服务。Shell 脚本内部的环境变量不会传回父终端，因此旧脚本和手输命令才会产生目录不一致。

`bash examples/piper_mujoco.sh start` 现在同时确保只读 Rerun 打开。原生 `cyr start` 负责设备生命周期：已有服务时直接复用，不重跑设备初始化，也不会重新弹窗。窗口被关闭时运行 `bash examples/piper_mujoco.sh viewer`；布局异常用 `viewer --reset-view`，不会重置物理场景。

| 想做的事 | 命令 | 结果 |
| --- | --- | --- |
| 回准备姿态 | `cyr call left_arm '{"op":"reset"}'` | 回 `[0,1.3,-1,0,0.5,0]`；若已在此姿态不会明显运动 |
| 六轴回零 | `cyr call left_arm '{"op":"reset","start_state":[0,0,0,0,0,0]}'` | 六轴目标全零，经过插值和到位检查 |
| 直接指定零位 | `cyr call left_arm '{"joint":[0,0,0,0,0,0]}'` | 同样是六轴绝对目标 |
| 重置碗和机械臂 | `bash examples/piper_mujoco.sh restart` | 重建整个场景 |

两种关节复位都保留夹爪目标和物体状态。命令没有运动时先看返回 JSON 的 accepted/error；不要把默认 reset 理解成全零或物理世界重置。当前目录没有服务时，`cyr stop` 会明确显示目录以及“未停止其他目录的服务”，不再误报已经停止全部进程。
