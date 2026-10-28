# RoboHarness 方法在 Piper harness 中的适配

> 历史实现说明：2026-10-28 默认入口已改为 [单 agent 工具工作区](harness_robowork.md)。本文中的阶段预算、纯 RGB 默认和 Planner/Verifier 描述仅对应旧 layered 模式。

本轮参考 [BinceQu/RoboHarness](https://github.com/BinceQu/RoboHarness)，核查版本为 `a0936867e0db7d0ef3eabe10688468edaea190d8`（2026-10-28）。该版本公开说明的环境是 BEHAVIOR 与 RoboDojo；未在代码中找到 RoboTwin 适配。本项目按其方法独立实现下述子集，没有复制其完整环境或抓取策略。

## 为什么增加这一层

原先 Controller 每轮看图后直接给动作：它可能反复换碗沿、误解相机方向，或把动作成功当成抓取成功。仅继续叠加提示词无法解决这些问题。现在把局部视觉证据、运动预览、执行验收分开，供 agent 检查自己的假设。

| 上游方法与源码 | 本地适配 | 明确边界 |
| --- | --- | --- |
| [动态点状态](https://github.com/BinceQu/RoboHarness/blob/a0936867e0db7d0ef3eabe10688468edaea190d8/harness/v2/ARX_X5/tools/util/dynamic_point_tracker.py)：跟踪与丢失显式表示 | RGB 局部特征、双向光流和图块一致性检查；丢失不自动重新绑定 | 不是碗检测器；不读取深度、物体位姿或接触真值 |
| [运动计划执行](https://github.com/BinceQu/RoboHarness/blob/a0936867e0db7d0ef3eabe10688468edaea190d8/harness/v2/ARX_X5/tools/exec_plan_pose.py)：预览与执行分离，检查计划时效 | Pinocchio IK 分段预检、短期计划缓存、实测漂移检查和单次执行 | IK 可解不代表路径无碰撞 |
| [腕部坐标控制](https://github.com/BinceQu/RoboHarness/blob/a0936867e0db7d0ef3eabe10688468edaea190d8/harness/v2/ARX_X5/tools/adjust_left_eef_pose_in_wrist_frame.py) | base/tool/head_camera/wrist_camera 下平移和旋转 | 坐标变换来自机器人/相机标定，不是图像推断的物体深度 |
| [闭合工具](https://github.com/BinceQu/RoboHarness/blob/a0936867e0db7d0ef3eabe10688468edaea190d8/harness/v2/ARX_X5/tools/close_gripper.py) 不把闭合报告成接触成功 | 保留独立 RGB Verifier；运动成功与图像判断分别记录 | 尚未移植上游持续闭合指令机制或 RGB-D 表面抓取求解器 |

## 现在的执行架构

```text
双视角 RGB + 机器人关节/夹爪/FK + 静态桌面参考
             ↓
Planner：确定阶段和完成条件
             ↓
Controller：腕图局部对齐、头图整体检查
  ├─ track_point / forget_track：维护当前局部视觉证据
  ├─ plan_motion：只读 IK 预览 → 下一轮查看结果
  └─ execute_plan 或原有直接动作
             ↓
检查新图像、状态漂移、单次计划 → PositionExecutor 分段执行
             ↓
实测运动结果 + 新 RGB → 继续调整 / 监督器恢复 / 独立图像验收
```

Controller 看得到跟踪像素和运动预览；Verifier 仍只收到阶段完成条件与双图，不读取这些工具结果、深度或仿真物体真值。像素距离只表示二维关系，不能换算为厘米或当作抓稳证据。

## 工具与单位

| 工具 | 输入 | 返回与用途 |
| --- | --- | --- |
| `track_point` | camera、当前 image_id、原图像素 `[u,v]`、label | track_id、observed/lost、像素与质量；最多 6 个点 |
| `forget_track` | track_id | 删除过时点，腾出容量 |
| `plan_motion` | 当前 observation_id、frame、delta_cm、可选 rot_deg/anchor_track_id | plan_id、基座增量、工具目标、内部段数；不写运动命令 |
| `execute_plan` | **当前** observation_id、已有 plan_id | 消费一次计划，沿预检路径执行；返回实测验收结果 |

例如先请求工具系下降/接近：

```json
{"action":"plan_motion","observation_id":"从当前反馈复制","frame":"tool","delta_cm":[0,0,2],"rot_deg":[0,0,0],"reason":"腕图确认沿当前工具方向接近"}
```

下一轮检查预览和新图，再把返回的 plan_id 与**新一轮** observation_id 用于 execute_plan。不要复制文档占位 ID。工具 +Z 是否向下取决于当前姿态；读取反馈的轴方向后再选择。

- 平移向量的长度为 1～5 cm，允许斜向；纯旋转时平移可为零。rot_deg 是旋转向量，不是欧拉角，长度不超过约 8.59°（0.15 rad）。
- 相机光学坐标 +X 向图像右、+Y 向下、+Z 离开相机。它描述**机器人运动方向**；静止物体在腕图中的运动可能相反。旋转绕工具原点，轴按所选坐标系表达。
- 计划缓存最多 4 条，120 秒到期；任何运动/夹爪命令使其余计划失效。计划被消费后即使执行中断也不能重放。
- 执行前重新取图和实测状态。关节漂移 >0.005 rad、夹爪变化 >0.02、工具漂移 >1 mm 或 >0.01 rad 时拒绝。可选 anchor 必须仍在当前图中被跟踪，较计划时移动超过 3 像素也拒绝。
- 弱纹理、遮挡、图像边缘可能导致登记拒绝或跟踪丢失。重新看图选点，也可以使用原有直接动作；不能把历史像素冒充当前位置。
- 只读工具也消耗 Controller 轮数，默认仍为 500 轮。不要每轮无意义重登记、重规划。续跑保留历史记录，但不会恢复任何可执行计划或有效跟踪 ID。

## 如何运行与观察

```bash
cd /Users/niantian/projects/control_your_robot
bash examples/piper_mujoco.sh agent
```

CLI 默认启用这组工具，原有基座/工具移动、关节调姿和夹爪动作仍可用。仅代码和提示词更新时，退出旧 agent 后重新启动即可；本轮没有改变相机位置、场景或 Rerun 布局。要做对照试验，可添加 `--no-grounded-tools`。底层 Python `run_loop` 为兼容已有调用，须显式传 `grounded_tools=True` 且 `legacy_actions=False`。

可选视觉环境依赖增加 `opencv-python-headless>=4.10,<6`（本机 harness 环境已有 OpenCV）；跟踪不下载模型权重。普通 Piper Sim 环境与 MuJoCo 可选安装方式不变。

日志中 `queried` 表示选点/忘点/预览，没有执行运动；`tool_result` 保存结果。`execute_plan` 才可能产生运动，`succeeded` 只说明机械臂到位。结合 `visual_panel`、独立图像验收和实际腕图判断是否抓住，不能只看模型文字。

## 尚未解决的问题

该改造改善的是证据连续性、坐标表达和执行协议，不能保证抓碗成功。纯 RGB 跟踪不提供目标三维面、力或接触。现有位置执行器运动时保持实测夹爪开度，尚不是上游持续闭合力控制；被碗阻挡的闭合可能超时，抬起也可能滑脱。下一步应独立验证夹爪持续闭合语义和接触后的控制，再评估自主抓取成功率，不能靠放宽停止条件掩盖这一限制。

回归覆盖跟踪平移/遮挡、过期图像、坐标变换、计划过期/单次使用、状态漂移、执行前新图检查、载荷边界及无运动预览。隔离 MuJoCo 使用脚本决策验证调用链；不等于在线模型自主抓取验收。

验证记录（2026-10-28）：完整回归第一轮 376 passed、2 skipped，跳过项需要外部 Marvin 资产。隔离 MuJoCo `save/grounded_smoke_32b4130ae9` 完成只读预览→执行 +Z 1 cm，最终 FK 误差约 0.025 mm / 0.000093 rad，临时服务已清理；没有调用在线模型。后续增加只读预览恰逢阶段复查时不消耗停滞预算的回归，以免计划/执行两轮制引入新的过早暂停。

调度修正后的相关回归：173 passed（新增工具、监督器、协议与提示资源）；两个仓库的 diff 空白检查通过。
