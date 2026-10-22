# 工具包

每个目录一份 `agent.md`。先跑该目录的 `check.py`。不通过时，按它打印的问题问用户，再继续。

- `gravity_compensation/`：辨识法兰质量和质心，写回 MIT 前馈。PiperSim 用 `config/runtime/piper_sim_gravity.yml`。点位用 `cyr call` 下发，不要拖臂。
- `impedance/`：检查配置里的 `joint_K` 和 `joint_D`。真机不套用仿真默认增益。PiperSim 用 `config/runtime/piper_sim_impedance.yml`。
- `handeye/`：Charuco 手眼。采集走 cyr `Session`，求解用 OpenCV。
- `handeye_charuco_roma/`：原来的 roma 采集前端，保持不动。
