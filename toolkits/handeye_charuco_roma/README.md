# MARVIN + 奥比 Charuco 手眼（roma 版）

Pipeline 版在 [`../handeye_charuco`](../handeye_charuco)。**本目录是另一套采集前端**，不替换、不覆盖那份代码。

核（`calibrate` / `verify` / `map-pixel`、`session.json`）仍是那边的包。这里只在 roma 管家已起时：

- `Observe` 读头相机 RGB + 臂关节（不 claim、不抢 UDP 4730）
- 离线 `Marvin_Kine` 把 `q` 变成法兰 TCP（与 Pipeline `eef` 同一套：X 前 / Y 左 / Z 上）

当前工位（`jinli-v1`）花名册：`left_arm` + GMSL `cam_head` / `cam_left` / `cam_right`。默认 **眼在手外**（头相机）。**眼在手上** 走左腕 `cam_left`，板固定在桌/地面，不要装在法兰上。

## 0. 环境

```bash
cd /path/to/niantian/handeye_charuco_roma

# 无真机测试（Robot / 带 opencv 的解释器）
PYTHONPATH="$PWD:$PWD/../handeye_charuco" python -m pytest tests -q

# 真机采集：conda roma（有 bus_py）+ opencv-python
# conda run -n roma python -c "import bus_py,cv2"
```

`scripts/run.sh` 会把 `handeye_charuco` 和天机 `SDK_PYTHON` 加进 `PYTHONPATH`。

## 1. 管家已起、只读

```bash
# 确认花名册与 FK（需要 roma 解释器）
./scripts/run.sh check-config

# 只打印绑定，不连总线
./scripts/run.sh check-config --offline
./scripts/run.sh check-board
```

## 2. teach 采集

示教器或已有遥操挪臂（本工具不 claim）。

眼在手外：板固连法兰，头相机看板。

```bash
./scripts/run.sh --setup eye_to_hand --side left capture --out runs/eth_roma
```

眼在手上：板固定在世界系，左腕 `cam_left` 看板。只挪臂，不要动板。

```bash
./scripts/run.sh --setup eye_in_hand --side left capture --out runs/eih_left
# 或：./scripts/run.sh --config configs/eye_in_hand.yml capture --out runs/eih_left
```

无 GUI：拖拽时回车**只记关节**，输入 `q` 后回放；每到一点先停 1s 再采图和 eef，再等 1s 去下一点。已有 `joints.json` 可 `--replay-only`。至少 12 帧有效。然后：

```bash
./scripts/run.sh calibrate runs/eih_left
./scripts/run.sh verify runs/eih_left
./scripts/run.sh verify-live runs/eih_left
```

## 3. 直播验证

```bash
./scripts/run.sh verify-live runs/eth_roma          # 无 GUI 抓一帧
./scripts/run.sh verify-live runs/eth_roma --display # 空格存帧
```

## 和 Pipeline 版的差别

| | `handeye_charuco` | 本包 |
|---|---|---|
| 臂 | `MarvinArmController` 占 UDP | 只用管家已发布的 `joint.q` |
| 相机 | `OrbbecSensor` 直开 | `Observe.get('cam_head')` |
| FK | SDK 读状态时算 | 同一份 `libKine.so`，不开链 |
| 自动走位 | `--mode auto` 会动臂 | **没有**（不 claim） |
| 分辨率 | 可开 1920×1080 | 跟 roma 槽走（当前目标 **1280×800**） |

`K` 按实际画面标定；外参米制刚体与分辨率无关。换分辨率后旧 session 的内参不能直接用，要重采或走 `adapt_intrinsics`。
