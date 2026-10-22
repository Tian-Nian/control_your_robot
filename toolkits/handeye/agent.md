# Charuco 手眼

采集用 cyr `Session` 读臂的 `eef`（xyz + wxyz）和相机的 `color`。求解优先调用 OpenCV `calibrateHandEye`。当前环境的 OpenCV 5 没有导出这个函数时，用同一组位姿解 AX = XB。`toolkits/handeye_charuco_roma` 保持不动。

先跑检查。`ok` 为假时，只把 `questions` 问给用户，不要假装仿真能看见板子。

```bash
python toolkits/handeye/check.py
```

## 条件

- 软件：服务在跑；至少一条臂的 `publish` 含 `eef`；至少一台相机的 `publish` 含 `color`；当前环境能 `import cv2`，并且有 `aruco`。有 `calibrateHandEye` 时用它，没有就用同一组位姿解 AX = XB。
- 硬件：一块和 `configs/default.yml` 一致的 Charuco。现用板是 14 列 × 10 行，格 40 mm，标记 28 mm，`DICT_5X5_100`，不是 legacy。还要这份相机的内参，以及眼在手上还是眼在手外。

PiperSim 的服务没有相机。检查会失败。问用户两件事：相机组件名，以及 `eye_in_hand` 还是 `eye_to_hand`。不要用仿真画面代替标定板。

## 用户答完以后

把答案写进 `toolkits/handeye/configs/default.yml` 的 `arm`、`camera`、`setup`、`camera_matrix`，然后：

```bash
python toolkits/handeye/check.py --confirm-board
```

通过后再采集。没有重力补偿时搬不动臂，换姿态用 `cyr call` 下发关节目标，走到停稳后再拍。每张样本是同一时刻的末端位姿和板在相机里的位姿。至少 12 张，姿态要张开，不要只平移。

```bash
python toolkits/handeye/capture.py --output samples.npz
python toolkits/handeye/solve.py samples.npz
```

`eye_in_hand` 的结果是相机相对法兰。`eye_to_hand` 的结果是相机相对基座。求解前把法兰位姿取逆再交给 OpenCV，因为板子装在手上、相机固定在基座上。
