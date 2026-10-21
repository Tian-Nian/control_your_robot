# 单臂 Piper

来自 RoboTwin 2.0 的 `embodiments/piper/piper.urdf`（数据集 `TianxingChen/RoboTwin2.0`，提交 `1230db540b24c0d644ce21fcb92c4bb8386bf6ee`）。

这份本体本来就是一条臂。这里去掉了腕部相机，也去掉了数据集里没有网格文件的碰撞体。夹爪 `joint7`、`joint8` 保留。`flange` 放在 `link6` 原点，`tool` 放在两侧手指的安装点，沿 `link6` 的 z 轴 0.13503 m。

画面网格是原来的 DAE，单位已经是米。仿真控制器是 `robot.controller.PiperSim_controller.PiperSimController`。
