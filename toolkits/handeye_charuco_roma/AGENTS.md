# handeye_charuco_roma 代理手册

Pipeline 版仍在 `../handeye_charuco`。**本目录是 roma 工位的采集前端**，不覆盖那份代码。

求解 / 验证 / 换 K / `session.json` 合同仍只在 `handeye_charuco`（`calibrate` `verify` `ingest` `geometry`）。本包只多两件事：

1. `bus_py.Observe` 读 `cam_*` RGB 和 `*_arm` 的 `q`（**不 claim**）
2. 用天机 `Marvin_Kine` **离线** FK（`libKine.so`，不开 UDP / 不 `OnLinkTo`）得到与 Pipeline `get_state()["eef"]` 同一套法兰 TCP

同一 Marvin IP 只能一个客户端。roma 管家占着 `192.168.1.190` 时不要再开 `MarvinArmController`。

## 不要做的事

- 不要改 `../handeye_charuco` 的 hardware / Pipeline compose
- 不要为了 FK 再连一遍 Marvin SDK
- 不要在本包里再抄一份 Tsai / `adapt_intrinsics`
- 不要 `ether` claim（采集是 teach）
- 核仍不 `import bus_py`；只有本包 `observe.py` 可以
