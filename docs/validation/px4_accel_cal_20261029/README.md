# Pixhawk 6X Pro 六面校准现场记录

2026-10-29，真实飞控；电调动力关闭、全部拆桨、停机标志有效，通过电脑 COM13 的 FMU Debug 操作。没有访问 NX，没有解锁或电机命令。操作指南见[实时六面校准](../../PX4_ACCEL_CALIBRATION_LIVE.md)。

## 第一次：只完成正放，未通过

只发送一次 `commander calibrate accel`。飞控完成 `down`（正放），随后五次报告 `down side already completed`，最后报告 `calibration failed: timeout: no motion` 和 `calibration failed: accel`。操作者确认看到了正放变绿，但还没有来得及翻转。

回读的全部受关注参数与启动前一致，包括三套加速度计偏置/比例、板载安装角与 `MOT_POLE_COUNT=24`。未解锁、停机标志有效、DShot 输出全零，动力观察为 0 V，COM13 已关闭。这轮不属于成功校准，不能用于评价新校准效果或宣称偏置预检问题已修复。

保存了[实际进度与参数前后值](attempt1_status.json)、[校准原始输出](attempt1_calibration_stream.txt)、[本次实际执行的工具源码](attempt1_host_tool.py)及[SHA256 清单](attempt1_manifest.json)。完整控制台留存在本地工作目录；公开记录不包含 `ver all` 的机板唯一序列信息。源码存档与执行时记录的 SHA256 一致。

下一轮将起始换面顺序调整为正放 → 左侧朝下 → 倒置 → 右侧朝下 → 机头朝下 → 机头朝上，并将换面提示固定在页面顶部。绿色后开始平稳换面，摆稳后保持静止；不要求在聊天中逐面回复。不修改固件判据，不自动重发校准命令。

后续尝试需要独立记录。网页恢复为 0/6 只表示新的本地引导会话已准备好，不代表飞控已经完成任何一面。
