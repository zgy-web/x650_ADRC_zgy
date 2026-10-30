# 拆桨 SB 实际停转验收：通过

本记录验证四个真实旋翼电机已旋转时，遥控 SB 停机是否令输出及实际转速归零。不是仅检查锁定标志，也不是 `actuator_test` 单电机测试。操作者确认四桨拆除、机架固定，使用 6S / 30000 mAh 动力电池；飞控版本为 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。

## 原始记录与判定

- [sb_stop_report.json](sb_stop_report.json)：117 组观测，包括最后一次状态回读，保留 NSH 原文。
- [sb_stop_audit.json](sb_stop_audit.json)：由 `scripts/px4_rc_powered_stop_summary.py` 重解析；`physical_sb_stop_verified=true`。
- 操作者现场原话：“电机都启动、SB下档后全部停稳”。遥测和现场确认共同构成通过依据。

| 顺序 | 记录索引 | 实际证据 |
|---|---:|---|
| 已启动 | 79 | 已解锁、kill=False、AUX1–4 非零、四路新鲜 RPM 全部大于零 |
| SB 停机 | 81 | 仍处于解锁状态，kill=True、四路输出为零、四路新鲜 RPM 为零 |
| 自动锁定 | 84 | 已锁定、kill=True、输出和转速保持零 |

被记录到的旋转段转速约 491–525 RPM。最终锁定原因编号 8，按该固件的事件枚举解释为 `kill_switch`。本窗口成功遥测计数增加 73374，CRC/超时计数均未增加；这只说明该窗口未复现 CRC，不表示历史偶发 CRC 已定位或消除。

NSH 各主题是依次读取，无法据此精确给出 SB 到实际停转的毫秒延迟。没有验证带桨、空中停机或 NX Offboard 人工接管。

## 电池信息修正

此前台式电源下预检阻止解锁；换用实际电池后仍发现电池串数为 0、剩余电量未知。固定范围工具 `scripts/px4_rc_battery_metadata.py` 将 `BAT1_N_CELLS` 从 0 设为 6、`BAT1_CAPACITY` 从 -1 设为 30000，并执行参数保存。具体写入、读回及其他电池参数未改的检查见 [battery_metadata_report.json](battery_metadata_report.json)。

当时读到约 24.52 V，修正后电池 warning 从 3 变为 0。没有修改电压阈值或绕过电池预检；电压与电流仍未用独立仪表标定。参数保存命令及即时读回成功，不以此记录替代断电持久性复测。

## 当前收尾与后续边界

操作者已断开动力电池。后续 Offboard 通信准备遇到飞控 Hard Fault，故未继续执行两项 Offboard 实验；该故障和当前安全状态单独保存在 [NX Offboard 故障档案](../nx_offboard_bench_20261030/README.md)。故障后的最终回读确认未解锁、实际 kill=True、输出全零、油门最低。

只读观察器：`scripts/px4_rc_powered_stop_observer.py`。离线重算（不访问硬件）：

```bash
python scripts/px4_rc_powered_stop_summary.py --report docs/validation/px4_powered_stop_20261030/sb_stop_report.json --output /tmp/sb_stop_audit_new.json --operator-all-started-and-stopped
```

此处操作者标志对应已记录的本次现场确认，不可用于未经现场确认的新实验。文件哈希及关联源码见 `manifest.json`。
