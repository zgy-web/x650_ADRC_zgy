# X650 POWER 独立供电验证

目标是确认已接电源模块可独立维持飞控／接收机工作，且撤去 USB 后没有被软件记录到复位。全程四桨未安装，飞控未解锁，SF负100%、SA上档、SB下档、油门最低，其他轴居中。由现场人员操作供电，程序只读 COM13，不启动电机、不改变参数、不访问 NX。

当前状态：用户上一轮确认 POWER1／POWER2 已接，模块型号待补。本轮首次 COM13 采集没有收到任何字节，已保留失败记录、释放串口。**没有完成本轮 POWER 验证，也没有证据判模块故障。**

## 两个独立窗口

1. **USB＋电源模块同时供电。** 先确认电源模块型号／接法和实际电池连接状态，保持开关位置不动，读取不少于10秒。要求实际未解锁、kill=True、RC有效／油门最低／开关正确、DShot全零，以及 POWER有效位、USB有效位和5V读数正常。没有读到有效POWER时先定位原因，不让操作者直接拔USB试运气。
2. **仅POWER供电。** 上一步通过后，由现场人员只拔飞控USB，保留电源模块供电和FMU Debug的TX/RX/GND；回复完成后采集至少60秒。检查USB有效位均为0，POWER位有效，电源读数持续更新，未解锁／停机／零输出状态保持，飞控与接收机仍在线。

两个窗口之间用电脑单调时钟与飞控时间戳的对应关系检查复位，不能仅凭“后一个飞控时间戳大于前一个”声称连续运行。若先断电再启动、等待足够久，时间戳仍可能重新超过旧值；这类情况应被拒绝。两次采集必须在同一电脑、同一电脑开机周期中完成。

期间若控制台中断、供电异常、飞控复位或任何锁定条件变化，程序保留部分记录并结束，本轮不算通过；先恢复可观察状态，再决定如何排查。不要求操作解锁开关，不通过关闭预检来继续。

## 判定与证据边界

- `brick_valid` 是电源有效位图，不能解释为测量电压。USB是否拔掉同时核对 `usb_connected` 和 `usb_valid`。
- 记录 `system_power.voltage5v_v`，按当前固定固件检查所使用的4.7–5.4V范围核对，另检查过流标志。这个字段是飞控外设5V总线采样，不是对电源模块接头直接用示波器测得的电压。
- `battery_status` 独立记录。6S、connected、有效剩余电量、无电池警告只能说明该监测链可继续核对，**不表示电压／电流比例已标定**。缺失数据不能填0，也不能用POWER有效位代替电池监测。
- `window_verified=true` 只表示当前窗口通过；仅在有效combined基线之后完成power_only窗口，才置 `independent_power_verified=true`。最终仍需人为核对物理连接和使用的电源模块。`flight_ready` 始终是false。
- 此测试不测瞬态纹波、带桨负载压降或独立双路供电冗余，不能据此证明所有飞行负载下的供电质量。5V与电池电压还应与现场仪表核对。

电压范围及位图意义按固定版本 [powerCheck.cpp](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/HealthAndArmingChecks/checks/powerCheck.cpp) 和 [SystemPower.msg](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/SystemPower.msg)核对。6X常见底板使用数字电源监测，模块型号和底板接法须匹配，不能按通用模拟电压比例猜测配置。[Holybro 数字电源说明](https://docs.holybro.com/power-module-and-pdb/power-module/digital-power-module-pm-setup)

## 程序入口

`scripts/px4_power_readonly.py` 只发送固定只读命令，带未解锁／零输出守卫，记录到电脑新目录；每个窗口10–120秒，不自动操作USB或改变供电。完整参数和传感器快照仍由 `scripts/px4_first_flight_readonly.py` 采集。

下面是现场条件确认后的工具用法，不是要求现在接电或拔线：

```powershell
python -B scripts/px4_power_readonly.py --phase combined --seconds 10 --port COM13 --output artifacts/<本次新目录>/combined
```

combined窗口通过、操作者完成拔USB后：

```powershell
python -B scripts/px4_power_readonly.py --phase power_only --seconds 60 --port COM13 --baseline artifacts/<本次新目录>/combined/report.json --output artifacts/<本次新目录>/power_only
```

软件检查覆盖仍有USB、无POWER、越界电压、过流、错误开关／油门、失效或过期数据、窗口间隐藏重启、失败后记录保留及串口释放。原始记录与复现结果在 [本轮记录](validation/px4_power_firstflight_20261030/README.md) 中区分“软件通过”与“硬件尚未验证”。
