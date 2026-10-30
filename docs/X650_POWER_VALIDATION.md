# X650 POWER 独立供电验证

目标是确认已接电源模块可独立维持飞控／接收机工作，且撤去 USB 后没有被软件记录到复位。全程四桨未安装，飞控未解锁，SF负100%、SA上档、SB下档、油门最低，其他轴居中。由现场人员操作供电，程序只读 COM13，不启动电机、不改变参数、不访问 NX。

当前状态：**2026-10-30 已完成静止、拆桨、未解锁条件下的 POWER1 独立供电验证。** 用户确认电池接 POWER1，并在组合供电窗口通过后只拔掉飞控USB。组合窗口10.781秒，独立供电窗口60.078秒；独立窗口40组采样，`brick_valid=1`、USB两个有效位均为0，5V总线5.09411–5.10791V，无报告过流，飞控时间关系连续，RC有效、实际kill=True、DShot输出全零。

本轮最早的COM13无回复记录、随后实际kill未置位而停止的记录均保留；操作者SB中→下后才重建有效基线。模块准确型号仍待确认，不能由驱动型号反推整个电源模块的品牌／额定功率。具体证据见 [验收记录](validation/px4_power_firstflight_20261030/README.md)。

## 两个独立窗口

1. **USB＋电源模块同时供电。** 先确认电源模块型号／接法和实际电池连接状态，保持开关位置不动，读取不少于10秒。要求实际未解锁、kill=True、RC有效／油门最低／开关正确、DShot全零，以及 POWER有效位、USB有效位和5V读数正常。没有读到有效POWER时先定位原因，不让操作者直接拔USB试运气。
2. **仅POWER供电。** 上一步通过后，由现场人员只拔飞控USB，保留电源模块供电和FMU Debug的TX/RX/GND；回复完成后采集至少60秒。检查USB有效位均为0，POWER位有效，电源读数持续更新，未解锁／停机／零输出状态保持，飞控与接收机仍在线。

两个窗口之间用电脑单调时钟与飞控时间戳的对应关系检查复位，不能仅凭“后一个飞控时间戳大于前一个”声称连续运行。若先断电再启动、等待足够久，时间戳仍可能重新超过旧值；这类情况应被拒绝。两次采集必须在同一电脑、同一电脑开机周期中完成。

期间若控制台中断、供电异常、飞控复位或任何锁定条件变化，程序保留部分记录并结束，本轮不算通过；先恢复可观察状态，再决定如何排查。不要求操作解锁开关，不通过关闭预检来继续。

## 判定与证据边界

- `brick_valid` 是电源有效位图，不能解释为测量电压。USB是否拔掉同时核对 `usb_connected` 和 `usb_valid`。
- 本机固定版本的 `BOARD_ADC_SERVO_VALID` 宏恒为1，因此不能把读到 `servo_valid=1` 当作额外电源确实存在、正在供电或已验证冗余的证据。[固定板级定义](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/boards/px4/fmu-v6x/src/board_config.h)
- 记录 `system_power.voltage5v_v`，按当前固定固件检查所使用的4.7–5.4V范围核对，另检查过流标志。这个字段是飞控外设5V总线采样，不是对电源模块接头直接用示波器测得的电压。
- `battery_status` 独立记录。6S、connected、有效剩余电量、无电池警告只能说明该监测链可继续核对，**不表示电压／电流比例已标定**。缺失数据不能填0，也不能用POWER有效位代替电池监测。
- `window_verified=true` 只表示当前窗口通过；仅在有效combined基线之后完成power_only窗口，才置 `independent_power_verified=true`。最终仍需人为核对物理连接和使用的电源模块。`flight_ready` 始终是false。
- 此测试不测瞬态纹波、带桨负载压降或独立双路供电冗余，不能据此证明所有飞行负载下的供电质量。5V与电池电压还应与现场仪表核对。

电压范围及位图意义按固定版本 [powerCheck.cpp](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/HealthAndArmingChecks/checks/powerCheck.cpp) 和 [SystemPower.msg](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/SystemPower.msg)核对。6X常见底板使用数字电源监测，模块型号和底板接法须匹配，不能按通用模拟电压比例猜测配置。[Holybro 数字电源说明](https://docs.holybro.com/power-module-and-pdb/power-module/digital-power-module-pm-setup)

## 本次电池监测核对

后续固定只读检查发现 `ina228 status` 为运行状态：I2C Bus 1、地址0x45、通信错误计数0。`battery_status.source=0` 对应电源模块类来源，区别于消息枚举中的ESC类来源2。两个显式使能参数为0不代表驱动没有启动；固定板级启动文件包含INA自动探测路径。本轮没有启动、停止或配置这些驱动。[消息定义](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/versioned/BatteryStatus.msg)、[板级启动逻辑](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/boards/px4/fmu-v6x/init/rc.board_sensors)

独立供电窗口电池电压为24.42492–24.43057V，后续只读快照约24.409V，识别6S、warning=0。用户随后确认“飞控显示的约24.4 V与电池实际电压一致”，记录为现场近似一致性核对通过；没有提供仪表精度、精确参考值或同步采样，不能宣称完成百分比精度标定。电流比例、逐节电芯电压和剩余电量仍未标定；低电策略的条件提案继续保留这些前提。

用户明确要求保持动力供电继续检查，因此收尾未执行拔电池或接回USB。最后只读快照仍为POWER1有效、USB无效、未解锁、kill=True、输出全零，串口已关闭。这是最后采样状态，不是无人值守的持续监控。

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

软件检查覆盖仍有USB、无POWER、越界电压、过流、错误开关／油门、失效或过期数据、窗口间隐藏重启、失败后记录保留及串口释放。新增电源驱动固定status查询后，两套工具合计36项离线测试通过；实际POWER结论来自上述实机窗口。原始记录、摘要与哈希在 [本轮记录](validation/px4_power_firstflight_20261030/README.md) 中关联。
