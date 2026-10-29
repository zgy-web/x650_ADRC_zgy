# Pixhawk 6X Pro 四旋翼电调与电机检查

本页对应真实 X650、Pixhawk 6X Pro、四路 Tekko32 F4 45A DShot 电调，以及电脑上的 FMU Debug 控制台。2026-10-29 已完成四台电机的拆桨单机点动、现场位置／方向／停止检查；M1、M2修正方向后，均通过断电再上电复测。共14次独立最低输出、1秒限时检查，其中保留了未看清、未起转及遥测故障的记录。当前电调动力已由用户关闭。这是空载台架验收，不是实际飞行通过证明，证据见[本次记录](validation/px4_rotor_commissioning_20261029/README.md)。

## 接线与当前分配

飞控由原低压供电连接供电；电脑通过 3.3 V TTL USB 转串口连接 FMU Debug，COM13、57600 baud、8N1、无流控。FMU Debug Pin 2（TX）接模块 RX，Pin 3（RX）接模块 TX，Pin 10 接 GND；不接模块 VCC。NX 与飞控原 USB 连接保留，本检查不访问或修改 NX。

用户已确认输出排针所在一端为飞控尾部，机头按机身实际前方确定。以下位置均相对机头，转向均从飞机上方向下看：

| 输出 | 飞控功能值 | 电机 | 应在的位置 | 目标转向 | 物理转动验证 |
|---|---:|---:|---|---|---|
| AUX1 | 101 | M1 | 右前 | CCW 逆时针 | 原为CW，已反向、保存并通过断电保持复测 |
| AUX2 | 102 | M2 | 左后 | CCW 逆时针 | 原为CW，已反向、保存并通过断电保持复测 |
| AUX3 | 103 | M3 | 左前 | CW 顺时针 | 用户确认位置、方向和停止；未修改方向 |
| AUX4 | 104 | M4 | 右后 | CW 顺时针 | 用户确认位置、方向和停止；未修改方向 |

2026-10-28 曾读到 AUX2=103、AUX3=102；2026-10-29 新读取变为上表，用户随后明确回复“与今天的新表一致”。检查程序未写过这些参数。保留历史记录，不把昨天的映射用于今天的点动。

DShot 电调控制信号接相应 AUX 信号针，地线与飞控信号地共地；独立 TX 遥测线使用串口 RX，不能当作 DShot 控制信号。用户报告遥测接 TELEM3，飞控参数也指向 TELEM3。四路遥测已实测，但过程中曾发生CRC连续增长、数据停止刷新的故障；断电再上电后恢复，最终M2断电复测、M3、M4各次记录窗口新增CRC错误和超时均为0。具体针脚／汇接照片、电流／电压和极数标定尚未完成。电调动力电源不接飞控信号排针的电源轨。

本次用户报告电源额定200 W、最大5 A，电调输入端已接配套电容。先前设定20 V／0.5 A，后在关闭输出后改为20 V／2 A并重新上电；用户确认静止及后续点动供电稳定。2 A是本轮单机拆桨最低输出检查的限流值，不代表带桨或飞行所需供电规格。电调遥测电流存在明显待标定偏置，不能用其读数代替电源面板的限流设定。

## 当前只读结果

固件固定核验为 PX4 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`（板型 FMUv6X，1.17.0）。本页命令语义以这份源码为准，不直接套用未来版本的参数名。

| 检查项 | 读数与含义 |
|---|---|
| 解锁状态 | `arming_state=1`，`actuator_armed.armed=False` |
| 输出协议 | AUX1–4 同属 DShot600，`PWM_AUX_TIM0=-3` |
| 输出分配 | AUX1–4=101–104；AUX5–8、MAIN1–8 均未分配 |
| 反向与可逆模式 | `PWM_AUX_REV=0`、`CA_R_REV=0`、`DSHOT_3D_ENABLE=0`；实际 `reversible_flags=0` |
| 输出上下限 | DShot mixer 最小运行值109、最大1999、停机值0；`DSHOT_MIN=0.055` |
| 当前四路输出 | 全部为0；驱动显示 Outputs on 仅表示驱动启用，不表示电机已转 |
| 推力曲线 | `THR_MDL_FAC=0` |
| 遥测 | `DSHOT_TEL_CFG=103`，驱动 `/dev/ttyS1`；通电后已验证刷新，故障及恢复记录独立保留；最终动力关闭后不要求继续收到ESC新数据 |
| 极数 | `MOT_POLE_COUNT=14` 为当前参数，尚未按实际电机确认；不得据此宣称 RPM 已标定 |
| 安全开关 | `CBRK_IO_SAFETY=22027` 为既有旁路值，本程序不改动；台架紧急停止依靠现场断开电调动力 |

原始读数与范围见 [本次记录](validation/px4_rotor_commissioning_20261029/README.md)。`SYS_USE_IO`、`PWM_AUX_TIM4` 在此固件中未读到有效参数，报告保留 `null`，不会以0代替“未知”。

## 代码入口

### 1. 只读盘点

[px4_motor_preflight_readonly.py](../scripts/px4_motor_preflight_readonly.py) 只允许固定的状态查询及参数读取，不接受任意 NSH 命令。先核验固件及未解锁状态，再读配置，最后确认飞控时间戳仍在推进。输出目录必须新建，避免覆盖前次证据。

```powershell
python -X utf8 scripts/px4_motor_preflight_readonly.py --port COM13 --output artifacts/rotor_checks/read_001
```

入口函数：`parse_parameter()` 保留未知值；`RotorReadConsole.query()` 检查固定命令集合；`collect()` 保存本次读取。日志只写执行这条命令的电脑。

### 2. 单电机有界点动

[px4_motor_single_pulse.py](../scripts/px4_motor_single_pulse.py) 默认仅输出计划，**不开串口、不发指令**：

```powershell
python -X utf8 scripts/px4_motor_single_pulse.py --motor 1
```

实际执行必须先由现场人员确认四桨全拆、机架固定、人员远离电机、可以随时切断动力，并核对接线和本次选择的电机。接通电调动力、准备观察后才可使用执行选项：

```powershell
python -X utf8 scripts/px4_motor_single_pulse.py --motor 1 --port COM13 --execute --props-removed --frame-secured --operator-ready --power-connected --wiring-confirmed --output artifacts/rotor_checks/m1_001
```

这些选项是操作者的现场确认记录，不是软件能检测到桨已拆或动力已接通。每次仅一台电机，程序不会自动升输出或轮转四台。

执行顺序与对应函数：

1. `preflight()` 重新读取固件、分配、单向模式、反向设置、输出上下限及未解锁状态，并要求两次四路遥测快照均新鲜、各路时间戳推进、停止转速为0、观测窗口内无新增CRC错误或超时。历史累计错误保留，不要求清零；250 ms新鲜度上限是本次台架诊断条件，不是PX4官方规格。若数据或配置不符，程序不给启动许可，绝不自动改参数。
2. `send_pulse()` 只发送一次 `actuator_test set -m 1 -v 0 -t 1`。此固件中，对非可逆电机，`-v 0` 表示**最小运行输出**，不是停机；当前映射到 mixer 数值109。它不是“零油门停机”，也不等于5.5%实际转速。
3. 飞控自己的 actuator-test 机制在约1秒超时后退出测试。电脑发送后回读 `dshot status`，检查只有选中通道输出109，随后检查所有通道返回0。回读是离散采样，不能据此声称已精确测出电气脉冲宽度。
4. `stop()` 额外发送该单向配置下的 `-v -1 -t 0.1`，并再次检查归零。异常路径也只尝试这条停止指令，不重试启动。若通信失效，停止确认可能失败，现场人员应直接断开动力；不得把软件超时当作硬件故障情况下的保证。
5. `pulse_report.json` 区分“看到了飞控通道输出”“归零回读成功”和“现场确认实际电机位置／转向”。物理观察不会由软件自动填成通过。`startup_telemetry`把ESC源时间戳与本次actuator_test时间窗关联；`available=true`只表示字段可解析，不能单独证明遥测新鲜或来源映射可靠。`console_check_passed`仅评价输出与停止链路，不代表遥测无错。CRC变化另见原始日志及[遥测计数审计](validation/px4_rotor_commissioning_20261029/telemetry_counter_audit.json)。

软件路径检查通过但未起转时，保留记录并检查电调供电、启动态与最低运行值，另行确定下一次试验；不自动提高输出。实际位置不符或方向错误时，先断开电调动力再排查接线／映射／电调方向，不带动力调整相线。

### 3. 单台停止状态下的电调转向设置

[px4_motor_direction.py](../scripts/px4_motor_direction.py) 复用上述实时预检，仅允许 `reverse`、`normal`、`save`，且必须显式给出1–4中的单个通道；不发电机运行指令。默认仅显示计划：

```powershell
python -X utf8 scripts/px4_motor_direction.py --motor 1 --action reverse
```

执行使用与单电机点动相同的 `--execute`、现场确认选项和新的 `--output` 目录。`dshot -m` 指的是物理输出编号，因此必须先确认 AUX1–4 与 M1–4 一一对应，不能泛化为任意输出分配下逻辑Motor编号都可直接使用。

当前固件提供 `dshot reverse -m 1` 和 `dshot save -m 1`，未提供可用的电调方向／固件设置回读命令。先临时反向、用真实点动确认方向，再保存；控制台命令成功只证明已派发，保存后的断电保持还须断电再上电复测。保存后的“normal”以新保存的方向为基准，不能把重复 `reverse` 当作已知绝对的CW或CCW。

本次M1执行一次 `reverse`、一次 `save`，随后通过断电保持复测。M2第一次临时反向后遇到遥测故障；调整电源并断电后重新执行一次 `reverse`，确认CCW且供电稳定后执行一次 `save`，随后也通过断电保持复测。共5条指定单台ESC配置命令；M3、M4未修改方向。PX4功能分配、机架几何、MPC和NX配置均未改动。

### 4. 停机状态的遥测诊断

[px4_motor_telemetry_check.py](../scripts/px4_motor_telemetry_check.py)只读采集三组飞控状态、输出与ESC数据，不点动、不修改参数：

```powershell
python -X utf8 scripts/px4_motor_telemetry_check.py --port COM13 --output artifacts/rotor_checks/telemetry_001
```

`telemetry_snapshot()`拒绝过期、缺失、重复、错误功能编号及停机时非零RPM的样本；`assess_samples()`要求各路时间戳和成功帧数推进，并检查CRC／超时计数变化。`observed_healthy=true`只覆盖本次停止状态的观察窗口，不能代表运行中永不丢帧。既有累计错误不被擦除，计数器重置也不会被当作零错误通过。

## 验证范围与下一步

当前61项无硬件软件测试通过，覆盖配置变化拒绝、缺失字段、重复启动拒绝、误通道输出、异常停止、转向命令范围，以及在线掩码正常但数据冻结／单路不推进／CRC增加时拒绝新的启动许可。四台真实点动均取得对应AUX通道109、其余通道0、超时后全部归零的记录，现场位置和方向由用户逐台确认。程序原报告的物理确认字段保持false，现场观察另记，不改写原始软件证据。

当前固件的串口遥测驱动会把 `esc_online_flags`、`esc_armed_flags` 填成配置数量对应的掩码，因此不能只看 `0b1111` 就声称四台均有新遥测或飞机已解锁。这里用每路实际时间戳、有效字段和成功帧计数判断遥测；飞控解锁状态仍用 `vehicle_status` 和 `actuator_armed`。实现见[实际固件的 DShot 驱动](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/drivers/dshot/DShot.cpp)。

后续先确认电机极数与遥测标定，并排查偶发CRC／帧错位的触发因素，再做遥控接管、失控保护和实际供电条件下的拆桨检查。真实定位、关节反馈及NX控制闭环另行验收。此前NX通信、MPC仿真与本次转子台架检查各自保留范围，不称为已具备实际飞行资格。

## 官方依据

- [PX4 1.17 执行器配置与测试](https://docs.px4.io/v1.17/en/config/actuators)：输出分配与拆桨测试流程。
- [当前固件的 actuator_test 命令](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/systemcmds/actuator_test/actuator_test.cpp)：单电机功能选择及显式超时参数。
- [当前固件的测试覆盖及超时退出](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/lib/mixer_module/actuator_test.cpp)。
- [当前固件的单向电机数值映射](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/lib/mixer_module/functions/FunctionMotors.hpp)：0映射至最小运行值，负值映射为 NaN，再由 mixer 使用停机值。
- [Holybro Tekko32 F4 45A 产品页](https://holybro.com/products/tekko32-f4-45a-esc)：确认电调系列；同名产品历史上存在不同固件，不能仅凭型号推断正在使用的电调固件或自动刷写。
