# Pixhawk 6X Pro 四旋翼电调与电机检查

本页对应真实 X650、Pixhawk 6X Pro、四路 Tekko32 F4 45A DShot 电调，以及电脑上的 FMU Debug 控制台。记录阶段为 **2026-10-29 通电前检查完成，实际旋转与转向尚待点动确认**。不把仿真编号图或软件输出回读当作物理接线已验证。

## 接线与当前分配

飞控由原低压供电连接供电；电脑通过 3.3 V TTL USB 转串口连接 FMU Debug，COM13、57600 baud、8N1、无流控。FMU Debug Pin 2（TX）接模块 RX，Pin 3（RX）接模块 TX，Pin 10 接 GND；不接模块 VCC。NX 与飞控原 USB 连接保留，本检查不访问或修改 NX。

用户已确认输出排针所在一端为飞控尾部，机头按机身实际前方确定。以下位置均相对机头，转向均从飞机上方向下看：

| 输出 | 飞控功能值 | 电机 | 应在的位置 | 目标转向 | 物理转动验证 |
|---|---:|---:|---|---|---|
| AUX1 | 101 | M1 | 右前 | CCW 逆时针 | 待点动 |
| AUX2 | 102 | M2 | 左后 | CCW 逆时针 | 待点动 |
| AUX3 | 103 | M3 | 左前 | CW 顺时针 | 待点动 |
| AUX4 | 104 | M4 | 右后 | CW 顺时针 | 待点动 |

2026-10-28 曾读到 AUX2=103、AUX3=102；2026-10-29 新读取变为上表，用户随后明确回复“与今天的新表一致”。检查程序未写过这些参数。保留历史记录，不把昨天的映射用于今天的点动。

DShot 电调控制信号接相应 AUX 信号针，地线与飞控信号地共地；独立 TX 遥测线使用串口 RX，不能当作 DShot 控制信号。用户报告遥测接 TELEM3，飞控参数也指向 TELEM3。**尚未以有效遥测帧验证具体针脚及四路汇接方式**。本次动力断开时成功帧为零，不能据此判定遥测硬件故障。电调动力电源不接飞控信号排针的电源轨。

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
| 遥测 | `DSHOT_TEL_CFG=103`，驱动 `/dev/ttyS1`，当前无有效帧 |
| 极数 | `MOT_POLE_COUNT=14` 为当前参数，尚未按实际电机确认；不得据此宣称 RPM 已标定 |
| 安全开关 | `CBRK_IO_SAFETY=22027` 为既有旁路值，本程序不改动；台架紧急停止依靠现场断开电调动力 |

原始读数与范围见 [本次记录](validation/px4_rotor_commissioning_20261029/README.md)。`SYS_USE_IO`、`PWM_AUX_TIM4` 在此固件中未读到有效参数，报告保留 `null`，不会以0代替“未知”。

## 两个代码入口

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

1. `preflight()` 重新读取固件、分配、单向模式、反向设置、输出上下限及未解锁状态。文件中保存的旧盘点不能替代这一步。若当前配置与本次台架计划不同，程序退出，交由操作者重新核对，绝不自动改参数。
2. `send_pulse()` 只发送一次 `actuator_test set -m 1 -v 0 -t 1`。此固件中，对非可逆电机，`-v 0` 表示**最小运行输出**，不是停机；当前映射到 mixer 数值109。它不是“零油门停机”，也不等于5.5%实际转速。
3. 飞控自己的 actuator-test 机制在约1秒超时后退出测试。电脑发送后回读 `dshot status`，检查只有选中通道输出109，随后检查所有通道返回0。回读是离散采样，不能据此声称已精确测出电气脉冲宽度。
4. `stop()` 额外发送该单向配置下的 `-v -1 -t 0.1`，并再次检查归零。异常路径也只尝试这条停止指令，不重试启动。若通信失效，停止确认可能失败，现场人员应直接断开动力；不得把软件超时当作硬件故障情况下的保证。
5. `pulse_report.json` 区分“看到了飞控通道输出”“归零回读成功”和“现场确认实际电机位置／转向”。后两项物理观察不会由软件自动填成通过。

软件路径检查通过但未起转时，保留记录并检查电调供电、启动态与最低运行值，另行确定下一次试验；不自动提高输出。实际位置不符或方向错误时，先断开电调动力再排查接线／映射／电调方向，不带动力调整相线。

## 验证范围与下一步

当前已验证只读连接、DShot 配置以及33项无硬件软件测试。测试覆盖配置变化拒绝、反向设置、缺失字段、重复启动拒绝、误通道输出、未观察到输出、写入结果不确定及异常停止等情况；它们不替代真实电机验收。

每台点动后记录：实际位置、从上方观察的方向、是否按期停止、回读输出、遥测是否收到。四台逐一通过后，再整理遥控接管、失控保护、真实定位、关节反馈及 NX 控制接入条件。此前 NX 通信、MPC 仿真与本次转子台架检查各自保留范围，不称为已具备实际飞行资格。

## 官方依据

- [PX4 1.17 执行器配置与测试](https://docs.px4.io/v1.17/en/config/actuators)：输出分配与拆桨测试流程。
- [当前固件的 actuator_test 命令](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/systemcmds/actuator_test/actuator_test.cpp)：单电机功能选择及显式超时参数。
- [当前固件的测试覆盖及超时退出](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/lib/mixer_module/actuator_test.cpp)。
- [当前固件的单向电机数值映射](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/lib/mixer_module/functions/FunctionMotors.hpp)：0映射至最小运行值，负值映射为 NaN，再由 mixer 使用停机值。
- [Holybro Tekko32 F4 45A 产品页](https://holybro.com/products/tekko32-f4-45a-esc)：确认电调系列；同名产品历史上存在不同固件，不能仅凭型号推断正在使用的电调固件或自动刷写。
