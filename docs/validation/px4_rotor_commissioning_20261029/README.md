# 四旋翼真实台架：编号、转向与有界点动

## 结论和边界

2026-10-29，经电脑COM13、57600 baud连接真实Pixhawk 6X Pro FMU Debug，完成四台电机的拆桨、单台最低输出、1秒限时检查。用户逐台确认实际位置、从机顶观察的方向和停止；M1、M2原方向均不符，修改并保存后通过了断电保持复测。最后用户确认电调动力已关闭。**本记录证明本轮单机空载通路与方向，不证明带桨推力、实际飞行或全功率供电能力。**

| 输出 | 功能 | 实际电机位置 | 最终方向（从机顶看） | 保存／上电状态 | 最终物理证据 |
|---|---:|---|---|---|---|
| AUX1 | 101 | M1 右前 | CCW 逆时针 | 原CW，已修改、保存并断电复测 | [m1_powercycle_v1](m1_powercycle_v1.json) |
| AUX2 | 102 | M2 左后 | CCW 逆时针 | 原CW，已修改、保存并断电复测 | [m2_powercycle_2a_v1](m2_powercycle_2a_v1.json) |
| AUX3 | 103 | M3 左前 | CW 顺时针 | 未修改，使用电调上电设置 | [m3_pulse_2a_v1](m3_pulse_2a_v1.json) |
| AUX4 | 104 | M4 右后 | CW 顺时针 | 未修改，使用电调上电设置 | [m4_pulse_2a_v1](m4_pulse_2a_v1.json) |

[operator_observations.json](operator_observations.json)保存用户原话、供电条件和方向判断。原程序报告中的 `physical_motor_confirmed=false`、`rotation_confirmed=false` 等字段保留；程序本身不能看到电机，不用人工结果覆盖原始软件证据。

14次脉冲均观察到选中AUX输出109、其余输出0，飞控超时后及额外停止指令后全部为0；各次均未解锁、非Offboard。**`console_check_passed=true`只描述输出和停止链路，不能代表遥测、实际起转或方向都通过。** 离散回读不等于电气脉宽的精确测量。共14条限时启动、14条附加停止、5条单台ESC配置命令，PX4参数写入0，NX访问／配置修改0。

## 供电与遥测故障

用户确认电源额定200 W、最大5 A，配套电容已接在电调电源输入端；型号未提供。最初设定20 V／0.5 A，其中0.5 A明确为限流上限，意味着在20 V下最多约10 W。M2初次有响声但未转；断电检查时用户确认手动转动平顺、三相线牢固。之后仍在0.5 A设置下曾成功起转，因此不能把第一次未起转的原因直接判定为唯一的供电问题。

`m2_after_reverse_v1`中，用户确认M2已逆时针，控制通道正常归零；但遥测CRC快速增加、有效帧停止在583,592，随后[只读记录](telemetry_fault_read_v1.json)显示数据已约251.5秒未更新，CRC累计100,612。故障时RPM还被报告到未被命令的槽位；这不足以证明其他电机实际转动。该组18.3–18.7 V、RPM和电流均为冻结旧数据，不能作为当前电压或转速。更早的 `m1_after_reverse_v1` 也出现127个新增CRC错误，但随后恢复，所有异常均保留。

在停止点动后，用户关闭电源输出，将20 V／0.5 A改为20 V／2 A，重新上电并确认CV（恒压）。[恢复后的三组读取](telemetry_after_2a_v1.json)中，有效帧598,648→600,269，新增CRC和超时均为0，各路时间戳推进、RPM为0。重新给M2临时反向后，[独立只读窗口](telemetry_after_reverse_2a_v1.json)又新增2,442个有效帧，无新增CRC或超时。

随后M2在20 V／2 A下点动时，用户确认逆时针、停止、全程CV及电压稳定；这次记录窗口新增1个CRC，但数据持续刷新，没有再次冻结。保存并断电重启后，M2复测、M3、M4各次记录窗口新增CRC／超时均为0。

**此次同时改变限流并进行了电调断电重启，不能据此分离两项措施的作用，故障根因尚未完全隔离。** 当前证据支持先保留稳定供电条件并监测遥测；不能宣称已证明0.5 A是全部故障的唯一原因。当前固件的[遥测解析实现](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/drivers/dshot/DShotTelemetry.cpp)按固定10字节帧校验，坏帧后继续请求下一路；源码显示没有逐字节滑窗重新对齐，因而帧对齐失步是待验证的可能因素。没有为此修改或刷写PX4固件，也没有重启NX。

电压／电流尚未标定，M1／M2静止时也会报告约1 A偏置，而M3／M4低速时电流仍报0；这些读数不能用作供电功率定量验收。`MOT_POLE_COUNT=14`尚未按MN4014实际极数确认，因此以下原始RPM仅表明非零响应，不作为已标定的机械转速。

## 全部点动记录

以下时间均为北京时间；每行是独立的一次1秒限时点动，没有自动重试或提高输出。CRC增量按该次日志最早／最晚 `dshot status` 计数计算，包含预检和停止后读取，不仅是1秒脉冲。详见[telemetry_counter_audit.json](telemetry_counter_audit.json)。

| 记录 | 开始时间 | 现场结果 | 新增CRC |
|---|---|---|---:|
| [m1_pulse_v1](m1_pulse_v1.json) | 13:16:42.703 | 未看清，用户要求复测 | 0 |
| [m1_pulse_v2](m1_pulse_v2.json) | 13:18:20.157 | 未看清方向 | 0 |
| [m1_pulse_v3](m1_pulse_v3.json) | 13:21:54.790 | 右前顺时针，与目标不符 | 0 |
| [m1_after_reverse_v1](m1_after_reverse_v1.json) | 13:26:29.175 | 临时反向后未看清 | 127 |
| [m1_after_reverse_v2](m1_after_reverse_v2.json) | 13:27:59.142 | 逆时针，已停止 | 0 |
| [m1_powercycle_v1](m1_powercycle_v1.json) | 13:31:44.044 | 保存并断电后仍逆时针，已停 | 0 |
| [m2_pulse_v1](m2_pulse_v1.json) | 13:32:49.344 | 有响声但没转；保留失败观察 | 0 |
| [m2_startup_diagnostic_v1](m2_startup_diagnostic_v1.json) | 13:40:39.720 | 用户确认起转，未确认方向 | 0 |
| [m2_direction_observe_v1](m2_direction_observe_v1.json) | 13:42:04.553 | 左后顺时针，已停 | 0 |
| [m2_after_reverse_v1](m2_after_reverse_v1.json) | 13:43:11.029 | 逆时针；遥测冻结，不能记为全部通过 | 1,312 |
| [m2_pulse_2a_v1](m2_pulse_2a_v1.json) | 14:03:49.766 | 左后逆时针且停；全程CV、供电稳定 | 1 |
| [m2_powercycle_2a_v1](m2_powercycle_2a_v1.json) | 14:06:02.769 | 保存并断电后仍逆时针，已停 | 0 |
| [m3_pulse_2a_v1](m3_pulse_2a_v1.json) | 14:06:43.362 | 左前顺时针且停；供电稳定 | 0 |
| [m4_pulse_2a_v1](m4_pulse_2a_v1.json) | 14:07:28.465 | 右后顺时针且停；供电稳定，随后关闭动力 | 0 |

最近四次脉冲约在源时间窗内0.39秒读取到被选中电机的非零RPM：M2为642、断电复测M2为671、M3为642、M4为642，其余三路均为0；停止后四路均为0。`startup_telemetry.available=true`仅表示字段可解析，必须同时查看源时间窗、数据年龄及错误计数，不能用于替代现场方向确认。

## 电调配置与断电保持

| 记录 | 时间 | 命令 | 范围与结果 |
|---|---|---|---|
| [m1_reverse_v1](m1_reverse_v1.json) | 13:26:00.981 | `dshot reverse -m 1` | 仅AUX1临时反向 |
| [m1_save_v1](m1_save_v1.json) | 13:29:04.163 | `dshot save -m 1` | 保存后已由现场断电复测验证 |
| [m2_reverse_v1](m2_reverse_v1.json) | 13:42:49.736 | `dshot reverse -m 2` | 仅AUX2临时反向，之后因排查供电而断电 |
| [m2_reverse_2a_v1](m2_reverse_2a_v1.json) | 14:00:07.574 | `dshot reverse -m 2` | 重新上电后恢复临时反向 |
| [m2_save_2a_v1](m2_save_2a_v1.json) | 14:05:06.361 | `dshot save -m 2` | 保存后已由现场断电复测验证 |

配置命令执行前后均确认未解锁、所有输出0，没有附带电机脉冲。控制台成功只表示命令派发；最终依靠用户在断电复测中观察方向确认持久化。M1、M2保存后的复测均未再发reverse。M3、M4未改配置。

## 初始读取与历史差异

固件为PX4 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`、1.17.0、FMUv6X。最新完整初始只读结果见[inventory_readonly.json](inventory_readonly.json)：13:08:43.778开始，飞控时间戳293284437→295317437微秒，未解锁、非Offboard，输出均0。用户当时确认动力断开，故遥测帧为0未记作接线失败。输出反向、可逆标志、推力曲线参数均0，DShot600启用；`SYS_USE_IO`和`PWM_AUX_TIM4`未知保留null。

| 本地原始目录（仓库artifacts/下） | 结果 |
|---|---|
| `px4_rotor_commissioning_20261028/baseline_v1` | 无NSH提示符，无配置写入或电机指令 |
| `px4_rotor_commissioning_20261028/baseline_v2` | 读取成功，当时AUX2=103、AUX3=102 |
| `px4_rotor_commissioning_20261029/baseline_v1` | COM13不存在，无电机指令 |
| `px4_rotor_commissioning_20261029/baseline_v2` | 恢复FMU Debug后读取成功，AUX2=102、AUX3=103 |
| `px4_rotor_commissioning_20261029/baseline_v3` | 补查反向、可逆、推力曲线设置 |

检查程序未改变映射，不推断是谁或哪个工具改变AUX2／3；按当天读取及用户接线确认执行测试。原始串口流含设备唯一标识，只留在本地对应artifacts目录；公开的报告保存console文件SHA-256供关联，不意味着仓库包含原始串口流。

## 代码与验证

代码入口和逐步命令见[检查指南](../../PX4_ROTOR_COMMISSIONING.md)。在已有只读／限时点动工具之上，新增单台ESC转向工具、脉冲源时间窗遥测和停止状态的独立遥测诊断。识别到冻结数据后，增加点动前的两次遥测检查：四路停止、数据年龄不超过250 ms、源时间戳和成功帧推进、观测窗口无新增CRC／超时。250 ms是本次台架诊断条件，不是官方频率或时延上限。保留累计历史错误，不因online/armed掩码为15而接受旧数据。

```powershell
python -X utf8 -m pytest tests/unit/test_px4_motor_preflight_readonly.py tests/unit/test_px4_motor_single_pulse.py tests/unit/test_px4_motor_direction.py tests/unit/test_px4_motor_telemetry_check.py -q
```

**61项测试通过**，包括未知配置拒绝、重复／过期许可、误通道输出、异常停止，以及冻结单路、伪在线、CRC变化或计数器重置时不给启动许可。另用本轮真实旧日志做[离线回放](telemetry_regression_replay.json)：已冻结251秒的记录被拒绝，恢复后的三组读取通过，未访问硬件。随后当前版本通过真实M2复测、M3和M4点动。每次报告绑定当时脚本及本地日志哈希；旧报告原样保存。

下一步先确认电机极数、遥测标定和偶发CRC来源，然后在拆桨状态下检查实际供电、遥控接管和失控停机。动捕、机械臂和NX控制闭环另行验收；当前不进入带桨飞行或MPC实飞。
