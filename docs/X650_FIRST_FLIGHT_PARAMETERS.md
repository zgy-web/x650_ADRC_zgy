# X650 室内首飞参数变更表（待实读、待确认，未应用）

适用范围：无机械臂 X650，约 3.02 kg，6S，现场飞手操作 PX4 Altitude 模式。动捕不可用。这里准备的是候选参数表，不是已安装配置，也不运行原研究 MPC。

固定固件为 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。下表“旧值”来自 **2026-10-30 15:23（北京时间）** 的真实只读快照；本轮 COM13 打开后无回复，所以这些值不能称为本轮最新实读。未曾采集的项明确写“待实读”，不以固件默认值补齐。应用前必须重新读取，并保存这一次的完整回退值。

建议值是针对本轮温和起降准备的工程起点，不是 PID 优化结论或厂家指定整机参数；飞手还需根据场地确认可用操纵范围。机器可读提案见 [x650_first_flight_v1.proposal.json](../configs/hardware/x650_first_flight_v1.proposal.json)，该文件不含自动写参入口。

## 第一组：模式与手动操纵范围

| 参数 | 旧值／当前已知含义 | 建议值 | 目的及依赖 | 回退 |
|---|---|---|---|---|
| `COM_FLTMODE1` | 1，SA上档 Altitude | 保持 1 | 保留现有定高主飞行档 | 不变 |
| `COM_FLTMODE4` | 2，SA中档 Position | **1，Altitude** | 避免拨到缺少水平定位的模式；与上档相同是有意安排 | 实施前实读值（历史为2） |
| `COM_FLTMODE6` | 7，SA下档 Offboard | **8，Stabilized** | 提供飞手熟悉的自稳备用档；该档油门直接控制推力，不定高。需飞手同意并重新贴标签／逐档回读 | 实施前实读值（历史为7） |
| `MPC_MAN_TILT_MAX` | 待实读 | **20°，待飞手确认** | 对 Stabilized／Altitude 的手动倾角生效；如果现值更小，不能未经评估放宽到20° | 实施前实读值 |
| `MPC_MAN_Y_MAX` | 待实读 | **60°/s，待飞手确认** | 限制手动偏航速度；同样不自动放宽已有更小限值 | 实施前实读值 |
| `MPC_Z_VEL_MAX_UP` | 3 m/s | **1.0 m/s** | 定高模式向上满杆速度上限；不是要求飞手用满杆起飞 | 实施前实读值（历史为3） |
| `MPC_Z_VEL_MAX_DN` | 1.5 m/s | **0.6 m/s** | 定高模式下降速度上限；实际下降由飞手操作 | 实施前实读值（历史为1.5） |
| `MPC_TKO_SPEED` | 1.5 m/s | **1.0 m/s** | 当前固件定义下限为1；不提交低于元数据下限的配置 | 实施前实读值（历史为1.5） |
| `MPC_LAND_SPEED` | 0.7 m/s | **0.6 m/s** | 当前固件定义下限为0.6；本参数不是所有手动落地阶段的唯一速度来源 | 实施前实读值（历史为0.7） |

三档最终意图是 **SA上定高／SA中定高／SA下自稳**。SF、SB功能保持不变，SB紧急切断推力不能作为正常降落。当前飞控还没有改成这套映射。

数值定义依据固定版本 [模式枚举](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/module.yaml)、[手动模式参数](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mc_pos_control/multicopter_stabilized_mode_params.c)、[速度限制](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mc_pos_control/multicopter_position_control_limits_params.c)、[起降参数](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mc_pos_control/multicopter_takeoff_land_params.c)。建议值由本项目提出，不冒充官方推荐值。

## 写独立限值前，先排除整体设置覆盖

| 参数 | 旧值 | 提议 | 原因 |
|---|---|---|---|
| `MPC_Z_VEL_ALL` | 待实读 | 若已为负数，保持；若≥0，先确认改为 **-3**，再设置独立速度 | 固定源码在≥0时覆盖上下速度、起飞速度和降落速度；单写上表可能被改回 |
| `SYS_VEHICLE_RESP` | 待实读 | 若已为负数，保持；若≥0，先确认改为 **-0.4**，并保留当前各项独立值 | 固定源码在≥0时会改写手动偏航速度、加速度、jerk及空中倾角等参数 |

这里负数用于停止整体设置覆盖，不意味着“负速度”。先备份所有可能被整体设置影响的独立参数；停止覆盖不会自动恢复历史独立参数。实际逻辑见固定 [MulticopterPositionControl.cpp](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mc_pos_control/MulticopterPositionControl.cpp)。

## 第二组：失联和电池处理，满足前提后单独确认

| 参数 | 旧值 | 建议 | 前提／限制 |
|---|---|---|---|
| `NAV_RCL_ACT` | 3，Land | 保持3 | 不依赖缺少定位的 Return；无水平定位时不能保证原地降落，须核对实际降级路径 |
| `COM_RC_LOSS_T` | 0.5 s | 保持0.5 s | 已有链路超时基线 |
| `COM_FAIL_ACT_T` | 5 s | **0 s** | 室内首轮减少触发失控保护后的附加等待；不代表总响应为0秒。改后在动力断开台架复核实际行为 |
| `COM_RCL_EXCEPT` | 0 | 保持0 | 不增加遥控失联豁免 |
| `COM_LOW_BAT_ACT` | 0，仅告警 | **2，Land** | 只有电池监测有效、来源及比例核对后才应用；不绕过预检，不依赖返航 |
| `BAT_LOW_THR` | 0.15 | **0.25** | 候选剩余电量告警点；不是单体电压 |
| `BAT_CRIT_THR` | 0.07 | **0.15** | 候选临界动作点，保持低于告警、高于紧急 |
| `BAT_EMERGEN_THR` | 0.05 | **0.10** | 候选紧急动作点；真实电池电量估计尚须验证 |

电池源读数无效、剩余电量未知时，第二组中的电池变更保持待定，不能用提高／降低阈值掩盖传感器问题。电池化学体系、充满／空载电压及电流标定尚未确认，所以本轮不修改 `BAT1_V_CHARGED`、`BAT1_V_EMPTY` 或电压电流比例。6S、30000 mAh保留已知设置。失控动作枚举及等待含义依据固定 [commander_params.c](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/commander_params.c)。

## 第三组：撤除台架旁路，满足硬件条件后确认

| 参数 | 旧值 | 提议 | 前提 |
|---|---|---|---|
| `CBRK_USB_CHK` | 197848，旁路启用 | **0，恢复检查** | POWER独立供电验证通过，并明确飞行时USB接法；当前台架还在用USB，不提前写入 |
| `CBRK_IO_SAFETY` | 22027，旁路启用 | **0，恢复安全开关** | 先确认实体安全开关可操作，再在拆桨状态下复核开关与预检；不自动应用 |
| `CBRK_SUPPLY_CHK` | 0 | 保持0 | 不关闭供电检查 |
| `COM_ARM_HFLT_CHK` | 1 | 保持1 | 不关闭故障检查；USB切换故障仍隔离 |
| `COM_ARM_WO_GPS` | 1 | 保持1 | 与无GPS的人工定高计划一致；不放行需要位置的模式 |

注意固定固件本身的两个旁路默认值就是启用值，所以“恢复检查”是本项目的首飞提议，不应称为“恢复出厂默认”。依据 [circuit_breaker_params.c](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/lib/circuit_breaker/circuit_breaker_params.c)。

## 本轮保持的控制参数

姿态与角速度 PID、滤波、已恢复的 IMU 校准、电机编号／旋向／极数保持；不把仿真整定数值直接写入真机。`MPC_THR_HOVER=0.5`、`MPC_USE_HTE=1`、`MPC_THR_MIN=0.12`、`MPC_THR_MAX=1` 暂时保留，真实带桨记录之后再判断。不能用随意限制最大推力来代替温和的速度／倾角范围，也不依据空载电机转速估算悬停推力。

`MPC_TKO_RAMP_T`、`MPC_ALT_MODE`、`MPC_THR_CURVE`、`MPC_MANTHR_MIN`、`COM_POWER_COUNT`、`COM_RC_IN_MODE` 和电源传感器使能项已加入只读采集清单；尚未实读的值不会写入回退表。这里的 `MPC_` 是 PX4 多旋翼位置控制参数前缀，与研究用模型预测控制算法不同。

## 实施与回退顺序

1. 恢复真实状态回读，完成 [POWER验证](X650_POWER_VALIDATION.md)。本表中的历史值重新核对，存完整参数备份。
2. 飞手确认模式布局、操纵范围和失控动作，用户确认具体变更组后，再准备受控写入；本提案没有直接执行 `param set` 的代码。
3. 每组写入后回读，若固件标记需要重启则重启后再核对。检查整体设置未覆盖独立值，SF／SA／SB 实际状态与标签相符。
4. 若回读或预检异常，在未解锁、拆桨状态下逐项恢复该次备份并再回读；不能拿网页默认值或本表历史值直接回退。
5. 对改变的模式和失联策略做动力断开台架复核，之后才进入首飞的现场准备。本表不批准解锁、启动电机或试飞。
