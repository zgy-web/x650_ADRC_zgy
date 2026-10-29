# Pixhawk 6X Pro：静置 IMU 偏置检查与 MN4014 极数修正

2026-10-29，真实飞控，固件 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。用户确认机架静置、电调动力关闭、COM13连接。通过电脑的FMU Debug串口采集；未使用或修改NX。本轮没有解锁、校准、重启或电机命令。

**结果：`MOT_POLE_COUNT`已由14改为24并保存、回读；加速度计偏置的预检触发条件持续存在，尚未修复。** 用户表示此前完整加速度计校准“不确定／没有做过”，下一步优先完成规范六面校准，再比较同样的静置数据。不能据本轮结果直接认定飞控硬件损坏，也不能把重新校准描述为已证明有效的修复。

## 1. 实际观察

先进行一次全量只读盘点，再进行13轮重复采样。重复采样首末轮开始时间跨度79.719秒，采样并非同时读取或高频振动测试。主估计器始终为instance 1（加速度计device ID 2818066）；每轮读取前后均核对了主实例。

| EKF/IMU实例 | X偏置范围 m/s² | Y偏置范围 m/s² | Z偏置范围 m/s² | X/Y越限次数 |
|---|---:|---:|---:|---:|
| 0 | −0.39965～−0.39963 | 0.39997～0.40000 | −0.07454～−0.07447 | 各13/13 |
| **1（主用）** | **−0.39947～−0.39942** | **0.39997～0.40000** | **−0.08312～−0.08303** | **各13/13** |
| 2 | −0.39885～−0.39880 | 0.39997～0.40000 | −0.10854～−0.10834 | 各13/13 |

三组的Z轴均未越限；陀螺仪三个轴也均未达到对应偏置告警条件。各组`vehicle_imu_status`中的加速度计/陀螺仪错误计数及饱和计数在13轮中都为0。这里仅说明观测窗口中的计数，不能覆盖带动力振动、所有温度条件或飞行性能。

当前`EKF2_ABL_LIM=0.4 m/s²`。当前[固件检查代码](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/HealthAndArmingChecks/checks/estimatorCheck.cpp)对选中实例的有效、新鲜偏置逐轴判断：

```text
|bias_axis| > 0.75 × accel_bias_limit + 3 × sqrt(max(variance_axis, 0))
```

主用IMU的X门限约0.35109，Y约0.35109～0.35196 m/s²，因此约0.40的偏置持续超限。门限根据NSH已舍入的方差计算，属于近似重算；超限余量远大于该显示精度。工具要求偏置年龄不超过2秒，比固件检查采用的30秒更严格。

本轮控制台没有重新打印`High Accelerometer Bias`文本；它可能在状态不变时不重复报告。这里依据新鲜偏置、当前主实例及相同固件判据确认触发条件仍成立。`health_report`本轮读到的是较早的状态变更快照，**未把其时间戳或日志安静当作“预检已通过”**。

`estimator_status_flags`显示静止识别成立、gravity vector融合开启、没有GNSS/外部视觉位置或速度融合；当地位置仍不可用。三组水平偏置接近同样的估计上限，同时没有传感器读取/饱和错误，提示应同时复核校准、机体水平/安装补偿以及估计器状态，不能只凭一行告警锁定某颗IMU故障。以上是排查方向，不是已识别的根因。

## 2. 校准记录与下一步

盘点已备份三组`CAL_ACC*_ID/PRIO/XOFF/YOFF/ZOFF/XSCALE/YSCALE/ZSCALE`，以及`SENS_BOARD_ROT`和板级角度补偿。当前：

- `CAL_ACC0`偏置为0、比例为1；另两组有非零偏置，比例为1。这些值本身不能证明完成过完整六面校准。
- `SENS_BOARD_ROT=0`；`SENS_BOARD_X_OFF=-3.107°`、`SENS_BOARD_Y_OFF=-2.2961°`。本轮没有修改或清零它们。
- 静置不等于严格水平，原始加速度的水平分量也可能来自重力投影。运行中的EKF偏置与`CAL_ACC`校准偏置不是同一个量，不能把约±0.4直接抄进校准参数。

建议下一阶段：

1. 确认飞控在机架内的实际朝向与当前配置相符，机架可以安全摆放六个方向，USB和Debug线有足够余量；继续保持电调动力关闭。
2. 按[PX4加速度计校准流程](https://docs.px4.io/v1.17/en/config/accelerometer)完成六面静置采集，每个方向保持不动，等待该方向完成提示后再换面。不要以单次“水平校准”代替六面加速度计校准。
3. 备份新的校准参数并与本报告比较。将机架放到经确认的水平面，核对姿态及安装补偿；需要时再做[水平校准](https://docs.px4.io/v1.17/en/config/level_horizon_calibration)。不预先把现有补偿清零。
4. 重启飞控并静置，回读24极及RC开关映射的断电保持，再执行相同三组IMU重复检查。校准完成提示只是过程完成，仍需检查有效性、偏置余量和预检状态；不调大`EKF2_ABL_LIM`来让本次数据通过。
5. 如完整校准后仍同样越限，保留前后记录，继续隔离温度、安装应力、姿态/航向与无外部定位时的估计状态；不在校准失败或告警未解决时进入带动力解锁验证。

本轮尚未启动上述校准。六面校准需要现场人员配合移动机架，静置只读确认不替代这项配合。

## 3. 极数修正与遥测边界

[MN4014 KV330厂家规格](https://www.ligpower.com/product/mn4014-kv330-motor-navigator-type.html)为18N24P：24个磁极、12对极。当前[DShot实现](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/drivers/dshot/DShot.cpp)由遥测字段换算：

```text
mechanical_rpm = integer(raw_erpm_field × 100 / (MOT_POLE_COUNT / 2))
```

实际唯一参数写入为`param set MOT_POLE_COUNT 24`，控制台返回`curr: 14 -> new: 24`；随后回读24、执行一次`param save`、保存后再读24。没有发送任何DShot转向、保存电调设置或测试输出命令。`IMU_GYRO_DNF_EN=0`沿用实读值。

对同一条有效eRPM，修正后的机械RPM约为旧值的14/24=0.58333（存在整数取整）。原始历史报告保持原样，不能改写为新实测；CRC污染或来源错位的样本也不能靠乘这个比例修好。极数修正不等于用光电转速计完成标定。

飞控本轮没有重启，因此尚未实测断电保持。电调动力关闭，也没有取得新的运行转速、电压/电流参考或CRC试验，**电流电压标定和偶发CRC根因仍是未完成项**。此前历史计数和故障记录见[旋翼验收](../px4_rotor_commissioning_20261029/README.md)；RC链路CRC与ESC遥测CRC分别处理。

最终回读：未解锁、Altitude模式、`failsafe=False`、`kill=True`、DShot全部9个通道值为0，电池观察0 V。COM13已释放。

## 4. 可复现代码与证据

- [采集及固定极数修正](../../../scripts/px4_imu_pole_check.py)：默认只读；`--apply-poles`才允许24极及参数保存两种固定写入。所有读取/写入均为白名单，写入前检查固件、未解锁、数据年龄、停机标志、零输出和动力关闭交叉读数。
- [离线汇总](../../../scripts/summarize_px4_imu_poles.py)：核对原始控制台SHA256，从原始偏置文本重算13轮结果，检查固定命令范围。它不打开串口。
- [20项软件测试](../../../tests/unit/test_px4_imu_pole_check.py)：实际偏置判据、误差方差、陈旧/无效样本、禁止校准/解锁/任意参数写入、未确认动力关闭、错误固件/极数、回读失败不重发和不保存。
- [summary.json](summary.json)：机器可读汇总；[manifest.json](manifest.json)：原始报告和源文件SHA256。
- [stationary_probe.json](stationary_probe.json)、[stationary_and_poles.json](stationary_and_poles.json)：原样复制的报告。原始控制台保留在电脑`artifacts/px4_imu_poles_20261029/<run>/console.json`，报告记录其哈希；控制台中的`ver all`设备唯一序号不随本轮公开报告上传。

```powershell
# 下一次静置复测：只读，不自动修改极数或进行校准
python -X utf8 scripts/px4_imu_pole_check.py --esc-power-off --rounds 13 --output artifacts/imu_checks/new_stationary_run

# 本轮已完成的固定极数修改命令，仅作为审计记录，毋须重复执行
# python -X utf8 scripts/px4_imu_pole_check.py --esc-power-off --apply-poles --rounds 13 --output <new-directory>

python -X utf8 -m pytest tests/unit/test_px4_imu_pole_check.py -q
```

`complete=true`仅表示采集/指定参数操作完成，**不是IMU健康或飞行验收通过**。本报告`flight_ready=false`。
