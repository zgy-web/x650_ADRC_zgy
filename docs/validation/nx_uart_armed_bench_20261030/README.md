# NX–PX4 已解锁、动力断开的 UART 台架记录

日期：2026-10-30。飞控 Pixhawk 6X Pro / PX4 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`，NX 使用此前已修复的 UART1，TELEM1 为 921600 bit/s。全部记录来自四桨拆下、电调动力物理断开的台架；没有电机动力、自动解锁或自动模式命令。本轮不运行 MPC。

## 初始预检与旧故障转储

开始时 SB 已解除停机，但解锁预检仍失败。`commander check` 明确报告 `Crash dumps present on SD`，根目录只有此前 USB MAVLink/DDS 切换事故的转储。两次读取完全一致，16923字节，SHA256 `d97375d97750fd7a8f93ec4979f5789ffe16a0aa9750f77ca5056b6ce154b9bb`。任务名、PC/LR、CFSR/HFSR与已记录事故吻合。

用户批准后，仅把 `/fs/microsd/fault_2026_10_29_09_01_06.log` 移至 `/fs/microsd/flyarm_fault_archive_20261030/`，电脑保留完整备份，`COM_ARM_HFLT_CHK=1` 未变。用户重启飞控、SB中档后，`Preflight check: OK`，当前Altitude模式可解锁。归档凭据和重启后预检见 [fault_archive_receipt.json](fault_archive_receipt.json)。固定固件的 [sdcardCheck.cpp](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/HealthAndArmingChecks/checks/sdcardCheck.cpp) 在每次启动扫描SD卡根目录。

这没有修复USB切换故障；该入口继续停用。归档仅释放已经识别并隔离的旧事故对本次无动力UART台架的预检阻塞，不授予带动力或飞行验收。

## 实机尝试

`manual_attempt_v1.json`保留首轮失败：实际解锁，但SA始终在上档，没有进入Offboard。操作者确认未及时看到／执行SA提示，随后出现地面自动锁定。它不是一次成功的接管实验。该轮串口同步后CRC为0，1357条零推力目标，最大发送间隔22.0119 ms，最后独立恢复回读通过。

复测页面提前展示SF→SA→接管顺序，用大字和可关闭的提示音标明当前动作；移除页面另设的4秒反应期限。飞控10秒地面自动锁定、2秒落地锁定、5秒kill锁定参数保持原值。只有独立记录中的真实已解锁Offboard→仍已解锁退出区间才可通过。

实际验收结论由 `scripts/check_nx_uart_armed.py` 生成的报告给出。页面的“已观察到退出”属于现场引导，不能替代原始记录复核。源代码和完整操作流程见 [NX_ARMED_UART_BENCH.md](../../NX_ARMED_UART_BENCH.md)。

| 已解锁台架工况 | 人工接管 `manual_v2` | 断流自动退出 `stream_stop_v1` |
|---|---:|---:|
| 本阶段验收 | 通过 | 通过 |
| 实际退出时仍已解锁 | 是 | 是 |
| 退出时实际模式 | Altitude | Altitude；failsafe=True，RC有效 |
| Offboard区间（PX4模式时间戳） | 3.768283 s | 4.065290 s |
| 零推力目标条数 | 583 | 438 |
| 最大发送间隔 | 21.9915 ms | 21.9047 ms |
| 独立内部目标一致样本 | 3 | 3 |
| 完整float32回传一致样本 | 53 | 53 |
| 同步后CRC错误 | 0 | 0 |
| 最后一条目标后持续观察 | 62.65 s | 66.16 s |
| NX目标流重新启动 | 否 | 否 |
| 最终未解锁、Altitude、kill、零输出回读 | 通过 | 通过 |

断流轮由NX观察到Offboard约3秒后主动停发。自动退出前SF保持正100%、SA保持下档、SB保持中档，遥控输入没有丢失。停止最后目标到NX观察到模式退出为约1.140 s，包含遥测与采样延迟，不能当成精确的失效保护执行时间。报告见 [manual_v2.json](manual_v2.json)、[stream_stop_v1.json](stream_stop_v1.json)。

### 收尾时的未解锁Offboard显示及分析程序修正

断流轮的目标退出先发生在**已解锁**状态；随后操作者SF锁定，SA尚未回上档时，两个Debug样本短暂显示未解锁Offboard。NX一直没有恢复目标发送，`offboard_control_signal_lost=True`，实际`actuator_armed.armed=False`。该固件[失效保护框架](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/failsafe/framework.cpp#L404-L420)在未解锁时选择`Action::None`，[Commander](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/Commander.cpp#L2142-L2172)据用户模式意图更新实际模式；记录与此机制一致。

最初离线分析器把退出后任何`nav_state=14`都当成控制重新进入，产生了 [初次拒绝记录](stream_stop_initial_checker.json)。复核固件与真实数据后，分析器v2分别检查“NX没有再发目标”和“没有已解锁的Offboard重入”，另外明确列出未解锁模式恢复样本。例外只接受实际未解锁、SF关闭、Offboard信号仍丢失、模式意图仍为Offboard的记录；真实已解锁重入、有效Offboard信号恢复或再次解锁仍拒绝，补充了相应反例测试。没有修改飞控或C++控制参数，也没有删除该段数据。

## 本地与NX软件检查

本机预检、操作状态机、独立证据判定和既有回读测试40项通过。真实NX内存编译后的C++合约测试通过；PTY过程测试11项通过，其中增加了“初态未解锁→实际解锁→人工退出／计划断流”的两项用例。早期PTY测试在READY之前模拟解锁，因初态守卫正确拒绝启动而失败；修正测试为基于真实READY事件后通过，没有因此放宽生产代码。

测试后的生产ELF SHA256为 `e9fddfce561033530579944bfc3972a0f38b2bf44194d30b0c2c04d70efa0b41`，与上一轮未解锁台架使用的ELF一致。代码只在NX内存运行；编译临时目录和memfd退出后清理，日志在电脑保存。见 [uart_ctest.log](uart_ctest.log)、[uart_pty_tests.log](uart_pty_tests.log)。

## 判据与范围

- 使用固定版本MAVLink字典解码；姿态目标必须为固定初始实际四元数、零推力，禁止解锁、模式、位置或执行器命令。
- COM13独立记录必须确认真实已解锁Offboard区间及仍已解锁退出。人工接管要求SA返回Altitude；断流退出要求SA、SF、SB不变、RC有效且Offboard信号丢失。
- 至少两个PX4内部原始姿态目标样本与发送一致，另核对完整float32回传；退出后停止发送且不自动重入，观察至少10秒，最终恢复锁定、Altitude、实际kill及零输出。
- 地面自动锁定、SB停机或SF锁定先于目标退出，均不能冒充断流保护通过。
- COM13顺序采样和MAVLink到达时刻含采样／传输延迟，只证明状态转换及字段一致性，不作为毫秒级退出延时认证。
- 原MPC的实时完整影子运行还缺有效水平位置／速度／航向、真实关节状态及执行历史；不以历史或零值填充缺失反馈。位置目标真实回读必须另立无动力检查范围，不由本次姿态台架结果替代。

完整原始记录保存在电脑项目 `artifacts/nx_uart_armed_bench_20261030/`，报告保留文件散列以供复核。

## 下一层真实状态影子MPC的输入核对

两项通过并恢复后，使用 `scripts/audit_px4_live_mpc_inputs.py` 再做一次固定命令只读快照，没有访问NX或发控制指令。机体姿态、本地位置和里程计主题都有新鲜时间戳，但 `xy_valid=False`、`v_xy_valid=False`、`heading_good_for_control=False`；高度和垂向速度有效。主题更新不等于水平状态有效。

另有真实TS20关节状态、实际已执行指令历史、MAVLink世界系加速度／重置语义适配与跨设备时间对齐尚未验收；这个仅PX4的快照无法替它们作证。完整影子MPC入口继续关闭，不补零、不借用历史状态。本轮已有的原MPC真实NX回放1281/1281结论保持，不能混称实时影子运行。见 [live_input_readiness.json](live_input_readiness.json)；下一步先补齐实际定位与关节反馈，再接入无输出的实时MPC，之后才准备PVA原始设定值回读。
