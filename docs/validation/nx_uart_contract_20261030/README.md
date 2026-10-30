# 2026-10-30：UART 消息合同与原 MPC 回放

本记录区分已实测、已配置、尚未验收三类结果。执行方案和代码导航见 [NX MAVLink 控制链](../../NX_MAVLINK_CONTROL_CHAIN.md)。

## 已完成

| 项目 | 结果 | 证据 |
|---|---|---|
| 固件／协议身份 | PX4 `d6f12ad1…`；对应 MAVLink 子模块 `33af200d…`，四个 XML 散列锁定 | `configs/hardware/nx_mavlink_dialect_lock.json` |
| 原 MPC 在真实 NX 回放 | 1281/1281；保持 711、受扰 570；原算法库和参数未改变 | [MPC 汇总](mpc_replay_summary.json)、[9 项 CTest](mpc_ctest.log) |
| 新 MAVLink PVA 编码 | 对原 MPC 新计算的输出进行序列化、CRC 解析、字段比较；1281 条全部通过 | 同上，`mavlink_wire_replay=true` |
| C++ 台架代码 | NX ARM64 构建、纯 C++ 测试、9 个虚拟串口场景通过 | [构建收据](bench_build_receipt.json)、[CTest](uart_ctest.log)、[PTY 记录](uart_pty_tests.log) |
| 真实 UART 原配置接收 | 20.0064 s，12797 字节，330 个 CRC 正确完整帧；发送 0 字节 | [只读采集](readonly_capture.json) |
| TELEM1 参数更新 | 用户批准的五项参数写入、回读、重启后回读全部一致 | [参数收据](profile_receipt.json) |
| 新配置真实接收 | 30.0064 s；姿态 49.894 Hz、RC 19.993 Hz、估计器 9.995 Hz；144451 字节均为 CRC 正确帧 | [实测速率](configured_rates.json) |
| 真实内部姿态目标 | 7 次独立 Debug 原始字段匹配；对应稳定区间 159 条完整 float32 目标遥测一致 | [回读与人工退出](manual_readback.json) |
| 未解锁 SA 人工退出 | 进入 Offboard 后由 SA 返回 Altitude；NX 停发并继续观察约45.14 s，无自动重入 | 同上；全程输出为零，不等价于已解锁接管 |
| 计划停流／PX4检测 | 第三轮真实进入Offboard，约3.01 s后停发；独立回读信号丢失；目标发送未恢复 | [第三轮分析](stream_stop_v3.json) |
| 最后恢复 | 操作者恢复SA上、SB下；独立回读未解锁、Altitude、kill=True、所有输出为零 | 同上及电脑原始restore记录 |
| 电脑端测试 | 本轮最终相关 Python 回归 100 项通过 | 测试文件见下文 |

NX 编译、回放和只读程序均在内存临时运行。已完成会话确认清理，新增持久实验文件为零；原始结果在电脑项目的 `artifacts/nx_uart_contract_20261030/`。既有算法库只读取，未迁移或重新安装。

1281 个输入是从既有仿真记录重建的消息与独立快照，逐个恢复原控制器记忆后重算；不是实时飞控状态，也不是 1281 次独立飞行。MPC 输出的 ENU/NED、P/V/A、yaw、忽略字段、float32 编码和发布后提交一致。输入 float32 量化导致的变化单独记录：三个机体速度量最大差约 `2.6871e-7 m/s`，两关节量最大差约 `4.3280e-7 rad/s`；不把量化输入与原双精度输入宣称为逐位相同。**本轮没有重新做延迟基准，不以此次回放时长代替 MPC 单步耗时。**

## 实测发现与修复

原配置为 57600 baud、MAV_0_RATE=1200 B/s。飞控 `tx rate mult=0.302`；实际 C++ 接收姿态四元数约 3.028 Hz、RC 约 1.514 Hz、CURRENT_MODE 约 0.151 Hz，明显不能用于选定的 50 Hz 台架。这是配置限速，不能解读为 MPC 求解性能不足。

用户批准后仅设置：

```text
SER_TEL1_BAUD = 921600
MAV_0_MODE = 1
MAV_0_RATE = 30000
MAV_0_FLOW_CTRL = 0
MAV_0_FORWARD = 0
```

重启后 COM13 独立回读确认 `/dev/ttyS6 @921600`、Custom、流控关闭、转发关闭、预算 30000 B/s、限速比例 1.000。前后飞控未解锁，所有 DShot 输出为零。旧值已保存在电脑，不需要全参数恢复。

台架代码补充了异步包顺序测试：模式包可以先于遥控拨杆包到达，旧的 SA 上档样本不应被当成“进入 Offboard 后再次拨回上档”。只有确认过 SA Offboard 档后的回拨，或 PX4 实际退出模式，才按人工／模式退出锁住当前会话；不放宽状态过期检查。退出后数据恢复也不自动重入。

NX 网络中断后，操作者将 NX 断电重启，SSH 恢复；原 UART 设备树修复仍有效，未新增 NX 配置修改。随后实测的姿态、RC、模式、心跳最大到达间隔分别约 33.29、64.39、510.93、1003.74 ms，满足本台架的100、150、1200、1500 ms守卫阈值。CURRENT_MODE 静态约 2 Hz，受源主题发布限制。电池主题在动力断开时没有发布，未伪造电池读数。

人工退出运行共发送1483条目标，最大发送间隔22.28 ms；这属于发送节奏测量，不是MPC计算耗时或端到端执行延迟。独立Debug确认Offboard维持约9.692 s、目标为实际初始姿态／零推力。最初分析对五位小数文本误用了半末位舍入误差界；本次真实显示为向零截断，现以精确显示形式校验，并以稳定区间的完整float32遥测交叉验证。修复发生在电脑上的验收脚本，NX控制代码未改变。模式／控制器切换边界的数据差异保留在汇总中，没有改写原始记录。

第三轮通过电脑实时引导页协调操作，避免聊天回复晚于发送窗口。操作者主动点击开始，再按页面提示拨杆；没有延长60 s发送上限。NX在观察到Offboard后3.00956 s停止目标，282条目标的最大发送间隔21.9649 ms，随后68.975 s没有恢复发送。Debug持续观察到丢失标志约81.604 s，内部控制模式时间戳不再更新。页内完成恢复后，额外COM13只读检查再次确认Altitude、未解锁、kill=True。当前NX程序已退出、内存可执行文件已关闭，新增NX持久文件为零。

未解锁时没有自动离开Offboard，这是本版本[失效保护框架未解锁分支](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/failsafe/framework.cpp#L417-L420)的行为；超时仍由[独立Offboard检查](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp#L36-L62)标记。这里不将自动退出标为通过，也不把该现象诊断为MPC计算或UART故障。最后停机档下`pre_flight_checks_pass=False`如实保留；不能据此直接开展解锁测试，后续须在具体试验初态重新检查预检原因。

## 尚未通过、不能外推的部分

- `cpp_rates_v1/v2/v3` 因 NX 网络故障未取得结果，不计为试验成功；恢复后 `cpp_rates_v4` 为有效新频率实测。
- 断流前两轮 `cpp_stream_stop_v1/v2` 均在发送窗口内未进入 Offboard，到60秒上限后停发，`seen_offboard=false`、`stop_reason=session_limit`。[首轮摘要](stream_stop_expired_v1.json)、[第二轮摘要](stream_stop_expired_v2.json)保留未完成条件；有效断流检测来自独立的第三轮。
- 本轮人工接管只验证未解锁模式切换。已解锁时的人工接管和Offboard失效保护尚未通过，不能用未解锁检查或软件PTY测试替代。
- USB MAVLink／DDS 切换的旧 Hard Fault 入口继续隔离。UART 方案不证明该固件问题已修复。
- 当前没有合格的水平定位、真实 TS20 完整反馈及同时间基准的全部 MPC 状态字段。ODOMETRY 不含世界系加速度，HIGHRES_IMU 是不同含义的比力，不能直接替代。
- 动力保持断开；本记录不构成带动力闭环或试飞通过。

带宽结果见 [bandwidth.json](bandwidth.json)：含异步余量，当前台架上下行分别为 8901 / 5971 B/s；后续完整 MPC 预留为 25181 / 6071 B/s。921600、8N1 的每方向容量为 92160 B/s。签名尺寸也另行计算，但本次没有启用签名。

## 下一步

1. 本轮已恢复SA上档、SF负100%、SB下档，动力继续断开。下一阶段先在具体试验初态读取预检原因，区分停机请求与其他问题。
2. 已解锁人工接管／断流自动退出要在动力断开、预检允许且操作者获知当次具体SF/SA操作后进行，不绕过预检；未解锁检测不能替代这一项。
3. 沿用电脑实时引导页协调限时动作，但任何新增解锁阶段须先补齐明确流程、记录和操作者确认；现有页面没有解锁或模式命令。结束时恢复并回读实际停机标志，若飞控重启不能只根据拨杆位置推定kill状态。
4. 补齐真实 MPC 状态后，先影子运行原 MPC，再做真实 PVA 内部回读。末端保持、普通八字、快速八字依次推进。

主要可复现入口：

```text
scripts/prepare_nx_mavlink_dialect.py
scripts/nx_mavlink_bandwidth.py
scripts/build_nx_mavlink_uart.py
scripts/run_nx_full_interface_replay.py --mavlink-generated <已校验协议目录>
scripts/summarize_nx_full_interface_replay.py
scripts/prepare_px4_uart_profile.py                 # 默认只读
scripts/run_nx_mavlink_uart_bench.py --case observe  # 默认零发送
scripts/observe_px4_uart_bench.py                   # 独立 Debug 只读
scripts/summarize_nx_mavlink_capture.py
scripts/check_nx_uart_manual_readback.py
scripts/nx_uart_operator_panel.py
scripts/check_nx_uart_stream_stop.py
```

电脑端本轮最终回归：`test_nx_mavlink_profile.py`、`test_x650_hardware_replay.py`、`test_nx_uart_mavlink_readonly.py`、`test_nx_offboard_bench.py`、`test_nx_setpoint_readback.py`。完整输入与生成协议头未复制到源码树；来源、散列和每批结果身份保留在记录中。

后续验收脚本修正已运行 `test_nx_uart_manual_readback.py` 和 `test_nx_mavlink_profile.py`，22项通过；新增测试覆盖真实截断样本、错误目标、轴交换、精度不足和非数值输入。真实人工退出记录也由同一复核器通过。

电脑引导页新增测试覆盖GET不启动硬件、显式本机POST、跨站／错误令牌拒绝、无解锁／配置入口、恢复状态证据和恢复后显示更新。引导页、原显示校验及profile三组共29项通过；第三轮真实记录另由`check_nx_uart_stream_stop.py`复核通过。所有控制目标仍由原`bench_build_v3`发出。
