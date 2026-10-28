# NX → Pixhawk 限时设定值与独立内部回读（2026-10-28）

**本阶段通过：实体 NX 的 C++ 发布器发出的测试字段已进入实体 Pixhawk 的内部 `trajectory_setpoint`。** 结论来自电脑 COM13 上的独立 NSH/uORB 读取，不是 NX 自己订阅发送话题。动力保持断开，全程未解锁、非 Offboard；未执行真实 MPC 闭环，也未验证电机或关节响应。

## 结果

| 检查 | 结果 |
|---|---|
| 发送窗口 | 固定 10 s，随后监测状态 3 s |
| NX 发布数量 | 184 条；A/B/C = 55/55/74 |
| 独立内部采样 | `listener trajectory_setpoint -n 150 -r 10`，150 个不同时间戳 |
| A/B/C 字段完整匹配 | 7/10/18 个样本，共 35 个 |
| 其他内部样本 | 115 个均符合发送前捕获的内部模式；0 个无法解释的组合 |
| 核对字段 | position、velocity、acceleration、yaw、jerk 和 yawspeed 的 NaN |
| 新鲜性和顺序 | 每种模式至少 2 个新时间戳；A→B→C，时间戳处于前后状态时间之间 |
| 台架状态 | 前后 arming_state=1、nav_state=4、hil_state=0、failsafe=False；过程未捕获解锁/Offboard/HIL/状态回退 |
| NX 飞控状态观测 | 28 次推进，最大接收年龄 0.5134 s；这是低频 VehicleStatus 的年龄，不是控制计算耗时 |
| 发送权限范围 | 解锁请求=0、Offboard 请求=0、执行器指令=0；没有相应发布器 |
| 清理 | 发布器、Agent 均停止；NX 临时 RAM 目录删除；COM13 关闭 |

模式 A 的实际内部样例：

```text
timestamp: 25070650
position: [-0.25000, 0.12500, -0.50000]
velocity: [-0.06250, 0.03125, -0.01562]
acceleration: [0.12500, 0.06250, 0.03125]
jerk: [nan, nan, nan]
yaw: 1.57080
yawspeed: nan
```

NX 原 ENU 输入为 `p=[0.125,-0.25,0.5]`、`v=[0.03125,-0.0625,0.015625]`、`a=[0.0625,0.125,-0.03125]`、yaw=0。内部 NED 轴序、正负号与 π/2 偏航符合预期。控制台打印只有有限小数位，例如 `-0.015625` 打印为 `-0.01562`，采用 `5.1e-6` 文本容差；不称为逐位传输验证。

115 个内部模式样本为 position 全 NaN、velocity `[NaN,NaN,0.7]`、acceleration `[0,0,NaN]`，jerk/yawspeed 为 NaN；yaw 在约 −2.81629 至 −2.81503 rad 间变化。这个模式已在发送前观察到。审计器只允许该既有模式的穿插，未知字段组合会失败，不能简单忽略所有不匹配数据。

Debug 仅按 10 Hz 观察，PX4 内部还存在其他发布者；35/184 **不是接收成功率，也不能推导丢包率**。本阶段验证三种字段模式确实送达，不测最大频率、单程延迟或实时性。

## 软件身份与测试

- 飞控固件：PX4 1.17.0，`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。
- 复用已部署 px4_msgs release/1.17 和 Agent 2.4.2，未更换依赖。
- 本次 ROS 包源码归档 SHA-256：`5e75ebfd8358b277b71243cf0d2b8ea6d1b4885b0b5571cbb60a9651b7ee0f98`。
- 执行的 ELF SHA-256：`539079861c31eb208201b9820b7a4f3ef68a55d519d9b826d436d1141e1133ad`。
- NX CTest 2/2；已有 ROS 状态/ACK 软件检查 3/3；新设定值 ROS 软件检查 7/7。
- 七种新检查：正常、没有启动触发、状态过期、解锁、Offboard、HIL、状态时间戳回退。异常工况均按预期不发送或停止。
- 电脑端 `test_nx_setpoint_readback.py`、`test_nx_hardware_link_summary.py`、`test_px4_usb_readonly_protocol.py`、`test_x650_hardware_replay.py` 合计 **64 passed**。

构建在 NX `/dev/shm` 进行，测试后的 ELF 回传电脑保存。实机执行时再传入 RAM，结束删除。没有在 NX 持久目录增加程序、实验输入、诊断记录或日志；既有 `/home/jetson/flyarm` 依赖保持原样。

## 前三次尝试和 USB 恢复

[attempts.json](attempts.json) 保留每次结果、发送计数和原始证据散列。前三次发送计数均为 **0**：

1. `live_v1`：DDS 尚未启动，其 domain 参数未标为 used，普通 `param show` 返回空列表，被准备程序拒绝。修正为 `param show -q` 精确只读查询，实际值为 0，没有改参数。
2. `live_v2`：停止 USB MAVLink 后启动 DDS，飞控返回 `open /dev/ttyACM0 failed (128)`，没有开始 NX 发送。
3. `live_v3`：先确认 NX Agent 已打开 USB，飞控仍返回同一错误。关闭测试进程，未触发发送。
4. 单独检查 NX USB 仍枚举，电源管理 `on/active`、累计 suspended=0。重启未解锁飞控后，`live_v4` 完成整个窗口并通过字段审计。

飞控进入 USB 未连接状态的根因仍待专项复现。本次不能归因于 MPC 或 NX 性能，也没有证据将它归因于 NX 自动休眠。没有修改 USB 电源策略、NX 系统配置、飞控持久参数或固件。重启只针对未解锁台架飞控。

测试结束停止 NX Agent；飞控仍保留临时 DDS 客户端。恢复原 USB MAVLink 可重启未解锁飞控。没有将临时 DDS 配为开机启动，也没有自动改变模式或解锁。

## 证据和离线复核

- [summary.json](summary.json)：完整原始记录的审计结果和散列；原始字节仅保存在电脑 `artifacts/nx_setpoint_readback_20261028/`。
- [evidence/](evidence/)：可离线复核的字段、发布、退出与清理记录。仅去掉固件输出中的硬件唯一编号和无关 Agent 日志；没有改设定值、时间戳或状态。原始与整理版结果逐项相同。
- [evidence_manifest.json](evidence_manifest.json)：原始/整理版散列和删减范围。
- [build.log](build.log)、[build_report.json](build_report.json)、[source_manifest.json](source_manifest.json)、[binary_manifest.json](binary_manifest.json)：实际 NX 构建、测试及 ELF 身份。
- [tools_manifest.json](tools_manifest.json)：电脑/NX 会话脚本、审计器和单元测试源码散列。
- [cleanup_independent.json](cleanup_independent.json)：结束后另一次只读 SSH 确认没有 Agent、发布器和对应 RAM 工作目录。

在项目根目录重算：

```powershell
python scripts/summarize_nx_setpoint_readback.py --run docs/validation/nx_setpoint_readback_20261028/evidence
```

下一步仍应区分“字段进入飞控”和“位置控制器按预期执行”。接管/退出、有效动捕定位、真实关节状态及动力响应不在本次验收范围内。具体接线、代码导航和实机复现命令见[限时发送指南](../../NX_PX4_SETPOINT_READBACK.md)。
