# Pixhawk 内部设定值回读：独立 Debug 串口方案

> **2026-10-30 更新：下文保留为 10 月 28 日的历史验收记录。随后 USB MAVLink／DDS 切换触发 Hard Fault，原实机入口已隔离；不要按本文重新切换 USB。当前执行方案改用 [TELEM1 UART 消息与验收流程](NX_MAVLINK_CONTROL_CHAIN.md)，FMU Debug 仍作独立回读。UART 方案不表示原固件故障已修复。**

适用设备：Orin NX（达妙载板）＋ Pixhawk 6X Pro，已核验 PX4 1.17 固件提交 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。2026-10-28 已完成 **NX C++ 限时发送 → USB DDS → 飞控内部 uORB → COM13 独立回读**。10 秒发送 184 条测试设定值；150 个内部采样中，A/B/C 分别有 7/10/18 个新时间戳样本逐字段匹配。其余 115 个匹配发送前捕获的内部模式，未发现无法解释的字段组合。全程未解锁、非 Offboard。见[本次验收记录](validation/nx_setpoint_readback_20261028/README.md)，此前的纯只读检查见[接入记录](validation/nx_px4_debug_20261028/README.md)。

## 为什么要第二条通道

当前固件的 [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.yaml) 包含 `/fmu/in/trajectory_setpoint` 输入，没有对应的 `trajectory_setpoint` 输出。NX 自己订阅这个输入话题只能看到自己的发布，不能证明消息进入了飞控。命令 401 的 ACK 也不能证明轨迹字段被接收。

推荐同时使用：

```text
NX USB 主机口 ──现有 USB 数据线── Pixhawk USB
           ROS 2 / Agent / XRCE-DDS：10 秒 A/B/C 测试设定值

电脑 USB ──3.3 V TTL USB 转串口── Pixhawk FMU Debug
           原生 NSH 控制台：listener 读取飞控内部 uORB
```

Debug 控制台接到电脑，日志直接保存在电脑；NX 不新增串口驱动、系统配置或实验日志。保留现有飞控固件与 DDS 话题配置。这里读到的是飞控内部 uORB 输入主题，**不是位置控制器已经执行，更不是电机已经响应**。

## 实际接线

Pixhawk 6X Pro 使用 FMU Debug 的 USART3，飞控内部设备名 `/dev/ttyS2`。下面是 **FMU Debug 10 针** 定义；不要接到 IO Debug，也不要用 6 针 Debug Mini 的编号。针脚方向按板上 Pin 1 标记和厂商图确认，不按线色猜测。[PX4 6X Pro 官方引脚说明](https://docs.px4.io/main/en/flight_controller/pixhawk6x_pro#debug-port)、[Holybro 基板引脚表](https://docs.holybro.com/autopilot/pixhawk-baseboards/pixhawk-baseboard-v1-ports#fmu-debug-port-jst-sh-1mm-pitch)。

| Pixhawk FMU Debug | USB 转串口 | 用途 |
|---|---|---|
| Pin 2，Console TX，3.3 V | RXD | 飞控输出送到电脑 |
| Pin 3，Console RX，3.3 V | TXD | 电脑发送读取命令 |
| Pin 10，GND | GND | 共地 |
| Pin 1，Vtref / 3.3 V | 不连接 | 本方案不从转串口给飞控供电 |
| Pin 4–9 | 不连接 | 本方案不使用 SWD、复位等信号 |

使用 **3.3 V TTL 信号电平**；确认模块的 TX/RX 电平确实为 3.3 V，仅将供电选择跳线切到 3.3 V 并不总能保证信号电平。RS-232 的电平不适用于该口。接线时断开供电；完成后恢复 NX 和飞控原供电，动力继续断开。

电脑端选择新增的 COM 口：**57600 baud、8 数据位、无校验、1 停止位、无 RTS/CTS 流控**。57600 来自当前固件的 [USART3 配置](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/boards/px4/fmu-v6x/nuttx-config/nsh/defconfig#L318)，不是 NX 上 USB Agent 的 921600。回车后应出现 `nsh>`；发送范围须由操作者明确批准。

## 只读确认命令

先记录 `ver all`，核对上述固件提交，再执行：

```text
uxrce_dds_client status
listener vehicle_status -n 1
listener trajectory_setpoint -n 1
```

这些是独立串口上的 NSH 命令，不是 ROS 命令。若轨迹主题从未发布，输出 `never published` 是合理的初始结果。[当前固件 listener 实现](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/systemcmds/topic_listener/listener_main.cpp) 可打印一次最近值，或用 `-n` 监听新值。

飞控若已重启，USB 可能已经恢复 MAVLink；开始 DDS 发送之前必须确认或重新获准执行临时 USB 协议切换。同一根 USB 不同时运行 MAVLink Shell 与 XRCE Agent。Debug 控制台不受这项 USB 占用限制。

本次发送前实际读到 `uxrce_dds_client: not running`，NX 上也没有 Agent，飞控 USB 识别为 `/dev/ttyACM0`。`vehicle_status` 显示未解锁、非 Offboard、非 HIL；内部 `trajectory_setpoint` 已存在。这些既有值不能证明 NX 消息送达，因此工具先记录内部基线，再以独特的 A/B/C 字段模式及新时间戳验收。

## 本次发送范围与顺序

用户授权本阶段后，使用单独的 C++ 工具，只发布 `TrajectorySetpoint`。它持续检查真实 `VehicleStatus`，状态接收超过 1 秒未更新、时间戳回退、出现解锁/Offboard/HIL 或失去 DDS 订阅端时终止。没有 `OffboardControlMode` 心跳、切模式、解锁、外部定位或执行器发布器。定位有效不是检查 uORB 字段的先决条件；动力保持断开。本轮没有修改 NX 系统配置或飞控保存参数。

1. 从 COM13 核验固件、未解锁状态、`SYS_USB_AUTO=2` 和 DDS domain 0，记录原有轨迹字段。
2. 停止 USB 上的 MAVLink 实例；NX Agent 打开该 USB 设备后，通过独立 Debug 口启动临时 DDS。代码核验 Agent 进程确实持有串口，随后等待真实状态与订阅匹配。
3. 先启动 `listener trajectory_setpoint -n 150 -r 10`，再向 NX 程序的控制管道写入一次 `start`。未触发时绝不发设定值。
4. A：0–3 s；B：3–6 s，A 的 P/V/A 取反；C：6–10 s，A 的 P/V/A 减半。yaw 始终为 ENU 0 → NED π/2。名义发送间隔 50 ms，最多 200 条；调度停顿后不补发突发消息。10 秒后停止发送，继续观察状态 3 秒。
5. 停止自建发布器和 Agent，删除 NX 内存工作目录，关闭 COM13；全部结果通过 SSH 回到电脑。

发送计数不是固定 20 Hz 压测指标。Debug 的 `-r 10` 只是观察采样率；它与 DDS 发送、PX4 内部其他发布者异步，因此不要求回读每条发送，也不能由 35/184 推断丢包率。

下面这组数值是实际采用的编码检查输入，**不是 MPC 飞行参考**。模式 A 的 ENU 输入及期望 PX4 内部 NED 字段如下：

| 字段 | NX ENU 模板 A | 飞控 NED 期望 |
|---|---|---|
| position，m | `[0.125, -0.25, 0.5]` | `[-0.25, 0.125, -0.5]` |
| velocity，m/s | `[0.03125, -0.0625, 0.015625]` | `[-0.0625, 0.03125, -0.015625]` |
| acceleration，m/s² | `[0.0625, 0.125, -0.03125]` | `[0.125, 0.0625, 0.03125]` |
| yaw，rad | `0` | `π/2`，按 float32 编码 |
| jerk、yawspeed | 不使用 | 全部 NaN，不补零 |

字段定义以当前固件 [TrajectorySetpoint.msg](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/versioned/TrajectorySetpoint.msg) 为准。模板二进制可精确表达的向量值便于观察符号和顺序；真正 MPC 输出还会有 float32 量化，控制台文本打印精度也有限。逐字段比较使用编码后的值和实际打印精度，不把文本打印称为逐位传输校验。

## 验收与时钟

1. 在飞控 NSH 原始输出中找到 `trajectory_setpoint`，每种模式至少两个不同的新时间戳样本，A/B/C 顺序正确，且时间戳严格位于前后飞控状态时间之间；重复时间戳不增加计数。
2. 同时核对 position、velocity、acceleration、yaw 和未使用字段的 NaN。重点检查 ENU/NED、rad、float32，以及没有叠加仿真 `ref_alt`、没有重复相加速度前馈。
3. 前后 `vehicle_status` 均保持未解锁、未进入 Offboard；NX 全过程状态记录也须符合台架条件。没有完整窗口或缺失退出记录不能通过。
4. NX ROS 时间与飞控启动时间不应直接相减。当前 [DDS 收发模板](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.h.em) 把会话时间偏移交给序列化/反序列化处理。本阶段记录时间戳推进和模式对应，不用串口文本抵达电脑的时间估算单程 USB 延迟。
5. 结束停止测试发布器，读回退出状态。保留原始串口文本、NX 发送清单、源码/消息/固件身份及结果在电脑。没有真实控制接管，所以不把停止测试发布器描述为实机失联保护验证。

回读不简单忽略所有“不匹配”的样本。只有与发送前捕获的 NaN-position 内部模式逐字段一致的样本才归入基线；该模式允许内部 yaw 随机体估计值变化。任何既不匹配 A/B/C、也不匹配这组已观测基线的字段组合都使验收失败。数值容差为打印精度对应的 `5.1e-6`；NaN 必须仍是 NaN。

## 代码对应与复现

| 文件 | 作用 |
|---|---|
| [px4_setpoint_probe.cpp](../ros2_ws/src/flyarm_hardware_state/src/px4_setpoint_probe.cpp) | C++ 状态检查、启动触发、10 秒 A/B/C 发送和自动停止 |
| [px4_message_adapter.hpp](../ros2_ws/src/flyarm_hardware_state/include/flyarm_hardware_state/px4_message_adapter.hpp) | 与完整接口回放共用的状态接收、PVA 编码、ROS 消息构造 |
| [probe_setpoint_ros.py](../ros2_ws/src/flyarm_hardware_state/test/probe_setpoint_ros.py) | 隔离 domain 210 的七类 ROS 软件检查，无 Agent、无 USB |
| [build_nx_setpoint_check.py](../scripts/build_nx_setpoint_check.py) | NX 内存编译与软件测试，将已测试 ELF 和散列带回电脑，不安装到持久目录 |
| [run_nx_setpoint_readback.py](../scripts/run_nx_setpoint_readback.py) | 电脑上的有限时长总控；先启动 observer，再触发发送 |
| [nx_setpoint_check_session_remote.py](../scripts/nx_setpoint_check_session_remote.py) | NX RAM 临时执行、Agent 串口所有权检查、进程/文件清理 |
| [px4_debug_console.py](../scripts/px4_debug_console.py) | COM13 固定命令白名单、NSH 原始字节和文本记录 |
| [summarize_nx_setpoint_readback.py](../scripts/summarize_nx_setpoint_readback.py) | 独立字段判据、完整性、身份、退出和清理检查 |
| [test_nx_setpoint_readback.py](../tests/unit/test_nx_setpoint_readback.py) | 错轴、错 NaN、缺模式、旧时间戳、异常内部值及不完整退出不能通过 |

在电脑运行，下列主机与路径为占位，执行发送前需确认本次授权及动力断开：

```powershell
python scripts/build_nx_setpoint_check.py --host NX_ADDRESS --identity SSH_KEY --known-hosts VERIFIED_KNOWN_HOSTS --output artifacts/setpoint_build
python scripts/run_nx_setpoint_readback.py --host NX_ADDRESS --identity SSH_KEY --known-hosts VERIFIED_KNOWN_HOSTS --build artifacts/setpoint_build --output artifacts/setpoint_run --port COM13 --approved-setpoint-check --bench-power-disconnected
python scripts/summarize_nx_setpoint_readback.py --run artifacts/setpoint_run
```

输出目录必须新建，避免覆盖失败记录。每次普通运行要求飞控 DDS 未运行且 USB MAVLink 可识别。只有在已核实前一次尝试确实停止了该 USB MAVLink、并已停止失败 DDS 客户端时，才使用 `--usb-already-reserved`；这个标志不跳过状态、固件、domain 或字段验收。

## USB 恢复注意事项与当前边界

本次在发送前捕获到 `open /dev/ttyACM0 failed (128)`。NX 端设备仍枚举，设备电源管理为 `on/active`、累计 suspend 为 0；Agent 打开后错误仍存在。只重启未解锁飞控后，临时 DDS 和完整检查恢复成功。**尚未确定飞控 USB 进入该状态的根因**；不能将其归因于 MPC 计算量、定位无效或 NX 性能。保留了失败记录，未通过改 ROS 门槛或关闭同步处理。

普通 `param show` 默认只显示已使用参数；DDS 尚未启动时可能查不到 domain。工具使用 `param show -q UXRCE_DDS_DOM_ID` 精确读取，而非改参数。[当前固件参数命令实现](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/systemcmds/param/param.cpp)。

结束后的 NX 发布器和 Agent 均已停止，临时 RAM 文件已删除。飞控临时 DDS 客户端没有被改成开机启动，也没有自动恢复 USB MAVLink；重启未解锁飞控可恢复原 `SYS_USB_AUTO=2` 的启动行为。状态打印中的短暂 `connected` 不能代替确认 NX Agent 仍在运行。

当前通过的是字段传输和编码；真实 MPC 持续运行、Offboard 接管/退出、有效动捕定位、关节反馈及电机响应各有独立验收范围。本测试没有验证这些环节，也没有给出单程通信延迟或端到端实时性结论。

若独立 Debug 接口不可用，可另行讨论 TELEM MAVLink 控制台或飞控内部日志；都需要核对接口/配置并另行批准。当前推荐方案不修改飞控固件以增加回传话题。
