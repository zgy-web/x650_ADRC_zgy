# Pixhawk 内部设定值回读：独立 Debug 串口方案

适用设备：Orin NX（达妙载板）＋ Pixhawk 6X Pro，已核验 PX4 1.17 固件提交 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。2026-10-28 已完成接线和真实 Debug 控制台只读检查：电脑 `COM13`、CH340、57600 8N1 能直接读取固件、状态及内部 `trajectory_setpoint`。**没有向真实飞控发送轨迹设定值，没有进行接管或解锁**；NX 发出设定值与飞控内部字段的对应验收仍未执行。见[接入记录](validation/nx_px4_debug_20261028/README.md)。

## 为什么要第二条通道

当前固件的 [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.yaml) 包含 `/fmu/in/trajectory_setpoint` 输入，没有对应的 `trajectory_setpoint` 输出。NX 自己订阅这个输入话题只能看到自己的发布，不能证明消息进入了飞控。命令 401 的 ACK 也不能证明轨迹字段被接收。

推荐同时使用：

```text
NX USB 主机口 ──现有 USB 数据线── Pixhawk USB
           ROS 2 / Agent / XRCE-DDS：后续发送测试设定值

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

电脑端选择新增的 COM 口：**57600 baud、8 数据位、无校验、1 停止位、无 RTS/CTS 流控**。57600 来自当前固件的 [USART3 配置](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/boards/px4/fmu-v6x/nuttx-config/nsh/defconfig#L318)，不是 NX 上 USB Agent 的 921600。回车后应出现 `nsh>`；控制台能使用后再进行发送授权。

## 只读确认命令

先记录 `ver all`，核对上述固件提交，再执行：

```text
uxrce_dds_client status
listener vehicle_status -n 1
listener trajectory_setpoint -n 1
```

这些是独立串口上的 NSH 命令，不是 ROS 命令。若轨迹主题从未发布，输出 `never published` 是合理的初始结果。[当前固件 listener 实现](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/systemcmds/topic_listener/listener_main.cpp) 可打印一次最近值，或用 `-n` 监听新值。

飞控若已重启，USB 可能已经恢复 MAVLink；开始 DDS 发送之前必须确认或重新获准执行临时 USB 协议切换。同一根 USB 不同时运行 MAVLink Shell 与 XRCE Agent。Debug 控制台不受这项 USB 占用限制。

本次实际读到 `uxrce_dds_client: not running`，NX 进程检查也没有 Agent，飞控 USB 在 NX 上仍识别为 `/dev/ttyACM0`。`vehicle_status` 显示未解锁、非 Offboard、非 HIL；内部 `trajectory_setpoint` 已存在且时间戳新鲜。这是实际读取到的既有内部值，不能因为话题存在就宣称 NX 的消息已经送达。后续测试须以独特的 A/B/C 字段模式和对应的新时间戳匹配，不能只检查话题存在或数据更新。

## 后续实机发送的明确范围（本次未执行）

单独准备只包含 `TrajectorySetpoint` 发布器的有限时长工具，等待用户批准部署与发送；使用持续的新鲜未解锁状态作为台架条件。没有 `OffboardControlMode` 心跳、切模式、解锁、外部定位或执行器发布器。定位有效不是检查 uORB 字段的先决条件，动力保持断开。

首先发送一个已标记的初始测试模式，使主题存在，再启动 `listener trajectory_setpoint -n 10 -r 5`，随后在 3 个可区分的模式间变化，总发送窗口限定为 10 秒，结束即停止。`-r 5` 是控制台打印采样率，本测试不测控制吞吐，也不要求回读每一条发送。发送器和 observer 的完整实现、部署及执行仍需下一步单独完成。

下面这组数值仅是后续编码检查模板，**不是 MPC 飞行参考**。模式 A 的 ENU 输入及期望 PX4 内部 NED 字段如下，B/C 改变符号和幅值以排除只读到旧缓存：

| 字段 | NX ENU 模板 A | 飞控 NED 期望 |
|---|---|---|
| position，m | `[0.125, -0.25, 0.5]` | `[-0.25, 0.125, -0.5]` |
| velocity，m/s | `[0.03125, -0.0625, 0.015625]` | `[-0.0625, 0.03125, -0.015625]` |
| acceleration，m/s² | `[0.0625, 0.125, -0.03125]` | `[0.125, 0.0625, 0.03125]` |
| yaw，rad | `0` | `π/2`，按 float32 编码 |
| jerk、yawspeed | 不使用 | 全部 NaN，不补零 |

字段定义以当前固件 [TrajectorySetpoint.msg](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/versioned/TrajectorySetpoint.msg) 为准。模板二进制可精确表达的向量值便于观察符号和顺序；真正 MPC 输出还会有 float32 量化，控制台文本打印精度也有限。逐字段比较使用编码后的值和实际打印精度，不把文本打印称为逐位传输校验。

## 验收与时钟

1. 在飞控 NSH 原始输出中找到 `trajectory_setpoint`，新时间戳及 A/B/C 字段模式与发送顺序对应；只读到旧时间戳或单一旧模式不通过。
2. 同时核对 position、velocity、acceleration、yaw 和未使用字段的 NaN。重点检查 ENU/NED、rad、float32，以及没有叠加仿真 `ref_alt`、没有重复相加速度前馈。
3. 前后 `vehicle_status` 均保持未解锁、未进入 Offboard；NX 全过程状态记录也须符合台架条件。没有完整窗口或缺失退出记录不能通过。
4. NX ROS 时间与飞控启动时间不应直接相减。当前 [DDS 收发模板](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.h.em) 把会话时间偏移交给序列化/反序列化处理。本阶段记录时间戳推进和模式对应，不用串口文本抵达电脑的时间估算单程 USB 延迟。
5. 结束停止测试发布器，读回退出状态。保留原始串口文本、NX 发送清单、源码/消息/固件身份及结果在电脑。没有真实控制接管，所以不把停止测试发布器描述为实机失联保护验证。

若独立 Debug 接口不可用，可另行讨论 TELEM MAVLink 控制台或飞控内部日志；都需要核对接口/配置并另行批准。当前推荐方案不修改飞控固件以增加回传话题。
