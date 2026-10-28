# Pixhawk 独立 Debug 回读接入（2026-10-28）

**COM13 只读控制台检查通过；NX→PX4 设定值传输未测试。** 用户修正最初接错的线后，电脑上的 CH340 转串口以 57600 8N1、无流控进入 Pixhawk FMU Debug 的 NSH 控制台。

| 检查 | 实际结果 |
|---|---|
| 回车响应 | `nsh>` |
| `ver all` | PX4_FMU_V6X，Release 1.17.0，预期固件提交匹配 |
| `uxrce_dds_client status` | `not running` |
| `listener vehicle_status -n 1` | arming_state=1（未解锁），nav_state=4（非 Offboard），hil_state=0，failsafe=False |
| `listener trajectory_setpoint -n 1` | 成功读取飞控内部消息，timestamp=47,547,379 µs，打印时年龄约 7.952 ms |
| 串口退出 | 所有读取命令返回 NSH 提示符，关闭 COM13 |

实际内部消息为：position 全 NaN；velocity `[NaN, NaN, 0.70000]`；acceleration `[0, 0, NaN]`；yaw `-2.49709` rad；jerk 和 yawspeed 为 NaN。这些数值仅记录当时内部主题的内容，**不是本次发出的测试指令、不是 MPC 输出，也不是悬停目标**。7.952 ms 是 PX4 在打印时报告的主题年龄，不是电脑与飞控之间的通信延迟。

同轮只读 SSH 检查表明 NX 可以登录、识别到 `/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00 → /dev/ttyACM0`，未发现 Micro XRCE Agent 或原硬件探针进程。此时飞控 DDS 客户端也未运行，因此不能把既有内部主题误认成 NX DDS 输入成功。

初次接线时，57600 下收到乱码；只被动接收比较 57600、115200、921600，没有读到控制台或通过已知消息 CRC 校验的 MAVLink 帧。期间 COM13 曾短暂消失，用户确认并修正了接线。之后恢复标准 57600，四条只读命令全部成功。初始错误未通过改变飞控波特率、参数、软件或刷固件处理。

原始成功串口字节与解码文本仅保存在电脑 `artifacts/nx_px4_debug_20261028/readonly_v2.json`，失败及被动接收记录保留在同目录；[summary.json](summary.json)记录命令、状态、结果与原始证据 SHA-256。没有向飞控发送轨迹、模式切换或执行器指令，没有启动 DDS 或更改 NX 配置。

此结果证明独立 Debug 通道可以读取飞控内部 uORB 消息；不证明任何尚未发送的 NX 设定值被接收或执行。下一阶段需要先准备有限时长发送器，再按既定授权范围恢复临时 DDS 并核对独特字段模式；不能仅用主题存在、自己的 ROS 订阅或时间戳推进代替字段对应。具体方案见[接线与验收说明](../../NX_PX4_SETPOINT_READBACK.md)。
