# NX → Pixhawk：UART 消息、带宽与分阶段验收

日期：2026-10-30。对象：Orin NX／达妙载板、Pixhawk 6X Pro、TELEM1 三线 UART。当前阶段为动力断开的通信台架。**五项通信参数已获批准、应用并在飞控重启后回读一致；新刷新率、真实设定值回读、人工接管和发送中断后的退出尚未验收。** 重启后 NX 网络暂时不可达，尚未成功下发新频率试验。

本次结果见 [验证记录](validation/nx_uart_contract_20261030/README.md)。使用的 PX4 固件为 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`；MAVLink 定义绑定到该固件的子模块提交 `33af200d25ec6f0925b49b1ba82bbf1294ea5f72`。不再切换 USB MAVLink／DDS，原 USB 路径的 Hard Fault 隔离措施继续有效。

## 1. 这次要验证什么

```text
Pixhawk 真实状态 ── TELEM1 / MAVLink ──> NX C++ 接收与检查
                                            │
                             台架：实际初始姿态 + 零推力
                             后续：原 MPC → PVA + yaw
                                            │
Pixhawk MAVLink 接收器 <── 50 Hz 周期发送 ──────┘
        │
        ├─ 模式允许时发布内部 uORB 设定值
        └─ FMU Debug → 电脑：独立回读与记录

遥控器 → PX4：由操作者选择 Offboard、接管或停机
```

分别判断三件事：消息是否正确到达、PX4 是否接受为当前内部设定值、操作者退出后 NX 是否停止并保持退出。串口写成功、配置命令 ACK、NX 收到自己发送的话题，都不能单独证明这三项。

MAVLink 的 `SET_ATTITUDE_TARGET`／`SET_POSITION_TARGET_LOCAL_NED` 不逐条返回执行 ACK。程序连续发送最新设定值，利用状态新鲜度、真实模式和独立回读确认链路；只有初始化消息频率的命令等待配置 ACK。遥控与模式包异步到达，测试覆盖“模式包先到、拨杆包后到”，避免把旧拨杆样本误判为新一次退出。

## 2. 消息与频率

唯一配置源为 [`nx_mavlink_uart_v1.json`](../configs/hardware/nx_mavlink_uart_v1.json)。下表是工程请求值／带宽预留值，**不是官方上限，也不代表已经实测达到**。原 MPC 求解为 25 Hz，执行端周期下发为 50 Hz；状态采样与求解不要求一包一算。

| 方向 | 消息 | 当前姿态台架 | 后续完整 MPC | 用途与边界 |
|---|---|---:|---:|---|
| PX4 → NX | ATTITUDE_QUATERNION | 50 Hz | 50 Hz | 实际姿态、角速度、启动时间戳 |
| PX4 → NX | RC_CHANNELS | 20 Hz | 20 Hz | 观察 SF／SA／SB 与油门，不注入遥控通道 |
| PX4 → NX | CURRENT_MODE | 请求 10 Hz | 请求 10 Hz | 实际模式；受 vehicle_status 更新频率约束，不能要求静态时必达 10 Hz |
| PX4 → NX | ESTIMATOR_STATUS | 10 Hz | 10 Hz | 估计器健康，不能替代所有原状态有效性标志 |
| PX4 → NX | HEARTBEAT | 1 Hz | 1 Hz | 身份、解锁状态与模式辅助核对 |
| PX4 → NX | EXTENDED_SYS_STATE / SYS_STATUS / BATTERY_STATUS | 各 2 Hz | 各 2 Hz | 着陆、健康、供电；动力断开时电池主题可能不更新 |
| PX4 → NX | ATTITUDE_TARGET | 20 Hz | 0 | 姿态目标遥测；仍需 Debug 核对原始 uORB 字段 |
| PX4 → NX | ODOMETRY | 0 | 50 Hz | 位置、速度、姿态、角速度、协方差及汇总重置计数 |
| PX4 → NX | HIGHRES_IMU | 0 | 50 Hz | 诊断比力；不能直接代替 MPC 世界系加速度 |
| PX4 → NX | POSITION_TARGET_LOCAL_NED | 0 | 20 Hz | 位置控制器下游目标，不是原始 trajectory_setpoint 的回显 |
| NX ↔ PX4 | TIMESYNC | 请求／响应各预留 10 Hz | 同左 | 时钟诊断；不重复启用同方向额外主动流 |
| NX → PX4 | HEARTBEAT | 1 Hz | 1 Hz | 伴随计算机身份；不能单独维持 MAVLink Offboard |
| NX → PX4 | SET_ATTITUDE_TARGET | 50 Hz | 0 | 当前台架限定实际初始姿态、零推力 |
| NX → PX4 | SET_POSITION_TARGET_LOCAL_NED | 0 | 50 Hz | 后续原 MPC 最终 PVA 与 yaw |

姿态台架忽略三个角速度目标，`type_mask=7`；位置编码采用 `LOCAL_NED`，`type_mask=2048`，忽略 yaw rate、保留 P/V/A/yaw，force 标志关闭。ENU→NED 为 `[y,x,-z]`；yaw 为 `wrap(π/2−yaw_ENU)`。这些编码已通过原 MPC 输出的真实 NX 离线回放；当前实机台架可执行文件只允许姿态零推力，不能发送位置目标。

## 3. 带宽与已批准的配置

8N1 下每字节占 10 个串行位。921600 baud 对应**每个方向** 92160 B/s；全双工上下行分别核算。按最大载荷、MAVLink 2 包开销及每方向 2500 B/s 异步余量计算，不依赖 MAVLink 尾部零字节截断来满足预算。

| 工况 | PX4 → NX，含余量 | NX → PX4，含余量 | UART 占用（下行／上行） |
|---|---:|---:|---:|
| 当前台架，未签名包 | 8901 B/s | 5971 B/s | 9.66% / 6.48% |
| 后续 MPC，未签名包 | 25181 B/s | 6071 B/s | 27.32% / 6.59% |
| 当前台架，预留签名尺寸 | 10552 B/s | 6764 B/s | 11.45% / 7.34% |
| 后续 MPC，预留签名尺寸 | 28132 B/s | 6864 B/s | 30.53% / 7.45% |

签名一行只是容量预留，本次没有配置或验收 MAVLink 签名。带宽不是端到端延迟：例如最大 65 字节的位置设定值，在该波特率下的串行占用约 0.71 ms，仍不含调度、排队、PX4 消费和控制响应。

2026-10-30 调整前实读 TELEM1 为 57600 baud、发送预算 1200 B/s，`tx rate mult=0.302`，不满足选定的消息组合。用户批准后仅修改以下五项；重启后回读全部一致，实际链路为 921600、Custom、流控关闭、转发关闭，`tx rate mult=1.000`。这只证明配置生效，新请求频率仍待实测。

| PX4 参数 | 修改前原值 | 已应用值 | 原因 |
|---|---:|---:|---|
| SER_TEL1_BAUD | 57600 | 921600 | 提供足够容量与串行发送余量 |
| MAV_0_MODE | 0 | 1 | Custom；由本项目显式请求消息流 |
| MAV_0_RATE | 1200 | 30000 | 为上述最大尺寸组合保留发送预算 |
| MAV_0_FLOW_CTRL | 2 | 0 | 三线 TX/RX/GND 未接 RTS/CTS |
| MAV_0_FORWARD | 1 | 0 | 避免无关链路转发占用此专用控制接口 |

MAV_0_CONFIG 保持 101（TELEM1，`/dev/ttyS6`）。参数应用工具默认只读；必须有单独批准才能加 `--apply-approved-profile --power-disconnected` 写入。原参数先保存到电脑，再逐项写入、回读。工具不重启飞控、不切 USB、不发解锁或模式命令。应用后由操作者重启飞控，再核对参数与 UART 实际状态。回退以电脑保存的原值为准，不能重置全部飞控参数。

NX 程序只在当前会话设置 `/dev/ttyTHS1` 的 termios，结束恢复原设置；使用内存可执行文件，不新增安装、系统服务或实验日志。日志回传电脑。

## 4. 三步台架验收

每一步单独形成证据，前一步不通过就定位该项。四桨拆下，电调动力物理断开；实际拨杆动作等待当次记录器就绪后指导。

1. **真实状态输入**：应用已批准配置并重启后，在 921600 下仅请求状态流，检查消息身份、CRC、实际刷新率、最大间隔、重复／倒退时间戳与 NX 内核串口错误。姿态要求 100 ms 内新鲜、RC 150 ms、心跳 1.5 s、CURRENT_MODE 1.2 s；这些是本台架守卫阈值，不是官方实时性标准。当前已完成原 57600 配置下的纯 C++ 零发送接收检查；不据此宣称新频率合格。
2. **内部设定值回读**：C++ 使用实际初始姿态、零推力，50 Hz 发送，单次流最长 60 s。操作者选择 Offboard，COM13 独立读取 `offboard_control_mode`、`vehicle_control_mode`、`vehicle_attitude_setpoint`、`vehicle_status`，核对真实模式、更新时间、`q_d`、`thrust_body=[0,0,0]`、忽略 yaw rate 的语义。输出计数或同一串口的遥测回显不作为唯一验收。
3. **接管与退出**：一次用 SA 切回 Altitude，验证模式退出、C++ 停发且不会自动重入；另一次进入 Offboard 后 3 s 主动停止设定值，继续观察 PX4 退出与最终状态。COM_OF_LOSS_T 当前为 1 s，COM_OBL_RC_ACT=0，但定位无效时可能发生进一步降级，必须记录实际结果。未解锁模式切换不能替代已解锁失效保护测试；后者仅在动力断开、预检允许且操作者按当次指引操作 SF 的条件下进行。当前 COM_RC_OVERRIDE=1 不提供 Offboard 摇杆夺权，本轮人工接管使用 SA。

本固件的 MAVLink 接收器只在已处于 Offboard 时向内部发布这些控制设定值。因此“发送了包但原始 uORB 未刷新”不能立即归因于丢包，要同时看模式。`POSITION_TARGET_LOCAL_NED` 来自下游 `vehicle_local_position_setpoint`，以后位置 MPC 的原始 PVA 验收仍要独立读 `trajectory_setpoint`。

状态过期、时钟倒退、异常油门、停机请求、发送积压或调度严重停顿会结束当前发送；状态恢复不自动重入。本工具从不发解锁、模式请求或执行器覆盖命令。没有设置放宽预检或隐藏故障的参数。

## 5. 之前的 MPC 什么时候测试

**已经在测：本次把原 ABI 4 MPC 再放到真实 NX 上，完成 1281 个独立快照回放，并新增 MAVLink 序列化／反序列化逐字段验证，1281/1281 通过。** 原算法库、预测模型和参数未修改。它检查真实 NX 上的计算、接口适配、输出编码与发布后提交规则，不开串口，不代表连续历史闭环或物理控制已通过。

下一层按以下顺序推进：

| 层级 | 内容 | 当前进度／进入条件 |
|---|---|---|
| 原 MPC 计算与输出接口 | 历史输入→原 C++ MPC→新 MAVLink 编码 | 本次已完成；真实 NX 1281/1281 |
| 真实通信与退出 | C++ 真实状态→PX4 内部设定值→人工接管／断流退出 | 原配置纯接收已通过；新配置已应用，待 NX 网络恢复后验收 |
| 真实状态影子运行 | 真实机体及关节状态→原 MPC，仅记录输出 | 需完整、同时间基准、有效的机体与 TS20 状态；禁止用零值或旧日志伪装缺失输入 |
| 位置 MPC 实机无动力发送 | 真实 MPC PVA→PX4 内部原始 trajectory_setpoint | 上一步通过、外部定位有效、状态重置语义完整后进行 |
| 动力闭环 | 限定空间末端保持→普通八字→快速八字 | 后续单独准备与验收，当前通信检查不批准这一层 |

当前还缺：有效水平位置／速度及航向、真实 TS20 关节角／角速度和实际指令历史，以及与原模型一致的世界系加速度与有效性／重置标志。标准 ODOMETRY 没有加速度；HIGHRES_IMU 是机体系比力，滤波与物理定义不同，不能直接填入原 MPC 的世界系加速度字段。应补齐经验证的状态适配或明确增加遥测字段，并测试时间对齐与重置行为，再进入实时影子 MPC。动捕维护期间，原 MPC 的离线回放和此前 NX 在环测试可以继续；真实位置闭环不以固定日期替代这些条件。

## 6. 代码导航与复现

| 文件 | 对应工作 |
|---|---|
| `configs/hardware/nx_mavlink_uart_v1.json` | 消息、刷新率、带宽与待批准参数的唯一配置源 |
| `configs/hardware/nx_mavlink_dialect_lock.json` | 固件对应消息定义与 XML 散列 |
| `scripts/prepare_nx_mavlink_dialect.py` | 电脑端验证定义、生成匹配的 C/Python 协议头 |
| `scripts/nx_mavlink_bandwidth.py` | 不连接硬件的上下行容量计算 |
| `scripts/prepare_px4_uart_profile.py` | 默认只读；显式批准后仅写入上述五项参数 |
| `hardware/nx/mavlink_uart/include/flyarm_mavlink/contract.hpp` | 状态新鲜度、一次性退出、ENU/NED 与姿态/PVA 编码 |
| `hardware/nx/mavlink_uart/src/bench.cpp` | C++ 串口接收、周期发送、内存记录；实机只允许姿态零推力 |
| `hardware/nx/mavlink_uart/tests/` | 纯 C++ 与虚拟串口故障测试 |
| `scripts/build_nx_mavlink_uart.py` | NX 内存构建／测试，ELF 回传电脑 |
| `scripts/run_nx_mavlink_uart_bench.py` | 运行已验证 ELF，默认 observe 零发送；内存退出清理 |
| `scripts/observe_px4_uart_bench.py` | COM13 独立限时只读记录模式、原始设定值与输出 |
| `scripts/summarize_nx_mavlink_capture.py` | 电脑端按固定协议定义复核 CRC、实测频率与时间戳 |
| `control_packages/x650_mpc_controller_v1/examples/hardware_replay.cpp` | 原 MPC 输出→CDR 与 MAVLink 双编码核验 |
| `scripts/run_nx_full_interface_replay.py` | 指定已锁定输入及 `--mavlink-generated`，在真实 NX 回放 |

先运行脚本的 `--help` 查看必需路径；所有输出使用新的电脑目录，避免覆盖历史证据。生成协议头、编译和离线回放都不要求 sudo。台架脚本、测试脚本和 MPC 算法库分开，不能把 PTY 专用可执行文件用于真实串口。

源码依据：[固定版本 MAVLink 接收器](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mavlink/mavlink_receiver.cpp)、[固定版本 ODOMETRY](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mavlink/streams/ODOMETRY.hpp)、[固定版本 CURRENT_MODE](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mavlink/streams/CURRENT_MODE.hpp)、[MAVLink 2 序列化](https://mavlink.io/en/guide/serialization.html)、[PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard)。
