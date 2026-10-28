# NX / Pixhawk ROS 2 只读状态通道

后续的 C++ 状态适配与一次性诊断命令验证见 [NX/PX4 台架指令链](NX_PX4_HARDWARE_LINK.md)。原 `px4_state_probe` 保持只读，新的诊断程序单独选择运行。

本阶段连接真实 Orin NX 与 Pixhawk 6X Pro，验证状态传输、消息版本、坐标变换和接收连续性。它不执行 MPC，不发布轨迹、模式、解锁或执行器命令。物理状态接收和数值坐标转换使用 C++；电脑端 Python 负责有限时的部署、启动、结果保存，不进入定周期控制链。

## 已部署的版本与目录

NX 使用现有 Ubuntu 22.04 / ROS 2 Humble。新增软件统一放在：

```text
/home/jetson/flyarm/ros2_hw_runtime/v1/
├── agent/                     Micro XRCE-DDS Agent 2.4.2
├── ros/                       PX4 1.17 消息接口
├── probe/                     flyarm_hardware_state C++ 状态检查程序
│   └── flyarm_probe_source.json
└── source_manifest.json
```

安装不使用 sudo，不修改 `/opt/ros`、网络配置、shell 配置或开机启动项。源码解包、编译缓存和 ROS 临时日志放 `/dev/shm`，结束时清理；实验记录通过 SSH 返回电脑。NX 持久保留编译代码、运行依赖及源码版本清单。原有其他项目不属于这次迁移范围。

| 项目 | 固定版本 |
|---|---|
| 实际飞控 | PX4 1.17.0，`d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| px4_msgs | release/1.17，`86d8239e962f6939e05c3737784f60c02fa884db` |
| Agent | v2.4.2，`57d086216d01ec43121845d385894a25987f8a2c` |
| Fast DDS | 复用 NX Humble 已安装的 2.6.11 |
| 传输 | USB CDC，Agent serial；过程域 `ROS_DOMAIN_ID=0` |

七个订阅消息的定义已与固定 PX4 版本源码逐个核对。源码归档提交和 SHA-256 位于 [依赖清单](../configs/hardware/nx_ros2_state_dependencies_v1.json)。固件换版或自定义消息后应重新核对，不能只按版本号推定兼容。

## 代码从哪里看

| 文件 | 阅读重点 |
|---|---|
| [px4_state_probe.cpp](../ros2_ws/src/flyarm_hardware_state/src/px4_state_probe.cpp) | 七类订阅、实际状态、有效标志、接收时刻、JSON 输出；没有飞控控制发布器 |
| [state_math.hpp](../ros2_ws/src/flyarm_hardware_state/include/flyarm_hardware_state/state_math.hpp) | NED/ENU、FRD/FLU、四元数变换、新鲜度与时间戳统计 |
| [state_math_test.cpp](../ros2_ws/src/flyarm_hardware_state/test/state_math_test.cpp) | 独立坐标基向量组合、错误四元数、版本话题和时序异常测试 |
| [prepare_nx_ros2_state_dependencies.py](../scripts/prepare_nx_ros2_state_dependencies.py) | 只在电脑下载固定公共依赖，校验归档散列 |
| [deploy_nx_ros2_state_dependencies.py](../scripts/deploy_nx_ros2_state_dependencies.py) | 经明确授权后，将依赖发送到 NX 内存并安装至用户目录 |
| [nx_ros2_dependencies_remote.py](../scripts/nx_ros2_dependencies_remote.py) | tmpfs 构建、现有系统依赖复用和清理；拒绝覆盖非空运行目录 |
| [deploy_nx_ros2_state_probe.py](../scripts/deploy_nx_ros2_state_probe.py) / [远端构建](../scripts/nx_ros2_probe_build_remote.py) | 打包当前 C++ 源码、编译、CTest、记录源码散列 |
| [run_nx_px4_ros2_state_session.py](../scripts/run_nx_px4_ros2_state_session.py) | 电脑端限时启动与完整记录保存；终端仅显示简短进度 |
| [px4_usb_dds_session_remote.py](../scripts/px4_usb_dds_session_remote.py) | 固定 NSH 命令白名单、未解锁检查、临时 USB 协议切换、Agent/探针清理 |
| [summarize_nx_ros2_state_session.py](../scripts/summarize_nx_ros2_state_session.py) | 电脑端结果摘要；缺失最终记录不能通过，通信通过不等于飞行通过 |

## 数据含义

```text
Pixhawk 传感器 / EKF → uORB → uXRCE-DDS Client → USB
    → NX 用户目录 Agent → ROS 2 Best Effort / KeepLast(1)
    → C++ 状态检查 → stdout → SSH → 电脑 JSONL
```

订阅 `/fmu/out/vehicle_attitude`、`vehicle_local_position_v1`、`vehicle_odometry`、`vehicle_status_v1`、`estimator_status_flags`、`sensor_combined`、`timesync_status`。版本后缀由消息的 `MESSAGE_VERSION` 确定。角速度来自 `VehicleOdometry.angular_velocity`；本版固件默认未输出独立的 `vehicle_angular_velocity` DDS 话题。

位置/速度：`ENU = [NED.y, NED.x, -NED.z]`；机体系角速度：`FLU = [FRD.x, -FRD.y, -FRD.z]`。姿态同时变换世界系和机体系，四元数顺序始终为 `w,x,y,z`。四元数不有限或模长明显异常时，输出不可用，不能用归一化掩盖无效初值。

`max_receipt_gap_ms` 使用 NX 单调时钟，衡量两次订阅回调之间的间隔；不包含 SSH 返回电脑的时间。它不是单向 USB 延迟，也不是 MPC 求解耗时。`timesync.round_trip_time_us` 是飞控报告的时间同步往返时间；摘要统计的是每秒快照，不是每个包的延迟分布。源时间戳与 NX 接收时钟分开记录，不能直接用两个不同时钟相减。

状态探针的 `core_topics_received` 只要求统计窗口内核心话题收到至少两条；进程返回 0 仅代表接收成功。单独的摘要继续检查完整统计窗口、核心接收新鲜度、时间戳及未解锁状态。250 ms 核心状态/1 s 飞控状态的新鲜度标志是本次诊断条件，不是最终 MPC 的执行准入门槛。估计器和同步状态属于较低频状态，不能套用 100 Hz 传感器的间隔要求。

`position_velocity_usable` 还要求 PX4 的 `xy_valid/z_valid/v_xy_valid/v_z_valid`。即使位置字段是有限数值，也不能替代这些有效标志。正式状态适配器还须加入航向可用性、估计器重置连续性处理、跨话题时间对齐、关节实测反馈以及故障退出；当前探针没有这些控制职责。

## 复现

先完成 [USB MAVLink 只读检查](NX_PX4_USB_BRINGUP.md)。下面使用示例主机 `192.0.2.10`，应替换成实际 NX 地址，SSH 主机记录须经过核对。执行配置或部署之前须获得使用者许可；命令行标志不能代替许可。

在电脑准备依赖，随后安装到 NX 用户目录（新机器只执行一次）：

```powershell
python scripts/prepare_nx_ros2_state_dependencies.py --output artifacts/nx_ros2_sources
python scripts/deploy_nx_ros2_state_dependencies.py --host 192.0.2.10 --ssh C:/Windows/System32/OpenSSH/ssh.exe --identity C:/Users/USER/.ssh/id_ed25519 --known-hosts artifacts/NX_KNOWN_HOSTS --sources artifacts/nx_ros2_sources --output artifacts/nx_ros2_dependency_build.log --apply-approved-user-directory
python scripts/deploy_nx_ros2_state_probe.py --host 192.0.2.10 --ssh C:/Windows/System32/OpenSSH/ssh.exe --identity C:/Users/USER/.ssh/id_ed25519 --known-hosts artifacts/NX_KNOWN_HOSTS --output artifacts/nx_ros2_probe_build.log --apply-approved-user-directory
```

电脑已经准备了与飞控匹配的 MAVLink 解析代码后，执行临时 DDS 状态检查：

```powershell
python scripts/run_nx_px4_ros2_state_session.py --host 192.0.2.10 --ssh C:/Windows/System32/OpenSSH/ssh.exe --identity C:/Users/USER/.ssh/id_ed25519 --known-hosts artifacts/NX_KNOWN_HOSTS --dialect-source artifacts/usb_dialect_v1_17_0/px4_development.py --expected-firmware-commit d6f12ad1c4f70ad3230afd7d86e971421e02fef4 --output artifacts/nx_ros2_state_run01.jsonl --approved-temporary-usb-switch
python scripts/summarize_nx_ros2_state_session.py --input artifacts/nx_ros2_state_run01.jsonl --output artifacts/nx_ros2_state_run01_summary.json
```

首次切换会先验证飞控未解锁、固件提交、DDS 未运行和 `SYS_USB_AUTO=2`，再执行固定命令：

```text
uxrce_dds_client start -t serial -d /dev/ttyACM0 -b 921600; mavlink stop -d /dev/ttyACM0
```

它改变当前 USB 的运行协议；不修改保存的飞控参数或启动项。原 MAVLink USB 在测试中不可用。状态记录包含前 15 秒初始化，再独立统计后 60 秒；未收到话题会明确失败，不自动关闭时间同步、延长门槛或发布控制命令。

若已经确认当前飞控仍处于这次临时 DDS 模式，可在同一命令追加 `--observe-existing-dds`，重启 Agent/探针并检查重连；该模式不在 DDS 串口上发送 MAVLink，也不会重新验证固件身份。**飞控已重启恢复 MAVLink 后，必须使用首次切换模式。** 单根 USB 不要同时运行 QGC/MAVLink 与 Agent。

测试结束自动停止 Agent 和 C++ 探针。飞控上的临时 DDS 客户端仍在；**重启未解锁飞控后，原 `SYS_USB_AUTO=2` 会恢复 USB MAVLink**。不要以 Agent 退出推断 USB 已恢复。停止用户目录程序即可停用本次 NX 依赖，不需要修改系统安装。

## 已取得的证据与限制

完整结果见 [2026-10-28 状态通道检查](validation/nx_ros2_state_20261028/README.md)。收到状态和定位有效是两个独立检查；目前水平定位与航向未就绪，仍需接入并核验实际动捕等定位来源。本阶段不修改 EKF 参数，不发送 Offboard、轨迹或解锁命令。

官方依据：[PX4 1.17 uXRCE-DDS](https://docs.px4.io/v1.17/en/middleware/uxrce_dds)、[实际固件 DDS 客户端](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/uxrce_dds_client.cpp)、[实际固件消息导出配置](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.yaml)。
