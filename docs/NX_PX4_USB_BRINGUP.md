# Orin NX 与 Pixhawk USB 通信检查

本指南对应真实 Orin NX＋Pixhawk 6X Pro 的第一阶段通信检查。它验证 USB、MAVLink 遥测、版本查询和参数读取，不启动 MPC、不接管飞控，不是实机飞行或实时性验收。

## 连接与权限

1. NX 使用载板原有电源；NX USB 主机口通过数据线连接飞控 USB。
2. 首次台架检查断开动力及关节电源，拆下螺旋桨。
3. 在 NX 查看 `ls -l /dev/serial/by-id/ /dev/ttyACM*`。本次枚举为 `/dev/ttyACM0`，稳定路径为 `/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00`。
4. 用户需要串口权限。本次通过 `sudo usermod -aG dialout jetson` 添加，重新建立 SSH 会话后生效。无需把设备设为所有用户可写，也无需重启。
5. 先退出其他占用同一串口的程序；诊断使用排他打开并在退出时恢复串口配置。多个外观相同的飞控可能没有唯一 USB 序列号，应进一步核对 MAVLink 硬件身份。

## 代码对应

| 文件 | 职责 |
|---|---|
| `scripts/probe_x650_nx_px4_usb.py` | 在电脑端启动 SSH，将解析代码送入 NX 内存，把结果保存到电脑 |
| `scripts/px4_usb_readonly_remote.py` | NX 上的限时串口采集、查询发送白名单、CRC 解析、频率和序号统计 |
| `scripts/prepare_px4_usb_dialect.py` | 按 PX4 发布版本固定的 MAVLink 子模块提交生成解析代码 |
| `tests/unit/test_px4_usb_readonly_protocol.py` | 验证查询白名单、危险命令拒绝、参数编码、MAVLink CRC 和帧头处理 |

这些 Python 文件只用于低频查询和通信诊断，不在正式 C++ MPC 的定周期控制链中。NX 无需安装 pymavlink 或 pyserial；代码通过 stdin 传入，关闭 Python 字节码缓存，不写源文件或实验结果到 NX。电脑端需要 Python 和 pymavlink。

## 复现命令

先读取实际固件版本，再选择对应的 PX4 release。下面示例中的 `192.0.2.10` 是文档占位地址，应替换成 NX 地址。SSH 主机记录使用已经确认的文件，严格校验主机身份。

```powershell
python scripts/probe_x650_nx_px4_usb.py --host 192.0.2.10 --ssh C:/Windows/System32/OpenSSH/ssh.exe --identity C:/Users/USER/.ssh/id_ed25519 --known-hosts artifacts/NX_KNOWN_HOSTS --duration 25 --output artifacts/usb_probe_initial.json
```

初次使用电脑安装的通用消息定义。如果出现 `UNKNOWN_n`，应按固件匹配消息定义，不把未知消息直接当成损坏数据或丢包。

```powershell
python scripts/prepare_px4_usb_dialect.py --px4-ref v1.17.0 --output artifacts/usb_dialect_v1_17_0
python scripts/probe_x650_nx_px4_usb.py --host 192.0.2.10 --ssh C:/Windows/System32/OpenSSH/ssh.exe --identity C:/Users/USER/.ssh/id_ed25519 --known-hosts artifacts/NX_KNOWN_HOSTS --dialect-source artifacts/usb_dialect_v1_17_0/px4_development.py --duration 45 --output artifacts/usb_probe_matched.json
```

消息定义来自 PX4 发布版本对应的 MAVLink gitlink，生成工具记录 URL、提交和 SHA-256。自定义固件可能需要它实际构建时的消息定义，不能仅凭版本字符串认定与官方发布完全一致。

发送白名单只允许指定参数的 `PARAM_REQUEST_READ`、指定消息的 `MAV_CMD_REQUEST_MESSAGE` 及版本兼容查询。没有解锁、模式切换、参数修改、流频率修改、HIL 注入、轨迹或执行器发送接口。查询缺失的历史参数名可能不返回响应，结果单独列出，不自动修改固件。

## 2026-10-27 实测结果

设备报告 PX4 **1.17.0**，MAVLink 2，未解锁，HIL 关闭。`UXRCE_DDS_CFG=0`，uXRCE-DDS 尚未启用。

NX 的 ROS 2 Humble 和 `rclcpp` 可用；本次加载 `/opt/ros/humble/setup.bash` 后未找到 `px4_msgs`，也未在当前 PATH 找到 `MicroXRCEAgent`。这表示仍需确认或准备对应工作空间，不能把 MAVLink 查询成功当成 DDS 已就绪。

完成 25 秒初查、60 秒匹配消息定义的复查，以及 45 秒启动/后续接收阶段检查。完整原始结果保存在电脑的 `artifacts/nx_px4_usb_20261027/`，未上传 NX。

最后一次 45 秒采集中，按预先设定的第 5 秒开始统计后续 40 秒：

| 消息 | 接收频率 | 最大接收间隔 |
|---|---:|---:|
| ATTITUDE | 100.01 Hz | 22.83 ms |
| HIGHRES_IMU | 50.00 Hz | 25.03 ms |
| ATTITUDE_QUATERNION | 49.99 Hz | 28.51 ms |
| LOCAL_POSITION_NED | 30.00 Hz | 38.12 ms |
| ODOMETRY | 29.99 Hz | 43.19 ms |

15 个指定参数的请求—响应均有记录，中位 **5.98 ms**，范围 **2.27～17.69 ms**。这些计时在 NX 本机进行，包含 PX4 处理、USB 及用户态接收调度，不包含电脑到 NX 的 SSH 往返；不是单向 USB 延迟，也不是 MPC 求解耗时。

开串口时有一个不完整数据前缀；两次序号不连续事件均发生在开始后约 **11 ms**。后续没有新的序号不连续事件、同步后的坏数据或未知消息。完整结果保留启动异常，不将其从全程记录删除。前次 60 秒记录的最大数百毫秒接收间隔发生于首秒，不能用它代表后续持续通信状态。现有证据将异常定位到开串口阶段；尚未用 USB 总线抓包区分飞控缓存、主机缓存和启动过程各自的贡献。

因此，**基础双向通信通过**，同时需要在正式程序中处理首次连接的旧数据、消息对齐和新鲜度。连接成功后不能立即把第一条收到的状态当成新鲜控制状态。

## 当前不能据此认定的事项

- `ESTIMATOR_STATUS.flags=165`：姿态、垂直速度和垂直位置相关标志存在，但没有有效水平速度和水平位置标志，并处于 constant-position 模式。收到位置数值不等于已获得可用于 MPC 的水平定位。需接入动捕或其他有效定位后再做位置闭环。
- USB MAVLink 通信成功不等于 ROS 2/uXRCE-DDS 已连通。应匹配真实固件的话题定义、消息版本与 QoS，再接入 C++ 状态接口。
- 本轮没有发送 Offboard、轨迹、解锁或关节指令，也没有验证接管退出与断链保护。
- 这是短时用户态通信观察，尚未在 MPC 负载下验证长期调度和新鲜度边界。

下一阶段先完成 C++ 状态接收与坐标/时间戳检查，再做算法旁路运行。接管和位置闭环需要有效定位及单独的无桨执行链测试。

参考：[PX4 v1.17 uXRCE-DDS](https://docs.px4.io/v1.17/en/middleware/uxrce_dds)、[MAVLink 消息与估计器标志](https://mavlink.io/en/messages/common.html#ESTIMATOR_STATUS_FLAGS)。
