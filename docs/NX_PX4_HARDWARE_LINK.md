# NX → C++ 状态处理 → PX4 指令处理：台架验证

本阶段承接 [ROS 2 状态接收](NX_PX4_ROS2_STATE.md)。当前实机缺少有效水平定位、航向和真实关节反馈，因此先完成状态适配与诊断指令闭环。**诊断指令返回成功不等于轨迹设定值验收，更不等于 MPC 已接管真机。**

2026-10-28 已完成本阶段实机检查：一次 401 请求返回匹配的 ACCEPTED，往返 10.88 ms；完整观察窗口中 3,001 次状态检查均正确禁止使用无效定位。见 [实机记录与限制](validation/nx_hardware_link_20261028/README.md)。

## 程序实际做什么

```text
PX4 真实姿态 / 位置 / 角速度 / 状态
             ↓ ROS 2
NX C++：时间戳、有效标志、坐标变换、估计器重置检查
             ├─ 状态不完整：禁止生成保持候选，输出具体原因
             ├─ 状态完整：仅在内存生成 ENU→NED 保持候选，不发布
             └─ 未解锁台架：一次 RUN_PREARM_CHECKS (401)
                                  ↓
                         PX4 Commander 处理
                                  ↓
                      VehicleCommandAck → NX 校验
```

两个运行模式：

- `shadow`（默认）：没有任何飞控输入发布器。
- `diagnostic_ack`：只发布一次固定编号 401，刷新预飞检查。目标系统/组件来自真实 `VehicleStatus`，所有参数为零。没有解锁、Offboard、轨迹、舵机或电机发布器。3 秒未收到匹配回复则失败，不自动重发。这个超时是单次诊断事务的截止时间，不进入连续控制的逐周期门控。

PX4 对 401 的 `ACCEPTED` 表示执行了检查请求，**不表示飞行前检查全部通过**。代码依据当前固件 [Commander.cpp](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/Commander.cpp#L1356)。

## 代码对应

| 文件 | 职责 |
|---|---|
| [link_contract.hpp](../ros2_ws/src/flyarm_hardware_state/include/flyarm_hardware_state/link_contract.hpp) | 真实状态质量检查、坐标转换、PVA 编码、一次性诊断请求与 ACK 匹配 |
| [px4_link_probe.cpp](../ros2_ws/src/flyarm_hardware_state/src/px4_link_probe.cpp) | ROS 2 订阅、50 Hz C++ 处理、默认旁路、显式诊断模式、每秒报告 |
| [link_contract_test.cpp](../ros2_ws/src/flyarm_hardware_state/test/link_contract_test.cpp) | 坐标、无效状态、时间戳停滞/倒退、重置、消息错配及超时 |
| [probe_link_ros.py](../ros2_ws/src/flyarm_hardware_state/test/probe_link_ros.py) | 独立 ROS 域中的模拟飞控，检验不发送、单次发送和丢失回复三个场景 |
| [build_nx_hardware_link.py](../scripts/build_nx_hardware_link.py) | 电脑端打包、源码散列和编译记录回收 |
| [nx_hardware_link_build_remote.py](../scripts/nx_hardware_link_build_remote.py) | 默认只在 NX `/dev/shm` 编译与测试；另行授权才安装到用户目录 |
| [run_nx_hardware_link_session.py](../scripts/run_nx_hardware_link_session.py) | 显式选择当前 USB 协议、限时运行、日志保存电脑 |
| [nx_hardware_link_session_remote.py](../scripts/nx_hardware_link_session_remote.py) | Agent/节点启动和退出；保持原 NX 系统配置 |
| [summarize_nx_hardware_link.py](../scripts/summarize_nx_hardware_link.py) | 完整窗口验收，禁止将诊断 ACK 写成轨迹或飞行验收 |

状态处理仍由 C++ 完成。Python 负责离线测试和进程启动、结果回收，不参与控制周期。

## 适配规则与限制

1. 原点沿用 PX4 本地原点，ENU/NED 互换为 `[y,x,-z]`。真实硬件编码不复用仿真的 `ref_alt` 高度偏置。姿态同时变换世界系和机体系。
2. 姿态、位置和里程计的新消息与新采样均须持续推进；重复消息不能刷新数据有效期。诊断预算为这三类 100 ms、飞控状态 1 s；不把它们宣称为最终 MPC 的时序参数。
3. 跨话题采样时刻差超过 50 ms 禁止生成保持候选。目前只检查时间接近，没有声称实现插值或预测到统一时刻。
4. 位置、速度、航向有效标志必须成立。四元数和数值必须有效。估计器重置计数变化会锁住本次保持候选，需要新会话重新初始化；尚未实现飞行中的参考系重定位。
5. PVA 的加速度为显式前馈值；未使用的 jerk 与 yaw rate 为 NaN。保持候选沿用当前机体位置/航向。这是编码检查，不调用全身 MPC 求解器。
6. 命令确认匹配编号 401、回复目标、内部来源、回复时间戳和 NX 本机事务时限。迟到、重复或属于其他组件的确认不能通过。

## 编译与软件测试

默认 `--action check` 在 `/dev/shm` 构建，使用已有 ROS 2 和消息依赖；不会安装程序，不启动 Agent，不打开 USB。软件测试限于 `ROS_DOMAIN_ID=210` 和 `ROS_LOCALHOST_ONLY=1`，这些只是子进程环境变量。生成物、ROS 日志和临时文件结束即清理，结果通过 SSH 保存到电脑。

```powershell
python scripts/build_nx_hardware_link.py --host NX_ADDRESS --ssh C:/Windows/System32/OpenSSH/ssh.exe --identity C:/Users/USER/.ssh/id_ed25519 --known-hosts artifacts/NX_KNOWN_HOSTS --output artifacts/link_build_check.log --action check
```

获使用者明确许可后，添加 `--action install --approved-user-deployment`。安装位置是 `/home/jetson/flyarm/ros2_hw_runtime/v1/link_check/<源码散列前12位>/`，保留之前的只读探针。不存在 sudo、网络配置、系统 ROS、启动项或 shell 配置修改；已有相同目录不覆盖。

## 实际台架执行与验收

实机部署、诊断发送前先征得用户许可，确认未解锁、动力断开。参数标志不代替实际许可。实机运行采用 15 秒初始化加 60 秒观察；完整日志保存电脑，NX 只保留程序、依赖、版本清单。

```powershell
python scripts/run_nx_hardware_link_session.py --host NX_ADDRESS --ssh C:/Windows/System32/OpenSSH/ssh.exe --identity C:/Users/USER/.ssh/id_ed25519 --known-hosts artifacts/NX_KNOWN_HOSTS --source-sha256 FULL_BUILD_HASH --usb-mode existing-dds --output artifacts/link_session.jsonl --approved-diagnostic-command --bench-power-disconnected
python scripts/summarize_nx_hardware_link.py --input artifacts/link_session.jsonl --output artifacts/link_summary.json
```

`existing-dds` 仅在已确认上一次临时 DDS 仍有效时使用；如果飞控已经重启恢复 MAVLink，改为 `fresh-mavlink` 并指定 `--dialect-source`。程序不会在 DDS 串口试发 MAVLink。切换沿用上一阶段固定命令，检查原固件和 USB 恢复配置，不修改持久飞控参数。

验收分开报告：

| 项目 | 本阶段判据 |
|---|---|
| 状态到 C++ | 完整窗口、消息持续推进、坐标变换正确；无效定位明确禁止保持候选 |
| 诊断请求到飞控 | 一次 401，有匹配的 PX4 ACK，结果为 ACCEPTED |
| 未越出范围 | 无解锁、无 Offboard、无飞行/执行器设定值；没有 HIL 注入 |
| 轨迹消息在 PX4 内部的字段 | `NOT_TESTED`，不能用诊断 ACK 替代 |
| 真实全身 MPC 与位置闭环 | 待有效定位、真实关节反馈及后续执行接口验收 |

Agent 和节点退出后，飞控仍可能保持临时 DDS。重启未解锁飞控恢复原 USB MAVLink。下一步在定位可用后，准备未解锁的轨迹字段回读验证，再安排接管退出；需要飞控内部证据，NX 自己订阅发送话题不能作为飞控已消费的证据。
