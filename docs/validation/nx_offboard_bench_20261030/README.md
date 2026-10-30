# NX Offboard 台架：软件检查通过，实机通信故障阻塞

## 结论

人工接管和发送中断退出两项实机验收均未完成。第一次尝试 DDS 未就绪，第二次停止 USB MAVLink 时飞控发生 Hard Fault 并重启。两次均未开始 Offboard 存活流或姿态设定值发送，动力电池已物理断开，也没有工具发出解锁、模式请求或执行器命令。

四电机旋转后的遥控 SB 实际停转是此前单独完成的通过项，见[其记录](../px4_powered_stop_20261030/README.md)。不能将它写作“NX Offboard 接管通过”。

## 事件顺序及证据

1. 第一次 `manual_takeover_v1`：原 USB MAVLink 停止正常；Agent 打开串口并建立 XRCE 会话和 participant，但未收到可用状态、未出现 `stream_started`。NX 程序先结束，旧清理流程先停 Agent，再停 PX4 DDS。
2. DDS 停止返回 `timeout, forcing stop`。旧工具随后启动 USB MAVLink，发现实例存在并写入 `usb_mavlink_restored=true`。保留 [attempt_1_report.json](attempt_1_report.json) 原样；**这个历史字段不能作为正常恢复或健康证明**。
3. 第二次 `manual_takeover_v2`：进入 Offboard 前的未解锁、kill、零输出和动力断开核对通过；执行 `mavlink stop -d /dev/ttyACM0` 时发生 Hard Fault，未再启动 Agent/DDS。
4. 飞控运行时间从约 2000 秒变为约 22 秒。操作者明确答复“没有操作飞控供电”。调试口故障转储直接记录了 Hard Fault，不能按人为重启解释。
5. 停止本轮实际 Offboard 测试。重启后 SB 虽在下档，实际 kill=False；操作者按要求 SB 中→下后，最终回读为 kill=True、未解锁、输出全零，随后关闭 COM13。

主要原始文件：

| 文件 | 内容 |
|---|---|
| [protocol_failure_records.json](protocol_failure_records.json) | 从两次串口记录摘取的强制停止、恢复命令和完整故障原文，含原始字节 base64 |
| [hardfault.txt](hardfault.txt) | 故障命令至任务/栈转储 |
| [attempt_1_remote.jsonl](attempt_1_remote.jsonl) | 第一次 NX Agent/探针事件；没有开始发送事件 |
| [attempt_2_report.json](attempt_2_report.json) | 第二次故障前状态及失败结果 |
| [incident.json](incident.json) | 证据界限、用户无供电操作确认、原始完整档案哈希 |
| [final_state_report.json](final_state_report.json) / [console](final_state_console.json) | 故障后最后只读回读，串口已释放 |

故障任务 `mavlink_if2`，PC `08030118`、LR `08022bfd`、CFSR `01000000`、HFSR `40000000`；R0/R2/R3 中出现 `deadbeef`。这些值保留供匹配固件 ELF 符号化，**不据此直接断言某个函数、内存越界或硬件损坏**。飞控版本 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`；当前没有完成匹配 ELF 的符号化。

## 能确定与尚不能确定的原因

能确定故障发生在 PX4 USB 通信协议停止/切换阶段，而不是 MPC 求解阶段。已发现旧测试工具的两个问题：过早停止 Agent；把 DDS 强制停止后的实例重建当作恢复成功。它们需要修正，但还没有证明“仅修改顺序即可修复这次固件 Hard Fault”。

固定版本 [DDS 初始化源码](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/uxrce_dds_client.cpp#L297-L331) 在 participant 创建后等待时间同步收敛，该循环本身没有退出请求检查。这与此次停在初始化、停止超时的现象相容，但不是对 Hard Fault 内存原因的证明。[MAVLink 停止实现](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mavlink/mavlink_main.cpp#L2922-L3025) 仍需结合本次固件的符号文件分析。

## 已修改的本地工具

- 发布器停止后，Agent 保持存活，先让飞控 DDS 客户端退出，再释放 Agent。
- 超时、强制停止、Hard Fault、缺少停止确认均导致失败；不再自动启动 MAVLink 或重试。
- 串口读到 Hard Fault 即锁住后续协议写入。
- `run_nx_offboard_bench.py` 的实机入口在打开 COM13/SSH 前直接拒绝运行；没有命令行绕过开关。软件回放、C++ 合同测试及只读工具仍可使用。

上述改动只做本地离线故障注入，未重新在真实飞控上验证，故实机故障仍记为未修复。解除停用须另行完成可审阅的协议/固件修复与动力断开验收，不是简单删除保护判断。

本地关联回归共 107 项通过（[JUnit 记录](local_regression.xml)），包含真实故障文本拒绝、退出顺序、DDS 强制退出、发布器停止未确认、Agent 清理未确认及恢复命令报错等情况。源码与所测 NX 构建清单逐文件一致；修订的是电脑协调器和 NX 启停包装脚本，未把此前软件测试冒充为新清理流程的实机验证。

## 软件测试来源

[nx_build_report.json](nx_build_report.json)、[nx_source_manifest.json](nx_source_manifest.json)、[nx_binary_manifest.json](nx_binary_manifest.json) 和 [nx_build.log](nx_build.log) 绑定本次 NX ARM64 编译及软件测试。C++ 核心 19 个用例，NX 上 3 项 CTest、3 个原通信场景、7 个原设定值场景、6 个新 Offboard 场景均通过。编译存在两处 misleading-indentation 警告，源文件与所测 ELF 字节保持一致，并未将警告掩盖为零警告构建。

6 个新场景为人工退出、计划中断、状态丢失、重复时间戳、倒退时间戳和油门异常，均使用隔离 ROS domain 210 / localhost 模拟消息，不连接真实飞控。NX 构建目录和运行目录位于 `/dev/shm`，日志回传电脑，构建临时目录删除成功。未安装新持久软件或修改 NX 系统配置。

## 恢复测试的顺序

1. 用与实际烧录固件匹配的 ELF/映射文件解析故障 PC/LR，检查 USB MAVLink 停止与 DDS 初始化退出路径；若需要固件或 NX 配置变更，先提交具体方案征求确认。
2. 在动力断开条件下先单独验证状态接收、同步完成及正常退出，不发送 Offboard 流。禁止沿用“强制停止后立即重建”的路径。
3. 通信生命周期验收后，再分别验证固定实际姿态、零推力的 SA 人工接管和发送中断退出，保留独立 Debug 回读；最后才安排完整控制代码与定位联调。

最后状态：飞控运行约 295 秒，arming_state=1、nav_state=1、kill=True，SF关、SA上、SB下、油门 -1，所有 DShot 输出零。USB 供电重启后电池主题从未发布，因此日志中的电池电压是 null；动力断开依据操作者确认，没有把“无电池主题”伪装成实测 0 V。

另经[只读 SSH 检查](nx_final_cleanup.json)，NX 上没有遗留 `MicroXRCEAgent` 或 `px4_offboard_bench` 进程，`/dev/shm/flyarm_offboard_*` 临时目录为空。文件及关联源码哈希见 `manifest.json`。
