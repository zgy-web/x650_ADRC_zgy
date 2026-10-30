# NX / Pixhawk Offboard 台架检查（2026-10-30）

本轮检查真实 NX 接收状态、发送 Offboard 存活流与姿态设定值，以及 PX4 模式归属和退出。仅用于电调动力断开台架。当前没有有效水平位置，使用实际姿态作为固定目标，推力设为零；这是通信与模式生命周期检查，不是位置 MPC 飞行验证。

**当前实机入口已停用。** 2026-10-30 实测在 USB MAVLink/DDS 切换阶段遇到 DDS 强制停止，随后 `mavlink stop` 触发飞控 Hard Fault；两项 Offboard 验收均未通过。详见[故障记录与恢复条件](validation/nx_offboard_bench_20261030/README.md)。下述流程是待修复后的验收设计，不是当前可执行的实机操作指南。C++ 与 ROS 软件测试可独立运行。

## 已准备的实现

| 代码 | 用途 |
|---|---|
| `ros2_ws/src/flyarm_hardware_state/src/px4_offboard_bench.cpp` | C++ 接收真实状态；收到一次显式 start 后，以 50 Hz 发布姿态设定值及 OffboardControlMode |
| `ros2_ws/src/flyarm_hardware_state/include/flyarm_hardware_state/offboard_bench_contract.hpp` | 状态新鲜度、时间倒退、遥控有效性、最低油门、姿态重置、一次性退出规则 |
| `ros2_ws/src/flyarm_hardware_state/test/probe_offboard_ros.py` | localhost、隔离 ROS domain 210 的软件测试，不连接飞控 |
| `scripts/build_nx_offboard_check.py` | NX `/dev/shm` 编译并测试；ELF、清单、日志回传电脑，临时目录删除 |
| `scripts/nx_offboard_bench_session_remote.py` | NX 内存运行被测 ELF 与既有 Agent；控制管道断开时退出，日志流向电脑 |
| `scripts/run_nx_offboard_bench.py` | 电脑通过 COM13 独立读回状态、控制标志、设定值、供电和输出；临时切换及恢复 USB 通信 |

C++ 发送窗口最长 60 秒。人工切走 Offboard 后立即停止发送，并将本次会话永久置为退出状态；恢复数据或模式消息不能使其再次接管。状态过期、重复不更新新鲜度、时间倒退、油门异常或 DDS 订阅端消失均停止发送。程序没有解锁、模式请求或执行器覆盖命令。

## 两种工况

1. **人工接管**：先建立稳定发送流，操作者使用 SA 进入 Offboard，再切回 SA 上档 Altitude。验收 PX4 曾处于 nav_state=14 且 Offboard 标志有效，随后回到高度模式，并且 NX 停止发送、没有自动重入。
2. **发送中断**：NX 观察到 Offboard 后等待 3 秒，停止存活流和设定值。独立 Debug 继续记录退出、失效保护、锁定和最终恢复。若要验证“已解锁时”的失效保护，须在电调动力物理断开的条件下，由操作者使用 SF 解锁；未解锁记录不能替代该项。

两项都保持拆桨，动力电池断开。开始时 SF 负100%、SA 上档、SB 下档、油门最低。每次动拨杆前由当前会话给出具体顺序；不预先操作。SF 解锁只在明确启用 `--operator-arming-allowed` 的工况中允许，工具不发送解锁指令。

## 权限及恢复范围

实际飞控测试须先取得用户确认：运行临时 NX 程序、启动既有 Agent、把 USB MAVLink 临时切换为 serial XRCE-DDS，以及完成两种台架工况。原有用户要求“改 NX 配置前先问”继续有效。

不修改 NX 系统 ROS、网络、开机项、shell 配置，不新增持久软件安装。代码与暂存文件使用 `/dev/shm`，结束删除；既有依赖仍在 `/home/jetson/flyarm/ros2_hw_runtime/v1/`。电脑保存日志及结果。本 Offboard 工具不修改 PX4 持久参数。

修订后的清理顺序是：停止 NX 发布器 → 保持 Agent 存活并停止 PX4 DDS → 确认 DDS 正常退出 → 释放 Agent、核对进程清理 → 才允许恢复原 USB MAVLink。遇到超时、强制停止、Hard Fault 或进程清理缺少确认，不再自动恢复或重试。`mavlink status` 显示实例存在，只能说明存在，不能证明协议切换后健康。这些修订目前只通过本地故障注入；**不能据此宣称固件故障已修复**。

## 当前证据及边界

2026-10-30：C++ 核心 19 个用例通过；NX ARM64 编译成功，3 项 CTest、原通信/设定值回归，以及新 6 个 ROS 软件场景通过。新场景包括人工退出、计划中断、状态缺失、重复时间戳、倒退时间戳和油门异常。构建源清单 SHA-256 `d26280593b016815e9ff6876b13ceffa710f5dd4115c167a975c879614d03cab`；被测 ELF SHA-256 `144a02e5b3c73bbe83bb26e5081202344cd99ab21cf30588d691f5658930aa39`。软件测试尚不代表实机 Offboard 接管通过；实机两次尝试均在发送控制流之前终止。

飞控参数 COM_RC_OVERRIDE=1，目前没有启用 Offboard 摇杆夺权；本轮人工接管使用 SA 模式开关。COM_OBL_RC_ACT=0 要求回到位置模式，但当前位置不可用，因此必须记录实际降级结果，不能把“配置为 Position”写成“已成功位置保持”。无有效定位时，本台架检查不能批准位置 MPC 试飞。

源码依据：[固定 PX4 版本的 DDS 主题表](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.yaml)、[USB 自启动实现](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/drivers/cdcacm_autostart/cdcacm_autostart.cpp)、[PX4 Offboard 文档](https://docs.px4.io/main/en/flight_modes/offboard)。
